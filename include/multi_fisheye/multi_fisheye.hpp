#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

class MultiFisheyePub : public rclcpp::Node
{
public:
    explicit MultiFisheyePub(const rclcpp::NodeOptions &options)
        : Node("multi_fisheye_pub", options)
    {
        gst_init(nullptr, nullptr);

        // V4L2 control table — name as it appears in YAML, V4L2 CID, sentinel = -1.
        const std::vector<std::pair<std::string, uint32_t>> v4l2_defs = {
            // exposure / framerate
            {"auto_exposure",          V4L2_CID_EXPOSURE_AUTO},
            {"exposure_absolute",      V4L2_CID_EXPOSURE_ABSOLUTE},
            {"dynamic_framerate",      V4L2_CID_EXPOSURE_AUTO_PRIORITY},
            // image quality
            {"brightness",             V4L2_CID_BRIGHTNESS},
            {"contrast",               V4L2_CID_CONTRAST},
            {"saturation",             V4L2_CID_SATURATION},
            {"hue",                    V4L2_CID_HUE},
            {"gamma",                  V4L2_CID_GAMMA},
            {"gain",                   V4L2_CID_GAIN},
            {"sharpness",              V4L2_CID_SHARPNESS},
            {"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION},
            // white balance
            {"white_balance_auto",     V4L2_CID_AUTO_WHITE_BALANCE},
            {"white_balance_temp",     V4L2_CID_WHITE_BALANCE_TEMPERATURE},
            // environment
            {"power_line_frequency",   V4L2_CID_POWER_LINE_FREQUENCY},
            // optical
            {"pan_absolute",           V4L2_CID_PAN_ABSOLUTE},
            {"tilt_absolute",          V4L2_CID_TILT_ABSOLUTE},
            {"zoom_absolute",          V4L2_CID_ZOOM_ABSOLUTE},
        };

        auto cam_names = declare_parameter<std::vector<std::string>>("cameras");
        if (cam_names.empty()) {
            throw std::runtime_error("`cameras` list is empty — nothing to open");
        }

        // Common defaults (required: capture format; optional: v4l2 controls).
        const int common_w   = declare_parameter<int>("common.cam_width");
        const int common_h   = declare_parameter<int>("common.cam_height");
        const int common_fps = declare_parameter<int>("common.cam_fps");
        const int common_rot = declare_parameter<int>("common.cam_rotation");

        std::map<std::string, int> common_v4l2;
        for (const auto &[name, _id] : v4l2_defs) {
            common_v4l2[name] = declare_parameter<int>("common." + name, -1);
        }

        // Build per-camera config (per-camera value falls back to common).
        for (const auto &cam : cam_names) {
            CameraConfig cfg;
            cfg.name     = cam;
            cfg.device   = declare_parameter<std::string>(cam + ".device");
            cfg.topic    = declare_parameter<std::string>(cam + ".topic");
            cfg.frame_id = declare_parameter<std::string>(cam + ".frame_id");

            cfg.cam_w   = declare_parameter<int>(cam + ".cam_width",    common_w);
            cfg.cam_h   = declare_parameter<int>(cam + ".cam_height",   common_h);
            cfg.cam_fps = declare_parameter<int>(cam + ".cam_fps",      common_fps);
            cfg.cam_rot = declare_parameter<int>(cam + ".cam_rotation", common_rot);

            if (cfg.cam_rot != 0 && cfg.cam_rot != 90 && cfg.cam_rot != 180 && cfg.cam_rot != 270) {
                throw std::runtime_error("[" + cam + "] cam_rotation must be one of 0, 90, 180, 270");
            }
            if (cfg.cam_rot == 90 || cfg.cam_rot == 270) {
                cfg.out_w = cfg.cam_h;
                cfg.out_h = cfg.cam_w;
            } else {
                cfg.out_w = cfg.cam_w;
                cfg.out_h = cfg.cam_h;
            }

            for (const auto &[name, id] : v4l2_defs) {
                int v = declare_parameter<int>(cam + "." + name, common_v4l2[name]);
                cfg.v4l2_params.push_back({name, id, v});
            }

            RCLCPP_INFO(get_logger(),
                "[%s] %s — capture %dx%d@%dfps, rotate=%d, publish %dx%d",
                cfg.name.c_str(), cfg.device.c_str(),
                cfg.cam_w, cfg.cam_h, cfg.cam_fps, cfg.cam_rot,
                cfg.out_w, cfg.out_h);

            cameras_.push_back(std::move(cfg));
        }

        auto qos = rclcpp::QoS(10).reliable().durability_volatile();

        for (size_t i = 0; i < cameras_.size(); i++) {
            pubs_.push_back(create_publisher<ImageMsg>(cameras_[i].topic, qos));
            open_camera(i);
            threads_.emplace_back(&MultiFisheyePub::capture_loop, this, i);
        }
        RCLCPP_INFO(get_logger(), "All %zu cameras started (raw)", cameras_.size());
    }

    ~MultiFisheyePub() override
    {
        running_ = false;
        for (auto &t : threads_)
            if (t.joinable()) t.join();
        for (size_t i = 0; i < pipelines_.size(); i++) {
            gst_element_set_state(pipelines_[i], GST_STATE_NULL);
            gst_object_unref(sinks_[i]);
            gst_object_unref(pipelines_[i]);
        }
    }

private:
    static const char *videoflip_method(int deg) {
        switch (deg) {
            case 90:  return "clockwise";
            case 180: return "rotate-180";
            case 270: return "counterclockwise";
            default:  return "none";
        }
    }

    struct V4L2Param {
        std::string name;
        uint32_t id;
        int value{-1};
    };

    struct CameraConfig {
        std::string name;
        std::string device;
        std::string topic;
        std::string frame_id;
        int cam_w{0}, cam_h{0}, cam_fps{0}, cam_rot{0};
        int out_w{0}, out_h{0};
        std::vector<V4L2Param> v4l2_params;
    };

    void apply_v4l2_controls(const CameraConfig &cfg)
    {
        int fd = ::open(cfg.device.c_str(), O_RDWR);
        if (fd < 0) {
            RCLCPP_WARN(get_logger(), "[%s] open() for v4l2 controls failed: %s",
                        cfg.device.c_str(), std::strerror(errno));
            return;
        }
        for (const auto &p : cfg.v4l2_params) {
            if (p.value < 0) continue;
            v4l2_control ctl{};
            ctl.id = p.id;
            ctl.value = p.value;
            if (::ioctl(fd, VIDIOC_S_CTRL, &ctl) < 0) {
                RCLCPP_WARN(get_logger(), "[%s] set %s=%d failed: %s",
                            cfg.device.c_str(), p.name.c_str(), p.value, std::strerror(errno));
            } else {
                RCLCPP_INFO(get_logger(), "[%s] %s = %d",
                            cfg.device.c_str(), p.name.c_str(), p.value);
            }
        }
        ::close(fd);
    }

    void open_camera(size_t idx)
    {
        const auto &cfg = cameras_[idx];
        apply_v4l2_controls(cfg);

        std::string flip_stage;
        if (cfg.cam_rot != 0) {
            flip_stage = std::string("videoflip method=") + videoflip_method(cfg.cam_rot) + " ! ";
        }

        std::string ps =
            "v4l2src device=" + cfg.device + " ! "
            "image/jpeg,width=" + std::to_string(cfg.cam_w) +
            ",height=" + std::to_string(cfg.cam_h) +
            ",framerate=" + std::to_string(cfg.cam_fps) + "/1 ! "
            "jpegdec ! videoconvert ! " +
            flip_stage +
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false";

        GError *err = nullptr;
        auto *pipeline = gst_parse_launch(ps.c_str(), &err);
        if (!pipeline || err) {
            std::string e = err ? err->message : "unknown";
            if (err) g_error_free(err);
            throw std::runtime_error("Pipeline failed for " + cfg.device + ": " + e);
        }

        auto *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        auto *bus = gst_element_get_bus(pipeline);
        auto *msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
            static_cast<GstMessageType>(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR));
        if (msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *gerr = nullptr;
            gst_message_parse_error(msg, &gerr, nullptr);
            std::string e = gerr ? gerr->message : "unknown";
            if (gerr) g_error_free(gerr);
            gst_message_unref(msg);
            gst_object_unref(bus);
            throw std::runtime_error("Camera open failed: " + cfg.device + ": " + e);
        }
        if (msg) gst_message_unref(msg);
        gst_object_unref(bus);

        pipelines_.push_back(pipeline);
        sinks_.push_back(sink);

        RCLCPP_INFO(get_logger(), "[%s] Opened %s — %dx%d@%dfps",
            cfg.frame_id.c_str(), cfg.device.c_str(),
            cfg.cam_w, cfg.cam_h, cfg.cam_fps);
    }

    void capture_loop(size_t idx)
    {
        const auto &cfg = cameras_[idx];
        int count = 0;
        auto log_time = steady_clock::now();
        const size_t frame_bytes = static_cast<size_t>(cfg.out_w) * cfg.out_h * 3;

        while (running_) {
            auto t0 = steady_clock::now();
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sinks_[idx]));
            if (!sample) continue;

            GstBuffer *buf = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            auto t1 = steady_clock::now();
            auto msg = std::make_unique<ImageMsg>();
            msg->header.stamp = this->now();
            msg->header.frame_id = cfg.frame_id;
            msg->height = cfg.out_h;
            msg->width  = cfg.out_w;
            msg->encoding = "bgr8";
            msg->step = cfg.out_w * 3;
            msg->data.resize(frame_bytes);
            std::memcpy(msg->data.data(), map.data, frame_bytes);
            auto t2 = steady_clock::now();

            pubs_[idx]->publish(std::move(msg));
            auto t3 = steady_clock::now();

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);
            count++;

            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(get_logger(),
                    "[%s] fps=%.1f  pull=%.1fms copy=%.1fms pub=%.1fms total=%.1fms",
                    cfg.frame_id.c_str(),
                    count / elapsed,
                    ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t0, t3));
                count = 0;
                log_time = steady_clock::now();
            }
        }
    }

    static double ms(steady_clock::time_point a, steady_clock::time_point b) {
        return duration<double, std::milli>(b - a).count();
    }

    std::atomic<bool> running_{true};
    std::vector<CameraConfig> cameras_;
    std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> pubs_;
    std::vector<GstElement *> pipelines_;
    std::vector<GstElement *> sinks_;
    std::vector<std::thread> threads_;
};
