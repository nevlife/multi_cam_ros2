#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

struct RawCamConfig {
    std::string device;
    std::string topic;
    std::string frame_id;
};

static const std::vector<RawCamConfig> RAW_CAMERAS = {
    {"/dev/cam_front", "/camera/front/image_raw", "cam_front"},
    {"/dev/cam_left",  "/camera/left/image_raw",  "cam_left"},
    {"/dev/cam_rear",  "/camera/rear/image_raw",  "cam_rear"},
    {"/dev/cam_right", "/camera/right/image_raw", "cam_right"},
};

class MultiFisheyePub : public rclcpp::Node
{
public:
    explicit MultiFisheyePub(const rclcpp::NodeOptions &options)
        : Node("multi_fisheye_pub", options)
    {
        gst_init(nullptr, nullptr);

        cam_w_   = declare_parameter<int>("cam_width");
        cam_h_   = declare_parameter<int>("cam_height");
        cam_fps_ = declare_parameter<int>("cam_fps");
        cam_rot_ = declare_parameter<int>("cam_rotation");

        if (cam_rot_ != 0 && cam_rot_ != 90 && cam_rot_ != 180 && cam_rot_ != 270) {
            throw std::runtime_error("cam_rotation must be one of 0, 90, 180, 270");
        }

        if (cam_rot_ == 90 || cam_rot_ == 270) {
            out_w_ = cam_h_;
            out_h_ = cam_w_;
        } else {
            out_w_ = cam_w_;
            out_h_ = cam_h_;
        }

        RCLCPP_INFO(get_logger(),
                     "Raw publish mode (no undistort) — capture %dx%d@%dfps, rotate=%d, publish %dx%d",
                     cam_w_, cam_h_, cam_fps_, cam_rot_, out_w_, out_h_);

        auto qos = rclcpp::QoS(10).reliable().durability_volatile();

        for (size_t i = 0; i < RAW_CAMERAS.size(); i++) {
            pubs_.push_back(create_publisher<ImageMsg>(RAW_CAMERAS[i].topic, qos));
            open_camera(i);
            threads_.emplace_back(&MultiFisheyePub::capture_loop, this, i);
        }
        RCLCPP_INFO(get_logger(), "All %zu cameras started (raw)", RAW_CAMERAS.size());
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

    void open_camera(size_t idx)
    {
        std::string flip_stage;
        if (cam_rot_ != 0) {
            flip_stage = std::string("videoflip method=") + videoflip_method(cam_rot_) + " ! ";
        }

        std::string ps =
            "v4l2src device=" + RAW_CAMERAS[idx].device + " ! "
            "image/jpeg,width=" + std::to_string(cam_w_) +
            ",height=" + std::to_string(cam_h_) +
            ",framerate=" + std::to_string(cam_fps_) + "/1 ! "
            "jpegdec ! videoconvert ! " +
            flip_stage +
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false";

        GError *err = nullptr;
        auto *pipeline = gst_parse_launch(ps.c_str(), &err);
        if (!pipeline || err) {
            std::string e = err ? err->message : "unknown";
            if (err) g_error_free(err);
            throw std::runtime_error("Pipeline failed for " + RAW_CAMERAS[idx].device + ": " + e);
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
            throw std::runtime_error("Camera open failed: " + RAW_CAMERAS[idx].device + ": " + e);
        }
        if (msg) gst_message_unref(msg);
        gst_object_unref(bus);

        pipelines_.push_back(pipeline);
        sinks_.push_back(sink);

        RCLCPP_INFO(get_logger(), "[%s] Opened %s — %dx%d@%dfps",
            RAW_CAMERAS[idx].frame_id.c_str(), RAW_CAMERAS[idx].device.c_str(),
            cam_w_, cam_h_, cam_fps_);
    }

    void capture_loop(size_t idx)
    {
        int count = 0;
        auto log_time = steady_clock::now();
        const size_t frame_bytes = static_cast<size_t>(out_w_) * out_h_ * 3;

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
            msg->header.frame_id = RAW_CAMERAS[idx].frame_id;
            msg->height = out_h_;
            msg->width  = out_w_;
            msg->encoding = "bgr8";
            msg->step = out_w_ * 3;
            msg->data.resize(frame_bytes);
            std::memcpy(msg->data.data(), map.data, frame_bytes);
            auto t2 = steady_clock::now();

            pubs_[idx]->publish(std::move(msg));
            auto t3 = steady_clock::now();

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);
            count++;

            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            // if (elapsed >= 5.0) {
            //     RCLCPP_INFO(get_logger(),
            //         "[%s] pull=%.1fms copy=%.1fms pub=%.1fms total=%.1fms fps=%.1f",
            //         RAW_CAMERAS[idx].frame_id.c_str(),
            //         ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t0, t3),
            //         count / elapsed);
            //     count = 0;
            //     log_time = steady_clock::now();
            // }
        }
    }

    static double ms(steady_clock::time_point a, steady_clock::time_point b) {
        return duration<double, std::milli>(b - a).count();
    }

    std::atomic<bool> running_{true};
    int cam_w_, cam_h_, cam_fps_, cam_rot_;
    int out_w_, out_h_;
    std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> pubs_;
    std::vector<GstElement *> pipelines_;
    std::vector<GstElement *> sinks_;
    std::vector<std::thread> threads_;
};
