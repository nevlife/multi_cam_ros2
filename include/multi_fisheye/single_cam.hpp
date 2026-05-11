#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

class SingleCamPub : public rclcpp::Node
{
public:
    explicit SingleCamPub(const rclcpp::NodeOptions &options)
        : Node("single_cam_pub", options)
    {
        gst_init(nullptr, nullptr);

        device_   = declare_parameter<std::string>("device");
        topic_    = declare_parameter<std::string>("topic");
        frame_id_ = declare_parameter<std::string>("frame_id");
        pixel_format_ = declare_parameter<std::string>("pixel_format", "mjpeg");
        width_  = declare_parameter<int>("width");
        height_ = declare_parameter<int>("height");
        fps_    = declare_parameter<int>("fps");

        RCLCPP_INFO(get_logger(),
            "[%s] %s — %dx%d@%dfps (%s) → %s",
            frame_id_.c_str(), device_.c_str(),
            width_, height_, fps_, pixel_format_.c_str(),
            topic_.c_str());

        auto qos = rclcpp::QoS(10).reliable().durability_volatile();
        pub_ = create_publisher<ImageMsg>(topic_, qos);

        open_pipeline();
        thread_ = std::thread(&SingleCamPub::capture_loop, this);
    }

    ~SingleCamPub() override
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            if (sink_) gst_object_unref(sink_);
            gst_object_unref(pipeline_);
        }
    }

private:
    void open_pipeline()
    {
        std::string src_caps;
        std::string decode_stage;
        if (pixel_format_ == "mjpeg" || pixel_format_ == "jpeg") {
            src_caps = "image/jpeg";
            decode_stage = "jpegdec ! videoconvert ! ";
        } else if (pixel_format_ == "yuyv") {
            src_caps = "video/x-raw,format=YUY2";
            decode_stage = "videoconvert ! ";
        } else {
            throw std::runtime_error("Unsupported pixel_format: " + pixel_format_
                                     + " (use mjpeg or yuyv)");
        }

        std::string ps =
            "v4l2src device=" + device_ + " ! " +
            src_caps + ",width=" + std::to_string(width_) +
            ",height=" + std::to_string(height_) +
            ",framerate=" + std::to_string(fps_) + "/1 ! " +
            decode_stage +
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false";

        GError *err = nullptr;
        pipeline_ = gst_parse_launch(ps.c_str(), &err);
        if (!pipeline_ || err) {
            std::string e = err ? err->message : "unknown";
            if (err) g_error_free(err);
            throw std::runtime_error("Pipeline failed for " + device_ + ": " + e);
        }

        sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        auto *bus = gst_element_get_bus(pipeline_);
        auto *msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
            static_cast<GstMessageType>(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR));
        if (msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *gerr = nullptr;
            gst_message_parse_error(msg, &gerr, nullptr);
            std::string e = gerr ? gerr->message : "unknown";
            if (gerr) g_error_free(gerr);
            gst_message_unref(msg);
            gst_object_unref(bus);
            throw std::runtime_error("Camera open failed: " + device_ + ": " + e);
        }
        if (msg) gst_message_unref(msg);
        gst_object_unref(bus);

        RCLCPP_INFO(get_logger(), "[%s] Pipeline running", frame_id_.c_str());
    }

    void capture_loop()
    {
        const size_t frame_bytes = static_cast<size_t>(width_) * height_ * 3;
        int count = 0;
        auto log_time = steady_clock::now();

        while (running_) {
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
            if (!sample) continue;

            GstBuffer *buf = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            auto msg = std::make_unique<ImageMsg>();
            msg->header.stamp = this->now();
            msg->header.frame_id = frame_id_;
            msg->height = height_;
            msg->width  = width_;
            msg->encoding = "bgr8";
            msg->step = width_ * 3;
            msg->data.resize(frame_bytes);
            std::memcpy(msg->data.data(), map.data, frame_bytes);

            pub_->publish(std::move(msg));

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);
            count++;

            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(get_logger(), "[%s] fps=%.1f",
                            frame_id_.c_str(), count / elapsed);
                count = 0;
                log_time = steady_clock::now();
            }
        }
    }

    std::string device_, topic_, frame_id_, pixel_format_;
    int width_{0}, height_{0}, fps_{0};

    std::atomic<bool> running_{true};
    rclcpp::Publisher<ImageMsg>::SharedPtr pub_;
    GstElement *pipeline_{nullptr};
    GstElement *sink_{nullptr};
    std::thread thread_;
};
