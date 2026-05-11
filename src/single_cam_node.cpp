#include <rclcpp/rclcpp.hpp>
#include "multi_fisheye/single_cam.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    auto node = std::make_shared<SingleCamPub>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
