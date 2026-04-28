#include "multi_fisheye/multi_fisheye.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<MultiFisheyePub>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
