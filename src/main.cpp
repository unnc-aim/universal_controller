/**
 * @file main.cpp
 * @brief Universal Controller 主入口
 */

#include <rclcpp/rclcpp.hpp>
#include "universal_controller/hub/hub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hub = std::make_shared<universal_controller::Hub>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(hub);

    RCLCPP_INFO(hub->get_logger(), "Universal Controller Framework started");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
