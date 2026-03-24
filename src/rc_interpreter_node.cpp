/**
 * @file rc_interpreter_node.cpp
 * @brief RC Interpreter 独立节点入口
 */

#include <rclcpp/rclcpp.hpp>
#include "universal_controller/interpreters/rc_interpreter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto interpreter = std::make_shared<universal_controller::RCInterpreter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(interpreter);

    RCLCPP_INFO(interpreter->get_logger(), "RC Interpreter Node started");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
