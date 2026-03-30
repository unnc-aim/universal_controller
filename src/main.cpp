/**
 * @file main.cpp
 * @brief Universal Controller 主入口
 */

#include <rclcpp/rclcpp.hpp>

#include "universal_controller/controllers/chassis_controller.hpp"
#include "universal_controller/controllers/fire_controller.hpp"
#include "universal_controller/controllers/gimbal_controller.hpp"
#include "universal_controller/hub/hub.hpp"
#include "universal_controller/interpreters/ndj_interpreter.hpp"
#include "universal_controller/interpreters/vtm_interpreter.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto vtm_interpreter = std::make_shared<universal_controller::VTMInterpreter>();
    auto ndj_interpreter = std::make_shared<universal_controller::NDJInterpreter>();
    auto chassis_controller = std::make_shared<universal_controller::ChassisController>();
    auto gimbal_controller = std::make_shared<universal_controller::GimbalController>();
    auto fire_controller = std::make_shared<universal_controller::FireController>();
    auto hub = std::make_shared<universal_controller::Hub>(
        chassis_controller, gimbal_controller, fire_controller);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(vtm_interpreter);
    executor.add_node(ndj_interpreter);
    executor.add_node(hub->get_chassis_node());
    executor.add_node(hub->get_gimbal_node());
    executor.add_node(hub->get_fire_node());
    executor.add_node(hub);

    RCLCPP_INFO(hub->get_logger(), "Universal Controller Framework started in single-process mode");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
