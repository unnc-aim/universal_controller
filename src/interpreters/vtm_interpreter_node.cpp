#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "universal_controller/interpreters/vtm_interpreter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<universal_controller::VTMInterpreter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
