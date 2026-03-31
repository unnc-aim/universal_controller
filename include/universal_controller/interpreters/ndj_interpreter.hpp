/**
 * @file ndj_interpreter.hpp
 * @brief NDJ 遥控器解释器（大疆新图传遥控器）
 *
 * 订阅 ReadDJIRC 消息，包含：
 * - 摇杆（底盘控制）
 * - 拨杆（模式切换）
 * - 键盘（WASD移动、QE旋转等）
 * - 鼠标（云台控制、发射）
 * - 拨轮（小陀螺速度调节）
 *
 * 支持通过 YAML 配置文件定义动作映射
 * 直接发布 UnifiedInput.msg 到 Hub
 */

#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/read_djirc.hpp"
#include "universal_controller/msg/unified_input.hpp"
#include "universal_controller/tools/input_processor.hpp"
#include "universal_controller/tools/keyboard_mouse_parser.hpp"
#include "universal_controller/tools/rc_action_types.hpp"

#include <array>
#include <memory>
#include <string>

namespace universal_controller {

class NDJInterpreter : public rclcpp::Node {
  public:
    explicit NDJInterpreter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~NDJInterpreter() override = default;

  private:
    void declare_parameters();
    void load_parameters();
    void load_trigger_definition(const std::string &file_path);
    void cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg);
    void process_input();
    void publish_unified();
    void execute_trigger_actions(const custom_msgs::msg::ReadDJIRC &rc);
    void execute_action_set(const ActionSet &actions);

    static int switch_to_dock(uint8_t switch_value);
    static int transition_index(int from_switch, int to_switch);

    // 键鼠映射
    KeyboardMouseInput map_keyboard_mouse(const custom_msgs::msg::ReadDJIRC &rc);

    // 订阅
    rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr sub_rc_;

    // 发布
    rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 原始数据
    custom_msgs::msg::ReadDJIRC::SharedPtr raw_rc_data_;
    bool connected_{false};
    rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
    double connection_timeout_s_{0.5};

    // 输出
    msg::UnifiedInput unified_output_;

    // 输入处理器
    InputProcessor input_processor_;
    InputProcessorConfig input_config_;

    // 键鼠解析器
    KeyboardMouseParser km_parser_;

    // 模式状态
    bool spin_mode_enabled_{false};
    bool nav_mode_enabled_{false};

    // 参数
    std::string topic_rc_read_;
    std::string topic_unified_output_;
    std::string ndj_definition_file_;
    std::string km_definition_file_;

    // 定义驱动状态
    NDJTriggerDefinition trigger_definition_;
    uint8_t last_left_switch_{0};
    uint8_t last_right_switch_{0};
    double last_dial_{0.0};

    bool autoaim_state_{false};
    bool emergency_state_{true};
    bool nav_topic_state_{false};
    bool behavior_tree_state_{false};
    bool friction_state_{false};
    bool feeder_state_{false}; // 拨弹盘开关状态
    bool burst_mode_{false};   // 连发模式状态

    rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
};

} // namespace universal_controller
