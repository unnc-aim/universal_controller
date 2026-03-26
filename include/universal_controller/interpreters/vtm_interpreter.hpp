/**
 * @file vtm_interpreter.hpp
 * @brief VTM 遥控器解释器（VT13 遥控器）
 *
 * 订阅 ReadVT13RemoteControl 消息，包含：
 * - 摇杆（底盘控制）
 * - 拨杆（模式切换）
 * - 按键（WASD移动、QE旋转等）
 * - 鼠标（云台控制、发射）
 * - 滚轮（小陀螺速度调节）
 * - 扳机（单发/连发）
 *
 * 支持通过 YAML 配置文件定义动作映射
 * 直接发布 UnifiedInput.msg 到 Hub
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/read_vt13_remote_control.hpp"
#include "universal_controller/msg/unified_input.hpp"
#include "universal_controller/tools/input_processor.hpp"
#include "universal_controller/tools/rc_action_types.hpp"

#include <array>
#include <memory>
#include <string>

namespace universal_controller
{

    class VTMInterpreter : public rclcpp::Node
    {
    public:
        explicit VTMInterpreter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~VTMInterpreter() override = default;

    private:
        void declare_parameters();
        void load_parameters();
        void load_trigger_definition(const std::string &file_path);
        void cb_rc(const custom_msgs::msg::ReadVT13RemoteControl::SharedPtr msg);
        void process_input();
        void publish_unified();

        // Trigger 动作执行
        void execute_trigger_actions(const custom_msgs::msg::ReadVT13RemoteControl &rc);
        void execute_action_set(const ActionSet &actions);

        // 按钮状态管理
        void handle_button_transition(bool current_pressed, bool &last_pressed,
                                      rclcpp::Time &press_start_time,
                                      bool &long_press_active,
                                      const ButtonDefinition &def);
        void handle_trigger_button(const custom_msgs::msg::ReadVT13RemoteControl &rc);

        // 订阅
        rclcpp::Subscription<custom_msgs::msg::ReadVT13RemoteControl>::SharedPtr sub_rc_;

        // 发布
        rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 原始数据
        custom_msgs::msg::ReadVT13RemoteControl::SharedPtr raw_rc_data_;
        bool connected_{false};
        rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
        double connection_timeout_s_{0.5};

        // 输出
        msg::UnifiedInput unified_output_;

        // 输入处理器
        InputProcessor input_processor_;
        InputProcessorConfig input_config_;

        // 模式状态
        bool spin_mode_enabled_{false};
        bool nav_mode_enabled_{false};

        // 参数
        std::string topic_rc_read_;
        std::string topic_unified_output_;
        std::string vtm_definition_file_;

        // Trigger 定义
        VTMTriggerDefinition trigger_definition_;

        // 上次状态（用于检测边沿）
        uint8_t last_gear_switching_{0};
        bool last_pause_button_{false};
        bool last_left_custom_button_{false};
        bool last_right_custom_button_{false};
        double last_thumb_wheel_{0.0};

        // 各按钮长按计时
        rclcpp::Time pause_button_press_start_time_{0, 0, RCL_ROS_TIME};
        bool pause_button_long_press_active_{false};

        rclcpp::Time left_custom_button_press_start_time_{0, 0, RCL_ROS_TIME};
        bool left_custom_button_long_press_active_{false};

        rclcpp::Time right_custom_button_press_start_time_{0, 0, RCL_ROS_TIME};
        bool right_custom_button_long_press_active_{false};

        // Trigger 按钮计时
        rclcpp::Time trigger_press_start_time_{0, 0, RCL_ROS_TIME};
        bool trigger_currently_pressed_{false};
        bool trigger_long_press_active_{false};

        // 状态变量
        bool autoaim_state_{false};
        bool emergency_state_{false};
        bool nav_topic_state_{false};
        bool behavior_tree_state_{false};
        bool friction_state_{false};
        bool feeder_state_{false};      // 拨弹盘开关状态
        bool burst_mode_{false};        // 连发模式状态

        // 小陀螺速度增量（由 dial 控制累积）
        double spin_speed_delta_{0.0};

        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
