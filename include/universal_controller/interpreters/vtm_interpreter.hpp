/**
 * @file vtm_interpreter.hpp
 * @brief VTM 遥控器解释器（VT13 遥控器）
 *
 * 订阅 ReadVT13RemoteControl 消息，包含：
 * - 摇杆（底盘控制）
 * - 拨杆（模式切换）
 * - 键盘（WASD移动、QE旋转等）
 * - 鼠标（云台控制、发射）
 *
 * 直接发布 UnifiedInput.msg 到 Hub
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/read_vt13_remote_control.hpp"
#include "universal_controller/msg/unified_input.hpp"
#include "universal_controller/tools/input_processor.hpp"

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
        void cb_rc(const custom_msgs::msg::ReadVT13RemoteControl::SharedPtr msg);
        void process_input();
        void publish_unified();

        // 订阅
        rclcpp::Subscription<custom_msgs::msg::ReadVT13RemoteControl>::SharedPtr sub_rc_;

        // 发布
        rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 原始数据
        custom_msgs::msg::ReadVT13RemoteControl::SharedPtr raw_rc_data_;
        bool connected_{false};

        // 输出
        msg::UnifiedInput unified_output_;

        // 输入处理器
        InputProcessor input_processor_;
        InputProcessorConfig input_config_;

        // 小陀螺模式
        bool spin_mode_enabled_{false};
        uint8_t last_left_switch_{0};
        uint8_t last_right_switch_{0};
        bool last_v_pressed_{false};
        bool nav_mode_enabled_{false};

        // 参数
        std::string topic_rc_read_;
        std::string topic_unified_output_;

        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
