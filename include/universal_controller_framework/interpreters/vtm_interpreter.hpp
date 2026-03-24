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
#include "universal_controller_framework/msg/unified_input.hpp"

#include <memory>
#include <cmath>
#include <string>

namespace universal_controller {

class VTMInterpreter : public rclcpp::Node {
public:
    explicit VTMInterpreter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VTMInterpreter() override = default;

private:
    void declare_parameters();
    void cb_rc(const custom_msgs::msg::ReadVT13RemoteControl::SharedPtr msg);
    void process_input();
    void publish_unified();
    double clamp(double value, double min_val, double max_val);

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

    // 速度模式
    double current_spd_mode_{3000.0};

    // 小陀螺模式
    bool spin_mode_enabled_{false};
    double spin_spd_{3000.0};
    uint8_t last_left_switch_{0};
    uint8_t last_right_switch_{0};
    bool last_v_pressed_{false};
    bool nav_mode_enabled_{false};

    // 云台目标
    double target_pitch_deg_{0.0};
    double target_yaw_rad_{0.0};

    // 参数
    std::string topic_rc_read_;
    std::string topic_unified_output_;

    double spin_spd_default_{3000.0};
    double spin_spd_min_{800.0};
    double spin_spd_max_{6500.0};
    double spin_dial_gain_per_tick_{6.0};
    double spin_key_gain_per_tick_{2.4};
    double spin_dial_deadband_{0.05};

    // 鼠标参数
    double mouse_sensitivity_{1.0};
    double mouse_yaw_gain_{0.75};
    double mouse_pitch_gain_{1.0};
    double mouse_limit_{100.0};
    double pitch_gain_coeff_{0.00005 * (180.0 / M_PI)};
    double yaw_gain_coeff_{10.0 * M_PI * 0.001 * 0.0025};

    // Pitch 限位
    double pitch_min_deg_{-25.0};
    double pitch_max_deg_{40.0};

    rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
};

}  // namespace universal_controller
