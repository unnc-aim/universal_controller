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
 * 参考：
 * - infantry_controller/gimbal_controller.py
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/read_dji_rc.hpp"
#include "universal_controller_framework/msg/unified_input.hpp"

#include <memory>
#include <cmath>

namespace universal_controller {

/**
 * @brief NDJ 遥控器解释器
 *
 * 处理 ReadDJIRC 消息中的所有输入：
 * - 摇杆: right_joystick, left_joystick
 * - 拨杆: gear_switching, pause_button, custom_buttons
 * - 拨轮: thumb_wheel
 * - 触发器: trigger
 * - 键盘: key_w/s/a/d, key_shift/ctrl, key_q/e/r/f/g/z/x/c/v/b
 * - 鼠标: mouse_x/y_axis, mouse_wheel, mouse_lb/rb/mb
 */
class NDJInterpreter : public rclcpp::Node {
public:
    explicit NDJInterpreter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~NDJInterpreter() override = default;

private:
    void declare_parameters();
    void cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg);
    void process_input();
    void publish_unified();
    double clamp(double value, double min_val, double max_val);

    // ========== 订阅 ==========
    rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr sub_rc_;

    // ========== 发布 ==========
    rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;

    // ========== 定时器 ==========
    rclcpp::TimerBase::SharedPtr timer_;

    // ========== 原始数据 ==========
    custom_msgs::msg::ReadDJIRC::SharedPtr raw_rc_data_;
    bool connected_{false};
    rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
    double connection_timeout_s_{0.5};

    // ========== 输出 ==========
    msg::UnifiedInput unified_output_;

    // ========== 速度模式 ==========
    double current_spd_mode_{3000.0};

    // ========== 小陀螺模式 ==========
    bool spin_mode_enabled_{false};
    double spin_spd_{3000.0};
    uint8_t last_gear_switch_{0};
    uint8_t last_pause_button_{0};
    bool last_v_pressed_{false};
    bool nav_mode_enabled_{false};

    // ========== 云台目标 ==========
    double target_pitch_deg_{0.0};
    double target_yaw_rad_{0.0};

    // ========== 参数 ==========
    std::string topic_rc_read_;
    std::string topic_unified_output_;

    double spin_spd_default_{3000.0};
    double spin_spd_min_{800.0};
    double spin_spd_max_{6500.0};
    double spin_dial_gain_per_tick_{6.0};
    double spin_key_gain_per_tick_{2.4};
    double spin_dial_deadband_{0.05};

    // 鼠标控制参数
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
