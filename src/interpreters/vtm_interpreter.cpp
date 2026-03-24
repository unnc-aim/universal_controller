/**
 * @file vtm_interpreter.cpp
 * @brief VTM 遥控器解释器实现（VT13 遥控器）
 *
 * 参考：
 * - sentry_controller/rc_interpreter.py（底盘控制）
 * - infantry_controller/gimbal_controller.py（鼠标映射）
 */

#include "universal_controller_framework/interpreters/vtm_interpreter.hpp"

namespace universal_controller {

VTMInterpreter::VTMInterpreter(const rclcpp::NodeOptions& options)
    : Node("vtm_interpreter", options) {
    declare_parameters();

    // 订阅 VTM 遥控器 (ReadVT13RemoteControl)
    sub_rc_ = this->create_subscription<custom_msgs::msg::ReadVT13RemoteControl>(
        topic_rc_read_, qos_best_effort_,
        std::bind(&VTMInterpreter::cb_rc, this, std::placeholders::_1));

    // 发布统一消息
    pub_unified_ = this->create_publisher<msg::UnifiedInput>(
        topic_unified_output_, qos_best_effort_);

    // 定时器（100Hz 发布）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&VTMInterpreter::publish_unified, this));

    // 初始化输出
    unified_output_.control_source = "VTM";
    unified_output_.connected = false;

    RCLCPP_INFO(this->get_logger(), "VTM Interpreter started (ReadVT13RemoteControl)");
}

void VTMInterpreter::declare_parameters() {
    // 话题
    this->declare_parameter("topic_rc_read", "/ecat/sn4653115/app1/read");
    this->declare_parameter("topic_unified_output", "/universal_controller/vtm_input");

    // 小陀螺参数
    this->declare_parameter("spin_speed_default", 3000.0);
    this->declare_parameter("spin_speed_min", 800.0);
    this->declare_parameter("spin_speed_max", 6500.0);
    this->declare_parameter("spin_dial_gain_per_tick", 6.0);
    this->declare_parameter("spin_key_gain_per_tick", 2.4);
    this->declare_parameter("spin_dial_deadband", 0.05);

    // 鼠标参数
    this->declare_parameter("mouse_sensitivity", 1.0);
    this->declare_parameter("mouse_yaw_gain", 0.75);
    this->declare_parameter("mouse_pitch_gain", 1.0);
    this->declare_parameter("mouse_limit", 100.0);

    // Pitch 限位
    this->declare_parameter("pitch_min_deg", -25.0);
    this->declare_parameter("pitch_max_deg", 40.0);

    // 读取参数
    topic_rc_read_ = this->get_parameter("topic_rc_read").as_string();
    topic_unified_output_ = this->get_parameter("topic_unified_output").as_string();

    spin_spd_default_ = this->get_parameter("spin_speed_default").as_double();
    spin_spd_min_ = this->get_parameter("spin_speed_min").as_double();
    spin_spd_max_ = this->get_parameter("spin_speed_max").as_double();
    spin_dial_gain_per_tick_ = this->get_parameter("spin_dial_gain_per_tick").as_double();
    spin_key_gain_per_tick_ = this->get_parameter("spin_key_gain_per_tick").as_double();
    spin_dial_deadband_ = this->get_parameter("spin_dial_deadband").as_double();

    mouse_sensitivity_ = this->get_parameter("mouse_sensitivity").as_double();
    mouse_yaw_gain_ = this->get_parameter("mouse_yaw_gain").as_double();
    mouse_pitch_gain_ = this->get_parameter("mouse_pitch_gain").as_double();
    mouse_limit_ = this->get_parameter("mouse_limit").as_double();

    pitch_min_deg_ = this->get_parameter("pitch_min_deg").as_double();
    pitch_max_deg_ = this->get_parameter("pitch_max_deg").as_double();
}

void VTMInterpreter::cb_rc(const custom_msgs::msg::ReadVT13RemoteControl::SharedPtr msg) {
    raw_rc_data_ = msg;
    connected_ = (msg->online == 1);
    process_input();
}

void VTMInterpreter::process_input() {
    if (!raw_rc_data_) {
        unified_output_.emergency_stop = true;
        unified_output_.connected = false;
        return;
    }

    const auto& rc = *raw_rc_data_;
    unified_output_.connected = connected_;
    unified_output_.control_source = "VTM";
    unified_output_.header.stamp = this->now();

    // ========== 1. 离线或急停检测 ==========
    // left_switch: 1=Up, 2=Down, 3=Mid
    if (!connected_ || rc.left_switch == 2) {
        nav_mode_enabled_ = false;
        unified_output_.vx = 0.0;
        unified_output_.vy = 0.0;
        unified_output_.wz = 0.0;
        unified_output_.spin_mode = false;
        unified_output_.emergency_stop = (rc.left_switch == 2);
        return;
    }
    unified_output_.emergency_stop = false;

    // ========== 2. 导航模式检测 ==========
    // 右拨杆中/上档时禁用遥控底盘输入
    // right_switch: 1=Up, 2=Down, 3=Mid
    if (rc.right_switch == 1 || rc.right_switch == 3) {
        if (rc.right_switch != last_right_switch_ || !nav_mode_enabled_) {
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
        }
        nav_mode_enabled_ = true;
        unified_output_.navigation_enabled = true;
        last_right_switch_ = rc.right_switch;
        return;
    }
    nav_mode_enabled_ = false;
    unified_output_.navigation_enabled = false;
    last_right_switch_ = rc.right_switch;

    // ========== 3. 拨杆边沿检测（小陀螺模式切换） ==========
    if (rc.left_switch == 1 && last_left_switch_ != 1) {
        spin_mode_enabled_ = !spin_mode_enabled_;
        if (spin_mode_enabled_) {
            spin_spd_ = spin_spd_default_;
        }
        RCLCPP_INFO(this->get_logger(), "Spin Mode: %s", spin_mode_enabled_ ? "ON" : "OFF");
    }
    last_left_switch_ = rc.left_switch;

    // ========== 4. 键盘 V 键边沿检测 ==========
    bool v_pressed = (rc.v == 1);
    if (v_pressed && !last_v_pressed_) {
        spin_mode_enabled_ = !spin_mode_enabled_;
        if (spin_mode_enabled_) {
            spin_spd_ = spin_spd_default_;
        }
        RCLCPP_INFO(this->get_logger(), "Spin Mode (V): %s", spin_mode_enabled_ ? "ON" : "OFF");
    }
    last_v_pressed_ = v_pressed;

    // ========== 5. 速度分档 (Shift/Ctrl) ==========
    if (rc.shift == 1) {
        current_spd_mode_ = std::min(8000.0, current_spd_mode_ + 6.0);
    } else if (rc.ctrl == 1) {
        current_spd_mode_ = std::max(0.0, current_spd_mode_ - 6.0);
    }

    // ========== 6. 小陀螺速度调节 ==========
    if (spin_mode_enabled_) {
        // 拨轮调节 (dial 是 -1 到 1)
        double dial_val = rc.dial;
        double dial_input = (std::abs(dial_val) < spin_dial_deadband_) ? 0.0 : dial_val;
        spin_spd_ += -dial_input * spin_dial_gain_per_tick_;

        // 键盘微调
        if (rc.shift == 1 && rc.ctrl == 0) {
            spin_spd_ += spin_key_gain_per_tick_;
        } else if (rc.ctrl == 1 && rc.shift == 0) {
            spin_spd_ -= spin_key_gain_per_tick_;
        }

        spin_spd_ = std::max(spin_spd_min_, std::min(spin_spd_max_, spin_spd_));
    }

    // ========== 7. 底盘速度解析 ==========
    double key_fb = static_cast<double>(rc.w) - static_cast<double>(rc.s);
    double key_lr = static_cast<double>(rc.d) - static_cast<double>(rc.a);

    // 右摇杆 + 键盘
    double v_x_g = rc.right_y * 8000.0 + key_fb * current_spd_mode_;
    double v_y_g = rc.right_x * 8000.0 + key_lr * current_spd_mode_;

    // 旋转速度
    double w_z = spin_mode_enabled_ ? spin_spd_ : 0.0;

    // 死区检测
    bool move_active = std::abs(v_x_g) > 100.0 || std::abs(v_y_g) > 100.0;
    bool rotate_active = std::abs(w_z) > 10.0;

    if (!move_active && !rotate_active) {
        unified_output_.vx = 0.0;
        unified_output_.vy = 0.0;
        unified_output_.wz = 0.0;
        unified_output_.spin_mode = false;
        return;
    }

    // 填充底盘输出
    unified_output_.vx = move_active ? v_x_g : 0.0;
    unified_output_.vy = move_active ? v_y_g : 0.0;
    unified_output_.wz = w_z;
    unified_output_.spin_mode = spin_mode_enabled_;
    unified_output_.spin_speed = spin_spd_;
    unified_output_.chassis_speed_scale = current_spd_mode_ / 8000.0;

    // ========== 8. 鼠标控制（参考 infantry_controller/gimbal_controller.py） ==========
    double mouse_x = static_cast<double>(rc.mouse_x) * mouse_sensitivity_;
    double mouse_y = static_cast<double>(rc.mouse_y) * mouse_sensitivity_;

    double left_right_offset = clamp(mouse_x * mouse_yaw_gain_, -mouse_limit_, mouse_limit_);
    double top_down_offset = clamp(-mouse_y * mouse_pitch_gain_, -mouse_limit_, mouse_limit_);

    // Pitch 增量 (度)
    double pitch_delta_deg = top_down_offset * pitch_gain_coeff_;
    target_pitch_deg_ += pitch_delta_deg;
    target_pitch_deg_ = clamp(target_pitch_deg_, pitch_min_deg_, pitch_max_deg_);

    // Yaw 增量 (弧度)
    double yaw_delta_rad = -left_right_offset * yaw_gain_coeff_;
    target_yaw_rad_ += yaw_delta_rad;
    // 归一化到 [-PI, PI]
    while (target_yaw_rad_ > M_PI) target_yaw_rad_ -= 2.0 * M_PI;
    while (target_yaw_rad_ < -M_PI) target_yaw_rad_ += 2.0 * M_PI;

    unified_output_.pitch_delta = pitch_delta_deg;
    unified_output_.yaw_delta = yaw_delta_rad;

    // ========== 9. 发射控制 ==========
    unified_output_.autoaim_enabled = rc.mouse_right_clicked == 1;
    unified_output_.fire_trigger = rc.mouse_left_clicked == 1;
    unified_output_.burst_mode = rc.mouse_left_clicked == 1;
    unified_output_.friction_on = true;  // 默认开启摩擦轮
    unified_output_.friction_speed = 6500.0;
}

void VTMInterpreter::publish_unified() {
    unified_output_.header.stamp = this->now();
    pub_unified_->publish(unified_output_);
}

double VTMInterpreter::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

}  // namespace universal_controller
