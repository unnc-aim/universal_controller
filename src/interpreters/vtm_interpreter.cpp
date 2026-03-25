/**
 * @file vtm_interpreter.cpp
 * @brief VTM 遥控器解释器实现（VT13 遥控器）
 */

#include "universal_controller/interpreters/vtm_interpreter.hpp"

namespace universal_controller
{

    VTMInterpreter::VTMInterpreter(const rclcpp::NodeOptions &options)
        : Node("vtm_interpreter", options),
          input_processor_()
    {
        declare_parameters();
        load_parameters();

        // 订阅遥控器 (ReadDJIRC)
        sub_rc_ = this->create_subscription<custom_msgs::msg::ReadDJIRC>(
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

        RCLCPP_INFO(this->get_logger(), "VTM Interpreter started (ReadDJIRC)");
    }

    void VTMInterpreter::declare_parameters()
    {
        // 话题
        this->declare_parameter("rc_interpreter.topic_vtm_rc", "/ecat/sn4653115/app5/read");
        this->declare_parameter("rc_interpreter.topic_vtm_output", "/universal_controller/input/vtm");

        // 输入处理器参数
        this->declare_parameter("rc_interpreter.joystick_max_output", 8000.0);
        this->declare_parameter("rc_interpreter.joystick_deadzone", 0.05);

        this->declare_parameter("rc_interpreter.keyboard_speed_default", 3000.0);
        this->declare_parameter("rc_interpreter.keyboard_speed_min", 0.0);
        this->declare_parameter("rc_interpreter.keyboard_speed_max", 8000.0);
        this->declare_parameter("rc_interpreter.keyboard_speed_step", 6.0);

        this->declare_parameter("rc_interpreter.spin_speed_default", 3000.0);
        this->declare_parameter("rc_interpreter.spin_speed_min", 800.0);
        this->declare_parameter("rc_interpreter.spin_speed_max", 6500.0);
        this->declare_parameter("rc_interpreter.spin_dial_gain", 6.0);
        this->declare_parameter("rc_interpreter.spin_key_gain", 2.4);
        this->declare_parameter("rc_interpreter.spin_dial_deadzone", 0.05);

        this->declare_parameter("rc_interpreter.mouse_sensitivity", 1.0);
        this->declare_parameter("rc_interpreter.mouse_yaw_gain", 0.75);
        this->declare_parameter("rc_interpreter.mouse_pitch_gain", 1.0);
        this->declare_parameter("rc_interpreter.mouse_limit", 100.0);

        this->declare_parameter("rc_interpreter.pitch_min_deg", -25.0);
        this->declare_parameter("rc_interpreter.pitch_max_deg", 40.0);
    }

    void VTMInterpreter::load_parameters()
    {
        topic_rc_read_ = this->get_parameter("rc_interpreter.topic_vtm_rc").as_string();
        topic_unified_output_ = this->get_parameter("rc_interpreter.topic_vtm_output").as_string();

        // 加载输入处理器配置
        input_config_.joystick_max_output = this->get_parameter("rc_interpreter.joystick_max_output").as_double();
        input_config_.joystick_deadzone = this->get_parameter("rc_interpreter.joystick_deadzone").as_double();

        input_config_.keyboard_speed_default = this->get_parameter("rc_interpreter.keyboard_speed_default").as_double();
        input_config_.keyboard_speed_min = this->get_parameter("rc_interpreter.keyboard_speed_min").as_double();
        input_config_.keyboard_speed_max = this->get_parameter("rc_interpreter.keyboard_speed_max").as_double();
        input_config_.keyboard_speed_step = this->get_parameter("rc_interpreter.keyboard_speed_step").as_double();

        input_config_.spin_speed_default = this->get_parameter("rc_interpreter.spin_speed_default").as_double();
        input_config_.spin_speed_min = this->get_parameter("rc_interpreter.spin_speed_min").as_double();
        input_config_.spin_speed_max = this->get_parameter("rc_interpreter.spin_speed_max").as_double();
        input_config_.spin_dial_gain = this->get_parameter("rc_interpreter.spin_dial_gain").as_double();
        input_config_.spin_key_gain = this->get_parameter("rc_interpreter.spin_key_gain").as_double();
        input_config_.spin_dial_deadzone = this->get_parameter("rc_interpreter.spin_dial_deadzone").as_double();

        input_config_.mouse_sensitivity = this->get_parameter("rc_interpreter.mouse_sensitivity").as_double();
        input_config_.mouse_yaw_gain = this->get_parameter("rc_interpreter.mouse_yaw_gain").as_double();
        input_config_.mouse_pitch_gain = this->get_parameter("rc_interpreter.mouse_pitch_gain").as_double();
        input_config_.mouse_limit = this->get_parameter("rc_interpreter.mouse_limit").as_double();

        input_config_.pitch_min_deg = this->get_parameter("rc_interpreter.pitch_min_deg").as_double();
        input_config_.pitch_max_deg = this->get_parameter("rc_interpreter.pitch_max_deg").as_double();

        // 更新输入处理器
        input_processor_.update_config(input_config_);
    }

    void VTMInterpreter::cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg)
    {
        raw_rc_data_ = msg;
        connected_ = (msg->online == 1);
        process_input();
    }

    void VTMInterpreter::process_input()
    {
        if (!raw_rc_data_)
        {
            unified_output_.emergency_stop = true;
            unified_output_.connected = false;
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }

        const auto &rc = *raw_rc_data_;
        unified_output_.connected = connected_;
        unified_output_.control_source = "VTM";
        unified_output_.header.stamp = this->now();

        // ========== 1. 离线或急停检测 ==========
        if (!connected_ || rc.left_switch == 2)
        {
            nav_mode_enabled_ = false;
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
            unified_output_.spin_mode = false;
            unified_output_.emergency_stop = (rc.left_switch == 2);
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }
        unified_output_.emergency_stop = false;

        // ========== 2. 导航模式检测 ==========
        if (rc.right_switch == 1 || rc.right_switch == 3)
        {
            if (rc.right_switch != last_right_switch_ || !nav_mode_enabled_)
            {
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

        // ========== 3. 小陀螺模式切换 ==========
        // 拨杆切换
        if (rc.left_switch == 1 && last_left_switch_ != 1)
        {
            spin_mode_enabled_ = !spin_mode_enabled_;
            if (spin_mode_enabled_)
            {
                input_processor_.reset_spin_speed();
            }
            RCLCPP_INFO(this->get_logger(), "Spin Mode: %s", spin_mode_enabled_ ? "ON" : "OFF");
        }
        last_left_switch_ = rc.left_switch;

        // V 键切换
        bool v_pressed = (rc.v == 1);
        if (v_pressed && !last_v_pressed_)
        {
            spin_mode_enabled_ = !spin_mode_enabled_;
            if (spin_mode_enabled_)
            {
                input_processor_.reset_spin_speed();
            }
            RCLCPP_INFO(this->get_logger(), "Spin Mode (V): %s", spin_mode_enabled_ ? "ON" : "OFF");
        }
        last_v_pressed_ = v_pressed;

        // ========== 4. 速度分档 ==========
        input_processor_.update_keyboard_speed(rc.shift == 1, rc.ctrl == 1);

        // ========== 5. 小陀螺速度调节 ==========
        if (spin_mode_enabled_)
        {
            input_processor_.update_spin_speed(rc.dial, rc.shift == 1, rc.ctrl == 1);
        }

        // ========== 6. 底盘速度计算 ==========
        auto chassis_vel = input_processor_.compute_chassis_velocity(
            rc.right_x, rc.right_y,
            rc.w, rc.s, rc.a, rc.d);

        unified_output_.vx = chassis_vel.vx;
        unified_output_.vy = chassis_vel.vy;
        unified_output_.wz = spin_mode_enabled_ ? input_processor_.get_spin_speed() : 0.0;
        unified_output_.spin_mode = spin_mode_enabled_;
        unified_output_.spin_speed = input_processor_.get_spin_speed();
        unified_output_.chassis_speed_scale = input_processor_.get_speed_scale();

        // ========== 7. 鼠标云台控制 ==========
        auto gimbal_delta = input_processor_.compute_gimbal_delta(
            static_cast<double>(rc.mouse_x),
            static_cast<double>(rc.mouse_y));

        unified_output_.pitch_delta = gimbal_delta.pitch_delta;
        unified_output_.yaw_delta = gimbal_delta.yaw_delta;

        // ========== 8. 发射控制 ==========
        unified_output_.autoaim_enabled = rc.mouse_right_clicked == 1;
        unified_output_.fire_trigger = rc.mouse_left_clicked == 1;
        unified_output_.burst_mode = rc.mouse_left_clicked == 1;
        unified_output_.friction_on = true;
        unified_output_.friction_speed = 6500.0;
    }

    void VTMInterpreter::publish_unified()
    {
        if (!raw_rc_data_ || !connected_)
        {
            return;
        }
        unified_output_.header.stamp = this->now();
        pub_unified_->publish(unified_output_);
    }

} // namespace universal_controller
