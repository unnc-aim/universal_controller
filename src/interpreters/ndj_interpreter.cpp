/**
 * @file ndj_interpreter.cpp
 * @brief NDJ 遥控器解释器实现（大疆新图传遥控器）
 */

#include "universal_controller/interpreters/ndj_interpreter.hpp"

namespace universal_controller
{

    NDJInterpreter::NDJInterpreter(const rclcpp::NodeOptions &options)
        : Node("ndj_interpreter", options),
          input_processor_()
    {
        declare_parameters();
        load_parameters();

        // 订阅 NDJ 遥控器 (ReadDJIRC)
        sub_rc_ = this->create_subscription<custom_msgs::msg::ReadDJIRC>(
            topic_rc_read_, qos_best_effort_,
            std::bind(&NDJInterpreter::cb_rc, this, std::placeholders::_1));

        // 发布统一消息
        pub_unified_ = this->create_publisher<msg::UnifiedInput>(
            topic_unified_output_, qos_best_effort_);

        // 定时器（100Hz 发布）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NDJInterpreter::publish_unified, this));

        // 初始化输出
        unified_output_.control_source = "NDJ";
        unified_output_.connected = false;

        RCLCPP_INFO(this->get_logger(), "NDJ Interpreter started (ReadDJIRC)");
    }

    void NDJInterpreter::declare_parameters()
    {
        // 话题
        this->declare_parameter("topic_rc_read", "/ecat/sn4587585/app1/read");
        this->declare_parameter("topic_unified_output", "/universal_controller/ndj_input");

        // 连接超时
        this->declare_parameter("connection_timeout_s", 0.5);

        // 输入处理器参数
        this->declare_parameter("joystick_max_output", 8000.0);
        this->declare_parameter("joystick_deadzone", 0.05);

        this->declare_parameter("keyboard_speed_default", 3000.0);
        this->declare_parameter("keyboard_speed_min", 0.0);
        this->declare_parameter("keyboard_speed_max", 8000.0);
        this->declare_parameter("keyboard_speed_step", 6.0);

        this->declare_parameter("spin_speed_default", 3000.0);
        this->declare_parameter("spin_speed_min", 800.0);
        this->declare_parameter("spin_speed_max", 6500.0);
        this->declare_parameter("spin_dial_gain", 6.0);
        this->declare_parameter("spin_key_gain", 2.4);
        this->declare_parameter("spin_dial_deadzone", 0.05);

        this->declare_parameter("mouse_sensitivity", 1.0);
        this->declare_parameter("mouse_yaw_gain", 0.75);
        this->declare_parameter("mouse_pitch_gain", 1.0);
        this->declare_parameter("mouse_limit", 100.0);

        this->declare_parameter("pitch_min_deg", -25.0);
        this->declare_parameter("pitch_max_deg", 40.0);
    }

    void NDJInterpreter::load_parameters()
    {
        topic_rc_read_ = this->get_parameter("topic_rc_read").as_string();
        topic_unified_output_ = this->get_parameter("topic_unified_output").as_string();
        connection_timeout_s_ = this->get_parameter("connection_timeout_s").as_double();

        // 加载输入处理器配置
        input_config_.joystick_max_output = this->get_parameter("joystick_max_output").as_double();
        input_config_.joystick_deadzone = this->get_parameter("joystick_deadzone").as_double();

        input_config_.keyboard_speed_default = this->get_parameter("keyboard_speed_default").as_double();
        input_config_.keyboard_speed_min = this->get_parameter("keyboard_speed_min").as_double();
        input_config_.keyboard_speed_max = this->get_parameter("keyboard_speed_max").as_double();
        input_config_.keyboard_speed_step = this->get_parameter("keyboard_speed_step").as_double();

        input_config_.spin_speed_default = this->get_parameter("spin_speed_default").as_double();
        input_config_.spin_speed_min = this->get_parameter("spin_speed_min").as_double();
        input_config_.spin_speed_max = this->get_parameter("spin_speed_max").as_double();
        input_config_.spin_dial_gain = this->get_parameter("spin_dial_gain").as_double();
        input_config_.spin_key_gain = this->get_parameter("spin_key_gain").as_double();
        input_config_.spin_dial_deadzone = this->get_parameter("spin_dial_deadzone").as_double();

        input_config_.mouse_sensitivity = this->get_parameter("mouse_sensitivity").as_double();
        input_config_.mouse_yaw_gain = this->get_parameter("mouse_yaw_gain").as_double();
        input_config_.mouse_pitch_gain = this->get_parameter("mouse_pitch_gain").as_double();
        input_config_.mouse_limit = this->get_parameter("mouse_limit").as_double();

        input_config_.pitch_min_deg = this->get_parameter("pitch_min_deg").as_double();
        input_config_.pitch_max_deg = this->get_parameter("pitch_max_deg").as_double();

        // 更新输入处理器
        input_processor_.update_config(input_config_);
    }

    void NDJInterpreter::cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg)
    {
        raw_rc_data_ = msg;
        last_rc_time_ = this->now();
        connected_ = true;
        process_input();
    }

    void NDJInterpreter::process_input()
    {
        // 检查连接超时
        if (connected_)
        {
            auto now = this->now();
            if ((now - last_rc_time_).seconds() > connection_timeout_s_)
            {
                connected_ = false;
            }
        }

        if (!raw_rc_data_ || !connected_)
        {
            unified_output_.emergency_stop = true;
            unified_output_.connected = false;
            return;
        }

        const auto &rc = *raw_rc_data_;
        unified_output_.connected = connected_;
        unified_output_.control_source = "NDJ";
        unified_output_.header.stamp = this->now();

        // ========== 1. 离线或急停检测 ==========
        bool emergency = (rc.pause_button == 1) || (rc.gear_switching == 2);
        if (!connected_ || emergency)
        {
            nav_mode_enabled_ = false;
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
            unified_output_.spin_mode = false;
            unified_output_.emergency_stop = emergency;
            return;
        }
        unified_output_.emergency_stop = false;

        // ========== 2. 导航模式检测 ==========
        if (rc.right_custom_button == 1)
        {
            if (rc.right_custom_button != last_pause_button_ || !nav_mode_enabled_)
            {
                unified_output_.vx = 0.0;
                unified_output_.vy = 0.0;
                unified_output_.wz = 0.0;
            }
            nav_mode_enabled_ = true;
            unified_output_.navigation_enabled = true;
            last_pause_button_ = rc.right_custom_button;
            return;
        }
        nav_mode_enabled_ = false;
        unified_output_.navigation_enabled = false;
        last_pause_button_ = rc.right_custom_button;

        // ========== 3. 小陀螺模式切换 ==========
        // 拨杆切换
        if (rc.left_custom_button == 1 && last_gear_switch_ != 1)
        {
            spin_mode_enabled_ = !spin_mode_enabled_;
            if (spin_mode_enabled_)
            {
                input_processor_.reset_spin_speed();
            }
            RCLCPP_INFO(this->get_logger(), "Spin Mode: %s", spin_mode_enabled_ ? "ON" : "OFF");
        }
        last_gear_switch_ = rc.left_custom_button;

        // V 键切换
        bool v_pressed = (rc.key_v == 1);
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
        input_processor_.update_keyboard_speed(rc.key_shift == 1, rc.key_ctrl == 1);

        // ========== 5. 小陀螺速度调节 ==========
        if (spin_mode_enabled_)
        {
            input_processor_.update_spin_speed(rc.thumb_wheel, rc.key_shift == 1, rc.key_ctrl == 1);
        }

        // ========== 6. 底盘速度计算 ==========
        auto chassis_vel = input_processor_.compute_chassis_velocity(
            rc.right_joystick_x, rc.right_joystick_y,
            rc.key_w, rc.key_s, rc.key_a, rc.key_d);

        unified_output_.vx = chassis_vel.vx;
        unified_output_.vy = chassis_vel.vy;
        unified_output_.wz = spin_mode_enabled_ ? input_processor_.get_spin_speed() : 0.0;
        unified_output_.spin_mode = spin_mode_enabled_;
        unified_output_.spin_speed = input_processor_.get_spin_speed();
        unified_output_.chassis_speed_scale = input_processor_.get_speed_scale();

        // ========== 7. 鼠标云台控制 ==========
        auto gimbal_delta = input_processor_.compute_gimbal_delta(
            rc.mouse_x_axis, rc.mouse_y_axis);

        unified_output_.pitch_delta = gimbal_delta.pitch_delta;
        unified_output_.yaw_delta = gimbal_delta.yaw_delta;

        // ========== 8. 发射控制 ==========
        unified_output_.autoaim_enabled = rc.mouse_rb == 1;
        unified_output_.fire_trigger = rc.mouse_lb == 1;
        unified_output_.burst_mode = rc.mouse_lb == 1;
        unified_output_.friction_on = true;
        unified_output_.friction_speed = 6500.0;
    }

    void NDJInterpreter::publish_unified()
    {
        // 检查连接超时
        if (connected_)
        {
            auto now = this->now();
            if ((now - last_rc_time_).seconds() > connection_timeout_s_)
            {
                connected_ = false;
                unified_output_.connected = false;
                unified_output_.emergency_stop = true;
            }
        }

        unified_output_.header.stamp = this->now();
        pub_unified_->publish(unified_output_);
    }

} // namespace universal_controller
