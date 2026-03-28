/**
 * @file vtm_interpreter.cpp
 * @brief VTM 遥控器解释器实现（VT13 遥控器）
 */

#include "universal_controller/interpreters/vtm_interpreter.hpp"
#include "universal_controller/tools/rc_yaml_parser.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace universal_controller
{

    VTMInterpreter::VTMInterpreter(const rclcpp::NodeOptions &options)
        : Node("vtm_interpreter", options),
          input_processor_()
    {
        declare_parameters();
        load_parameters();

        // 订阅 VT13 遥控器 (ReadVT13RemoteControl)
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

    void VTMInterpreter::declare_parameters()
    {
        // 话题
        this->declare_parameter("rc_interpreter.topic_vtm_rc", "/ecat/vt13/app1/read");
        this->declare_parameter("rc_interpreter.topic_vtm_output", "/universal_controller/input/vtm");
        this->declare_parameter("rc_interpreter.vtm_definition_file", "");

        // 连接超时
        this->declare_parameter("rc_interpreter.connection_timeout_s", 0.5);

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
        vtm_definition_file_ = this->get_parameter("rc_interpreter.vtm_definition_file").as_string();
        connection_timeout_s_ = this->get_parameter("rc_interpreter.connection_timeout_s").as_double();

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

        // 加载 YAML 配置文件
        if (vtm_definition_file_.empty())
        {
            try
            {
                vtm_definition_file_ = ament_index_cpp::get_package_share_directory("universal_controller") +
                                       "/config/vtm_definition.sentry.yaml";
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to resolve share config path: %s", e.what());
            }
        }

        if (!vtm_definition_file_.empty())
        {
            load_trigger_definition(vtm_definition_file_);
        }
    }

    void VTMInterpreter::load_trigger_definition(const std::string &file_path)
    {
        try
        {
            const YAML::Node root = YAML::LoadFile(file_path);
            const YAML::Node def = root["vtm_trigger_def"];
            if (!def)
            {
                RCLCPP_WARN(this->get_logger(), "vtm_trigger_def not found in %s", file_path.c_str());
                return;
            }

            // 解析 gear_switching
            const auto gear = def["gear_switching"];
            if (gear)
            {
                const std::array<std::string, 3> dock_names = {"left", "mid", "right"};
                parse_dock_points_three(gear["dock_points"], trigger_definition_.gear_dock_points, dock_names);

                // 解析四档切换：left_to_mid, mid_to_left, mid_to_right, right_to_mid
                const auto trans = gear["transitions"];
                if (trans)
                {
                    const std::array<std::string, 4> names = {
                        "left_to_mid", "mid_to_left", "mid_to_right", "right_to_mid"};
                    parse_transitions(trans, trigger_definition_.gear_transitions, names);
                }
            }

            // 解析按钮定义
            parse_button_definition(def["pause_button"], trigger_definition_.pause_button);
            parse_button_definition(def["left_custom_button"], trigger_definition_.left_custom_button);
            parse_button_definition(def["right_custom_button"], trigger_definition_.right_custom_button);

            // 解析滚轮
            parse_dial_action(def["thumb_wheel_up"], trigger_definition_.thumb_wheel_up);
            parse_dial_action(def["thumb_wheel_down"], trigger_definition_.thumb_wheel_down);

            // 解析扳机
            parse_button_definition(def["trigger"], trigger_definition_.trigger);

            trigger_definition_.loaded = true;
            RCLCPP_INFO(this->get_logger(), "Loaded VTM trigger definition: %s", file_path.c_str());
        }
        catch (const std::exception &e)
        {
            trigger_definition_.loaded = false;
            RCLCPP_WARN(this->get_logger(), "Failed to load VTM definition %s: %s", file_path.c_str(), e.what());
        }
    }

    void VTMInterpreter::cb_rc(const custom_msgs::msg::ReadVT13RemoteControl::SharedPtr msg)
    {
        raw_rc_data_ = msg;
        last_rc_time_ = this->now();
        connected_ = true; // VT13 假设有数据就是连接状态
        process_input();
    }

    void VTMInterpreter::process_input()
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
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }

        const auto &rc = *raw_rc_data_;
        unified_output_.connected = connected_;
        unified_output_.control_source = "VTM";
        unified_output_.header.stamp = this->now();
        spin_speed_delta_ = 0.0;

        // 执行 YAML 定义的 trigger 动作
        if (trigger_definition_.loaded)
        {
            execute_trigger_actions(rc);
        }

        // ========== 1. 急停检测 ==========
        bool emergency = emergency_state_;
        if (emergency)
        {
            nav_mode_enabled_ = false;
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
            unified_output_.spin_mode = false;
            unified_output_.emergency_stop = true;
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }
        unified_output_.emergency_stop = false;

        // ========== 2. 导航模式 ==========
        nav_mode_enabled_ = nav_topic_state_ || behavior_tree_state_;
        unified_output_.navigation_enabled = nav_mode_enabled_;

        // ========== 3. 速度分档 ==========
        input_processor_.update_keyboard_speed(rc.key_shift == 1, rc.key_ctrl == 1);

        // ========== 4. 小陀螺速度调节（滚轮 + 键盘） ==========
        if (spin_mode_enabled_)
        {
            // 滚轮调节
            input_processor_.update_spin_speed(rc.thumb_wheel, rc.key_shift == 1, rc.key_ctrl == 1);

            // YAML 定义的额外增量
            if (spin_speed_delta_ > 0.0)
            {
                for (int i = 0; i < static_cast<int>(spin_speed_delta_); ++i)
                {
                    input_processor_.update_spin_speed(-1.0, false, false); // 加速
                }
            }
            else if (spin_speed_delta_ < 0.0)
            {
                for (int i = 0; i < static_cast<int>(-spin_speed_delta_); ++i)
                {
                    input_processor_.update_spin_speed(1.0, false, false); // 减速
                }
            }
        }

        // ========== 5. 底盘速度计算 ==========
        if (nav_mode_enabled_)
        {
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
        }
        else
        {
            auto chassis_vel = input_processor_.compute_chassis_velocity(
                rc.right_joystick_x, rc.right_joystick_y,
                rc.key_w, rc.key_s, rc.key_a, rc.key_d);
            unified_output_.vx = chassis_vel.vx;
            unified_output_.vy = chassis_vel.vy;
            unified_output_.wz = spin_mode_enabled_ ? input_processor_.get_spin_speed() : 0.0;
        }

        unified_output_.spin_mode = spin_mode_enabled_;
        unified_output_.spin_speed = input_processor_.get_spin_speed();
        unified_output_.chassis_speed_scale = input_processor_.get_speed_scale();

        // ========== 6. 云台控制（左摇杆 + 鼠标，和 NDJ 对齐） ==========
        const double mouse_x = static_cast<double>(rc.mouse_x_axis) * input_config_.mouse_sensitivity;
        const double mouse_y = static_cast<double>(rc.mouse_y_axis) * input_config_.mouse_sensitivity;

        // 左右偏移 = 左摇杆X + 鼠标X
        const double left_right_offset =
            static_cast<double>(rc.left_joystick_x) * 100.0 +
            InputProcessor::clamp(
                mouse_x * input_config_.mouse_yaw_gain,
                -input_config_.mouse_limit,
                input_config_.mouse_limit);

        // 上下偏移 = 左摇杆Y + 鼠标Y
        const double top_down_offset =
            static_cast<double>(rc.left_joystick_y) * 100.0 +
            InputProcessor::clamp(
                mouse_y * input_config_.mouse_pitch_gain,
                -input_config_.mouse_limit,
                input_config_.mouse_limit);

        unified_output_.pitch_delta = top_down_offset * input_config_.pitch_gain_coeff;
        unified_output_.yaw_delta = left_right_offset * input_config_.yaw_gain_coeff;

        // ========== 7. 发射与模式控制 ==========
        const bool mouse_autoaim = (rc.mouse_rb == 1);
        const bool mouse_fire = (rc.mouse_lb == 1);

        unified_output_.autoaim_enabled = autoaim_state_ || mouse_autoaim;
        unified_output_.burst_mode = burst_mode_;
        unified_output_.fire_trigger = feeder_state_ || burst_mode_ || mouse_fire;
        unified_output_.friction_on = friction_state_;
        unified_output_.friction_speed = friction_state_ ? 6500.0 : 0.0;
    }

    void VTMInterpreter::execute_trigger_actions(const custom_msgs::msg::ReadVT13RemoteControl &rc)
    {
        // ========== gear_switching 三档拨杆 ==========
        // 值: 左=0, 中=1, 右=2
        const int gear = static_cast<int>(rc.gear_switching);
        
        // --- 增加对摇杆与拨杆的 DEBUG log ---
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "execute_trigger_actions running. gear: %d, pause: %d, left_btn: %d, right_btn: %d, trigger: %d", 
            gear, rc.pause_button, rc.left_custom_button, rc.right_custom_button, rc.trigger);

        if (gear >= 0 && gear <= 2)
        {
            execute_action_set(trigger_definition_.gear_dock_points[static_cast<size_t>(gear)]);
        }

        // 检测切换瞬间，执行 transitions
        // gear_transitions: [left_to_mid(0), mid_to_left(1), mid_to_right(2), right_to_mid(3)]
        if (last_gear_switching_ != 255 && gear != 255 && last_gear_switching_ != gear)
        {
            int trans_idx = -1;
            if (last_gear_switching_ == 0 && gear == 1)      trans_idx = 0;  // left_to_mid
            else if (last_gear_switching_ == 1 && gear == 0) trans_idx = 1;  // mid_to_left
            else if (last_gear_switching_ == 1 && gear == 2) trans_idx = 2;  // mid_to_right
            else if (last_gear_switching_ == 2 && gear == 1) trans_idx = 3;  // right_to_mid

            if (trans_idx >= 0)
            {
                execute_action_set(trigger_definition_.gear_transitions[static_cast<size_t>(trans_idx)]);
            }
        }
        last_gear_switching_ = rc.gear_switching;

        // ========== 按钮处理 ==========
        handle_button_transition(rc.pause_button == 1, last_pause_button_,
                     pause_button_press_start_time_,
                     pause_button_long_press_active_,
                     trigger_definition_.pause_button);
        handle_button_transition(rc.left_custom_button == 1, last_left_custom_button_,
                     left_custom_button_press_start_time_,
                     left_custom_button_long_press_active_,
                     trigger_definition_.left_custom_button);
        handle_button_transition(rc.right_custom_button == 1, last_right_custom_button_,
                     right_custom_button_press_start_time_,
                     right_custom_button_long_press_active_,
                     trigger_definition_.right_custom_button);

        // ========== 扳机处理 ==========
        handle_trigger_button(rc);

        // ========== 滚轮处理 ==========
        const double thumb_now = static_cast<double>(rc.thumb_wheel);

        // 向上（加速）
        if (last_thumb_wheel_ <= trigger_definition_.thumb_wheel_up.threshold &&
            thumb_now > trigger_definition_.thumb_wheel_up.threshold)
        {
            execute_action_set(trigger_definition_.thumb_wheel_up.actions);
        }

        // 向下（减速）
        const double down_threshold = -std::abs(trigger_definition_.thumb_wheel_down.threshold);
        if (last_thumb_wheel_ >= down_threshold && thumb_now < down_threshold)
        {
            execute_action_set(trigger_definition_.thumb_wheel_down.actions);
        }

        last_thumb_wheel_ = thumb_now;
    }

    void VTMInterpreter::handle_button_transition(bool current_pressed, bool &last_pressed,
                                                   rclcpp::Time &press_start_time,
                                                   bool &long_press_active,
                                                   const ButtonDefinition &def)
    {
        if (!def.loaded)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Button not loaded, ignoring.");
            return;
        }

        const auto now = this->now();

        // 按下边沿
        if (current_pressed && !last_pressed)
        {
            RCLCPP_INFO(this->get_logger(), "Button pressed edge detected.");
            press_start_time = now;
            long_press_active = false;
            execute_action_set(def.on_press);
        }

        // 长按检测（持续按下状态）
        if (current_pressed && !long_press_active)
        {
            const double hold_time = (now - press_start_time).seconds();
            if (hold_time >= def.long_press_threshold_s)
            {
                long_press_active = true;
                execute_action_set(def.on_long_press_reached);
            }
        }

        // 释放边沿
        if (!current_pressed && last_pressed)
        {
            // 先执行 on_release（任何释放都触发）
            execute_action_set(def.on_release);

            if (!long_press_active)
            {
                // 短按释放
                execute_action_set(def.on_short_press_released);
            }
            else
            {
                // 长按释放
                execute_action_set(def.on_long_press_released);
            }
        }

        last_pressed = current_pressed;
    }

    void VTMInterpreter::handle_trigger_button(const custom_msgs::msg::ReadVT13RemoteControl &rc)
    {
        if (!trigger_definition_.trigger.loaded)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Trigger not loaded, ignoring.");
            return;
        }

        const bool trigger_pressed = (rc.trigger == 1);
        const auto now = this->now();

        // 按下边沿
        if (trigger_pressed && !trigger_currently_pressed_)
        {
            RCLCPP_INFO(this->get_logger(), "Trigger pressed edge detected.");
            trigger_press_start_time_ = now;
            trigger_long_press_active_ = false;
            execute_action_set(trigger_definition_.trigger.on_press);
        }

        // 长按检测
        if (trigger_pressed && !trigger_long_press_active_)
        {
            const double hold_time = (now - trigger_press_start_time_).seconds();
            if (hold_time >= trigger_definition_.trigger.long_press_threshold_s)
            {
                trigger_long_press_active_ = true;
                execute_action_set(trigger_definition_.trigger.on_long_press_reached);
            }
        }

        // 释放边沿
        if (!trigger_pressed && trigger_currently_pressed_)
        {
            // 先执行 on_release
            execute_action_set(trigger_definition_.trigger.on_release);

            if (!trigger_long_press_active_)
            {
                // 短按释放
                execute_action_set(trigger_definition_.trigger.on_short_press_released);
            }
            else
            {
                // 长按释放
                execute_action_set(trigger_definition_.trigger.on_long_press_released);
            }
        }

        trigger_currently_pressed_ = trigger_pressed;
    }

    void VTMInterpreter::execute_action_set(const ActionSet &actions)
    {
        apply_tri_state_action(actions.emergency_stop, emergency_state_);
        apply_tri_state_action(actions.autoaim, autoaim_state_);
        apply_tri_state_action(actions.nav_topic, nav_topic_state_);
        apply_tri_state_action(actions.behavior_tree_topic, behavior_tree_state_);
        apply_tri_state_action(actions.friction_wheel, friction_state_);
        apply_tri_state_action(actions.spin_mode, spin_mode_enabled_);
        apply_tri_state_action(actions.feeder, feeder_state_);
        apply_tri_state_action(actions.feeder_burst, burst_mode_);

        // 小陀螺速度控制
        if (actions.spin_control.accelerate)
        {
            spin_speed_delta_ += 1.0;
        }
        if (actions.spin_control.decelerate)
        {
            spin_speed_delta_ -= 1.0;
        }
    }

    void VTMInterpreter::publish_unified()
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

        if (!raw_rc_data_ || !connected_)
        {
            return;
        }

        unified_output_.header.stamp = this->now();
        pub_unified_->publish(unified_output_);
    }

} // namespace universal_controller
