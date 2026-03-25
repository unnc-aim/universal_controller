/**
 * @file ndj_interpreter.cpp
 * @brief NDJ 遥控器解释器实现（大疆新图传遥控器）
 */

#include "universal_controller/interpreters/ndj_interpreter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
    bool yaml_bool(const YAML::Node &node, const char *key)
    {
        return (node && node[key]) ? node[key].as<bool>() : false;
    }

    void parse_tristate(const YAML::Node &actions_node,
                        const char *key,
                        universal_controller::NDJInterpreter::TriStateAction &out)
    {
        const auto n = actions_node[key];
        if (!n)
        {
            return;
        }
        out.on = yaml_bool(n, "on");
        out.off = yaml_bool(n, "off");
        out.toggle = yaml_bool(n, "toggle");
    }

    void parse_actions(const YAML::Node &actions_node,
                       universal_controller::NDJInterpreter::ActionSet &out)
    {
        if (!actions_node)
        {
            return;
        }

        parse_tristate(actions_node, "autoaim", out.autoaim);
        parse_tristate(actions_node, "emergency_stop", out.emergency_stop);
        parse_tristate(actions_node, "nav_topic", out.nav_topic);
        parse_tristate(actions_node, "behavior_tree_topic", out.behavior_tree_topic);
        parse_tristate(actions_node, "friction_wheel", out.friction_wheel);
        parse_tristate(actions_node, "spin_mode", out.spin_mode);

        const auto feeder = actions_node["feeder"];
        if (feeder)
        {
            out.feeder.single_shot_once = yaml_bool(feeder, "single_once");
            out.feeder.continuous_start = yaml_bool(feeder, "burst_on");
            out.feeder.continuous_stop = yaml_bool(feeder, "burst_off");
            out.feeder.continuous_toggle = yaml_bool(feeder, "burst_toggle");
        }

        const auto spin_control = actions_node["spin_control"];
        if (spin_control)
        {
            out.spin_control.switch_toggle = yaml_bool(spin_control, "switch_toggle");
            out.spin_control.accelerate = yaml_bool(spin_control, "accelerate");
            out.spin_control.decelerate = yaml_bool(spin_control, "decelerate");
        }
    }
} // namespace

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
        this->declare_parameter("rc_interpreter.topic_ndj_rc", "/ecat/sn4653115/app1/read");
        this->declare_parameter("rc_interpreter.topic_ndj_output", "/universal_controller/input/ndj");
        this->declare_parameter("rc_interpreter.ndj_definition_file", "");

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

    void NDJInterpreter::load_parameters()
    {
        topic_rc_read_ = this->get_parameter("rc_interpreter.topic_ndj_rc").as_string();
        topic_unified_output_ = this->get_parameter("rc_interpreter.topic_ndj_output").as_string();
        ndj_definition_file_ = this->get_parameter("rc_interpreter.ndj_definition_file").as_string();
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

        if (ndj_definition_file_.empty())
        {
            try
            {
                ndj_definition_file_ = ament_index_cpp::get_package_share_directory("universal_controller") +
                                       "/config/ndj_definition.sentry.yaml";
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to resolve share config path: %s", e.what());
            }
        }

        if (!ndj_definition_file_.empty())
        {
            load_trigger_definition(ndj_definition_file_);
        }
    }

    void NDJInterpreter::load_trigger_definition(const std::string &file_path)
    {
        try
        {
            const YAML::Node root = YAML::LoadFile(file_path);
            const YAML::Node def = root["ndj_trigger_def"];
            if (!def)
            {
                RCLCPP_WARN(this->get_logger(), "ndj_trigger_def not found in %s", file_path.c_str());
                return;
            }

            auto parse_dock_points = [](const YAML::Node &dock_points,
                                        std::array<ActionSet, 3> &out)
            {
                const std::array<std::string, 3> names = {"up", "mid", "down"};

                for (size_t i = 0; i < names.size(); ++i)
                {
                    const YAML::Node dock_node = dock_points[names[i]];
                    if (!dock_node)
                    {
                        continue;
                    }
                    parse_actions(dock_node["actions"], out[i]);
                }
            };

            auto parse_transitions = [](const YAML::Node &transitions,
                                        std::array<ActionSet, 4> &out)
            {
                // NDJ 原始拨杆值: 上/中/下 = 1/3/2
                // 仅支持四种合法相邻瞬间: 13, 31, 23, 32
                const std::array<std::string, 4> names = {
                    "up_to_mid",
                    "mid_to_up",
                    "down_to_mid",
                    "mid_to_down"};

                for (size_t i = 0; i < names.size(); ++i)
                {
                    const YAML::Node trans_node = transitions[names[i]];
                    if (!trans_node)
                    {
                        continue;
                    }
                    parse_actions(trans_node["actions"], out[i]);
                }
            };

            const auto left = def["left_trigger"];
            if (left)
            {
                parse_dock_points(left["dock_points"], trigger_definition_.left_dock_points);
                parse_transitions(left["transitions"], trigger_definition_.left_transitions);
            }

            const auto right = def["right_trigger"];
            if (right)
            {
                parse_dock_points(right["dock_points"], trigger_definition_.right_dock_points);
                parse_transitions(right["transitions"], trigger_definition_.right_transitions);
            }

            const auto dial_up = def["dial_up"];
            if (dial_up)
            {
                if (dial_up["threshold"])
                {
                    trigger_definition_.dial_up.threshold = dial_up["threshold"].as<double>();
                }
                parse_actions(dial_up["actions"], trigger_definition_.dial_up.actions);
            }

            const auto dial_down = def["dial_down"];
            if (dial_down)
            {
                if (dial_down["threshold"])
                {
                    trigger_definition_.dial_down.threshold = dial_down["threshold"].as<double>();
                }
                parse_actions(dial_down["actions"], trigger_definition_.dial_down.actions);
            }

            trigger_definition_.loaded = true;
            RCLCPP_INFO(this->get_logger(), "Loaded NDJ trigger definition: %s", file_path.c_str());
        }
        catch (const std::exception &e)
        {
            trigger_definition_.loaded = false;
            RCLCPP_WARN(this->get_logger(), "Failed to load NDJ definition %s: %s", file_path.c_str(), e.what());
        }
    }

    void NDJInterpreter::cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg)
    {
        raw_rc_data_ = msg;
        last_rc_time_ = this->now();
        connected_ = (msg->online == 1);
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
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }

        const auto &rc = *raw_rc_data_;
        unified_output_.connected = connected_;
        unified_output_.control_source = "NDJ";
        unified_output_.header.stamp = this->now();
        fire_single_pulse_ = false;

        if (trigger_definition_.loaded)
        {
            execute_trigger_actions(rc);
        }

        // ========== 1. 离线或急停检测 ==========
        bool emergency = (rc.left_switch == 2) || emergency_state_;
        if (!connected_ || emergency)
        {
            nav_mode_enabled_ = false;
            unified_output_.vx = 0.0;
            unified_output_.vy = 0.0;
            unified_output_.wz = 0.0;
            unified_output_.spin_mode = false;
            unified_output_.emergency_stop = emergency;
            unified_output_.friction_on = false;
            unified_output_.fire_trigger = false;
            unified_output_.burst_mode = false;
            return;
        }
        unified_output_.emergency_stop = false;

        // ========== 2. 配置触发动作 ==========
        if (!trigger_definition_.loaded)
        {
            // 保留旧行为作为回退路径
            if (rc.right_switch == 1 || rc.right_switch == 3)
            {
                if (rc.right_switch != last_pause_button_ || !nav_mode_enabled_)
                {
                    unified_output_.vx = 0.0;
                    unified_output_.vy = 0.0;
                    unified_output_.wz = 0.0;
                }
                nav_mode_enabled_ = true;
                unified_output_.navigation_enabled = true;
                last_pause_button_ = rc.right_switch;
            }
            else
            {
                nav_mode_enabled_ = false;
                unified_output_.navigation_enabled = false;
                last_pause_button_ = rc.right_switch;
            }

            if (rc.left_switch == 1 && last_gear_switch_ != 1)
            {
                spin_mode_enabled_ = !spin_mode_enabled_;
                if (spin_mode_enabled_)
                {
                    input_processor_.reset_spin_speed();
                }
                RCLCPP_INFO(this->get_logger(), "Spin Mode: %s", spin_mode_enabled_ ? "ON" : "OFF");
            }
            last_gear_switch_ = rc.left_switch;

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
        }

        // ========== 3. 速度分档 ==========
        input_processor_.update_keyboard_speed(rc.shift == 1, rc.ctrl == 1);

        // ========== 4. 小陀螺速度调节 ==========
        if (spin_mode_enabled_)
        {
            input_processor_.update_spin_speed(rc.dial, rc.shift == 1, rc.ctrl == 1);
        }

        nav_mode_enabled_ = trigger_definition_.loaded ? (nav_topic_state_ || behavior_tree_state_)
                                                       : unified_output_.navigation_enabled;
        unified_output_.navigation_enabled = nav_mode_enabled_;

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
                rc.right_x, rc.right_y,
                rc.w, rc.s, rc.a, rc.d);
            unified_output_.vx = chassis_vel.vx;
            unified_output_.vy = chassis_vel.vy;
            unified_output_.wz = spin_mode_enabled_ ? input_processor_.get_spin_speed() : 0.0;
        }

        unified_output_.spin_mode = spin_mode_enabled_;
        unified_output_.spin_speed = input_processor_.get_spin_speed();
        unified_output_.chassis_speed_scale = input_processor_.get_speed_scale();

        // ========== 6. 云台控制（对齐 sentry_controller 逻辑） ==========
        // legacy mapping:
        // left_right_offset = left_x * 100 + clamp(mouse_x * 0.75, -100, 100)
        // top_down_offset   = left_y * 100 + clamp(mouse_y,        -100, 100)
        // pitch_delta = top_down_offset * 0.00005 * (180 / pi)
        // yaw_delta   = -left_right_offset * 10 * pi * 0.001 * 0.0025
        const double mouse_x = static_cast<double>(rc.mouse_x) * input_config_.mouse_sensitivity;
        const double mouse_y = static_cast<double>(rc.mouse_y) * input_config_.mouse_sensitivity;

        const double left_right_offset =
            static_cast<double>(rc.left_x) * 100.0 +
            InputProcessor::clamp(
                mouse_x * input_config_.mouse_yaw_gain,
                -input_config_.mouse_limit,
                input_config_.mouse_limit);

        const double top_down_offset =
            static_cast<double>(rc.left_y) * 100.0 +
            InputProcessor::clamp(
                mouse_y * input_config_.mouse_pitch_gain,
                -input_config_.mouse_limit,
                input_config_.mouse_limit);

        unified_output_.pitch_delta = top_down_offset * input_config_.pitch_gain_coeff;
        unified_output_.yaw_delta = -left_right_offset * input_config_.yaw_gain_coeff;

        // ========== 7. 发射与模式控制 ==========
        if (trigger_definition_.loaded)
        {
            const bool mouse_autoaim = (rc.mouse_right_clicked == 1);
            const bool mouse_fire = (rc.mouse_left_clicked == 1);

            unified_output_.autoaim_enabled = autoaim_state_ || mouse_autoaim;
            unified_output_.burst_mode = feeder_continuous_state_;
            unified_output_.fire_trigger = fire_single_pulse_ || feeder_continuous_state_ || mouse_fire;
            unified_output_.friction_on = friction_state_;
            unified_output_.friction_speed = friction_state_ ? 6500.0 : 0.0;
        }
        else
        {
            unified_output_.autoaim_enabled = rc.mouse_right_clicked == 1;
            unified_output_.fire_trigger = rc.mouse_left_clicked == 1;
            unified_output_.burst_mode = rc.mouse_left_clicked == 1;
            unified_output_.friction_on = true;
            unified_output_.friction_speed = 6500.0;
        }
    }

    void NDJInterpreter::execute_trigger_actions(const custom_msgs::msg::ReadDJIRC &rc)
    {
        const int left_dock = switch_to_dock(rc.left_switch);
        const int right_dock = switch_to_dock(rc.right_switch);

        if (left_dock >= 1 && left_dock <= 3)
        {
            execute_action_set(trigger_definition_.left_dock_points[static_cast<size_t>(left_dock - 1)]);
        }
        if (right_dock >= 1 && right_dock <= 3)
        {
            execute_action_set(trigger_definition_.right_dock_points[static_cast<size_t>(right_dock - 1)]);
        }

        if (last_left_switch_ != 0 && rc.left_switch != 0 && last_left_switch_ != rc.left_switch)
        {
            const int idx = transition_index(last_left_switch_, rc.left_switch);
            if (idx >= 0)
            {
                execute_action_set(trigger_definition_.left_transitions[static_cast<size_t>(idx)]);
            }
        }

        if (last_right_switch_ != 0 && rc.right_switch != 0 && last_right_switch_ != rc.right_switch)
        {
            const int idx = transition_index(last_right_switch_, rc.right_switch);
            if (idx >= 0)
            {
                execute_action_set(trigger_definition_.right_transitions[static_cast<size_t>(idx)]);
            }
        }

        const double dial_now = static_cast<double>(rc.dial);
        if (last_dial_ <= trigger_definition_.dial_up.threshold &&
            dial_now > trigger_definition_.dial_up.threshold)
        {
            execute_action_set(trigger_definition_.dial_up.actions);
        }

        const double down_threshold = -std::abs(trigger_definition_.dial_down.threshold);
        if (last_dial_ >= down_threshold && dial_now < down_threshold)
        {
            execute_action_set(trigger_definition_.dial_down.actions);
        }

        last_left_switch_ = rc.left_switch;
        last_right_switch_ = rc.right_switch;
        last_dial_ = dial_now;
    }

    void NDJInterpreter::execute_action_set(const ActionSet &actions)
    {
        apply_tri_state_action(actions.emergency_stop, emergency_state_);
        apply_tri_state_action(actions.autoaim, autoaim_state_);
        apply_tri_state_action(actions.nav_topic, nav_topic_state_);
        apply_tri_state_action(actions.behavior_tree_topic, behavior_tree_state_);
        apply_tri_state_action(actions.friction_wheel, friction_state_);
        apply_tri_state_action(actions.spin_mode, spin_mode_enabled_);
        apply_feeder_action(actions.feeder);
        apply_spin_control_action(actions.spin_control);
    }

    void NDJInterpreter::apply_tri_state_action(const TriStateAction &action, bool &state)
    {
        if (action.on)
        {
            state = true;
        }
        if (action.off)
        {
            state = false;
        }
        if (action.toggle)
        {
            state = !state;
        }
    }

    void NDJInterpreter::apply_feeder_action(const FeederAction &action)
    {
        if (action.single_shot_once)
        {
            fire_single_pulse_ = true;
        }
        if (action.continuous_start)
        {
            feeder_continuous_state_ = true;
        }
        if (action.continuous_stop)
        {
            feeder_continuous_state_ = false;
        }
        if (action.continuous_toggle)
        {
            feeder_continuous_state_ = !feeder_continuous_state_;
        }
    }

    void NDJInterpreter::apply_spin_control_action(const SpinControlAction &action)
    {
        if (action.switch_toggle)
        {
            spin_mode_enabled_ = !spin_mode_enabled_;
            if (spin_mode_enabled_)
            {
                input_processor_.reset_spin_speed();
            }
        }
        if (action.accelerate)
        {
            input_processor_.update_spin_speed(-1.0, false, false);
        }
        if (action.decelerate)
        {
            input_processor_.update_spin_speed(1.0, false, false);
        }
    }

    int NDJInterpreter::switch_to_dock(uint8_t switch_value)
    {
        if (switch_value == 1)
        {
            return 1;
        }
        if (switch_value == 3)
        {
            return 2;
        }
        if (switch_value == 2)
        {
            return 3;
        }
        return 0;
    }

    int NDJInterpreter::transition_index(int from_switch, int to_switch)
    {
        // NDJ: 上/中/下 = 1/3/2
        // transitions index: 0=13, 1=31, 2=23, 3=32
        if (from_switch == 1 && to_switch == 3)
        {
            return 0;
        }
        if (from_switch == 3 && to_switch == 1)
        {
            return 1;
        }
        if (from_switch == 2 && to_switch == 3)
        {
            return 2;
        }
        if (from_switch == 3 && to_switch == 2)
        {
            return 3;
        }
        return -1;
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

        if (!raw_rc_data_ || !connected_)
        {
            return;
        }

        unified_output_.header.stamp = this->now();
        pub_unified_->publish(unified_output_);
    }

} // namespace universal_controller
