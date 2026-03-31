/**
 * @file ndj_interpreter.cpp
 * @brief NDJ 遥控器解释器实现（大疆新图传遥控器）
 */

#include "universal_controller/interpreters/ndj_interpreter.hpp"
#include "universal_controller/tools/rc_yaml_parser.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace universal_controller {

NDJInterpreter::NDJInterpreter(const rclcpp::NodeOptions &options)
    : Node("ndj_interpreter", options),
      input_processor_(),
      km_parser_() {
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
    unified_output_.control_source = "ndj";
    unified_output_.connected = false;

    RCLCPP_INFO(this->get_logger(), "NDJ Interpreter started (ReadDJIRC)");
}

void NDJInterpreter::declare_parameters() {
    // 话题
    this->declare_parameter("rc_interpreter.topic_ndj_rc", "/ecat/sn4653115/app1/read");
    this->declare_parameter("rc_interpreter.topic_ndj_output", "/universal_controller/input/ndj");
    this->declare_parameter("rc_interpreter.ndj_definition_file", "");
    this->declare_parameter("rc_interpreter.km_definition_file", "");

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

void NDJInterpreter::load_parameters() {
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

    // 更新键鼠解析器
    km_parser_.update_config(input_config_);

    if (ndj_definition_file_.empty()) {
        try {
            ndj_definition_file_ = ament_index_cpp::get_package_share_directory("universal_controller") +
                                   "/config/ndj_definition.sentry.yaml";
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to resolve share config path: %s", e.what());
        }
    }

    if (!ndj_definition_file_.empty()) {
        load_trigger_definition(ndj_definition_file_);
    }

    // 加载键鼠 YAML 定义
    km_definition_file_ = this->get_parameter("rc_interpreter.km_definition_file").as_string();
    if (km_definition_file_.empty()) {
        try {
            km_definition_file_ = ament_index_cpp::get_package_share_directory("universal_controller") +
                                  "/config/km_definition.yaml";
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to resolve KM config path: %s", e.what());
        }
    }
    if (!km_definition_file_.empty()) {
        if (km_parser_.load_definition(km_definition_file_)) {
            RCLCPP_INFO(this->get_logger(), "Loaded KM definition: %s", km_definition_file_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load KM definition: %s", km_definition_file_.c_str());
        }
    }
}

void NDJInterpreter::load_trigger_definition(const std::string &file_path) {
    try {
        const YAML::Node root = YAML::LoadFile(file_path);
        const YAML::Node def = root["ndj_trigger_def"];
        if (!def) {
            RCLCPP_WARN(this->get_logger(), "ndj_trigger_def not found in %s", file_path.c_str());
            return;
        }

        // 使用工具解析
        const auto left = def["left_switch"];
        if (left) {
            parse_dock_points(left["dock_points"], trigger_definition_.left_dock_points);

            const std::array<std::string, 4> trans_names = {
                "up_to_mid", "mid_to_up", "down_to_mid", "mid_to_down"};
            parse_transitions(left["transitions"], trigger_definition_.left_transitions, trans_names);
        }

        const auto right = def["right_switch"];
        if (right) {
            parse_dock_points(right["dock_points"], trigger_definition_.right_dock_points);

            const std::array<std::string, 4> trans_names = {
                "up_to_mid", "mid_to_up", "down_to_mid", "mid_to_down"};
            parse_transitions(right["transitions"], trigger_definition_.right_transitions, trans_names);
        }

        parse_dial_action(def["dial_up"], trigger_definition_.dial_up);
        parse_dial_action(def["dial_down"], trigger_definition_.dial_down);

        trigger_definition_.loaded = true;
        RCLCPP_INFO(this->get_logger(), "Loaded NDJ trigger definition: %s", file_path.c_str());
    } catch (const std::exception &e) {
        trigger_definition_.loaded = false;
        RCLCPP_WARN(this->get_logger(), "Failed to load NDJ definition %s: %s", file_path.c_str(), e.what());
    }
}

void NDJInterpreter::cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg) {
    raw_rc_data_ = msg;
    last_rc_time_ = this->now();
    connected_ = (msg->online == 1);
    process_input();
}

KeyboardMouseInput NDJInterpreter::map_keyboard_mouse(
    const custom_msgs::msg::ReadDJIRC &rc) {
    KeyboardMouseInput kmi;
    kmi.key_w = rc.w;
    kmi.key_s = rc.s;
    kmi.key_a = rc.a;
    kmi.key_d = rc.d;
    kmi.key_q = rc.q;
    kmi.key_e = rc.e;
    kmi.key_r = rc.r;
    kmi.key_f = rc.f;
    kmi.key_g = rc.g;
    kmi.key_z = rc.z;
    kmi.key_x = rc.x;
    kmi.key_c = rc.c;
    kmi.key_v = rc.v;
    kmi.key_b = rc.b;
    kmi.key_shift = rc.shift == 1;
    kmi.key_ctrl = rc.ctrl == 1;
    kmi.mouse_x = static_cast<double>(rc.mouse_x);
    kmi.mouse_y = static_cast<double>(rc.mouse_y);
    kmi.mouse_left = rc.mouse_left_clicked == 1;
    kmi.mouse_right = rc.mouse_right_clicked == 1;
    return kmi;
}

void NDJInterpreter::process_input() {
    // 检查连接超时
    if (connected_) {
        auto now = this->now();
        if ((now - last_rc_time_).seconds() > connection_timeout_s_) {
            connected_ = false;
        }
    }

    if (!raw_rc_data_ || !connected_) {
        unified_output_.emergency_stop = true;
        unified_output_.connected = false;
        unified_output_.friction_on = false;
        unified_output_.fire_trigger = false;
        unified_output_.burst_mode = false;
        return;
    }

    const auto &rc = *raw_rc_data_;
    unified_output_.connected = connected_;
    unified_output_.control_source = "ndj";
    unified_output_.header.stamp = this->now();

    if (trigger_definition_.loaded) {
        execute_trigger_actions(rc);
    }

    // ========== 键鼠解析 ==========
    auto kmi = map_keyboard_mouse(rc);
    km_parser_.set_spin_mode(spin_mode_enabled_);

    // 处理 KM 按钮事件（边沿检测 + 长按）→ ActionSet
    if (km_parser_.definition_loaded()) {
        double time_s = this->now().seconds();
        auto km_actions = km_parser_.process_button_events(kmi, time_s);
        execute_action_set(km_actions);
    }

    auto km_out = km_parser_.parse(kmi);

    // ========== 1. 小陀螺调速（拨轮 RC + 键盘） ==========
    km_parser_.update_spin_speed(rc.dial, kmi.key_shift, kmi.key_ctrl);

    // ========== 2. 急停检测 ==========
    bool emergency = (rc.left_switch == 2) || emergency_state_;
    if (!connected_ || emergency) {
        nav_mode_enabled_ = false;
        unified_output_.vx = 0.0;
        unified_output_.vy = 0.0;
        unified_output_.wz = 0.0;
        unified_output_.spin_mode = false;
        unified_output_.spin_speed = km_parser_.get_spin_speed();
        unified_output_.chassis_speed_scale = km_parser_.get_speed_scale();
        unified_output_.emergency_stop = emergency;
        unified_output_.friction_on = false;
        unified_output_.fire_trigger = false;
        unified_output_.burst_mode = false;
        return;
    }
    unified_output_.emergency_stop = false;

    // ========== 3. 导航模式 ==========
    nav_mode_enabled_ = trigger_definition_.loaded ? (nav_topic_state_ || behavior_tree_state_)
                                                   : unified_output_.navigation_enabled;
    unified_output_.navigation_enabled = nav_mode_enabled_;

    // ========== 4. 底盘速度 ==========
    if (nav_mode_enabled_) {
        unified_output_.vx = 0.0;
        unified_output_.vy = 0.0;
        unified_output_.wz = 0.0;
    } else {
        // 摇杆（RC 硬件）+ 键盘方向（parser 输出）
        double joystick_vx = input_processor_.process_joystick(rc.right_y);
        double joystick_vy = input_processor_.process_joystick(rc.right_x);
        unified_output_.vx = joystick_vx + km_out.key_vx;
        unified_output_.vy = joystick_vy + km_out.key_vy;
        unified_output_.wz = spin_mode_enabled_ ? km_parser_.get_spin_speed() : 0.0;
    }

    unified_output_.spin_mode = spin_mode_enabled_;
    unified_output_.spin_speed = km_parser_.get_spin_speed();
    unified_output_.chassis_speed_scale = km_parser_.get_speed_scale();

    // ========== 5. 云台控制（左摇杆 RC + 鼠标 parser 输出） ==========
    double joystick_pitch = static_cast<double>(rc.left_y) * 100.0 *
                            input_config_.pitch_gain_coeff;
    double joystick_yaw = static_cast<double>(rc.left_x) * 100.0 *
                          input_config_.yaw_gain_coeff;
    unified_output_.pitch_delta = joystick_pitch + km_out.pitch_delta;
    unified_output_.yaw_delta = -(joystick_yaw + km_out.yaw_delta); // NDJ yaw 取反

    // ========== 6. 发射与模式控制 ==========
    if (trigger_definition_.loaded) {
        unified_output_.autoaim_enabled = autoaim_state_ || km_out.mouse_autoaim;
        unified_output_.burst_mode = burst_mode_;
        unified_output_.fire_trigger = feeder_state_ || burst_mode_ || km_out.mouse_fire;
        unified_output_.friction_on = friction_state_;
        unified_output_.friction_speed = friction_state_ ? 6500.0 : 0.0;
    } else {
        unified_output_.autoaim_enabled = km_out.mouse_autoaim;
        unified_output_.fire_trigger = km_out.mouse_fire;
        unified_output_.burst_mode = km_out.mouse_fire;
        unified_output_.friction_on = true;
        unified_output_.friction_speed = 6500.0;
    }
}

void NDJInterpreter::execute_trigger_actions(const custom_msgs::msg::ReadDJIRC &rc) {
    const int left_dock = switch_to_dock(rc.left_switch);
    const int right_dock = switch_to_dock(rc.right_switch);

    if (left_dock >= 1 && left_dock <= 3) {
        execute_action_set(trigger_definition_.left_dock_points[static_cast<size_t>(left_dock - 1)]);
    }
    if (right_dock >= 1 && right_dock <= 3) {
        execute_action_set(trigger_definition_.right_dock_points[static_cast<size_t>(right_dock - 1)]);
    }

    if (last_left_switch_ != 0 && rc.left_switch != 0 && last_left_switch_ != rc.left_switch) {
        const int idx = transition_index(last_left_switch_, rc.left_switch);
        if (idx >= 0) {
            execute_action_set(trigger_definition_.left_transitions[static_cast<size_t>(idx)]);
        }
    }

    if (last_right_switch_ != 0 && rc.right_switch != 0 && last_right_switch_ != rc.right_switch) {
        const int idx = transition_index(last_right_switch_, rc.right_switch);
        if (idx >= 0) {
            execute_action_set(trigger_definition_.right_transitions[static_cast<size_t>(idx)]);
        }
    }

    const double dial_now = static_cast<double>(rc.dial);
    if (last_dial_ <= trigger_definition_.dial_up.threshold &&
        dial_now > trigger_definition_.dial_up.threshold) {
        execute_action_set(trigger_definition_.dial_up.actions);
    }

    const double down_threshold = -std::abs(trigger_definition_.dial_down.threshold);
    if (last_dial_ >= down_threshold && dial_now < down_threshold) {
        execute_action_set(trigger_definition_.dial_down.actions);
    }

    last_left_switch_ = rc.left_switch;
    last_right_switch_ = rc.right_switch;
    last_dial_ = dial_now;
}

void NDJInterpreter::execute_action_set(const ActionSet &actions) {
    // 使用抽象工具函数
    apply_tri_state_action(actions.emergency_stop, emergency_state_);
    apply_tri_state_action(actions.autoaim, autoaim_state_);
    apply_tri_state_action(actions.nav_topic, nav_topic_state_);
    apply_tri_state_action(actions.behavior_tree_topic, behavior_tree_state_);
    apply_tri_state_action(actions.friction_wheel, friction_state_);
    apply_tri_state_action(actions.spin_mode, spin_mode_enabled_);
    apply_tri_state_action(actions.feeder, feeder_state_);
    apply_tri_state_action(actions.feeder_burst, burst_mode_);

    // 小陀螺速度控制
    if (actions.spin_control.accelerate) {
        km_parser_.update_spin_speed(-1.0, false, false);
    }
    if (actions.spin_control.decelerate) {
        km_parser_.update_spin_speed(1.0, false, false);
    }
}

int NDJInterpreter::switch_to_dock(uint8_t switch_value) {
    if (switch_value == 1) {
        return 1;
    }
    if (switch_value == 3) {
        return 2;
    }
    if (switch_value == 2) {
        return 3;
    }
    return 0;
}

int NDJInterpreter::transition_index(int from_switch, int to_switch) {
    // NDJ: 上/中/下 = 1/3/2
    // transitions index: 0=13, 1=31, 2=23, 3=32
    if (from_switch == 1 && to_switch == 3) {
        return 0;
    }
    if (from_switch == 3 && to_switch == 1) {
        return 1;
    }
    if (from_switch == 2 && to_switch == 3) {
        return 2;
    }
    if (from_switch == 3 && to_switch == 2) {
        return 3;
    }
    return -1;
}

void NDJInterpreter::publish_unified() {
    // 检查连接超时
    if (connected_) {
        auto now = this->now();
        if ((now - last_rc_time_).seconds() > connection_timeout_s_) {
            connected_ = false;
            unified_output_.connected = false;
            unified_output_.emergency_stop = true;
        }
    }

    if (!raw_rc_data_) {
        return;
    }

    if (!connected_) {
        unified_output_.connected = false;
        unified_output_.emergency_stop = true;
        unified_output_.friction_on = false;
        unified_output_.fire_trigger = false;
        unified_output_.burst_mode = false;
        unified_output_.header.stamp = this->now();
        pub_unified_->publish(unified_output_);
    }

    unified_output_.header.stamp = this->now();
    pub_unified_->publish(unified_output_);
}

} // namespace universal_controller
