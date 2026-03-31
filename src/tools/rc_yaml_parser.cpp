/**
 * @file rc_yaml_parser.cpp
 * @brief 遥控器 YAML 配置解析工具实现
 */

#include "universal_controller/tools/rc_yaml_parser.hpp"

namespace universal_controller {

bool yaml_bool(const YAML::Node &node, const char *key) {
    return (node && node[key]) ? node[key].as<bool>() : false;
}

double yaml_double(const YAML::Node &node, const char *key, double default_value) {
    return (node && node[key]) ? node[key].as<double>() : default_value;
}

void parse_tristate(const YAML::Node &actions_node, const char *key, TriStateAction &out) {
    const auto n = actions_node[key];
    if (!n) {
        return;
    }
    out.on = yaml_bool(n, "on");
    out.off = yaml_bool(n, "off");
    out.toggle = yaml_bool(n, "toggle");
}

void parse_spin_control_action(const YAML::Node &spin_control_node, SpinControlAction &out) {
    if (!spin_control_node) {
        return;
    }
    out.accelerate = yaml_bool(spin_control_node, "accelerate");
    out.decelerate = yaml_bool(spin_control_node, "decelerate");
}

void parse_actions(const YAML::Node &actions_node, ActionSet &out) {
    if (!actions_node) {
        return;
    }

    parse_tristate(actions_node, "autoaim", out.autoaim);
    parse_tristate(actions_node, "emergency_stop", out.emergency_stop);
    parse_tristate(actions_node, "nav_topic", out.nav_topic);
    parse_tristate(actions_node, "behavior_tree_topic", out.behavior_tree_topic);
    parse_tristate(actions_node, "friction_wheel", out.friction_wheel);
    parse_tristate(actions_node, "spin_mode", out.spin_mode);
    parse_tristate(actions_node, "feeder", out.feeder);
    parse_tristate(actions_node, "feeder_burst", out.feeder_burst);

    const auto spin_control = actions_node["spin_control"];
    if (spin_control) {
        parse_spin_control_action(spin_control, out.spin_control);
    }
}

void parse_dock_points_three(const YAML::Node &dock_points_node, std::array<ActionSet, 3> &out, const std::array<std::string, 3> &names) {
    if (!dock_points_node) {
        return;
    }

    for (size_t i = 0; i < names.size(); ++i) {
        const YAML::Node dock_node = dock_points_node[names[i]];
        if (!dock_node) {
            continue;
        }
        parse_actions(dock_node["actions"], out[i]);
    }
}

void parse_dock_points(const YAML::Node &dock_points_node, std::array<ActionSet, 3> &out) {
    const std::array<std::string, 3> names = {"up", "mid", "down"};
    parse_dock_points_three(dock_points_node, out, names);
}

void parse_dock_points_two(const YAML::Node &dock_points_node, std::array<ActionSet, 2> &out) {
    if (!dock_points_node) {
        return;
    }

    const std::array<std::string, 2> names = {"left", "right"};
    for (size_t i = 0; i < names.size(); ++i) {
        const YAML::Node dock_node = dock_points_node[names[i]];
        if (!dock_node) {
            continue;
        }
        parse_actions(dock_node["actions"], out[i]);
    }
}

void parse_transitions(const YAML::Node &transitions_node, std::array<ActionSet, 4> &out, const std::array<std::string, 4> &names) {
    if (!transitions_node) {
        return;
    }

    for (size_t i = 0; i < names.size(); ++i) {
        const YAML::Node trans_node = transitions_node[names[i]];
        if (!trans_node) {
            continue;
        }
        parse_actions(trans_node["actions"], out[i]);
    }
}

void parse_dial_action(const YAML::Node &dial_node, DialAction &out) {
    if (!dial_node) {
        return;
    }

    out.threshold = yaml_double(dial_node, "threshold", 0.5);
    parse_actions(dial_node["actions"], out.actions);
}

void parse_button_definition(const YAML::Node &button_node, ButtonDefinition &out) {
    if (!button_node) {
        return;
    }

    const auto event_actions = [](const YAML::Node &event_node) -> YAML::Node {
        if (!event_node) {
            return YAML::Node();
        }
        if (event_node["actions"]) {
            return event_node["actions"];
        }
        return event_node;
    };

    out.loaded = true;
    out.long_press_threshold_s = yaml_double(button_node, "long_press_threshold_s", 0.2);
    parse_actions(event_actions(button_node["on_press"]), out.on_press);
    parse_actions(event_actions(button_node["on_short_press_released"]), out.on_short_press_released);
    parse_actions(event_actions(button_node["on_long_press_reached"]), out.on_long_press_reached);
    parse_actions(event_actions(button_node["on_long_press_released"]), out.on_long_press_released);
    parse_actions(event_actions(button_node["on_release"]), out.on_release);

    // 解析 on_released_after_s: {时间阈值: {actions: ...}}
    const auto released_after = button_node["on_released_after_s"];
    if (released_after && released_after.IsMap()) {
        for (const auto &entry : released_after) {
            double threshold = entry.first.as<double>();
            ActionSet actions;
            parse_actions(event_actions(entry.second), actions);
            out.on_released_after_s[threshold] = actions;
        }
    }
}

// ========== 动作执行工具实现 ==========

void apply_tri_state_action(const TriStateAction &action, bool &state) {
    if (action.on) {
        state = true;
    }
    if (action.off) {
        state = false;
    }
    if (action.toggle) {
        state = !state;
    }
}

void apply_spin_control_action(const SpinControlAction &action, double &spin_speed_delta) {
    if (action.accelerate) {
        spin_speed_delta += 1.0;
    }
    if (action.decelerate) {
        spin_speed_delta -= 1.0;
    }
}

void execute_action_set(const ActionSet &actions, ActionStates &states) {
    apply_tri_state_action(actions.emergency_stop, states.emergency_state);
    apply_tri_state_action(actions.autoaim, states.autoaim_state);
    apply_tri_state_action(actions.nav_topic, states.nav_topic_state);
    apply_tri_state_action(actions.behavior_tree_topic, states.behavior_tree_state);
    apply_tri_state_action(actions.friction_wheel, states.friction_state);
    apply_tri_state_action(actions.spin_mode, states.spin_mode);
    apply_tri_state_action(actions.feeder, states.feeder_state);
    apply_tri_state_action(actions.feeder_burst, states.burst_mode);
    apply_spin_control_action(actions.spin_control, states.spin_speed_delta);
}

// ========== 键鼠定义解析 ==========

void parse_km_button_event(const YAML::Node &event_node, KMButtonEvent &out) {
    if (!event_node) {
        return;
    }

    out.speed = yaml_double(event_node, "speed", -1.0);
    parse_actions(event_node["actions"], out.actions);
}

void parse_km_button_definition(const YAML::Node &button_node, KMButtonDefinition &out) {
    if (!button_node) {
        return;
    }

    out.loaded = true;
    out.long_press_threshold_s = yaml_double(button_node, "long_press_threshold_s", 0.2);
    parse_km_button_event(button_node["on_press"], out.on_press);
    parse_km_button_event(button_node["on_short_press_released"], out.on_short_press_released);
    parse_km_button_event(button_node["on_long_press_reached"], out.on_long_press_reached);
    parse_km_button_event(button_node["on_long_press_released"], out.on_long_press_released);
    parse_km_button_event(button_node["on_release"], out.on_release);

    // 解析 on_released_after_s: {时间阈值: {actions: ..., speed: ...}}
    const auto released_after = button_node["on_released_after_s"];
    if (released_after && released_after.IsMap()) {
        for (const auto &entry : released_after) {
            double threshold = entry.first.as<double>();
            KMButtonEvent event;
            parse_km_button_event(entry.second, event);
            out.on_released_after_s[threshold] = event;
        }
    }
}

bool parse_km_trigger_definition(const std::string &file_path, KMTriggerDefinition &out) {
    try {
        const YAML::Node root = YAML::LoadFile(file_path);
        const YAML::Node def = root["km_trigger_def"];
        if (!def) {
            return false;
        }

        out.speed_default = yaml_double(def, "speed_default", 5000.0);

        // 速度控制键
        parse_km_button_definition(def["key_shift"], out.key_shift);
        parse_km_button_definition(def["key_ctrl"], out.key_ctrl);

        // 功能键
        parse_km_button_definition(def["key_q"], out.key_q);
        parse_km_button_definition(def["key_e"], out.key_e);
        parse_km_button_definition(def["key_r"], out.key_r);
        parse_km_button_definition(def["key_f"], out.key_f);
        parse_km_button_definition(def["key_g"], out.key_g);
        parse_km_button_definition(def["key_z"], out.key_z);
        parse_km_button_definition(def["key_x"], out.key_x);
        parse_km_button_definition(def["key_c"], out.key_c);
        parse_km_button_definition(def["key_v"], out.key_v);
        parse_km_button_definition(def["key_b"], out.key_b);

        // 鼠标按钮
        parse_km_button_definition(def["mouse_left"], out.mouse_left);
        parse_km_button_definition(def["mouse_right"], out.mouse_right);
        parse_km_button_definition(def["mouse_middle"], out.mouse_middle);

        // 鼠标滚轮
        parse_dial_action(def["mouse_wheel_up"], out.mouse_wheel_up);
        parse_dial_action(def["mouse_wheel_down"], out.mouse_wheel_down);

        out.loaded = true;
        return true;
    } catch (const std::exception &e) {
        return false;
    }
}

} // namespace universal_controller
