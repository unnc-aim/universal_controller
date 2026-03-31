/**
 * @file keyboard_mouse_parser.cpp
 * @brief 统一键鼠解析器实现
 */

#include "universal_controller/tools/keyboard_mouse_parser.hpp"
#include "universal_controller/tools/rc_yaml_parser.hpp"

#include <algorithm>
#include <cmath>

namespace universal_controller {

KeyboardMouseParser::KeyboardMouseParser(const InputProcessorConfig &config)
    : config_(config),
      current_spd_mode_(config.keyboard_speed_default),
      spin_spd_(config.spin_speed_default) {
}

// ========== 按钮事件处理 ==========

void KeyboardMouseParser::process_one_button(
    bool current_pressed, BtnState &state,
    const KMButtonDefinition &def,
    double current_time_s, ActionSet &accum, double &speed_out)
{
    if (!def.loaded) {
        return;
    }

    auto flags = detect_button_events(
        current_pressed,
        state.last_pressed,
        state.press_start_time_s,
        state.long_press_active,
        current_time_s,
        def.long_press_threshold_s);

    // 按下
    if (flags.press_edge) {
        if (def.on_press.speed >= 0) {
            speed_out = def.on_press.speed;
        }
        merge_action_set(accum, def.on_press.actions);
    }

    // 短按释放
    if (flags.short_release) {
        if (def.on_short_press_released.speed >= 0) {
            speed_out = def.on_short_press_released.speed;
        }
        merge_action_set(accum, def.on_short_press_released.actions);
    }

    // 长按到达
    if (flags.long_reached) {
        if (def.on_long_press_reached.speed >= 0) {
            speed_out = def.on_long_press_reached.speed;
        }
        merge_action_set(accum, def.on_long_press_reached.actions);
    }

    // 长按释放
    if (flags.long_release) {
        if (def.on_long_press_released.speed >= 0) {
            speed_out = def.on_long_press_released.speed;
        }
        merge_action_set(accum, def.on_long_press_released.actions);
    }

    // 任何释放
    if (flags.release_edge) {
        if (def.on_release.speed >= 0) {
            speed_out = def.on_release.speed;
        }
        merge_action_set(accum, def.on_release.actions);
    }
}

ActionSet KeyboardMouseParser::process_button_events(
    const KeyboardMouseInput &input, double current_time_s)
{
    ActionSet accum;
    double speed_val = -1.0; // < 0 means no change

    // 速度控制键
    process_one_button(input.key_shift, btn_shift_, km_def_.key_shift,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_ctrl, btn_ctrl_, km_def_.key_ctrl,
                       current_time_s, accum, speed_val);

    // 功能键
    process_one_button(input.key_q, btn_q_, km_def_.key_q,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_e, btn_e_, km_def_.key_e,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_r, btn_r_, km_def_.key_r,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_f, btn_f_, km_def_.key_f,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_g, btn_g_, km_def_.key_g,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_z, btn_z_, km_def_.key_z,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_x, btn_x_, km_def_.key_x,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_c, btn_c_, km_def_.key_c,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_v, btn_v_, km_def_.key_v,
                       current_time_s, accum, speed_val);
    process_one_button(input.key_b, btn_b_, km_def_.key_b,
                       current_time_s, accum, speed_val);

    // 鼠标按钮
    process_one_button(input.mouse_left, btn_mouse_left_, km_def_.mouse_left,
                       current_time_s, accum, speed_val);
    process_one_button(input.mouse_right, btn_mouse_right_, km_def_.mouse_right,
                       current_time_s, accum, speed_val);
    process_one_button(input.mouse_middle, btn_mouse_middle_, km_def_.mouse_middle,
                       current_time_s, accum, speed_val);

    // 应用速度设置
    if (speed_val >= 0) {
        current_spd_mode_ = std::clamp(speed_val, config_.keyboard_speed_min, config_.keyboard_speed_max);
    }

    // 鼠标滚轮（DialAction 模式）
    const double wheel_now = input.mouse_wheel;
    if (km_def_.mouse_wheel_up.actions.spin_control.accelerate ||
        km_def_.mouse_wheel_up.actions.spin_control.decelerate) {
        if (last_mouse_wheel_ <= km_def_.mouse_wheel_up.threshold &&
            wheel_now > km_def_.mouse_wheel_up.threshold) {
            merge_action_set(accum, km_def_.mouse_wheel_up.actions);
        }
    }
    if (km_def_.mouse_wheel_down.actions.spin_control.accelerate ||
        km_def_.mouse_wheel_down.actions.spin_control.decelerate) {
        const double down_threshold = -std::abs(km_def_.mouse_wheel_down.threshold);
        if (last_mouse_wheel_ >= down_threshold &&
            wheel_now < down_threshold) {
            merge_action_set(accum, km_def_.mouse_wheel_down.actions);
        }
    }
    last_mouse_wheel_ = wheel_now;

    return accum;
}

bool KeyboardMouseParser::load_definition(const std::string &file_path) {
    bool ok = parse_km_trigger_definition(file_path, km_def_);
    if (ok) {
        current_spd_mode_ = km_def_.speed_default;
    }
    return ok;
}

bool KeyboardMouseParser::definition_loaded() const {
    return km_def_.loaded;
}

// ========== 键鼠解析 ==========

KeyboardMouseOutput KeyboardMouseParser::parse(const KeyboardMouseInput &input) {
    KeyboardMouseOutput out;

    // ========== 1. 速度分档 ==========
    if (km_def_.loaded) {
        // KM 定义加载后：速度由 process_button_events() 中的 speed 字段控制
        // 此处不再做增量调整
    } else {
        // 未加载 KM 定义：保持原有增量逻辑
        if (input.key_shift && !input.key_ctrl) {
            current_spd_mode_ = std::min(config_.keyboard_speed_max, current_spd_mode_ + config_.keyboard_speed_step);
        } else if (input.key_ctrl && !input.key_shift) {
            current_spd_mode_ = std::max(config_.keyboard_speed_min, current_spd_mode_ - config_.keyboard_speed_step);
        }
    }

    out.keyboard_speed = current_spd_mode_;
    out.speed_scale = current_spd_mode_ / config_.keyboard_speed_max;

    // ========== 2. 键盘方向（WASD） ==========
    double key_fb = (input.key_w ? 1.0 : 0.0) - (input.key_s ? 1.0 : 0.0);
    double key_lr = (input.key_d ? 1.0 : 0.0) - (input.key_a ? 1.0 : 0.0);
    out.key_vx = key_fb * current_spd_mode_;
    out.key_vy = key_lr * current_spd_mode_;

    // ========== 3. 鼠标云台增量 ==========
    double mx = input.mouse_x * config_.mouse_sensitivity;
    double my = input.mouse_y * config_.mouse_sensitivity;

    double mouse_yaw_offset = std::clamp(
        mx * config_.mouse_yaw_gain,
        -config_.mouse_limit, config_.mouse_limit);
    double mouse_pitch_offset = std::clamp(
        my * config_.mouse_pitch_gain,
        -config_.mouse_limit, config_.mouse_limit);

    out.pitch_delta = mouse_pitch_offset * config_.pitch_gain_coeff * (M_PI / 180.0);
    out.yaw_delta = mouse_yaw_offset * config_.yaw_gain_coeff;

    // ========== 4. 小陀螺 ==========
    out.spin_mode = spin_mode_;
    out.spin_speed = spin_spd_;

    // ========== 5. 鼠标按键 + 滚轮 ==========
    if (km_def_.loaded) {
        // KM 定义已加载：鼠标按钮由 action 系统控制，不再直接传递
        out.mouse_autoaim = false;
        out.mouse_fire = false;
    } else {
        // 未加载 KM 定义：直接传递鼠标按钮状态（向后兼容）
        out.mouse_autoaim = input.mouse_right;
        out.mouse_fire = input.mouse_left;
    }
    out.mouse_middle = input.mouse_middle;
    out.mouse_wheel = input.mouse_wheel;

    return out;
}

// ========== 小陀螺速度控制 ==========

void KeyboardMouseParser::update_spin_speed(double dial_value, bool shift, bool ctrl) {
    double dial_input = (std::abs(dial_value) < config_.spin_dial_deadzone) ? 0.0 : dial_value;
    spin_spd_ += -dial_input * config_.spin_dial_gain;

    // 速度控制键对小陀螺的影响
    if (km_def_.loaded) {
        // KM 定义加载后：spin speed 由 dial_only 控制
        // shift/ctrl 对小陀螺的影响已通过 process_button_events 的 actions 处理
    } else {
        if (shift && !ctrl) {
            spin_spd_ += config_.spin_key_gain;
        } else if (ctrl && !shift) {
            spin_spd_ -= config_.spin_key_gain;
        }
    }

    spin_spd_ = std::clamp(spin_spd_, config_.spin_speed_min, config_.spin_speed_max);
}

double KeyboardMouseParser::get_spin_speed() const {
    return spin_spd_;
}

void KeyboardMouseParser::set_spin_mode(bool enabled) {
    spin_mode_ = enabled;
}

bool KeyboardMouseParser::get_spin_mode() const {
    return spin_mode_;
}

double KeyboardMouseParser::get_speed_scale() const {
    return current_spd_mode_ / config_.keyboard_speed_max;
}

void KeyboardMouseParser::update_config(const InputProcessorConfig &config) {
    config_ = config;
}

} // namespace universal_controller
