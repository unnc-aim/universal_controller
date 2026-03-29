/**
 * @file keyboard_mouse_parser.cpp
 * @brief 统一键鼠解析器实现
 */

#include "universal_controller/tools/keyboard_mouse_parser.hpp"
#include <cmath>
#include <algorithm>

namespace universal_controller
{

    KeyboardMouseParser::KeyboardMouseParser(const InputProcessorConfig &config)
        : config_(config),
          current_spd_mode_(config.keyboard_speed_default),
          spin_spd_(config.spin_speed_default)
    {
    }

    KeyboardMouseOutput KeyboardMouseParser::parse(const KeyboardMouseInput &input)
    {
        KeyboardMouseOutput out;

        // ========== 1. 速度分档 ==========
        if (input.key_shift && !input.key_ctrl)
        {
            current_spd_mode_ = std::min(config_.keyboard_speed_max,
                                         current_spd_mode_ + config_.keyboard_speed_step);
        }
        else if (input.key_ctrl && !input.key_shift)
        {
            current_spd_mode_ = std::max(config_.keyboard_speed_min,
                                         current_spd_mode_ - config_.keyboard_speed_step);
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

        out.pitch_delta = mouse_pitch_offset * config_.pitch_gain_coeff;
        out.yaw_delta = mouse_yaw_offset * config_.yaw_gain_coeff;

        // ========== 4. 鼠标按键 ==========
        out.mouse_autoaim = input.mouse_right;
        out.mouse_fire = input.mouse_left;

        return out;
    }

    void KeyboardMouseParser::update_spin_speed(double dial_value, bool shift, bool ctrl)
    {
        double dial_input = (std::abs(dial_value) < config_.spin_dial_deadzone) ? 0.0 : dial_value;
        spin_spd_ += -dial_input * config_.spin_dial_gain;

        if (shift && !ctrl)
        {
            spin_spd_ += config_.spin_key_gain;
        }
        else if (ctrl && !shift)
        {
            spin_spd_ -= config_.spin_key_gain;
        }

        spin_spd_ = std::clamp(spin_spd_, config_.spin_speed_min, config_.spin_speed_max);
    }

    double KeyboardMouseParser::get_spin_speed() const
    {
        return spin_spd_;
    }

    double KeyboardMouseParser::get_speed_scale() const
    {
        return current_spd_mode_ / config_.keyboard_speed_max;
    }

    void KeyboardMouseParser::update_config(const InputProcessorConfig &config)
    {
        config_ = config;
    }

} // namespace universal_controller
