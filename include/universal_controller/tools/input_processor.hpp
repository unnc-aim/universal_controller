/**
 * @file input_processor.hpp
 * @brief 输入处理工具类
 *
 * 统一处理摇杆、键盘、鼠标的编解码逻辑：
 * - 死区处理
 * - 速度计算（摇杆 + 键盘组合）
 * - 小陀螺速度调节
 * - 鼠标云台控制
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace universal_controller {

/**
 * @brief 输入处理参数配置
 */
struct InputProcessorConfig {
    // 摇杆参数
    double joystick_max_output{8000.0}; // 摇杆最大输出速度
    double joystick_deadzone{0.05};     // 摇杆死区

    // 键盘参数
    double keyboard_speed_default{3000.0}; // 键盘默认速度
    double keyboard_speed_min{0.0};        // 键盘最小速度
    double keyboard_speed_max{8000.0};     // 键盘最大速度
    double keyboard_speed_step{6.0};       // 每次速度调整步长

    // 小陀螺参数
    double spin_speed_default{3000.0}; // 小陀螺默认速度
    double spin_speed_min{800.0};      // 小陀螺最小速度
    double spin_speed_max{6500.0};     // 小陀螺最大速度
    double spin_dial_gain{6.0};        // 拨轮调节增益
    double spin_key_gain{2.4};         // 键盘调节增益
    double spin_dial_deadzone{0.05};   // 拨轮死区

    // 鼠标参数
    double mouse_sensitivity{1.0}; // 鼠标灵敏度
    double mouse_yaw_gain{0.75};   // Yaw 增益
    double mouse_pitch_gain{1.0};  // Pitch 增益
    double mouse_limit{100.0};     // 鼠标偏移限幅
    double pitch_gain_coeff{0.00005 * (180.0 / M_PI)};
    double yaw_gain_coeff{10.0 * M_PI * 0.001 * 0.0025};

    // Pitch 限位
    double pitch_min_deg{-25.0};
    double pitch_max_deg{40.0};
};

/**
 * @brief 底盘速度输出
 */
struct ChassisVelocity {
    double vx{0.0};     // X 方向速度（前进）
    double vy{0.0};     // Y 方向速度（横移）
    double wz{0.0};     // 旋转角速度
    bool active{false}; // 是否有有效输入
};

/**
 * @brief 云台增量输出
 */
struct GimbalDelta {
    double pitch_delta{0.0};  // Pitch 增量（度）
    double yaw_delta{0.0};    // Yaw 增量（弧度）
    double target_pitch{0.0}; // 目标 Pitch（度）
    double target_yaw{0.0};   // 目标 Yaw（弧度）
};

/**
 * @brief 输入处理工具类
 */
class InputProcessor {
  public:
    explicit InputProcessor(const InputProcessorConfig &config = InputProcessorConfig{})
        : config_(config),
          current_spd_mode_(config.keyboard_speed_default),
          spin_spd_(config.spin_speed_default),
          target_pitch_deg_(0.0),
          target_yaw_rad_(0.0) {
    }

    /**
     * @brief 处理摇杆输入（带死区）
     * @param joystick_value 摇杆原始值（-1 到 1）
     * @return 处理后的速度值
     */
    double process_joystick(double joystick_value) const {
        if (std::abs(joystick_value) < config_.joystick_deadzone) {
            return 0.0;
        }
        return joystick_value * config_.joystick_max_output;
    }

    /**
     * @brief 计算底盘速度（摇杆 + 键盘组合）
     * @param joystick_x 摇杆 X 值
     * @param joystick_y 摇杆 Y 值
     * @param key_forward 前进键（W）
     * @param key_backward 后退键（S）
     * @param key_left 左移键（A）
     * @param key_right 右移键（D）
     * @return 底盘速度
     */
    ChassisVelocity compute_chassis_velocity(
        double joystick_x, double joystick_y,
        uint8_t key_forward, uint8_t key_backward,
        uint8_t key_left, uint8_t key_right) const {
        // 摇杆处理
        double vx_joystick = process_joystick(joystick_y);
        double vy_joystick = process_joystick(joystick_x);

        // 键盘处理
        double key_fb = static_cast<double>(key_forward) - static_cast<double>(key_backward);
        double key_lr = static_cast<double>(key_right) - static_cast<double>(key_left);
        double vx_keyboard = key_fb * current_spd_mode_;
        double vy_keyboard = key_lr * current_spd_mode_;

        // 组合
        ChassisVelocity vel;
        vel.vx = vx_joystick + vx_keyboard;
        vel.vy = vy_joystick + vy_keyboard;

        // 死区检测
        vel.active = (std::abs(vel.vx) > 100.0 || std::abs(vel.vy) > 100.0);

        if (!vel.active) {
            vel.vx = 0.0;
            vel.vy = 0.0;
        }

        return vel;
    }

    /**
     * @brief 更新键盘速度分档
     * @param shift_pressed Shift 键状态
     * @param ctrl_pressed Ctrl 键状态
     */
    void update_keyboard_speed(bool shift_pressed, bool ctrl_pressed) {
        if (shift_pressed && !ctrl_pressed) {
            current_spd_mode_ = std::min(config_.keyboard_speed_max, current_spd_mode_ + config_.keyboard_speed_step);
        } else if (ctrl_pressed && !shift_pressed) {
            current_spd_mode_ = std::max(config_.keyboard_speed_min, current_spd_mode_ - config_.keyboard_speed_step);
        }
    }

    /**
     * @brief 更新小陀螺速度（拨轮 + 键盘）
     * @param dial_value 拨轮值（-1 到 1）
     * @param shift_pressed Shift 键状态
     * @param ctrl_pressed Ctrl 键状态
     */
    void update_spin_speed(double dial_value, bool shift_pressed, bool ctrl_pressed) {
        // 拨轮调节（带死区）
        double dial_input = (std::abs(dial_value) < config_.spin_dial_deadzone) ? 0.0 : dial_value;
        spin_spd_ += -dial_input * config_.spin_dial_gain;

        // 键盘微调
        if (shift_pressed && !ctrl_pressed) {
            spin_spd_ += config_.spin_key_gain;
        } else if (ctrl_pressed && !shift_pressed) {
            spin_spd_ -= config_.spin_key_gain;
        }

        // 限幅
        spin_spd_ = clamp(spin_spd_, config_.spin_speed_min, config_.spin_speed_max);
    }

    /**
     * @brief 获取当前小陀螺速度
     */
    double get_spin_speed() const {
        return spin_spd_;
    }

    /**
     * @brief 重置小陀螺速度为默认值
     */
    void reset_spin_speed() {
        spin_spd_ = config_.spin_speed_default;
    }

    /**
     * @brief 计算鼠标云台控制
     * @param mouse_x 鼠标 X 轴移动量
     * @param mouse_y 鼠标 Y 轴移动量
     * @return 云台增量
     */
    GimbalDelta compute_gimbal_delta(double mouse_x, double mouse_y) {
        // 应用灵敏度
        double mx = mouse_x * config_.mouse_sensitivity;
        double my = mouse_y * config_.mouse_sensitivity;

        // 计算偏移量（带限幅）
        double left_right_offset = clamp(mx * config_.mouse_yaw_gain, -config_.mouse_limit, config_.mouse_limit);
        double top_down_offset = clamp(-my * config_.mouse_pitch_gain, -config_.mouse_limit, config_.mouse_limit);

        // Pitch 增量（度）
        double pitch_delta = top_down_offset * config_.pitch_gain_coeff;
        target_pitch_deg_ += pitch_delta;
        target_pitch_deg_ = clamp(target_pitch_deg_, config_.pitch_min_deg, config_.pitch_max_deg);

        // Yaw 增量（弧度）
        double yaw_delta = -left_right_offset * config_.yaw_gain_coeff;
        target_yaw_rad_ += yaw_delta;
        normalize_angle(target_yaw_rad_);

        GimbalDelta delta;
        delta.pitch_delta = pitch_delta;
        delta.yaw_delta = yaw_delta;
        delta.target_pitch = target_pitch_deg_;
        delta.target_yaw = target_yaw_rad_;

        return delta;
    }

    /**
     * @brief 重置云台目标位置
     */
    void reset_gimbal_target() {
        target_pitch_deg_ = 0.0;
        target_yaw_rad_ = 0.0;
    }

    /**
     * @brief 获取当前键盘速度比例
     */
    double get_speed_scale() const {
        return current_spd_mode_ / config_.keyboard_speed_max;
    }

    /**
     * @brief 获取当前键盘速度
     */
    double get_keyboard_speed() const {
        return current_spd_mode_;
    }

    /**
     * @brief 设置键盘速度
     */
    void set_keyboard_speed(double speed) {
        current_spd_mode_ = clamp(speed, config_.keyboard_speed_min, config_.keyboard_speed_max);
    }

    /**
     * @brief 更新配置
     */
    void update_config(const InputProcessorConfig &config) {
        config_ = config;
    }

    /**
     * @brief 获取配置
     */
    const InputProcessorConfig &get_config() const {
        return config_;
    }

    /**
     * @brief 通用限幅函数
     */
    static double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

  private:
    /**
     * @brief 归一化角度到 [-PI, PI]
     */
    static void normalize_angle(double &angle_rad) {
        while (angle_rad > M_PI)
            angle_rad -= 2.0 * M_PI;
        while (angle_rad < -M_PI)
            angle_rad += 2.0 * M_PI;
    }

    InputProcessorConfig config_;
    double current_spd_mode_;
    double spin_spd_;
    double target_pitch_deg_;
    double target_yaw_rad_;
};

} // namespace universal_controller
