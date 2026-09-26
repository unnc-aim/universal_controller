/**
 * @file types.hpp
 * @brief 通用类型定义
 *
 * 定义控制模式枚举、电机类型、指令结构体等通用类型
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace universal_controller {

/**
 * @brief 控制模式枚举
 */
enum class ControlMode : uint8_t {
    EMERGENCY_STOP = 0, ///< 急停模式
    MANUAL = 1,         ///< 手动遥控模式
    AUTOAIM = 2,        ///< 自瞄模式
    NAVIGATION = 3,     ///< 导航模式
    BEHAVIOR_TREE = 4   ///< 行为树模式
};

/**
 * @brief 控制模式转字符串
 */
inline std::string control_mode_to_string(ControlMode mode) {
    switch (mode) {
        case ControlMode::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case ControlMode::MANUAL:
            return "MANUAL";
        case ControlMode::AUTOAIM:
            return "AUTOAIM";
        case ControlMode::NAVIGATION:
            return "NAVIGATION";
        case ControlMode::BEHAVIOR_TREE:
            return "BEHAVIOR_TREE";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 电机类型枚举
 */
enum class MotorType : uint8_t {
    DJI = 0, ///< 大疆电机（位置模式）
    LK = 1   ///< 雷赛电机（电流/力矩模式）
};

/**
 * @brief 电机类型转字符串
 */
inline std::string motor_type_to_string(MotorType type) {
    return type == MotorType::DJI ? "DJI" : "LK";
}

/**
 * @brief 输入源枚举
 */
enum class InputSource : uint8_t {
    RC = 0,           ///< 遥控器
    KEYBOARD = 1,     ///< 键盘
    NAVIGATION = 2,   ///< 导航
    BEHAVIOR_TREE = 3 ///< 行为树
};

/**
 * @brief 子系统输入源枚举
 *
 * 每个子系统（底盘、云台、发射）独立追踪由哪个输入源控制。
 */
enum class SubsystemInput : uint8_t {
    NONE = 0,      ///< 无输入（控制器应保持安全状态）
    RC = 1,        ///< 遥控器输入
    AUTOAIM = 2,   ///< 自瞄系统输入
    NAVIGATION = 3, ///< 导航速度输入（/cmd_vel）
    SCAN = 4       ///< 云台扫描输入（行为树速度扫描）
};

/**
 * @brief SubsystemInput 转字符串
 */
inline std::string subsystem_input_to_string(SubsystemInput input) {
    switch (input) {
        case SubsystemInput::NONE:
            return "NONE";
        case SubsystemInput::RC:
            return "RC";
        case SubsystemInput::AUTOAIM:
            return "AUTOAIM";
        case SubsystemInput::NAVIGATION:
            return "NAVIGATION";
        case SubsystemInput::SCAN:
            return "SCAN";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 仲裁结果结构体
 *
 * 每个子系统独立追踪输入源，而非使用全局单一模式。
 * emergency_stop 是全局覆盖：若为 true，所有控制器立即停止。
 */
struct ArbitrationResult {
    bool emergency_stop{true};
    SubsystemInput chassis{SubsystemInput::NONE};
    SubsystemInput gimbal{SubsystemInput::NONE};
    SubsystemInput fire{SubsystemInput::NONE};

    std::string to_string() const {
        return std::string("Arbitration{estop=") +
               (emergency_stop ? "Y" : "N") +
               ", chassis=" + subsystem_input_to_string(chassis) +
               ", gimbal=" + subsystem_input_to_string(gimbal) +
               ", fire=" + subsystem_input_to_string(fire) + "}";
    }
};

/**
 * @brief 底盘指令结构体
 */
struct ChassisCommand {
    double vx_gimbal{0.0};  ///< X方向速度（云台系）
    double vy_gimbal{0.0};  ///< Y方向速度（云台系）
    double wz{0.0};         ///< 旋转角速度
    bool spin_mode{false};  ///< 小陀螺模式
    double spin_speed{0.0}; ///< 小陀螺速度

    ChassisCommand() = default;
    ChassisCommand(double vx, double vy, double w, bool spin = false, double spin_spd = 0.0)
        : vx_gimbal(vx),
          vy_gimbal(vy),
          wz(w),
          spin_mode(spin),
          spin_speed(spin_spd) {
    }
};

/**
 * @brief 云台指令结构体
 */
struct GimbalCommand {
    double pitch_deg{0.0};   ///< 目标 Pitch 角度（度）
    double yaw_rad{0.0};     ///< 目标 Yaw 角度（弧度）
    bool from_action{false}; ///< 是否来自 Action Server
    bool absolute{true};     ///< true=绝对角度, false=增量

    // 导航平移速度已转换为实际云台坐标系方向。
    bool follow_navigation{false};
    double navigation_vx{0.0};
    double navigation_vy{0.0};
    int64_t navigation_stamp_ns{0};

    // 扫描模式（速度扫描，由行为树驱动）
    bool scan_mode{false};       ///< 是否为扫描模式
    double scan_vel_yaw{0.0};    ///< Yaw 扫描速度 (rad/s)
    double scan_vel_pitch{0.0};  ///< Pitch 扫描速度 (rad/s)
    double scan_yaw_min{-M_PI};  ///< Yaw 扫描下限 (rad)
    double scan_yaw_max{M_PI};   ///< Yaw 扫描上限 (rad)
    double scan_pitch_min{0.0};  ///< Pitch 扫描下限 (rad)
    double scan_pitch_max{0.0};  ///< Pitch 扫描上限 (rad)

    GimbalCommand() = default;
    GimbalCommand(double pitch, double yaw, bool action = false, bool abs = true)
        : pitch_deg(pitch),
          yaw_rad(yaw),
          from_action(action),
          absolute(abs) {
    }
};

/**
 * @brief 发射指令结构体
 */
struct FireCommand {
    bool friction_on{false};       ///< 摩擦轮开关
    bool trigger_fire{false};      ///< 发射触发
    bool burst_mode{false};        ///< 连发模式
    double friction_speed{6500.0}; ///< 摩擦轮速度
    bool from_action{false};       ///< 是否来自 Action Server

    FireCommand() = default;
    FireCommand(bool friction, bool trigger, bool burst = false, double speed = 6500.0, bool action = false)
        : friction_on(friction),
          trigger_fire(trigger),
          burst_mode(burst),
          friction_speed(speed),
          from_action(action) {
    }
};

/**
 * @brief 发射器状态枚举
 */
enum class FeederState : uint8_t {
    IDLE = 0,    ///< 空闲状态
    LOADING = 1, ///< 装填状态
    READY = 2    ///< 就绪状态
};

/**
 * @brief 发射器状态转字符串
 */
inline std::string feeder_state_to_string(FeederState state) {
    switch (state) {
        case FeederState::IDLE:
            return "IDLE";
        case FeederState::LOADING:
            return "LOADING";
        case FeederState::READY:
            return "READY";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 裁判系统约束数据
 */
struct RefereeConstraints {
    double heat{0.0};        ///< 当前热量
    double heat_limit{0.0};  ///< 热量上限
    double power{0.0};       ///< 当前功率
    double power_limit{0.0}; ///< 功率上限
    bool fire_allowed{true}; ///< 是否允许发射
    double speed_scale{1.0}; ///< 速度缩放因子
    double timestamp{0.0};   ///< 时间戳

    bool is_valid(double now, double timeout = 0.5) const {
        return (now - timestamp) < timeout;
    }
};

/**
 * @brief IMU 数据
 */
struct IMUData {
    double pitch_rad{0.0}; ///< Pitch 角度（弧度）
    double yaw_rad{0.0};   ///< Yaw 角度（弧度）
    double gyro_z{0.0};    ///< Z 轴角速度（rad/s）
    double timestamp{0.0}; ///< 时间戳
};

/**
 * @brief 四元数
 */
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/**
 * @brief 从四元数提取 Yaw 角
 */
inline double quaternion_to_yaw(const Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief 从四元数提取 Pitch 角
 */
inline double quaternion_to_pitch(const Quaternion &q) {
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0) {
        return std::copysign(M_PI / 2.0, sinp);
    }
    return std::asin(sinp);
}

/**
 * @brief 角度归一化到 [-PI, PI]
 */
inline double normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 数值限幅
 */
template <typename T>
inline T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(max_val, value));
}

/**
 * @brief 编码器差值计算（考虑 0-8191 循环）
 */
inline int16_t ecd_diff(uint16_t current, uint16_t target, uint16_t range = 8192) {
    int32_t diff = static_cast<int32_t>(target) - static_cast<int32_t>(current);
    if (diff > static_cast<int32_t>(range / 2)) {
        diff -= range;
    } else if (diff < -static_cast<int32_t>(range / 2)) {
        diff += range;
    }
    return static_cast<int16_t>(diff);
}

/**
 * @brief 统一键鼠输入（由各 interpreter 的 mapper 填充）
 */
struct KeyboardMouseInput {
    // 键盘 16 键
    bool key_w{false}, key_s{false}, key_a{false}, key_d{false};
    bool key_q{false}, key_e{false}, key_r{false}, key_f{false};
    bool key_g{false}, key_z{false}, key_x{false}, key_c{false};
    bool key_v{false}, key_b{false};
    bool key_shift{false}, key_ctrl{false};

    // 鼠标
    double mouse_x{0.0}, mouse_y{0.0};
    double mouse_wheel{0.0};
    bool mouse_left{false}, mouse_right{false}, mouse_middle{false};
};

/**
 * @brief 键鼠解析输出
 */
struct KeyboardMouseOutput {
    // ========== 速度分档 ==========
    double keyboard_speed{0.0}; ///< 当前键盘速度
    double speed_scale{0.0};    ///< 速度比例 (0-1)

    // ========== 键盘方向（WASD） ==========
    double key_vx{0.0}; ///< 键盘前后方向速度分量
    double key_vy{0.0}; ///< 键盘左右方向速度分量

    // ========== 鼠标云台增量（统一弧度） ==========
    double pitch_delta{0.0}; ///< 鼠标 Pitch 增量（弧度）
    double yaw_delta{0.0};   ///< 鼠标 Yaw 增量（弧度）

    // ========== 小陀螺 ==========
    bool spin_mode{false};  ///< 小陀螺模式开关
    double spin_speed{0.0}; ///< 当前小陀螺速度

    // ========== 鼠标按键 ==========
    bool mouse_autoaim{false}; ///< 鼠标右键自瞄
    bool mouse_fire{false};    ///< 鼠标左键发射
    bool mouse_middle{false};  ///< 鼠标中键

    // ========== 鼠标滚轮 ==========
    double mouse_wheel{0.0}; ///< 鼠标滚轮值
};

} // namespace universal_controller
