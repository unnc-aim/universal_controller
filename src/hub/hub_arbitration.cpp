/**
 * @file hub_arbitration.cpp
 * @brief Hub 模式仲裁与指令分发
 */

#include "universal_controller/hub/hub.hpp"
#include <cmath>

namespace universal_controller {

bool Hub::is_nav_vel_valid() const {
    if (!nav_cmd_vel_) {
        return false;
    }
    if (nav_vel_last_time_.nanoseconds() == 0) {
        return false;
    }
    return (this->now() - nav_vel_last_time_).seconds() < nav_vel_timeout_s_;
}

bool Hub::is_gimbal_scan_valid() const {
    if (!gimbal_scan_cmd_) {
        return false;
    }
    if (gimbal_scan_last_time_.nanoseconds() == 0) {
        return false;
    }
    return (this->now() - gimbal_scan_last_time_).seconds() < gimbal_scan_timeout_s_;
}

ArbitrationResult Hub::arbitrate() {
    ArbitrationResult result;

    // 全局急停检查
    if (!unified_input_ ||
        unified_input_->emergency_stop ||
        !unified_input_->connected) {
        result.emergency_stop = true;
        return result;
    }
    result.emergency_stop = false;

    // --- 底盘：Navigation > RC ---
    if (unified_input_->navigation_enabled && is_nav_vel_valid()) {
        result.chassis = SubsystemInput::NAVIGATION;
    } else {
        result.chassis = SubsystemInput::RC;
    }

    // --- 云台仲裁 ---
    if (unified_input_->navigation_enabled) {
        // 导航模式：自瞄 > 扫描（不允许 RC 控制）
        if (is_autoaim_valid_nav()) {
            result.gimbal = SubsystemInput::AUTOAIM;
        } else if (is_gimbal_scan_valid()) {
            result.gimbal = SubsystemInput::SCAN;
        } else {
            result.gimbal = SubsystemInput::SCAN; // 默认扫描
        }
    } else {
        // 非导航模式：自瞄 > RC
        if (is_autoaim_valid()) {
            result.gimbal = SubsystemInput::AUTOAIM;
        } else {
            result.gimbal = SubsystemInput::RC;
        }
    }

    // --- 发射：RC ---
    result.fire = SubsystemInput::RC;

    return result;
}

bool Hub::is_autoaim_valid() const {
    if (!unified_input_ || !unified_input_->autoaim_enabled) {
        return false;
    }
    if (!autoaim_cmd_ || !autoaim_cmd_->control) {
        return false;
    }
    double autoaim_timeout = 0.2;
    bool autoaim_fresh = (this->now().seconds() - autoaim_last_time_) < autoaim_timeout;
    return autoaim_fresh;
}

bool Hub::is_autoaim_valid_nav() const {
    // 导航模式下的自瞄有效性：不需要 RC 的 autoaim_enabled 标志
    // 只要自瞄指令有效即自动启用
    if (!autoaim_cmd_ || !autoaim_cmd_->control) {
        return false;
    }
    double autoaim_timeout = 0.2;
    bool autoaim_fresh = (this->now().seconds() - autoaim_last_time_) < autoaim_timeout;
    return autoaim_fresh;
}

void Hub::dispatch_commands() {
    if (arbitration_.emergency_stop) {
        chassis_->stop();
        gimbal_->stop();
        fire_->stop();
        return;
    }

    if (!unified_input_)
        return;

    dispatch_chassis();
    dispatch_gimbal();
    dispatch_fire();
}

void Hub::dispatch_chassis() {
    // 传递裁判系统功率上限
    if (referee_.power_limit > 0.0) {
        chassis_->set_power_limit(referee_.power_limit);
    }

    switch (arbitration_.chassis) {
        case SubsystemInput::RC: {
            ChassisCommand cmd;
            cmd.vx_gimbal = unified_input_->vx;
            cmd.vy_gimbal = unified_input_->vy;
            cmd.wz = unified_input_->wz;
            cmd.spin_mode = unified_input_->spin_mode;
            cmd.spin_speed = unified_input_->spin_speed;
            chassis_->set_command(cmd);
            break;
        }
        case SubsystemInput::NAVIGATION: {
            // 使用导航速度指令（TwistStamped）
            ChassisCommand cmd;
            cmd.vx_gimbal = nav_cmd_vel_->twist.linear.x * 1348;
            cmd.vy_gimbal = nav_cmd_vel_->twist.linear.y * -1348;
            cmd.wz = nav_cmd_vel_->twist.angular.z;
            cmd.spin_mode = (std::abs(nav_spin_speed_) > 1e-3f);
            cmd.spin_speed = static_cast<double>(nav_spin_speed_);
            chassis_->set_command(cmd);
            break;
        }
        default:
            break;
    }
}

void Hub::dispatch_gimbal() {
    switch (arbitration_.gimbal) {
        case SubsystemInput::RC: {
            GimbalCommand cmd;
            cmd.pitch_deg = unified_input_->pitch_delta;
            cmd.yaw_rad = unified_input_->yaw_delta;
            cmd.absolute = false;
            gimbal_->set_command(cmd);
            break;
        }
        case SubsystemInput::AUTOAIM: {
            GimbalCommand cmd;
            cmd.yaw_rad = autoaim_cmd_->yaw;
            cmd.pitch_deg = -autoaim_cmd_->pitch * (180.0 / M_PI);
            cmd.absolute = true;
            gimbal_->set_command(cmd);
            break;
        }
        case SubsystemInput::SCAN: {
            GimbalCommand cmd;
            cmd.scan_mode = true;
            if (is_gimbal_scan_valid() && gimbal_scan_cmd_) {
                cmd.scan_vel_yaw = static_cast<double>(gimbal_scan_cmd_->velocity.yaw);
                cmd.scan_vel_pitch = static_cast<double>(gimbal_scan_cmd_->velocity.pitch);
                cmd.scan_yaw_min = static_cast<double>(gimbal_scan_cmd_->velocity.yaw_min_range);
                cmd.scan_yaw_max = static_cast<double>(gimbal_scan_cmd_->velocity.yaw_max_range);
                cmd.scan_pitch_min = static_cast<double>(gimbal_scan_cmd_->velocity.pitch_min_range);
                cmd.scan_pitch_max = static_cast<double>(gimbal_scan_cmd_->velocity.pitch_max_range);
            } else {
                // 默认扫描参数
                cmd.scan_vel_yaw = 1.0;
                cmd.scan_vel_pitch = 0.0;
                cmd.scan_yaw_min = -M_PI;
                cmd.scan_yaw_max = 0.0;
            }
            gimbal_->set_command(cmd);
            break;
        }
        default:
            break;
    }
}

void Hub::dispatch_fire() {
    switch (arbitration_.fire) {
        case SubsystemInput::RC: {
            FireCommand cmd;
            cmd.friction_on = unified_input_->friction_on;
            cmd.trigger_fire = unified_input_->fire_trigger;
            cmd.burst_mode = unified_input_->burst_mode;
            cmd.friction_speed = unified_input_->friction_speed;
            fire_->set_command(cmd);
            break;
        }
        default:
            break;
    }
}

void Hub::publish_auto_aim_switch() {
    // 导航模式下由行为树（PublishAutoAim）全权管理 auto_aim_switch，
    // Hub 不再发布，避免双发布者状态冲突。
    if (unified_input_ && unified_input_->navigation_enabled) {
        return;
    }

    std_msgs::msg::Int32 msg;
    if (arbitration_.emergency_stop) {
        msg.data = 0;
    } else if (unified_input_ && unified_input_->autoaim_enabled) {
        // 非导航模式：跟随 RC 自瞄开关
        msg.data = 1;
    } else {
        msg.data = 0;
    }
    pub_auto_aim_switch_->publish(msg);
}

} // namespace universal_controller
