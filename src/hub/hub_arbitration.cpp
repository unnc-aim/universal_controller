/**
 * @file hub_arbitration.cpp
 * @brief Hub 模式仲裁与指令分发
 */

#include "universal_controller/hub/hub.hpp"
#include <cmath>

namespace {

/** 云台系线速度 (m/s) → 底盘系，与 ChassisController::transform_to_chassis_frame 一致 */
void gimbal_vel_to_chassis_mps(double vx_g, double vy_g, double theta, double &vx_c, double &vy_c) {
    vx_c = vx_g * std::cos(theta) + vy_g * std::sin(theta);
    vy_c = -vx_g * std::sin(theta) + vy_g * std::cos(theta);
}

} // namespace

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
        bool follow_ready = false;
        bool moving = false;
        if (gimbal_config_.follow_navigation && result.chassis == SubsystemInput::NAVIGATION) {
            const auto stamp = rclcpp::Time(nav_cmd_vel_->header.stamp);
            const double age = (this->now() - stamp).seconds();
            const double speed = std::hypot(nav_cmd_vel_->twist.linear.x, nav_cmd_vel_->twist.linear.y);
            follow_ready = nav_cmd_vel_->header.frame_id == gimbal_config_.navigation_frame &&
                           stamp.nanoseconds() > 0 && age >= -0.05 &&
                           age < gimbal_config_.follow_input_timeout_s && std::isfinite(speed);
            moving = follow_ready && speed >= gimbal_config_.follow_min_speed;
        }
        const bool scan_valid = is_gimbal_scan_valid();
        // 新许可只作用于行进中的云台；显式扫描及发射条件继续沿用原有处理。
        const bool navigation_aim_allowed = gimbal_config_.navigation_autoaim_takeover &&
            moving && !scan_valid && autoaim_cmd_ &&
            autoaim_last_time_ > 0.0 && this->now().seconds() >= autoaim_last_time_ &&
            std::isfinite(autoaim_cmd_->yaw) && std::isfinite(autoaim_cmd_->pitch);
        if ((nav_fire_allowed_ || navigation_aim_allowed) && is_autoaim_valid_nav()) {
            result.gimbal = SubsystemInput::AUTOAIM;
        } else if (scan_valid) {
            result.gimbal = SubsystemInput::SCAN;
        } else if (follow_ready && (moving || arbitration_.gimbal == SubsystemInput::NAVIGATION ||
                                   arbitration_.gimbal == SubsystemInput::AUTOAIM)) {
            result.gimbal = SubsystemInput::NAVIGATION;
        } else {
            result.gimbal = SubsystemInput::SCAN;
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
    // 导航侧自瞄：尊重 RC autoaim_enabled（左拨杆），与 full_auto / 拨弹门控一致
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

void Hub::dispatch_commands() {
    if (arbitration_.emergency_stop) {
        chassis_controllers::msg::ChassisControl estop_msg;
        estop_msg.emergency_stop = 1;
        estop_msg.power_limit = 100;
        pub_chassis_command_->publish(estop_msg);

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
    bool ext_spin = (std::abs(nav_spin_speed_) > 1e-3f);

    if (!game_started_) {
        ext_spin = false;
    }
    if (arbitration_.chassis == SubsystemInput::NAVIGATION) {
        ext_spin = false;
    }

    chassis_controllers::msg::ChassisControl msg;
    msg.emergency_stop = 0;
    msg.power_limit = 100;

    switch (arbitration_.chassis) {
        case SubsystemInput::RC: {
            const double vx_g = unified_input_->vx / chassis_cmd_k_linear_;
            const double vy_g = unified_input_->vy / chassis_cmd_k_linear_;
            double theta = chassis_->gimbal_yaw_angle();
            if (ext_spin || unified_input_->spin_mode) {
                // ext_spin：行为树 /cmd_spin；否则 RC 小陀螺用 unified wz（与 spin_speed 同量纲）
                const double wz_enc =
                    ext_spin ? static_cast<double>(nav_spin_speed_) : unified_input_->wz;
                theta += chassis_config_.spin_compensation_k * wz_enc;
            }
            double vx_c = 0.0;
            double vy_c = 0.0;
            gimbal_vel_to_chassis_mps(vx_g, vy_g, theta, vx_c, vy_c);
            msg.x_speed = static_cast<float>(vx_c);
            msg.y_speed = static_cast<float>(vy_c);
            if (ext_spin || unified_input_->spin_mode) {
                const double spd =
                    ext_spin ? static_cast<double>(nav_spin_speed_) : unified_input_->wz;
                msg.spin_speed = static_cast<float>(spd / chassis_cmd_k_spin_);
            } else {
                msg.spin_speed = 0.0f;
            }
            break;
        }
        case SubsystemInput::NAVIGATION: {
            const double vx_g = -nav_cmd_vel_->twist.linear.x;
            const double vy_g = -nav_cmd_vel_->twist.linear.y;
            const double theta = chassis_->gimbal_yaw_angle();
            double vx_c = 0.0;
            double vy_c = 0.0;
            gimbal_vel_to_chassis_mps(vx_g, vy_g, theta, vx_c, vy_c);
            msg.x_speed = static_cast<float>(vx_c);
            msg.y_speed = static_cast<float>(vy_c);
            msg.spin_speed = static_cast<float>(nav_cmd_vel_->twist.angular.z);
            break;
        }
        default:
            break;
    }

    pub_chassis_command_->publish(msg);
}

void Hub::dispatch_gimbal() {
    switch (arbitration_.gimbal) {
        case SubsystemInput::NAVIGATION: {
            GimbalCommand cmd;
            cmd.follow_navigation = true;
            // 与 dispatch_chassis 的速度方向转换一致。
            cmd.navigation_vx = -nav_cmd_vel_->twist.linear.x;
            cmd.navigation_vy = -nav_cmd_vel_->twist.linear.y;
            cmd.navigation_stamp_ns = rclcpp::Time(nav_cmd_vel_->header.stamp).nanoseconds();
            gimbal_->set_command(cmd);
            break;
        }
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
            if (!is_gimbal_scan_valid() || !gimbal_scan_cmd_) {
                // In autonomous mode, if scan command is temporarily missing,
                // hold current gimbal pose instead of keeping stale scan state.
                GimbalCommand cmd;
                cmd.scan_mode = false;
                cmd.absolute = false;
                cmd.pitch_deg = 0.0;
                cmd.yaw_rad = 0.0;
                gimbal_->set_command(cmd);
                break;
            }
            GimbalCommand cmd;
            cmd.scan_mode = true;
            cmd.scan_vel_yaw = static_cast<double>(gimbal_scan_cmd_->velocity.yaw);
            cmd.scan_vel_pitch = static_cast<double>(gimbal_scan_cmd_->velocity.pitch);
            cmd.scan_yaw_min = static_cast<double>(gimbal_scan_cmd_->velocity.yaw_min_range);
            cmd.scan_yaw_max = static_cast<double>(gimbal_scan_cmd_->velocity.yaw_max_range);
            cmd.scan_pitch_min = static_cast<double>(gimbal_scan_cmd_->velocity.pitch_min_range);
            cmd.scan_pitch_max = static_cast<double>(gimbal_scan_cmd_->velocity.pitch_max_range);
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
            cmd.burst_mode = unified_input_->burst_mode;
            cmd.friction_speed = unified_input_->friction_speed;

            bool full_auto = unified_input_->navigation_enabled
                          && unified_input_->autoaim_enabled;
            if (full_auto) {
                // 全自主模式（左上+右中/上）：拨弹三重门控
                //   nav_fire_allowed_  — auto_aim_switch 到达目标后由 BT 发布
                //   is_autoaim_valid_nav() — 自瞄确实锁敌
                bool aim_locked = nav_fire_allowed_ && is_autoaim_valid_nav();
                cmd.trigger_fire = unified_input_->fire_trigger && aim_locked;
            } else {
                // 手动 / 半自动模式：右拨杆直接控制
                cmd.trigger_fire = unified_input_->fire_trigger;
            }
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
