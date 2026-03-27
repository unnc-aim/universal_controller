/**
 * @file hub_arbitration.cpp
 * @brief Hub 模式仲裁与指令分发
 */

#include "universal_controller/hub/hub.hpp"
#include <cmath>

namespace universal_controller
{

    ArbitrationResult Hub::arbitrate()
    {
        ArbitrationResult result;

        // 全局急停检查
        if (!unified_input_ ||
            unified_input_->emergency_stop ||
            !unified_input_->connected)
        {
            result.emergency_stop = true;
            return result;
        }
        result.emergency_stop = false;

        // --- 底盘：Action > RC ---
        result.chassis = chassis_action_active_
                             ? SubsystemInput::ACTION
                             : SubsystemInput::RC;

        // --- 云台：Action > Autoaim > RC ---
        if (gimbal_action_active_)
        {
            result.gimbal = SubsystemInput::ACTION;
        }
        else if (is_autoaim_valid())
        {
            result.gimbal = SubsystemInput::AUTOAIM;
        }
        else
        {
            result.gimbal = SubsystemInput::RC;
        }

        // --- 发射：Action > RC ---
        result.fire = fire_action_active_
                          ? SubsystemInput::ACTION
                          : SubsystemInput::RC;

        return result;
    }

    bool Hub::is_autoaim_valid() const
    {
        if (!unified_input_ || !unified_input_->autoaim_enabled)
        {
            return false;
        }
        if (!autoaim_cmd_ || !autoaim_cmd_->control)
        {
            return false;
        }
        double autoaim_timeout = 0.2;
        bool autoaim_fresh = (this->now().seconds() - autoaim_last_time_) < autoaim_timeout;
        return autoaim_fresh;
    }

    void Hub::dispatch_commands()
    {
        if (!unified_input_)
            return;

        if (arbitration_.emergency_stop)
        {
            chassis_->stop();
            gimbal_->stop();
            fire_->stop();
            return;
        }

        dispatch_chassis();
        dispatch_gimbal();
        dispatch_fire();
    }

    void Hub::dispatch_chassis()
    {
        switch (arbitration_.chassis)
        {
        case SubsystemInput::RC:
        {
            ChassisCommand cmd;
            cmd.vx_gimbal = unified_input_->vx;
            cmd.vy_gimbal = unified_input_->vy;
            cmd.wz = unified_input_->wz;
            cmd.spin_mode = unified_input_->spin_mode;
            cmd.spin_speed = unified_input_->spin_speed;
            chassis_->set_command(cmd);
            break;
        }
        case SubsystemInput::ACTION:
            // Action Server 已直接调用 set_command()，不覆盖
            break;
        default:
            break;
        }
    }

    void Hub::dispatch_gimbal()
    {
        switch (arbitration_.gimbal)
        {
        case SubsystemInput::RC:
        {
            GimbalCommand cmd;
            cmd.pitch_deg = unified_input_->pitch_delta;
            cmd.yaw_rad = unified_input_->yaw_delta;
            cmd.autoaim_enabled = false;
            cmd.from_action = false;
            cmd.absolute = false;
            gimbal_->set_command(cmd);
            break;
        }
        case SubsystemInput::AUTOAIM:
        {
            GimbalCommand cmd;
            cmd.yaw_rad = autoaim_cmd_->yaw;
            cmd.pitch_deg = -autoaim_cmd_->pitch * (180.0 / M_PI);
            cmd.autoaim_enabled = true;
            cmd.from_action = false;
            cmd.absolute = true;
            gimbal_->set_command(cmd);
            break;
        }
        case SubsystemInput::ACTION:
            // Action Server 已直接调用 set_command()，不覆盖
            break;
        default:
            break;
        }
    }

    void Hub::dispatch_fire()
    {
        switch (arbitration_.fire)
        {
        case SubsystemInput::RC:
        {
            FireCommand cmd;
            cmd.friction_on = unified_input_->friction_on;
            cmd.trigger_fire = unified_input_->fire_trigger;
            cmd.burst_mode = unified_input_->burst_mode;
            cmd.friction_speed = unified_input_->friction_speed;
            cmd.from_action = false;
            fire_->set_command(cmd);
            break;
        }
        case SubsystemInput::ACTION:
            // Action Server 已直接调用 set_command()，不覆盖
            break;
        default:
            break;
        }
    }

} // namespace universal_controller
