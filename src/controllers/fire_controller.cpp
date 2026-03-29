/**
 * @file fire_controller.cpp
 * @brief 发射控制器实现
 *
 * 参考 infantry_controller/fire_controller.py
 */

#include "universal_controller/controllers/fire_controller.hpp"

namespace universal_controller
{

    // 一发弹丸对应的编码器增量 (36:8 减速比)
    static constexpr double ONE_BULLET_ECD = 36.0 * 8192.0 / 8.0;

    FireController::FireController() = default;

    void FireController::init(rclcpp::Node *node, const ConfigLoader &cfg)
    {
        set_node(node);

        // 加载配置
        config_.load(cfg);

        // 初始化拨盘 PID
        pid_trigger_pos_ = std::make_unique<PID>(
            config_.trigger_pos_pid.kp,
            config_.trigger_pos_pid.ki,
            config_.trigger_pos_pid.kd,
            config_.trigger_pos_pid.max_out,
            config_.trigger_pos_pid.max_iout);

        pid_trigger_spd_ = std::make_unique<PID>(
            config_.trigger_spd_pid.kp,
            config_.trigger_spd_pid.ki,
            config_.trigger_spd_pid.kd,
            config_.trigger_spd_pid.max_out,
            config_.trigger_spd_pid.max_iout);

        // 订阅拨盘电机反馈
        sub_motor_ = node->create_subscription<custom_msgs::msg::ReadDJIMotor>(
            config_.topic_fire_read, qos_best_effort_,
            std::bind(&FireController::cb_motor_feedback, this, std::placeholders::_1));

        // 发布摩擦轮+拨盘指令
        pub_motor_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
            config_.topic_fire_write, qos_best_effort_);

        friction_speed_target_ = config_.friction_speed_default;

        set_initialized(true);
        RCLCPP_INFO(node->get_logger(), "FireController initialized");
    }

    void FireController::set_command(const FireCommand &cmd)
    {
        command_ = cmd;
        command_valid_ = true;
    }

    void FireController::update_referee_constraints(const RefereeConstraints &constraints)
    {
        referee_ = constraints;
    }

    void FireController::update(double dt)
    {
        if (!config_.enabled || !command_valid_)
        {
            trigger_target_ecd_ = total_ecd_;
            trigger_has_fired_ = false;
            stop_all();
            return;
        }
        compute_control(dt);
        publish_commands();
    }

    void FireController::stop()
    {
        stop_all();
    }

    void FireController::cb_motor_feedback(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg)
    {
        motor3_current_ = msg->motor3_current;
        motor3_rpm_ = msg->motor3_rpm;
        double current_ecd = static_cast<double>(msg->motor3_ecd);

        if (!motor_initialized_)
        {
            motor3_ecd_ = current_ecd;
            total_ecd_ = current_ecd;
            trigger_target_ecd_ = current_ecd;
            motor_initialized_ = true;
        }
        else
        {
            // 计算最短路径差值
            double delta = current_ecd - motor3_ecd_;
            if (delta > 4096.0)
            {
                delta -= 8192.0;
            }
            else if (delta < -4096.0)
            {
                delta += 8192.0;
            }
            total_ecd_ += delta;
            motor3_ecd_ = current_ecd;
        }
    }

    void FireController::compute_control(double dt)
    {
        update_state_machine(dt);
    }

    void FireController::update_state_machine(double dt)
    {
        (void)dt;
        double now = node_->now().seconds();
        bool fire_locked = referee_.is_valid(now, config_.referee_timeout_s) && !referee_.fire_allowed;

        // 状态机转换
        switch (feeder_state_)
        {
        case FeederState::IDLE:
            if (command_.friction_on)
            {
                feeder_state_ = FeederState::LOADING;
                burst_mode_ = command_.burst_mode;
                // RCLCPP_INFO(node_->get_logger(), "Fire: LOADING");
            }
            break;

        case FeederState::LOADING:
            if (fire_locked || std::abs(motor3_current_) > config_.load_current_threshold)
            {
                feeder_state_ = FeederState::READY;
                // RCLCPP_INFO(node_->get_logger(), "Fire: READY");
            }
            else
            {
                trigger_target_ecd_ += config_.load_speed_ecd;
            }
            break;

        case FeederState::READY:
            if (!command_.friction_on)
            {
                feeder_state_ = FeederState::IDLE;
                trigger_target_ecd_ = total_ecd_;
            }
            else
            {
                bool should_fire = command_.trigger_fire && !fire_locked;

                if (should_fire)
                {
                    // 只有当上一发转得差不多才允许下一发
                    if (std::abs(trigger_target_ecd_ - total_ecd_) < 8192.0)
                    {
                        if (burst_mode_)
                        {
                            if ((now - last_shot_time_) > (config_.shot_period_ms / 1000.0))
                            {
                                trigger_target_ecd_ += ONE_BULLET_ECD;
                                last_shot_time_ = now;
                            }
                        }
                        else
                        {
                            if (!trigger_has_fired_)
                            {
                                trigger_target_ecd_ += ONE_BULLET_ECD;
                                trigger_has_fired_ = true;
                            }
                        }
                    }
                }
                else
                {
                    trigger_has_fired_ = false;
                }
            }
            break;
        }
    }

    void FireController::publish_commands()
    {
        auto msg = custom_msgs::msg::WriteDJIMotor();

        // 摩擦轮
        if (command_.friction_on)
        {
            double friction_cmd = friction_speed_target_;
            msg.motor1_enable = 1;
            msg.motor1_cmd = static_cast<int16_t>(friction_cmd);
            msg.motor2_enable = 1;
            msg.motor2_cmd = static_cast<int16_t>(-friction_cmd);
        }
        else
        {
            // 与 Python 版一致：停机时保持使能，仅下发 0 转速
            msg.motor1_enable = 1;
            msg.motor1_cmd = 0;
            msg.motor2_enable = 1;
            msg.motor2_cmd = 0;
        }

        // 拨盘
        double motor3_torque = 0.0;
        if (feeder_state_ != FeederState::IDLE)
        {
            double angle_error = trigger_target_ecd_ - total_ecd_;
            double target_rpm = pid_trigger_pos_->update(angle_error);
            double speed_error = target_rpm - motor3_rpm_;
            motor3_torque = pid_trigger_spd_->update(speed_error);
        }
        else
        {
            pid_trigger_pos_->reset();
            pid_trigger_spd_->reset();
        }

        msg.motor3_enable = (feeder_state_ != FeederState::IDLE) ? 1 : 0;
        msg.motor3_cmd = static_cast<int16_t>(motor3_torque);

        pub_motor_->publish(msg);
    }

    void FireController::stop_all()
    {
        auto msg = custom_msgs::msg::WriteDJIMotor();
        msg.motor1_enable = 1;
        msg.motor1_cmd = 0;
        msg.motor2_enable = 1;
        msg.motor2_cmd = 0;

        msg.motor3_enable = 0;
        msg.motor3_cmd = 0;
        
        pub_motor_->publish(msg);

        feeder_state_ = FeederState::IDLE;
    }

} // namespace universal_controller
