/**
 * @file chassis_controller.cpp
 * @brief 底盘控制器实现
 *
 * 参考 infantry_controller/chassis_controller.py 和 sentry_controller/chassis_controller.py
 */

#include "universal_controller/controllers/chassis_controller.hpp"
#include <cmath>

namespace universal_controller
{

    ChassisController::ChassisController()
    {
        // 初始化运动学解算器
        kinematics_ = std::make_unique<SwerveKinematics>(0.372, 0.372, 8192);
    }

    void ChassisController::init(rclcpp::Node *node, const ConfigLoader &cfg)
    {
        set_node(node);

        // 加载配置
        config_.load(cfg);

        // 设置电机类型
        motor_type_ = config_.motor_type;

        // 初始化运动学
        uint16_t ecd_range = (motor_type_ == MotorType::DJI) ? 8192 : 65536;
        kinematics_ = std::make_unique<SwerveKinematics>(
            config_.wheel_track, config_.wheel_base, ecd_range);

        // 初始化 PID (LK 模式)
        if (motor_type_ == MotorType::LK)
        {
            for (int i = 0; i < 4; ++i)
            {
                steer_angle_pids_[i] = PID(
                    config_.steer_angle_pid.kp,
                    config_.steer_angle_pid.ki,
                    config_.steer_angle_pid.kd,
                    config_.steer_angle_pid.max_out,
                    config_.steer_angle_pid.max_iout);

                steer_speed_pids_[i] = PID(
                    config_.steer_speed_pid.kp,
                    config_.steer_speed_pid.ki,
                    config_.steer_speed_pid.kd,
                    config_.steer_speed_pid.max_out,
                    config_.steer_speed_pid.max_iout);

                drive_speed_pids_[i] = PID(
                    config_.drive_speed_pid.kp,
                    config_.drive_speed_pid.ki,
                    config_.drive_speed_pid.kd,
                    config_.drive_speed_pid.max_out,
                    config_.drive_speed_pid.max_iout);
            }
        }

        // 创建订阅者
        if (motor_type_ == MotorType::DJI)
        {
            sub_steer_dji_ = node->create_subscription<custom_msgs::msg::ReadDJIMotor>(
                config_.topic_steer_read, qos_best_effort_,
                std::bind(&ChassisController::cb_steer_dji, this, std::placeholders::_1));

            pub_drive_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
                config_.topic_drive_write, qos_best_effort_);
            pub_steer_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
                config_.topic_steer_write, qos_best_effort_);
        }
        else
        {
            sub_steer_lk_ = node->create_subscription<custom_msgs::msg::ReadLkMotorMulti>(
                config_.topic_steer_read, qos_best_effort_,
                std::bind(&ChassisController::cb_steer_lk, this, std::placeholders::_1));
            sub_drive_lk_ = node->create_subscription<custom_msgs::msg::ReadLkMotorMulti>(
                config_.topic_drive_read, qos_best_effort_,
                std::bind(&ChassisController::cb_drive_lk, this, std::placeholders::_1));

            pub_drive_lk_ = node->create_publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>(
                config_.topic_drive_write, qos_best_effort_);
            pub_steer_lk_ = node->create_publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>(
                config_.topic_steer_write, qos_best_effort_);
        }

        // Yaw 电机订阅
        sub_yaw_ = node->create_subscription<custom_msgs::msg::ReadLkMotor>(
            config_.topic_yaw_read, qos_best_effort_,
            std::bind(&ChassisController::cb_yaw, this, std::placeholders::_1));

        set_initialized(true);
        RCLCPP_INFO(node->get_logger(), "ChassisController initialized (motor type: %s)",
                    motor_type_ == MotorType::DJI ? "DJI" : "LK");
    }

    void ChassisController::set_command(const ChassisCommand &cmd)
    {
        command_ = cmd;
        command_valid_ = true;
    }

    void ChassisController::update(double dt)
    {
        if (!command_valid_)
        {
            return;
        }
        compute_control(dt);

        if (motor_type_ == MotorType::DJI)
        {
            publish_dji_commands();
        }
        else
        {
            publish_lk_commands();
        }
    }

    void ChassisController::stop()
    {
        if (motor_type_ == MotorType::DJI)
        {
            auto msg = custom_msgs::msg::WriteDJIMotor();
            pub_drive_dji_->publish(msg);
            pub_steer_dji_->publish(msg);
        }
        else
        {
            auto msg = custom_msgs::msg::WriteLkMotorBroadcastCurrentControl();
            pub_drive_lk_->publish(msg);
            pub_steer_lk_->publish(msg);
        }
    }

    void ChassisController::compute_control(double dt)
    {
        // 死区检测
        if (std::abs(command_.vx_gimbal) < 100.0 &&
            std::abs(command_.vy_gimbal) < 100.0 &&
            std::abs(command_.wz) < 100.0)
        {
            // 停止输出
            for (int i = 0; i < 4; ++i)
            {
                target_drive_speeds_[i] = 0.0;
                target_steer_ecds_[i] = current_steer_ecds_[i];
            }
            return;
        }

        // 1. 云台系 → 底盘系坐标转换
        double vx_c, vy_c;
        transform_to_chassis_frame(command_.vx_gimbal, command_.vy_gimbal, vx_c, vy_c);

        // 2. 小陀螺模式速度抑制
        double wz = command_.wz;
        if (command_.spin_mode && std::abs(command_.wz) > 10.0)
        {
            wz = compute_spin_suppression(vx_c, vy_c, command_.wz);
        }

        // 3. 运动学解算
        auto [drive_speeds, steer_ecds] = kinematics_->calculate_motion(
            vx_c, vy_c, wz,
            current_steer_ecds_, config_.ecd_zeros);

        target_drive_speeds_ = drive_speeds;
        target_steer_ecds_ = steer_ecds;
    }

    void ChassisController::transform_to_chassis_frame(double vx_g, double vy_g,
                                                       double &vx_c, double &vy_c) const
    {
        double theta = gimbal_yaw_angle_;
        vx_c = vx_g * std::cos(theta) + vy_g * std::sin(theta);
        vy_c = -vx_g * std::sin(theta) + vy_g * std::cos(theta);
    }

    double ChassisController::compute_spin_suppression(double vx_c, double vy_c, double wz_cmd) const
    {
        double current_speed = std::sqrt(vx_c * vx_c + vy_c * vy_c);
        double k = current_speed / 8000.0;

        // 限幅系数
        if (k > 0.4)
            k = 0.4;
        if (k < 0.2)
            k = 0.2;

        // 速度越快，自旋越慢
        return wz_cmd * (1.0 - k);
    }

    void ChassisController::publish_dji_commands()
    {
        // 舵向指令
        auto steer_msg = custom_msgs::msg::WriteDJIMotor();
        steer_msg.motor1_enable = 1;
        steer_msg.motor1_cmd = target_steer_ecds_[0];
        steer_msg.motor2_enable = 1;
        steer_msg.motor2_cmd = target_steer_ecds_[1];
        steer_msg.motor3_enable = 1;
        steer_msg.motor3_cmd = target_steer_ecds_[2];
        steer_msg.motor4_enable = 1;
        steer_msg.motor4_cmd = target_steer_ecds_[3];
        pub_steer_dji_->publish(steer_msg);

        // 驱动指令
        auto drive_msg = custom_msgs::msg::WriteDJIMotor();
        drive_msg.motor1_enable = 1;
        drive_msg.motor1_cmd = static_cast<int16_t>(target_drive_speeds_[0]);
        drive_msg.motor2_enable = 1;
        drive_msg.motor2_cmd = static_cast<int16_t>(target_drive_speeds_[1]);
        drive_msg.motor3_enable = 1;
        drive_msg.motor3_cmd = static_cast<int16_t>(target_drive_speeds_[2]);
        drive_msg.motor4_enable = 1;
        drive_msg.motor4_cmd = static_cast<int16_t>(target_drive_speeds_[3]);
        pub_drive_dji_->publish(drive_msg);
    }

    void ChassisController::publish_lk_commands()
    {
        // 舵向级联 PID
        auto steer_msg = custom_msgs::msg::WriteLkMotorBroadcastCurrentControl();
        std::array<int16_t, 4> steer_currents;

        for (int i = 0; i < 4; ++i)
        {
            int32_t err = static_cast<int32_t>(target_steer_ecds_[i]) - current_steer_ecds_[i];
            // 处理编码器跳变
            if (err > 32768)
                err -= 65536;
            if (err < -32768)
                err += 65536;

            double target_rpm = steer_angle_pids_[i].update(static_cast<double>(err));
            double current_out = steer_speed_pids_[i].update(
                target_rpm - current_steer_speeds_[i]);
            steer_currents[i] = static_cast<int16_t>(current_out);
        }

        steer_msg.motor1_cmd = steer_currents[0];
        steer_msg.motor2_cmd = steer_currents[1];
        steer_msg.motor3_cmd = steer_currents[2];
        steer_msg.motor4_cmd = steer_currents[3];
        pub_steer_lk_->publish(steer_msg);

        // 驱动速度环 PID
        auto drive_msg = custom_msgs::msg::WriteLkMotorBroadcastCurrentControl();
        std::array<int16_t, 4> drive_currents;

        for (int i = 0; i < 4; ++i)
        {
            double current_out = drive_speed_pids_[i].update(
                target_drive_speeds_[i] - current_drive_speeds_[i]);
            drive_currents[i] = static_cast<int16_t>(current_out);
        }

        drive_msg.motor1_cmd = drive_currents[0];
        drive_msg.motor2_cmd = drive_currents[1];
        drive_msg.motor3_cmd = drive_currents[2];
        drive_msg.motor4_cmd = drive_currents[3];
        pub_drive_lk_->publish(drive_msg);
    }

    void ChassisController::cb_steer_dji(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg)
    {
        // DJI 电机 ID 映射: motor1=FL, motor2=BR, motor3=BL, motor4=FR
        current_steer_ecds_[0] = msg->motor1_ecd;
        current_steer_ecds_[1] = msg->motor4_ecd;
        current_steer_ecds_[2] = msg->motor3_ecd;
        current_steer_ecds_[3] = msg->motor2_ecd;
    }

    void ChassisController::cb_steer_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg)
    {
        current_steer_ecds_[0] = msg->motor1_encoder;
        current_steer_ecds_[1] = msg->motor2_encoder;
        current_steer_ecds_[2] = msg->motor3_encoder;
        current_steer_ecds_[3] = msg->motor4_encoder;
        current_steer_speeds_[0] = msg->motor1_speed;
        current_steer_speeds_[1] = msg->motor2_speed;
        current_steer_speeds_[2] = msg->motor3_speed;
        current_steer_speeds_[3] = msg->motor4_speed;
    }

    void ChassisController::cb_drive_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg)
    {
        current_drive_speeds_[0] = msg->motor1_speed;
        current_drive_speeds_[1] = msg->motor2_speed;
        current_drive_speeds_[2] = msg->motor3_speed;
        current_drive_speeds_[3] = msg->motor4_speed;
    }

    void ChassisController::cb_yaw(const custom_msgs::msg::ReadLkMotor::SharedPtr msg)
    {
        int32_t relative_ecd = static_cast<int32_t>(msg->encoder) - yaw_center_ecd_;
        // 处理 16 位有符号溢出
        if (relative_ecd > 32767)
            relative_ecd -= 65536;
        if (relative_ecd < -32768)
            relative_ecd += 65536;

        gimbal_yaw_angle_ = (relative_ecd / 65535.0) * 2.0 * M_PI;
    }

} // namespace universal_controller
