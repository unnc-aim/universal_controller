/**
 * @file gimbal_controller.cpp
 * @brief 云台控制器实现
 *
 * 参考 infantry_controller/gimbal_controller.py
 */

#include "universal_controller/controllers/gimbal_controller.hpp"
#include <cmath>

namespace universal_controller
{

    GimbalController::GimbalController() = default;

    void GimbalController::init(rclcpp::Node *node, const ConfigLoader &cfg)
    {
        set_node(node);

        // 加载配置
        config_.load(cfg);

        // 初始化 Yaw 级联 PID
        pid_yaw_pos_ = std::make_unique<PID>(
            config_.yaw_pos_pid.kp,
            config_.yaw_pos_pid.ki,
            config_.yaw_pos_pid.kd,
            config_.yaw_pos_pid.max_out,
            config_.yaw_pos_pid.max_iout);

        pid_yaw_spd_ = std::make_unique<PID>(
            config_.yaw_spd_pid.kp,
            config_.yaw_spd_pid.ki,
            config_.yaw_spd_pid.kd,
            config_.yaw_spd_pid.max_out,
            config_.yaw_spd_pid.max_iout);

        // 订阅 IMU
        sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
            config_.topic_imu_read, qos_best_effort_,
            std::bind(&GimbalController::cb_imu, this, std::placeholders::_1));

        // 订阅 Pitch 电机反馈
        sub_pitch_ = node->create_subscription<custom_msgs::msg::ReadDJIMotor>(
            config_.topic_pitch_read, qos_best_effort_,
            std::bind(&GimbalController::cb_pitch_feedback, this, std::placeholders::_1));

        // 订阅 Yaw 电机反馈
        sub_yaw_ = node->create_subscription<custom_msgs::msg::ReadLkMotor>(
            config_.topic_yaw_read, qos_best_effort_,
            std::bind(&GimbalController::cb_yaw_feedback, this, std::placeholders::_1));

        // 订阅自瞄指令
        sub_autoaim_ = node->create_subscription<sp_msgs::msg::AutoAimCommandMsg>(
            config_.topic_autoaim_cmd, qos_best_effort_,
            std::bind(&GimbalController::cb_autoaim, this, std::placeholders::_1));

        // 发布 Pitch 指令
        pub_pitch_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
            config_.topic_pitch_write, qos_best_effort_);

        // 发布 Yaw 指令
        pub_yaw_ = node->create_publisher<custom_msgs::msg::WriteLkMotorTorqueControl>(
            config_.topic_yaw_write, qos_best_effort_);

        set_initialized(true);
        RCLCPP_INFO(node->get_logger(), "GimbalController initialized @ 1000Hz");
    }

    void GimbalController::set_command(const GimbalCommand &cmd)
    {
        command_ = cmd;
        command_valid_ = true;
    }

    void GimbalController::update(double dt)
    {
        if (!command_valid_)
        {
            return;
        }

        // 计算控制
        compute_pitch_control();
        compute_yaw_control();

        // 发布指令
        publish_commands();
    }

    void GimbalController::stop()
    {
        stop_all();
    }

    void GimbalController::cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto &q = msg->orientation;
        imu_pitch_rad_ = quaternion_to_pitch({q.w, q.x, q.y, q.z});
        imu_yaw_rad_ = quaternion_to_yaw({q.w, q.x, q.y, q.z});
        imu_gyro_z_ = msg->angular_velocity.z;
    }

    void GimbalController::cb_pitch_feedback(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg)
    {
        // Pitch 电机反馈（可用于前馈）
    }

    void GimbalController::cb_yaw_feedback(const custom_msgs::msg::ReadLkMotor::SharedPtr msg)
    {
        yaw_motor_speed_ = msg->speed;
        yaw_motor_pos_ = (msg->encoder / 65535.0) * 2.0 * M_PI;
    }

    void GimbalController::cb_autoaim(const sp_msgs::msg::AutoAimCommandMsg::SharedPtr msg)
    {
        autoaim_control_ = msg->control;
        autoaim_yaw_ = msg->yaw;
        autoaim_pitch_ = msg->pitch;
        autoaim_last_time_ = node_->now().seconds();
    }

    void GimbalController::compute_pitch_control()
    {
        double now = node_->now().seconds();
        bool autoaim_fresh = (now - autoaim_last_time_) < config_.autoaim_timeout_s;

        // 判断是否使用自瞄
        bool use_autoaim = command_.autoaim_enabled && autoaim_control_ && autoaim_fresh;

        if (use_autoaim)
        {
            target_pitch_deg_ = -autoaim_pitch_ * (180.0 / M_PI);
        }
        else if (!command_.from_action)
        {
            // 遥控器增量控制
            target_pitch_deg_ += command_.pitch_deg;
        }
        // from_action 时直接使用 command_.pitch_deg 作为目标

        // 限幅
        target_pitch_deg_ = clamp(target_pitch_deg_, config_.pitch_min_deg, config_.pitch_max_deg);
    }

    void GimbalController::compute_yaw_control()
    {
        double now = node_->now().seconds();
        bool autoaim_fresh = (now - autoaim_last_time_) < config_.autoaim_timeout_s;

        bool use_autoaim = command_.autoaim_enabled && autoaim_control_ && autoaim_fresh;

        if (use_autoaim)
        {
            // 自瞄模式
            target_yaw_rad_ = std::atan2(std::sin(autoaim_yaw_), std::cos(autoaim_yaw_));
        }
        else if (!command_.from_action)
        {
            // 遥控器增量控制
            target_yaw_rad_ += command_.yaw_rad;
            // 归一化到 [-PI, PI]
            target_yaw_rad_ = std::atan2(std::sin(target_yaw_rad_), std::cos(target_yaw_rad_));
        }
    }

    void GimbalController::publish_commands()
    {
        // ========== Pitch 控制 (DJI 电机位置模式) ==========
        double current_pitch_deg = -imu_pitch_rad_ * (180.0 / M_PI);
        double pitch_error_deg = target_pitch_deg_ - current_pitch_deg;

        // 转换为编码器值 (8192 units per 360 degrees)
        int ecd_offset = static_cast<int>(pitch_error_deg * (8192.0 / 360.0));
        int pitch_cmd_ecd = config_.pitch_center_ecd + ecd_offset;

        auto pitch_msg = custom_msgs::msg::WriteDJIMotor();
        pitch_msg.motor4_enable = 1;
        pitch_msg.motor4_cmd = pitch_cmd_ecd;
        pub_pitch_->publish(pitch_msg);

        // ========== Yaw 控制 (LK 电机力矩模式，级联 PID) ==========
        // 1. 位置环
        double pos_error = target_yaw_rad_ - imu_yaw_rad_;
        pos_error = std::atan2(std::sin(pos_error), std::cos(pos_error)); // 归一化

        // 前馈角速度
        double target_ang_vel = -command_.yaw_rad * (10.0 * M_PI * 0.001);

        // D 项输入
        double d_input = target_ang_vel - imu_gyro_z_;

        double pid_pos_out = pid_yaw_pos_->update(pos_error, d_input);

        // 2. 速度环
        double target_spd = target_ang_vel + pid_pos_out;
        double spd_error = target_spd - imu_gyro_z_;
        double torque_cmd = pid_yaw_spd_->update(spd_error);

        auto yaw_msg = custom_msgs::msg::WriteLkMotorTorqueControl();
        yaw_msg.enable = 1;
        yaw_msg.torque = static_cast<int16_t>(torque_cmd);
        pub_yaw_->publish(yaw_msg);
    }

    void GimbalController::stop_all()
    {
        // 停止 Pitch
        auto pitch_msg = custom_msgs::msg::WriteDJIMotor();
        pitch_msg.motor4_enable = 0;
        pitch_msg.motor4_cmd = 0;
        pub_pitch_->publish(pitch_msg);

        // 停止 Yaw
        auto yaw_msg = custom_msgs::msg::WriteLkMotorTorqueControl();
        yaw_msg.enable = 0;
        yaw_msg.torque = 0;
        pub_yaw_->publish(yaw_msg);
    }

} // namespace universal_controller
