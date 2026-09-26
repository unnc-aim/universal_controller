/**
 * @file gimbal_controller.cpp
 * @brief 云台控制器实现
 *
 * 参考 infantry_controller/gimbal_controller.py
 */

#include "universal_controller/controllers/gimbal_controller.hpp"
#include <cmath>

namespace universal_controller {

GimbalController::GimbalController() = default;

void GimbalController::init(rclcpp::Node *node, const ConfigLoader &cfg) {
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

    // 发布 Pitch 指令
    pub_pitch_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
        config_.topic_pitch_write, qos_best_effort_);

    // 发布 Yaw 指令
    pub_yaw_ = node->create_publisher<custom_msgs::msg::WriteLkMotorTorqueControl>(
        config_.topic_yaw_write, qos_best_effort_);

    set_initialized(true);
    RCLCPP_INFO(node->get_logger(), "GimbalController initialized @ 1000Hz");
}

void GimbalController::set_command(const GimbalCommand &cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_follow_active_ && !cmd.follow_navigation) {
        reset_navigation_follow();
    }
    command_ = cmd;
    command_valid_ = true;
}

void GimbalController::update(double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!command_valid_) {
        return;
    }

    dt_ = dt;

    if (command_.follow_navigation) {
        if (!navigation_inputs_ready(node_->now().nanoseconds())) {
            reset_navigation_follow();
            stop_all();
            return;
        }
    }

    // 计算控制
    compute_pitch_control();
    compute_yaw_control();

    // 发布指令
    publish_commands();
}

void GimbalController::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_follow_active_) {
        reset_navigation_follow();
    }
    stop_all();
}

void GimbalController::cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto &q = msg->orientation;
    const double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    navigation_imu_valid_ = stamp > 0 && std::isfinite(norm) && std::abs(norm - 1.0) <= 0.01 &&
                            std::isfinite(msg->angular_velocity.z) && msg->orientation_covariance[0] >= 0.0;
    if (!navigation_imu_valid_) {
        return;
    }
    if (stamp < imu_stamp_ns_) {
        return;
    }
    imu_stamp_ns_ = stamp;
    imu_pitch_rad_ = quaternion_to_pitch({q.w, q.x, q.y, q.z});
    imu_yaw_rad_ = quaternion_to_yaw({q.w, q.x, q.y, q.z});
    imu_gyro_z_ = msg->angular_velocity.z;
}

void GimbalController::cb_pitch_feedback(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg) {
    (void)msg;
    // Pitch 电机反馈（可用于前馈）
}

void GimbalController::cb_yaw_feedback(const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    yaw_motor_speed_ = msg->speed;
    yaw_motor_pos_ = (msg->encoder / 65535.0) * 2.0 * M_PI;
}

void GimbalController::compute_pitch_control() {
    if (command_.follow_navigation) {
        return;
    }
    if (command_.scan_mode) {
        // 扫描模式：速度驱动，到达限位反转
        target_pitch_deg_ += command_.scan_vel_pitch * scan_direction_pitch_ * dt_ * (180.0 / M_PI);
        double pitch_min_deg = command_.scan_pitch_min * (180.0 / M_PI);
        double pitch_max_deg = command_.scan_pitch_max * (180.0 / M_PI);
        if (target_pitch_deg_ >= pitch_max_deg) {
            target_pitch_deg_ = pitch_max_deg;
            scan_direction_pitch_ = -1.0;
        } else if (target_pitch_deg_ <= pitch_min_deg) {
            target_pitch_deg_ = pitch_min_deg;
            scan_direction_pitch_ = 1.0;
        }
    } else if (command_.absolute) {
        // 绝对目标（自瞄/Action）
        target_pitch_deg_ = command_.pitch_deg;
    } else {
        // 遥控器增量控制
        target_pitch_deg_ += command_.pitch_deg;
    }

    // 限幅
    target_pitch_deg_ = clamp(target_pitch_deg_, config_.pitch_min_deg, config_.pitch_max_deg);
}

void GimbalController::compute_yaw_control() {
    if (command_.follow_navigation) {
        compute_navigation_yaw_control();
    } else if (command_.scan_mode) {
        // 速度扫描模式：按速度移动，到达限幅后反转方向
        target_yaw_rad_ += command_.scan_vel_yaw * scan_direction_yaw_ * dt_;
        if (target_yaw_rad_ >= command_.scan_yaw_max) {
            target_yaw_rad_ = command_.scan_yaw_max;
            scan_direction_yaw_ = -1.0;
        } else if (target_yaw_rad_ <= command_.scan_yaw_min) {
            target_yaw_rad_ = command_.scan_yaw_min;
            scan_direction_yaw_ = 1.0;
        }
    } else if (command_.absolute) {
        // 绝对目标（自瞄/Action）
        target_yaw_rad_ = std::atan2(std::sin(command_.yaw_rad), std::cos(command_.yaw_rad));
    } else {
        // 遥控器增量控制
        target_yaw_rad_ += command_.yaw_rad;
        target_yaw_rad_ = std::atan2(std::sin(target_yaw_rad_), std::cos(target_yaw_rad_));
    }
}

bool GimbalController::navigation_inputs_ready(int64_t now_ns) const {
    const double imu_age = static_cast<double>(now_ns - imu_stamp_ns_) * 1e-9;
    const double command_age = static_cast<double>(now_ns - command_.navigation_stamp_ns) * 1e-9;
    return config_.follow_navigation && navigation_imu_valid_ &&
           std::isfinite(dt_) && dt_ > 0.0 && dt_ <= config_.follow_input_timeout_s &&
           imu_age >= -0.05 && imu_age < config_.follow_input_timeout_s &&
           command_.navigation_stamp_ns > 0 && command_.navigation_stamp_ns >= navigation_stamp_ns_ &&
           command_age >= -0.05 && command_age < config_.follow_input_timeout_s &&
           std::isfinite(std::hypot(command_.navigation_vx, command_.navigation_vy));
}

void GimbalController::reset_navigation_follow() {
    navigation_follow_active_ = false;
    navigation_moving_ = false;
    navigation_stamp_ns_ = 0;
    navigation_yaw_rate_ = 0.0;
    if (pid_yaw_pos_) pid_yaw_pos_->reset();
    if (pid_yaw_spd_) pid_yaw_spd_->reset();
}

void GimbalController::compute_navigation_yaw_control() {
    if (!navigation_follow_active_) {
        reset_navigation_follow();
        target_yaw_rad_ = imu_yaw_rad_;
        target_pitch_deg_ = clamp(-imu_pitch_rad_ * (180.0 / M_PI), config_.pitch_min_deg, config_.pitch_max_deg);
        navigation_follow_active_ = true;
    }
    if (command_.navigation_stamp_ns != navigation_stamp_ns_) {
        navigation_moving_ = std::hypot(command_.navigation_vx, command_.navigation_vy) >= config_.follow_min_speed;
        navigation_target_yaw_ = normalize_angle(
            imu_yaw_rad_ + std::atan2(command_.navigation_vy, command_.navigation_vx));
        navigation_stamp_ns_ = command_.navigation_stamp_ns;
    }

    const double error = normalize_angle(navigation_target_yaw_ - target_yaw_rad_);
    // 转动段保留最低目标速度，到位后平滑减速至零。
    double desired_rate = 0.0;
    if (navigation_moving_ && std::abs(error) > config_.follow_yaw_tolerance) {
        const double braking_rate = std::sqrt(
            2.0 * config_.follow_max_yaw_accel * (std::abs(error) - config_.follow_yaw_tolerance));
        desired_rate = std::copysign(clamp(braking_rate, config_.follow_min_yaw_rate,
                                          config_.follow_max_yaw_rate), error);
    }
    const double max_change = config_.follow_max_yaw_accel * dt_;
    navigation_yaw_rate_ += clamp(desired_rate - navigation_yaw_rate_, -max_change, max_change);
    target_yaw_rad_ = normalize_angle(target_yaw_rad_ + navigation_yaw_rate_ * dt_);
}

void GimbalController::publish_commands() {
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

    // 导航跟随使用目标角速度；RC 使用原有增量换算。
    double target_ang_vel = command_.follow_navigation
        ? navigation_yaw_rate_
        : ((!command_.absolute && !command_.scan_mode)
            ? (-command_.yaw_rad * (10.0 * M_PI * 0.001)) : 0.0);

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

void GimbalController::stop_all() {
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
