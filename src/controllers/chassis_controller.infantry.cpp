/**
 * @file chassis_controller.infantry.cpp
 * @brief 步兵底盘控制器实现 (DJI 电机)
 *
 * 基于 infantry_controller/chassis_controller.py
 * 仅支持 DJI 电机：舵向位置模式 + 驱动速度模式
 * 步兵电机接线与哨兵不同：
 *   反馈: motor1→[0], motor4→[1], motor3→[2], motor2→[3]
 *   发送: motor1→[0], motor2→[1], motor3→[2], motor4→[3]
 */

#include "universal_controller/controllers/chassis_controller.hpp"
#include <cmath>

namespace universal_controller {

ChassisController::ChassisController() {
    kinematics_ = std::make_unique<SwerveKinematics>(0.372, 0.372, 8192);
}

void ChassisController::init(rclcpp::Node *node, const ConfigLoader &cfg) {
    set_node(node);

    // 加载配置
    config_.load(cfg);
    motor_type_ = config_.motor_type;
    yaw_center_ecd_ = config_.yaw_center_ecd;

    // DJI 编码器周期 8192
    kinematics_ = std::make_unique<SwerveKinematics>(
        config_.wheel_track, config_.wheel_base, 8192u);

    // 订阅舵向 DJI 电机反馈
    sub_steer_dji_ = node->create_subscription<custom_msgs::msg::ReadDJIMotor>(
        config_.topic_steer_read, qos_best_effort_,
        std::bind(&ChassisController::cb_steer_dji, this, std::placeholders::_1));

    // 发布驱动和舵向指令
    pub_drive_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
        config_.topic_drive_write, qos_best_effort_);
    pub_steer_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
        config_.topic_steer_write, qos_best_effort_);

    // Yaw 电机订阅
    sub_yaw_ = node->create_subscription<custom_msgs::msg::ReadLkMotor>(
        config_.topic_yaw_read, qos_best_effort_,
        std::bind(&ChassisController::cb_yaw, this, std::placeholders::_1));

    set_initialized(true);
    RCLCPP_INFO(node->get_logger(), "ChassisController (Infantry) initialized (DJI motors)");
    RCLCPP_INFO(node->get_logger(), "Chassis yaw_center_ecd=%d", yaw_center_ecd_);
}

void ChassisController::set_command(const ChassisCommand &cmd) {
    command_ = cmd;
    command_valid_ = true;
}

void ChassisController::set_power_limit(double limit) {
    (void)limit; // 步兵不使用功率限制
}

void ChassisController::update(double dt) {
    if (!command_valid_) {
        return;
    }
    compute_control(dt);
    publish_dji_commands();
}

void ChassisController::stop() {
    auto msg = custom_msgs::msg::WriteDJIMotor();
    pub_drive_dji_->publish(msg);
    pub_steer_dji_->publish(msg);
}

void ChassisController::compute_control(double dt) {
    (void)dt;
    // 死区检测
    if (std::abs(command_.vx_gimbal) < 100.0 &&
        std::abs(command_.vy_gimbal) < 100.0 &&
        std::abs(command_.wz) < 100.0) {
        for (int i = 0; i < 4; ++i) {
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
    if (command_.spin_mode && std::abs(command_.wz) > 10.0) {
        wz = compute_spin_suppression(vx_c, vy_c, command_.wz);
    }

    // 3. 运动学解算
    auto [drive_speeds, steer_ecds] = kinematics_->calculate_motion(
        vx_c, vy_c, wz,
        current_steer_ecds_, config_.ecd_zeros);

    target_drive_speeds_ = drive_speeds;
    target_steer_ecds_ = steer_ecds;
}

void ChassisController::transform_to_chassis_frame(double vx_g, double vy_g, double &vx_c, double &vy_c) const {
    double theta = gimbal_yaw_angle_;
    vx_c = vx_g * std::cos(theta) + vy_g * std::sin(theta);
    vy_c = -vx_g * std::sin(theta) + vy_g * std::cos(theta);
}

double ChassisController::compute_spin_suppression(double vx_c, double vy_c, double wz_cmd) const {
    double current_speed = std::sqrt(vx_c * vx_c + vy_c * vy_c);
    double k = current_speed / 8000.0;

    // 限幅系数
    if (k > 0.4)
        k = 0.4;
    if (k < 0.2)
        k = 0.2;

    // 步兵抑制公式: (1.0 - k)，比哨兵 (1.0 - 2.0 * k) 更温和
    return wz_cmd * (1.0 - k);
}

void ChassisController::publish_dji_commands() {
    // 舵向指令
    // 步兵发送映射: motor1=[0], motor2=[1], motor3=[2], motor4=[3]
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

void ChassisController::cb_steer_dji(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg) {
    // 步兵舵向反馈映射（物理接线与发送不同）:
    //   motor1→[0], motor4→[1], motor3→[2], motor2→[3]
    current_steer_ecds_[0] = msg->motor1_ecd;
    current_steer_ecds_[1] = msg->motor4_ecd;
    current_steer_ecds_[2] = msg->motor3_ecd;
    current_steer_ecds_[3] = msg->motor2_ecd;
}

void ChassisController::cb_yaw(const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
    int32_t relative_ecd = static_cast<int32_t>(msg->encoder) - yaw_center_ecd_;
    if (relative_ecd > 32767)
        relative_ecd -= 65536;
    if (relative_ecd < -32768)
        relative_ecd += 65536;

    gimbal_yaw_angle_ = (relative_ecd / 65535.0) * 2.0 * M_PI;
}

} // namespace universal_controller
