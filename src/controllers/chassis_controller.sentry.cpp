/**
 * @file chassis_controller.cpp
 * @brief 底盘控制器实现
 *
 * 参考 infantry_controller/chassis_controller.py 和 sentry_controller/chassis_controller.py
 */

#include "universal_controller/controllers/chassis_controller.hpp"
#include <cmath>

namespace universal_controller {

ChassisController::ChassisController() {
    // 初始化运动学解算器
    kinematics_ = std::make_unique<SwerveKinematics>(0.372, 0.372, 8192);
}

void ChassisController::init(rclcpp::Node *node, const ConfigLoader &cfg) {
    set_node(node);

    // 加载配置
    config_.load(cfg);

    // 设置电机类型
    motor_type_ = config_.motor_type;
    yaw_center_ecd_ = config_.yaw_center_ecd;

    // 初始化运动学
    // 对齐旧 sentry_controller：LK 舵向编码器按 32768 周期处理
    uint32_t ecd_range = (motor_type_ == MotorType::DJI) ? 8192u : 32768u;
    kinematics_ = std::make_unique<SwerveKinematics>(
        config_.wheel_track, config_.wheel_base, ecd_range);

    // 初始化 PID (LK 模式)
    if (motor_type_ == MotorType::LK) {
        for (int i = 0; i < 4; ++i) {
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
    if (motor_type_ == MotorType::DJI) {
        sub_steer_dji_ = node->create_subscription<custom_msgs::msg::ReadDJIMotor>(
            config_.topic_steer_read, qos_best_effort_,
            std::bind(&ChassisController::cb_steer_dji, this, std::placeholders::_1));

        pub_drive_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
            config_.topic_drive_write, qos_best_effort_);
        pub_steer_dji_ = node->create_publisher<custom_msgs::msg::WriteDJIMotor>(
            config_.topic_steer_write, qos_best_effort_);
    } else {
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

    // 超级电容订阅（功率限制）
    if (config_.power_limit_enabled && !config_.topic_supercap.empty()) {
        sub_supercap_ = node->create_subscription<custom_msgs::msg::ReadSuperCap>(
            config_.topic_supercap, qos_best_effort_,
            std::bind(&ChassisController::cb_supercap, this, std::placeholders::_1));
        k_dynamic_ = config_.power_K;
        power_limit_ = config_.power_limit_default;
        RCLCPP_INFO(node->get_logger(), "ChassisController power limiting enabled (R=%.4f, K=%.4f, P0=%.4f)",
                    config_.power_R, config_.power_K, config_.power_P0);
    }

    set_initialized(true);
    RCLCPP_INFO(node->get_logger(), "ChassisController initialized (motor type: %s)",
                motor_type_ == MotorType::DJI ? "DJI" : "LK");
    RCLCPP_INFO(node->get_logger(), "Chassis yaw_center_ecd=%d", yaw_center_ecd_);
}

void ChassisController::set_command(const ChassisCommand &cmd) {
    command_ = cmd;
    command_valid_ = true;
}

void ChassisController::set_power_limit(double limit) {
    if (limit > 0.0) {
        power_limit_ = limit;
    }
}

void ChassisController::update(double dt) {
    if (!command_valid_) {
        return;
    }
    compute_control(dt);

    if (motor_type_ == MotorType::DJI) {
        publish_dji_commands();
    } else {
        publish_lk_commands();
    }
}

void ChassisController::stop() {
    if (motor_type_ == MotorType::DJI) {
        auto msg = custom_msgs::msg::WriteDJIMotor();
        pub_drive_dji_->publish(msg);
        pub_steer_dji_->publish(msg);
    } else {
        auto msg = custom_msgs::msg::WriteLkMotorBroadcastCurrentControl();
        pub_drive_lk_->publish(msg);
        pub_steer_lk_->publish(msg);
    }
}

void ChassisController::compute_control(double dt) {
    (void)dt;
    // 死区检测
    if (std::abs(command_.vx_gimbal) < 100.0 &&
        std::abs(command_.vy_gimbal) < 100.0 &&
        std::abs(command_.wz) < 100.0) {
        // 停止输出
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

void ChassisController::transform_to_chassis_frame(double vx_g, double vy_g,
                                                   double &vx_c, double &vy_c) const {
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

    // 速度越快，自旋越慢
    return wz_cmd * (1.0 - 2.0 * k);
}

void ChassisController::publish_dji_commands() {
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

void ChassisController::publish_lk_commands() {
    // 舵向级联 PID
    auto steer_msg = custom_msgs::msg::WriteLkMotorBroadcastCurrentControl();
    std::array<int16_t, 4> steer_currents;

    for (int i = 0; i < 4; ++i) {
        int32_t err = static_cast<int32_t>(target_steer_ecds_[i]) - current_steer_ecds_[i];
        // 处理编码器跳变
        if (err > 16384)
            err -= 32768;
        if (err < -16384)
            err += 32768;

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

    for (int i = 0; i < 4; ++i) {
        double current_out = drive_speed_pids_[i].update(
            target_drive_speeds_[i] - current_drive_speeds_[i]);
        drive_currents[i] = static_cast<int16_t>(current_out);
    }

    // 功率限制：缩放驱动电流
    if (config_.power_limit_enabled) {
        apply_power_limit(drive_currents);
    }

    drive_msg.motor1_cmd = drive_currents[0];
    drive_msg.motor2_cmd = drive_currents[1];
    drive_msg.motor3_cmd = drive_currents[2];
    drive_msg.motor4_cmd = drive_currents[3];
    pub_drive_lk_->publish(drive_msg);
}

void ChassisController::cb_steer_dji(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg) {
    // DJI 电机 ID 映射: motor1=FL, motor2=FR, motor3=BL, motor4=BR
    current_steer_ecds_[0] = msg->motor1_ecd;
    current_steer_ecds_[1] = msg->motor2_ecd;
    current_steer_ecds_[2] = msg->motor3_ecd;
    current_steer_ecds_[3] = msg->motor4_ecd;
}

void ChassisController::cb_steer_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg) {
    current_steer_ecds_[0] = msg->motor1_encoder;
    current_steer_ecds_[1] = msg->motor2_encoder;
    current_steer_ecds_[2] = msg->motor3_encoder;
    current_steer_ecds_[3] = msg->motor4_encoder;
    current_steer_speeds_[0] = msg->motor1_speed;
    current_steer_speeds_[1] = msg->motor2_speed;
    current_steer_speeds_[2] = msg->motor3_speed;
    current_steer_speeds_[3] = msg->motor4_speed;
}

void ChassisController::cb_drive_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg) {
    current_drive_speeds_[0] = msg->motor1_speed;
    current_drive_speeds_[1] = msg->motor2_speed;
    current_drive_speeds_[2] = msg->motor3_speed;
    current_drive_speeds_[3] = msg->motor4_speed;
}

void ChassisController::cb_supercap(const custom_msgs::msg::ReadSuperCap::SharedPtr msg) {
    supercap_online_ = (msg->online != 0);
    supercap_chassis_only_power_ = static_cast<double>(msg->chassis_only_power);
}

void ChassisController::apply_power_limit(std::array<int16_t, 4> &drive_currents) {
    // 1. 用实测功率在线标定 K_dynamic
    if (supercap_online_ && supercap_chassis_only_power_ > 0.0) {
        double last_i2 = 0.0, last_wi = 0.0;
        for (int i = 0; i < 4; ++i) {
            double I = static_cast<double>(last_drive_currents_[i]);
            last_i2 += I * I;
            last_wi += I * current_drive_speeds_[i];
        }
        if (std::abs(last_wi) > 1.0) {
            // 从实测功率反推 K: P = R*ΣI² + K*Σ(ω*I) + P0
            double k_measured = (supercap_chassis_only_power_ - config_.power_P0 - config_.power_R * last_i2) / last_wi;
            k_dynamic_ = k_dynamic_ * (1.0 - config_.power_filter_alpha) + k_measured * config_.power_filter_alpha;
            k_dynamic_ = clamp(k_dynamic_, config_.power_K_min, config_.power_K_max);
        }
    }

    // 2. 预测发送功率: P = R*ΣI² + K*Σ(ω*I) + P0
    double i2_sum = 0.0, wi_sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        double I = static_cast<double>(drive_currents[i]);
        i2_sum += I * I;
        wi_sum += I * current_drive_speeds_[i];
    }

    double a = config_.power_R * i2_sum;
    double b = k_dynamic_ * wi_sum;
    double c = config_.power_P0;
    double predicted = a + b + c;

    // 3. 超功率则求解缩放因子 k_c
    if (predicted > power_limit_ && power_limit_ > 0.0) {
        double k_c = 1.0;
        if (a > 1e-9) {
            double disc = b * b - 4.0 * a * (c - power_limit_);
            if (disc < 0.0) {
                k_c = clamp(-b / (2.0 * a), 0.0, 1.0);
            } else {
                double sqrt_disc = std::sqrt(disc);
                k_c = clamp((-b + sqrt_disc) / (2.0 * a), 0.0, 1.0);
            }
        }
        for (int i = 0; i < 4; ++i) {
            drive_currents[i] = static_cast<int16_t>(static_cast<double>(drive_currents[i]) * k_c);
        }
    }

    // 4. 记录本轮电流，用于下一周期标定
    for (int i = 0; i < 4; ++i) {
        last_drive_currents_[i] = drive_currents[i];
    }
}

void ChassisController::cb_yaw(const custom_msgs::msg::ReadLkMotor::SharedPtr msg) {
    int32_t relative_ecd = static_cast<int32_t>(msg->encoder) - yaw_center_ecd_;
    // 处理 16 位有符号溢出
    if (relative_ecd > 32767)
        relative_ecd -= 65536;
    if (relative_ecd < -32768)
        relative_ecd += 65536;

    gimbal_yaw_angle_ = (relative_ecd / 65535.0) * 2.0 * M_PI;
}

} // namespace universal_controller
