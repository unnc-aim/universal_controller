#include "rclcpp/rclcpp.hpp"
#include "chassis_controllers/dji_full_wheel_chassis_controller.hpp"
#include "chassis_controllers/utils/rls.hpp"

custom_msgs::msg::ReadDJIMotor DJIFullWheelChassisController::steer_motor_status_{};
custom_msgs::msg::ReadDJIMotor DJIFullWheelChassisController::wheel_motor_status_{};
custom_msgs::msg::WriteDJIMotor DJIFullWheelChassisController::steer_motor_command_{};
custom_msgs::msg::WriteDJIMotor DJIFullWheelChassisController::wheel_motor_command_{};
chassis_controllers::msg::ChassisControl DJIFullWheelChassisController::control_command_{};

#ifdef POWER_FITTING
void DJIFullWheelChassisController::rls_update_loop() {
    if (control_command_.extra_info.empty()) {
        return;
    }

#if POWER_FITTING == STEER
    const float steer_rpm[4] = {
        static_cast<float>(steer_motor_status_.motor1_rpm),
        static_cast<float>(steer_motor_status_.motor2_rpm),
        static_cast<float>(steer_motor_status_.motor3_rpm),
        static_cast<float>(steer_motor_status_.motor4_rpm)
    };
    const float steer_current[4] = {
        static_cast<float>(steer_motor_status_.motor1_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor2_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor3_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor4_current) / 16384.f * 3.f
    };
    steer_motor_limiter_.update_rls(steer_rpm, steer_current, control_command_.extra_info[0]);
#elif POWER_FITTING == WHEEL
    const float wheel_rpm[4] = {
        static_cast<float>(wheel_motor_status_.motor1_rpm),
        static_cast<float>(wheel_motor_status_.motor2_rpm),
        static_cast<float>(wheel_motor_status_.motor3_rpm),
        static_cast<float>(wheel_motor_status_.motor4_rpm)
    };
    const float wheel_current[4] = {
        static_cast<float>(wheel_motor_status_.motor1_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor2_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor3_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor4_current) / 16384.f * 20.f
    };
    wheel_motor_limiter_.update_rls(wheel_rpm, wheel_current, control_command_.extra_info[0]);
#endif
}
#endif

void DJIFullWheelChassisController::control_loop() {
    if (control_command_.emergency_stop) {
        steer_motor_command_.motor1_enable = 0;
        steer_motor_command_.motor2_enable = 0;
        steer_motor_command_.motor3_enable = 0;
        steer_motor_command_.motor4_enable = 0;
        this->steer_motor_pub_->publish(steer_motor_command_);

        wheel_motor_command_.motor1_enable = 0;
        wheel_motor_command_.motor2_enable = 0;
        wheel_motor_command_.motor3_enable = 0;
        wheel_motor_command_.motor4_enable = 0;
        this->wheel_motor_pub_->publish(wheel_motor_command_);
        return;
    }

    const double vx_fl = control_command_.x_speed - control_command_.spin_speed * chassis_l_;
    const double vy_fl = -control_command_.y_speed - control_command_.spin_speed * chassis_l_;
    chassis_ik_solution_.v_front_left =
            std::sqrt(vx_fl * vx_fl + vy_fl * vy_fl);
    chassis_ik_solution_.direction_front_left =
            calc_atan2(vy_fl, vx_fl);

    const double vx_fr = control_command_.x_speed + control_command_.spin_speed * chassis_l_;
    const double vy_fr = -control_command_.y_speed - control_command_.spin_speed * chassis_l_;
    chassis_ik_solution_.v_front_right =
            std::sqrt(vx_fr * vx_fr + vy_fr * vy_fr);
    chassis_ik_solution_.direction_front_right =
            calc_atan2(vy_fr, vx_fr);

    const double vx_bl = control_command_.x_speed - control_command_.spin_speed * chassis_l_;
    const double vy_bl = -control_command_.y_speed + control_command_.spin_speed * chassis_l_;
    chassis_ik_solution_.v_back_left =
            std::sqrt(vx_bl * vx_bl + vy_bl * vy_bl);
    chassis_ik_solution_.direction_back_left =
            calc_atan2(vy_bl, vx_bl);

    const double vx_br = control_command_.x_speed + control_command_.spin_speed * chassis_l_;
    const double vy_br = -control_command_.y_speed + control_command_.spin_speed * chassis_l_;
    chassis_ik_solution_.v_back_right =
            std::sqrt(vx_br * vx_br + vy_br * vy_br);
    chassis_ik_solution_.direction_back_right =
            calc_atan2(vy_br, vx_br);

    double rpm_fl = mpstorpm(chassis_ik_solution_.v_front_left);
    double rpm_fr = mpstorpm(chassis_ik_solution_.v_front_right);
    double rpm_bl = mpstorpm(chassis_ik_solution_.v_back_left);
    double rpm_br = mpstorpm(chassis_ik_solution_.v_back_right);

    const double max_rpm =
            std::max({
                std::abs(rpm_fl),
                std::abs(rpm_fr),
                std::abs(rpm_bl),
                std::abs(rpm_br)
            });

    if (max_rpm > 8500.0) {
        const double scale = 8500.0 / max_rpm;

        rpm_fl *= scale;
        rpm_fr *= scale;
        rpm_bl *= scale;
        rpm_br *= scale;
    }

    chassis_ik_solution_.v_front_left = rpm_fl;
    chassis_ik_solution_.v_front_right = rpm_fr;
    chassis_ik_solution_.v_back_left = rpm_bl;
    chassis_ik_solution_.v_back_right = rpm_br;

    const int16_t target_front_left_ecd =
            static_cast<int16_t>((chassis_ik_solution_.direction_front_left / (2.0f * M_PI)) * 8192 +
                                 front_left_ecd_zero_) % 8192;
    const int16_t target_front_right_ecd =
            static_cast<int16_t>((chassis_ik_solution_.direction_front_right / (2.0f * M_PI)) * 8192 +
                                 front_right_ecd_zero_) % 8192;
    const int16_t target_back_left_ecd =
            static_cast<int16_t>((chassis_ik_solution_.direction_back_left / (2.0f * M_PI)) * 8192 +
                                 back_left_ecd_zero_) % 8192;
    const int16_t target_back_right_ecd =
            static_cast<int16_t>((chassis_ik_solution_.direction_back_right / (2.0f * M_PI)) * 8192 +
                                 back_right_ecd_zero_) % 8192;

    // optimize path
    const ShortestPath fl_t = calc_shortest_path(steer_motor_status_.motor1_ecd, target_front_left_ecd);
    const int16_t front_left_ecd_err = fl_t.err;
    chassis_ik_solution_.v_front_left *= fl_t.direction;

    const ShortestPath fr_t = calc_shortest_path(steer_motor_status_.motor2_ecd, target_front_right_ecd);
    const int16_t front_right_ecd_err = fr_t.err;
    chassis_ik_solution_.v_front_right *= fr_t.direction;

    const ShortestPath bl_t = calc_shortest_path(steer_motor_status_.motor3_ecd, target_back_left_ecd);
    const int16_t back_left_ecd_err = bl_t.err;
    chassis_ik_solution_.v_back_left *= bl_t.direction;

    const ShortestPath br_t = calc_shortest_path(steer_motor_status_.motor4_ecd, target_back_right_ecd);
    const int16_t back_right_ecd_err = br_t.err;
    chassis_ik_solution_.v_back_right *= br_t.direction;

    if (chassis_ik_solution_.v_front_left != 0 ||
        chassis_ik_solution_.v_front_right != 0 ||
        chassis_ik_solution_.v_back_left != 0 ||
        chassis_ik_solution_.v_back_right != 0
    ) {
        const float temp_front_left_angle_pid_out = -steer_angle_pids_[0].calculate(front_left_ecd_err, 0);
        const float temp_front_right_angle_pid_out = -steer_angle_pids_[1].calculate(front_right_ecd_err, 0);
        const float temp_back_left_angle_pid_out = -steer_angle_pids_[2].calculate(back_left_ecd_err, 0);
        const float temp_back_right_angle_pid_out = -steer_angle_pids_[3].calculate(back_right_ecd_err, 0);

        steer_motor_command_.motor1_cmd = steer_speed_pids_[0].calculate(
            steer_motor_status_.motor1_rpm, temp_front_left_angle_pid_out);
        steer_motor_command_.motor2_cmd = steer_speed_pids_[1].calculate(
            steer_motor_status_.motor2_rpm, temp_front_right_angle_pid_out);
        steer_motor_command_.motor3_cmd = steer_speed_pids_[2].calculate(
            steer_motor_status_.motor3_rpm, temp_back_left_angle_pid_out);
        steer_motor_command_.motor4_cmd = steer_speed_pids_[3].calculate(
            steer_motor_status_.motor4_rpm, temp_back_right_angle_pid_out);
    } else {
        steer_motor_command_.motor1_cmd = 0;
        steer_motor_command_.motor2_cmd = 0;
        steer_motor_command_.motor3_cmd = 0;
        steer_motor_command_.motor4_cmd = 0;
    }

    wheel_motor_command_.motor1_cmd = wheel_speed_pids_[0].calculate(wheel_motor_status_.motor1_rpm,
                                                                     chassis_ik_solution_.v_front_left);
    wheel_motor_command_.motor2_cmd = wheel_speed_pids_[1].calculate(wheel_motor_status_.motor2_rpm,
                                                                     chassis_ik_solution_.v_front_right);
    wheel_motor_command_.motor3_cmd = wheel_speed_pids_[2].calculate(wheel_motor_status_.motor3_rpm,
                                                                     chassis_ik_solution_.v_back_left);
    wheel_motor_command_.motor4_cmd = wheel_speed_pids_[3].calculate(wheel_motor_status_.motor4_rpm,
                                                                     chassis_ik_solution_.v_back_right);
    // ---

    const float steer_rpm[4] = {
        static_cast<float>(steer_motor_status_.motor1_rpm),
        static_cast<float>(steer_motor_status_.motor2_rpm),
        static_cast<float>(steer_motor_status_.motor3_rpm),
        static_cast<float>(steer_motor_status_.motor4_rpm)
    };
    const float steer_current[4] = {
        static_cast<float>(steer_motor_status_.motor1_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor2_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor3_current) / 16384.f * 3.f,
        static_cast<float>(steer_motor_status_.motor4_current) / 16384.f * 3.f
    };
    const float steer_outputs[4] = {
        static_cast<float>(steer_motor_command_.motor1_cmd) / 16384.f * 3.f,
        static_cast<float>(steer_motor_command_.motor2_cmd) / 16384.f * 3.f,
        static_cast<float>(steer_motor_command_.motor3_cmd) / 16384.f * 3.f,
        static_cast<float>(steer_motor_command_.motor4_cmd) / 16384.f * 3.f
    };

    const float wheel_rpm[4] = {
        static_cast<float>(wheel_motor_status_.motor1_rpm),
        static_cast<float>(wheel_motor_status_.motor2_rpm),
        static_cast<float>(wheel_motor_status_.motor3_rpm),
        static_cast<float>(wheel_motor_status_.motor4_rpm)
    };
    const float wheel_current[4] = {
        static_cast<float>(wheel_motor_status_.motor1_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor2_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor3_current) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_status_.motor4_current) / 16384.f * 20.f
    };
    const float wheel_outputs[4] = {
        static_cast<float>(wheel_motor_command_.motor1_cmd) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_command_.motor2_cmd) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_command_.motor3_cmd) / 16384.f * 20.f,
        static_cast<float>(wheel_motor_command_.motor4_cmd) / 16384.f * 20.f
    };

    // ---
#ifdef POWER_FITTING
#if POWER_FITTING == WHEEL
    wheel_motor_limiter_.update_status(wheel_rpm, wheel_current, wheel_outputs, control_command_.power_limit);
    RCLCPP_INFO(get_logger(), "%f %f %f", wheel_motor_limiter_.get_estimated_power(),
                wheel_motor_limiter_.get_command_power(), control_command_.extra_info.empty() ? 0.0 : control_command_.extra_info[0]);
#elif POWER_FITTING == STEER
    steer_motor_limiter_.update_status(steer_rpm, steer_current, steer_outputs, control_command_.power_limit);
    RCLCPP_INFO(get_logger(), "%f %f %f", steer_motor_limiter_.get_estimated_power(),
                steer_motor_limiter_.get_command_power(), control_command_.extra_info.empty() ? 0.0 : control_command_.extra_info[0]);
#endif
#else
    const float max_power = control_command_.power_limit;

    steer_motor_limiter_.update_status(steer_rpm, steer_current, steer_outputs,
                                       max_power * steer_power_distribution_ratio);
    wheel_motor_limiter_.update_status(wheel_rpm, wheel_current, wheel_outputs,
                                       max_power - std::fmin(steer_motor_limiter_.get_command_power(),
                                                             max_power * steer_power_distribution_ratio));

    const float *steer_res = steer_motor_limiter_.get_decay_power(steer_rpm, steer_outputs);
    const float *wheel_res = wheel_motor_limiter_.get_decay_power(wheel_rpm, wheel_outputs);

    wheel_motor_command_.motor1_cmd = limit_max_min(wheel_res[0] / 20.f * 16384.f, 16384.f, -16384.f);
    wheel_motor_command_.motor2_cmd = limit_max_min(wheel_res[1] / 20.f * 16384.f, 16384.f, -16384.f);
    wheel_motor_command_.motor3_cmd = limit_max_min(wheel_res[2] / 20.f * 16384.f, 16384.f, -16384.f);
    wheel_motor_command_.motor4_cmd = limit_max_min(wheel_res[3] / 20.f * 16384.f, 16384.f, -16384.f);

    steer_motor_command_.motor1_cmd = limit_max_min(steer_res[0] / 3.f * 16384.f, 16384.f, -16384.f);
    steer_motor_command_.motor2_cmd = limit_max_min(steer_res[1] / 3.f * 16384.f, 16384.f, -16384.f);
    steer_motor_command_.motor3_cmd = limit_max_min(steer_res[2] / 3.f * 16384.f, 16384.f, -16384.f);
    steer_motor_command_.motor4_cmd = limit_max_min(steer_res[3] / 3.f * 16384.f, 16384.f, -16384.f);
#endif

    // ---
    steer_motor_command_.motor1_enable = 1;
    steer_motor_command_.motor2_enable = 1;
    steer_motor_command_.motor3_enable = 1;
    steer_motor_command_.motor4_enable = 1;
    this->steer_motor_pub_->publish(steer_motor_command_);

    wheel_motor_command_.motor1_enable = 1;
    wheel_motor_command_.motor2_enable = 1;
    wheel_motor_command_.motor3_enable = 1;
    wheel_motor_command_.motor4_enable = 1;
    this->wheel_motor_pub_->publish(wheel_motor_command_);
}

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    const auto node =
            std::make_shared<DJIFullWheelChassisController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
