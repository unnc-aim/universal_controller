//
// Created by aim on 2026/4/1.
//

#ifndef BUILD_DJI_FULL_WHEEL_CHASSIS_CONTROLLER_HPP
#define BUILD_DJI_FULL_WHEEL_CHASSIS_CONTROLLER_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <std_msgs/msg/string.hpp>

#include "chassis_controllers/msg/chassis_control.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/read_dji_motor.hpp"
#include "utils/pid.hpp"
#include "utils/power_limiter.hpp"
#include "utils/rls.hpp"

using namespace std::chrono_literals;
using namespace aim::algorithm;

class DJIFullWheelChassisController : public rclcpp::Node {
public:
    explicit DJIFullWheelChassisController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("dji_full_wheel_chassis_controller", options) {
        this->declare_parameter<uint16_t>("front_left_ecd_zero", 3040);
        this->declare_parameter<uint16_t>("front_right_ecd_zero", 999);
        this->declare_parameter<uint16_t>("back_left_ecd_zero", 5102);
        this->declare_parameter<uint16_t>("back_right_ecd_zero", 7235);

        this->declare_parameter<double>("wheel_radius", 0.06);
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();

        this->declare_parameter<double>("chassis_radius", 0.30);
        chassis_radius_ = this->get_parameter("chassis_radius").as_double();
        chassis_l_ = chassis_radius_ / std::sqrt(2.0);

        this->declare_parameter<std::string>("steer_motor_write_topic", "/steer_motor_write");
        this->declare_parameter<std::string>("steer_motor_read_topic", "/steer_motor_read");
        this->declare_parameter<std::string>("wheel_motor_write_topic", "/wheel_motor_write");
        this->declare_parameter<std::string>("wheel_motor_read_topic", "/wheel_motor_read");
        this->declare_parameter<std::string>("chassis_cmd_read_topic", "/chassis_command");

        front_left_ecd_zero_ =
                this->get_parameter("front_left_ecd_zero").as_int();
        front_right_ecd_zero_ =
                this->get_parameter("front_right_ecd_zero").as_int();
        back_left_ecd_zero_ =
                this->get_parameter("back_left_ecd_zero").as_int();
        back_right_ecd_zero_ =
                this->get_parameter("back_right_ecd_zero").as_int();

        chassis_command_sub_ =
                this->create_subscription<
                    chassis_controllers::msg::ChassisControl>(
                    this->get_parameter("chassis_cmd_read_topic").as_string(),
                    rclcpp::SensorDataQoS(),
                    [this](chassis_controllers::msg::ChassisControl::SharedPtr msg) {
                        this->control_command_ = *msg;
                    }
                );

        steer_motor_sub_ =
                this->create_subscription<
                    custom_msgs::msg::ReadDJIMotor>(
                    this->get_parameter("steer_motor_read_topic").as_string(),
                    rclcpp::SensorDataQoS(),
                    [this](custom_msgs::msg::ReadDJIMotor::SharedPtr msg) {
                        this->steer_motor_status_ = *msg;
                    }
                );

        wheel_motor_sub_ =
                this->create_subscription<
                    custom_msgs::msg::ReadDJIMotor>(
                    this->get_parameter("wheel_motor_read_topic").as_string(),
                    rclcpp::SensorDataQoS(),
                    [this](custom_msgs::msg::ReadDJIMotor::SharedPtr msg) {
                        this->wheel_motor_status_ = *msg;
                    }
                );

        steer_motor_pub_ =
                this->create_publisher<
                    custom_msgs::msg::WriteDJIMotor>(
                    this->get_parameter("steer_motor_write_topic").as_string(),
                    rclcpp::SensorDataQoS()
                );

        wheel_motor_pub_ =
                this->create_publisher<
                    custom_msgs::msg::WriteDJIMotor>(
                    this->get_parameter("wheel_motor_write_topic").as_string(),
                    rclcpp::SensorDataQoS()
                );

        control_timer_ =
                this->create_wall_timer(
                    2ms,
                    [this] { control_loop(); });

#ifdef POWER_FITTING
        fit_loop_timer_ = this->create_wall_timer(
            1ms,
            [this] { rls_update_loop(); });
#endif

        RCLCPP_INFO(this->get_logger(), "DJI full-wheel chassis controller started");
    }

private:
    uint16_t front_left_ecd_zero_;
    uint16_t front_right_ecd_zero_;
    uint16_t back_left_ecd_zero_;
    uint16_t back_right_ecd_zero_;

    double wheel_radius_;
    double chassis_radius_;
    double chassis_l_;

    struct {
        double v_front_left;
        double v_front_right;
        double v_back_left;
        double v_back_right;

        double direction_front_left;
        double direction_front_right;
        double direction_back_left;
        double direction_back_right;
    } chassis_ik_solution_;

    struct ShortestPath {
        int16_t err;

        // 1=clockwise
        // -1=counter-clockwise
        int8_t direction;
    };

    constexpr static float steer_speed_pid_kp = 125.0;
    constexpr static float steer_speed_pid_ki = 0.0;
    constexpr static float steer_speed_pid_kd = 0.0;
    constexpr static float steer_speed_pid_maxout = 16384.0;
    constexpr static float steer_speed_pid_maxiout = 0.0;
    PID steer_speed_pids_[4]{
        PID(steer_speed_pid_kp, steer_speed_pid_ki, steer_speed_pid_kd, steer_speed_pid_maxout,
            steer_speed_pid_maxiout),
        PID(steer_speed_pid_kp, steer_speed_pid_ki, steer_speed_pid_kd, steer_speed_pid_maxout,
            steer_speed_pid_maxiout),
        PID(steer_speed_pid_kp, steer_speed_pid_ki, steer_speed_pid_kd, steer_speed_pid_maxout,
            steer_speed_pid_maxiout),
        PID(steer_speed_pid_kp, steer_speed_pid_ki, steer_speed_pid_kd, steer_speed_pid_maxout,
            steer_speed_pid_maxiout)
    };

    constexpr static float steer_angle_pid_kp = 0.35;
    constexpr static float steer_angle_pid_ki = 0.0;
    constexpr static float steer_angle_pid_kd = 0.1;
    constexpr static float steer_angle_pid_maxout = 320.0;
    constexpr static float steer_angle_pid_maxiout = 0.0;
    PID steer_angle_pids_[4]{
        PID(steer_angle_pid_kp, steer_angle_pid_ki, steer_angle_pid_kd, steer_angle_pid_maxout,
            steer_angle_pid_maxiout),
        PID(steer_angle_pid_kp, steer_angle_pid_ki, steer_angle_pid_kd, steer_angle_pid_maxout,
            steer_angle_pid_maxiout),
        PID(steer_angle_pid_kp, steer_angle_pid_ki, steer_angle_pid_kd, steer_angle_pid_maxout,
            steer_angle_pid_maxiout),
        PID(steer_angle_pid_kp, steer_angle_pid_ki, steer_angle_pid_kd, steer_angle_pid_maxout,
            steer_angle_pid_maxiout)
    };

    constexpr static float wheel_speed_pid_kp = 13.5;
    constexpr static float wheel_speed_pid_ki = 0.5;
    constexpr static float wheel_speed_pid_kd = 0.0;
    constexpr static float wheel_speed_pid_maxout = 16384.0;
    constexpr static float wheel_speed_pid_maxiout = 2000.0;
    PID wheel_speed_pids_[4]{
        PID(wheel_speed_pid_kp, wheel_speed_pid_ki, wheel_speed_pid_kd, wheel_speed_pid_maxout,
            wheel_speed_pid_maxiout),
        PID(wheel_speed_pid_kp, wheel_speed_pid_ki, wheel_speed_pid_kd, wheel_speed_pid_maxout,
            wheel_speed_pid_maxiout),
        PID(wheel_speed_pid_kp, wheel_speed_pid_ki, wheel_speed_pid_kd, wheel_speed_pid_maxout,
            wheel_speed_pid_maxiout),
        PID(wheel_speed_pid_kp, wheel_speed_pid_ki, wheel_speed_pid_kd, wheel_speed_pid_maxout,
            wheel_speed_pid_maxiout)
    };

    static custom_msgs::msg::ReadDJIMotor steer_motor_status_;
    static custom_msgs::msg::ReadDJIMotor wheel_motor_status_;
    static custom_msgs::msg::WriteDJIMotor steer_motor_command_;
    static custom_msgs::msg::WriteDJIMotor wheel_motor_command_;
    static chassis_controllers::msg::ChassisControl control_command_;

    rclcpp::Subscription<
        chassis_controllers::msg::ChassisControl>::SharedPtr
    chassis_command_sub_;

    rclcpp::Publisher<
        custom_msgs::msg::WriteDJIMotor>::SharedPtr
    steer_motor_pub_;

    rclcpp::Publisher<
        custom_msgs::msg::WriteDJIMotor>::SharedPtr
    wheel_motor_pub_;

    rclcpp::Subscription<
        custom_msgs::msg::ReadDJIMotor>::SharedPtr
    steer_motor_sub_;

    rclcpp::Subscription<
        custom_msgs::msg::ReadDJIMotor>::SharedPtr
    wheel_motor_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr fit_loop_timer_;

    PowerLimiter steer_motor_limiter_{0.741, 1e-5, 13.9, 2.0, 4};
    PowerLimiter wheel_motor_limiter_{0.3 / (3591.f / 187.f), 0.01, 450.f, 3.0, 4};
    static constexpr float steer_power_distribution_ratio = 0.8f;

    void control_loop();

#ifdef POWER_FITTING
    void rls_update_loop();
#endif

    int16_t mpstorpm(const float mps) const {
        return 60.f * mps / (2. * M_PI * wheel_radius_) * (3591.f / 187.f);
    }

    double static calc_atan2(const double y, const double x) {
        const double atan2_res = atan2(y, x);
        if (atan2_res >= 0) {
            return atan2_res;
        }
        return M_PI * 2 + atan2_res;
    }

    ShortestPath static calc_shortest_path(const int16_t current_ecd, const int16_t target_ecd) {
        const int16_t forwardDistanceA = (target_ecd - current_ecd + 8192) % 8192;
        const int16_t backwardDistanceA = (current_ecd - target_ecd + 8192) % 8192;

        const int16_t forwardDistanceB = (target_ecd - current_ecd + 8192 + 4096) % 8192;
        const int16_t backwardDistanceB = (current_ecd - target_ecd + 8192 + 4096) % 8192;

        int16_t rotationAngle;
        ShortestPath res{};

        if (std::min(forwardDistanceA, backwardDistanceA) <= std::min(forwardDistanceB, backwardDistanceB)) {
            rotationAngle = (forwardDistanceA <= backwardDistanceA) ? forwardDistanceA : -backwardDistanceA;
            res.err = rotationAngle;
            res.direction = 1;
        } else {
            rotationAngle = (forwardDistanceB <= backwardDistanceB) ? forwardDistanceB : -backwardDistanceB;
            res.err = rotationAngle;
            res.direction = -1;
        }
        return res;
    }
};

#endif //BUILD_DJI_FULL_WHEEL_CHASSIS_CONTROLLER_HPP
