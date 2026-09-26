/**
 * @file gimbal_controller.hpp
 * @brief 云台控制器
 *
 * 实现云台的双轴控制：
 * - Pitch: DJI 电机位置模式
 * - Yaw: LK 电机力矩模式（级联 PID）
 */

#pragma once

#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "universal_controller/common/config.hpp"
#include "universal_controller/common/types.hpp"
#include "universal_controller/controllers/base_controller.hpp"
#include "universal_controller/tools/pid.hpp"

#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/write_lk_motor_torque_control.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace universal_controller {

/**
 * @brief 云台控制器
 */
class GimbalController : public BaseController {
  public:
    GimbalController();
    ~GimbalController() override = default;

    void init(rclcpp::Node *node, const ConfigLoader &cfg) override;
    void update(double dt) override;
    void stop() override;
    std::string name() const override {
        return "GimbalController";
    }

    void set_command(const GimbalCommand &cmd);

  private:
    friend struct NavigationGimbalTest;

    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cb_pitch_feedback(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg);
    void cb_yaw_feedback(const custom_msgs::msg::ReadLkMotor::SharedPtr msg);

    void compute_pitch_control();
    void compute_yaw_control();
    bool navigation_inputs_ready(int64_t now_ns) const;
    void compute_navigation_yaw_control();
    void reset_navigation_follow();
    void publish_commands();
    void stop_all();

    // 配置
    GimbalConfig config_;

    // PID 控制器
    std::unique_ptr<PID> pid_yaw_pos_;
    std::unique_ptr<PID> pid_yaw_spd_;

    // 指令
    GimbalCommand command_;
    bool command_valid_{false};
    std::mutex mutex_;

    // IMU 状态
    double imu_pitch_rad_{0.0};
    double imu_yaw_rad_{0.0};
    double imu_gyro_z_{0.0};
    int64_t imu_stamp_ns_{0};
    bool navigation_imu_valid_{false};

    // Yaw 电机状态
    double yaw_motor_speed_{0.0};
    double yaw_motor_pos_{0.0};

    // 目标状态
    double target_pitch_deg_{0.0};
    double target_yaw_rad_{0.0};

    // 目标方向按导航消息更新，角度轨迹按控制周期更新。
    bool navigation_follow_active_{false};
    bool navigation_moving_{false};
    int64_t navigation_stamp_ns_{0};
    double navigation_target_yaw_{0.0};
    double navigation_yaw_rate_{0.0};

    // 扫描状态
    double dt_{0.0};
    double scan_direction_yaw_{1.0};
    double scan_direction_pitch_{1.0};

    // ROS2 接口
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<custom_msgs::msg::ReadDJIMotor>::SharedPtr sub_pitch_;
    rclcpp::Subscription<custom_msgs::msg::ReadLkMotor>::SharedPtr sub_yaw_;

    rclcpp::Publisher<custom_msgs::msg::WriteDJIMotor>::SharedPtr pub_pitch_;
    rclcpp::Publisher<custom_msgs::msg::WriteLkMotorTorqueControl>::SharedPtr pub_yaw_;

    rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
};

} // namespace universal_controller
