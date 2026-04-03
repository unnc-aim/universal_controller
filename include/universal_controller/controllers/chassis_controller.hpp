/**
 * @file chassis_controller.hpp
 * @brief 底盘控制器
 *
 * 支持舵轮底盘控制，兼容 DJI 和 LK 两种电机类型
 */

#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "universal_controller/common/config.hpp"
#include "universal_controller/common/types.hpp"
#include "universal_controller/controllers/base_controller.hpp"
#include "universal_controller/tools/pid.hpp"
#include "universal_controller/tools/swerve_kinematics.hpp"

#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/read_lk_motor_multi.hpp"
#include "custom_msgs/msg/read_super_cap.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"
#include "custom_msgs/msg/write_lk_motor_broadcast_current_control.hpp"
#include "custom_msgs/msg/write_super_cap.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <array>
#include <memory>

namespace universal_controller {

/**
 * @brief 底盘控制器
 *
 * 实现舵轮底盘的完整控制功能：
 * - 四轮舵轮运动学解算
 * - 云台-底盘坐标系转换
 * - 小陀螺模式（速度抑制）
 * - 支持 DJI 电机位置控制
 * - 支持 LK 电机级联 PID 控制
 */
class ChassisController : public BaseController {
  public:
    ChassisController();
    ~ChassisController() override = default;

    // ========== BaseController 接口实现 ==========
    void init(rclcpp::Node *node, const ConfigLoader &cfg) override;
    void update(double dt) override;
    void stop() override;
    std::string name() const override {
        return "ChassisController";
    }

    // ========== 指令设置 ==========
    void set_command(const ChassisCommand &cmd);
    void set_motor_type(MotorType type) {
        motor_type_ = type;
    }
    void set_power_limit(double limit);
    void set_chassis_power(double power);

    // ========== 反馈回调 ==========
    void cb_steer_dji(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg);
    void cb_steer_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg);
    void cb_drive_lk(const custom_msgs::msg::ReadLkMotorMulti::SharedPtr msg);
    void cb_yaw(const custom_msgs::msg::ReadLkMotor::SharedPtr msg);
    void cb_supercap(const custom_msgs::msg::ReadSuperCap::SharedPtr msg);

  private:
    // ========== 控制逻辑 ==========
    void compute_control(double dt);
    void publish_dji_commands();
    void publish_lk_commands();
    void apply_power_limit(std::array<int16_t, 4> &drive_currents);
    void apply_dji_power_limit(std::array<double, 4> &drive_speeds);
    void publish_supercap_command(int max_watt, int allow_watt);

    // 坐标转换
    void transform_to_chassis_frame(double vx_g, double vy_g, double &vx_c, double &vy_c) const;

    // 小陀螺抑制
    double compute_spin_suppression(double vx_c, double vy_c, double wz_cmd) const;

    // ========== 配置 ==========
    ChassisConfig config_;
    MotorType motor_type_{MotorType::DJI};

    // ========== 运动学 ==========
    std::unique_ptr<SwerveKinematics> kinematics_;

    // ========== PID 控制器 (LK 模式) ==========
    std::array<PID, 4> steer_angle_pids_;
    std::array<PID, 4> steer_speed_pids_;
    std::array<PID, 4> drive_speed_pids_;

    // ========== 状态变量 ==========
    ChassisCommand command_;
    bool command_valid_{false};

    // 舵向编码器反馈
    std::array<uint16_t, 4> current_steer_ecds_{0, 0, 0, 0};
    std::array<double, 4> current_steer_speeds_{0, 0, 0, 0};
    std::array<double, 4> current_drive_speeds_{0, 0, 0, 0};
    std::array<uint16_t, 4> target_steer_ecds_{0, 0, 0, 0};
    std::array<double, 4> target_drive_speeds_{0, 0, 0, 0};

    // Yaw 电机角度
    double gimbal_yaw_angle_{0.0};
    int yaw_center_ecd_{0};

    // ========== 功率限制状态 ==========
    double power_limit_{80.0};
    double k_dynamic_{0.0131};
    double supercap_chassis_only_power_{0.0};
    bool supercap_online_{false};
    std::array<int16_t, 4> last_drive_currents_{0, 0, 0, 0};

    // DJI 速度模式功率限制（基于裁判系统实测功率反馈）
    double referee_chassis_power_{0.0};
    double dji_power_scale_{1.0};

    // ========== ROS2 接口 ==========
    rclcpp::Subscription<custom_msgs::msg::ReadDJIMotor>::SharedPtr sub_steer_dji_;
    rclcpp::Subscription<custom_msgs::msg::ReadLkMotorMulti>::SharedPtr sub_steer_lk_;
    rclcpp::Subscription<custom_msgs::msg::ReadLkMotorMulti>::SharedPtr sub_drive_lk_;
    rclcpp::Subscription<custom_msgs::msg::ReadLkMotor>::SharedPtr sub_yaw_;
    rclcpp::Subscription<custom_msgs::msg::ReadSuperCap>::SharedPtr sub_supercap_;

    rclcpp::Publisher<custom_msgs::msg::WriteDJIMotor>::SharedPtr pub_drive_dji_;
    rclcpp::Publisher<custom_msgs::msg::WriteDJIMotor>::SharedPtr pub_steer_dji_;
    rclcpp::Publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>::SharedPtr pub_drive_lk_;
    rclcpp::Publisher<custom_msgs::msg::WriteLkMotorBroadcastCurrentControl>::SharedPtr pub_steer_lk_;
    rclcpp::Publisher<custom_msgs::msg::WriteSuperCap>::SharedPtr pub_supercap_;

    rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
};

} // namespace universal_controller
