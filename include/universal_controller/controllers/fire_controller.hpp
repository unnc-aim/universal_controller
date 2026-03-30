/**
 * @file fire_controller.hpp
 * @brief 发射控制器
 *
 * 实现摩擦轮和拨盘的完整控制：
 * - 摩擦轮速度控制
 * - 拨盘串级 PID 控制
 * - 裁判系统约束
 */

#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include "universal_controller/common/config.hpp"
#include "universal_controller/common/types.hpp"
#include "universal_controller/controllers/base_controller.hpp"
#include "universal_controller/tools/pid.hpp"

#include "custom_msgs/msg/read_dji_motor.hpp"
#include "custom_msgs/msg/write_dji_motor.hpp"

namespace universal_controller {

/**
 * @brief 发射控制器
 */
class FireController : public BaseController {
  public:
    FireController();
    ~FireController() override = default;

    void init(rclcpp::Node *node, const ConfigLoader &cfg) override;
    void update(double dt) override;
    void stop() override;
    std::string name() const override {
        return "FireController";
    }

    void set_command(const FireCommand &cmd);

    // 裁判系统约束更新
    void update_referee_constraints(const RefereeConstraints &constraints);

  private:
    void cb_motor_feedback(const custom_msgs::msg::ReadDJIMotor::SharedPtr msg);
    void compute_control(double dt);
    void publish_commands();
    void stop_all();

    // 状态机
    void update_state_machine(double dt);

    // 配置
    FireConfig config_;

    // PID 控制器
    std::unique_ptr<PID> pid_trigger_pos_;
    std::unique_ptr<PID> pid_trigger_spd_;

    // 指令
    FireCommand command_;
    bool command_valid_{false};

    // 状态机
    FeederState feeder_state_{FeederState::IDLE};

    // 拨盘状态
    double trigger_target_ecd_{0.0};
    double total_ecd_{0.0};  // 多圈累计位置
    double motor3_ecd_{0.0}; // 单圈位置
    int16_t motor3_rpm_{0};
    int16_t motor3_current_{0};
    bool motor_initialized_{false};

    // 发射控制
    bool burst_mode_{false};
    bool trigger_has_fired_{false};
    double last_shot_time_{0.0};

    // 摩擦轮
    double friction_speed_target_{6500.0};

    // 裁判系统约束
    RefereeConstraints referee_;

    // ROS2 接口
    rclcpp::Subscription<custom_msgs::msg::ReadDJIMotor>::SharedPtr sub_motor_;
    rclcpp::Publisher<custom_msgs::msg::WriteDJIMotor>::SharedPtr pub_motor_;
    rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
};

} // namespace universal_controller
