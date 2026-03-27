/**
 * @file hub.hpp
 * @brief 中心决策 Hub
 *
 * Hub 是 universal_controller 的核心组件，负责：
 * - 接收并聚合所有输入源（RC、自瞄、导航）
 * - 模式仲裁（急停 > 导航 > 自瞄 > 手动）
 * - 分发指令到各子控制器
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <vector>

#include "universal_controller/common/types.hpp"
#include "universal_controller/common/config.hpp"
#include "universal_controller/controllers/chassis_controller.hpp"
#include "universal_controller/controllers/gimbal_controller.hpp"
#include "universal_controller/controllers/fire_controller.hpp"

#include "universal_controller/msg/unified_input.hpp"
#include "sp_msgs/msg/auto_aim_command_msg.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace universal_controller
{

    /**
     * @brief 中心决策 Hub
     */
    class Hub : public rclcpp::Node
    {
    public:
        Hub(std::shared_ptr<ChassisController> chassis,
            std::shared_ptr<GimbalController> gimbal,
            std::shared_ptr<FireController> fire);
        ~Hub() override = default;

        rclcpp::Node::SharedPtr get_chassis_node() const { return chassis_node_; }
        rclcpp::Node::SharedPtr get_gimbal_node() const { return gimbal_node_; }
        rclcpp::Node::SharedPtr get_fire_node() const { return fire_node_; }

    private:
        // ========== 参数 ==========
        void declare_parameters();

        // ========== 控制循环 ==========
        void control_loop();

        // ========== RC 输入融合 ==========
        void rc_fuse();
        void cb_vtm_input(const msg::UnifiedInput::SharedPtr msg);
        void cb_ndj_input(const msg::UnifiedInput::SharedPtr msg);
        bool is_rc_source_connected(const msg::UnifiedInput::SharedPtr &msg, const rclcpp::Time &stamp) const;
        std::optional<msg::UnifiedInput::SharedPtr> get_rc_source_by_name(const std::string &name) const;

        // ========== 外部输入回调 ==========
        void cb_nav_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
        void cb_autoaim(const sp_msgs::msg::AutoAimCommandMsg::SharedPtr msg);
        void cb_referee(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void cb_game_status(const std_msgs::msg::String::SharedPtr msg);

        // ========== 模式仲裁 ==========
        ArbitrationResult arbitrate();
        void dispatch_commands();
        void dispatch_chassis();
        void dispatch_gimbal();
        void dispatch_fire();
        bool is_autoaim_valid() const;
        bool is_nav_vel_valid() const;

        // ========== 控制器 ==========
        std::shared_ptr<ChassisController> chassis_;
        std::shared_ptr<GimbalController> gimbal_;
        std::shared_ptr<FireController> fire_;

        // 控制器独立 ROS 节点
        rclcpp::Node::SharedPtr chassis_node_;
        rclcpp::Node::SharedPtr gimbal_node_;
        rclcpp::Node::SharedPtr fire_node_;

        // ========== 配置 ==========
        ConfigLoader config_loader_;
        ChassisConfig chassis_config_;
        GimbalConfig gimbal_config_;
        FireConfig fire_config_;

        // ========== 状态变量 ==========
        ArbitrationResult arbitration_{};
        msg::UnifiedInput::SharedPtr unified_input_;
        sp_msgs::msg::AutoAimCommandMsg::SharedPtr autoaim_cmd_;
        bool autoaim_valid_{false};
        double autoaim_last_time_{0.0};
        geometry_msgs::msg::Twist::SharedPtr nav_cmd_vel_;
        rclcpp::Time nav_vel_last_time_{0, 0, RCL_ROS_TIME};
        double nav_vel_timeout_s_{0.2};

        RefereeConstraints referee_;
        bool game_started_{false};

        // ========== RC 融合状态 ==========
        msg::UnifiedInput::SharedPtr vtm_input_;
        msg::UnifiedInput::SharedPtr ndj_input_;
        rclcpp::Time vtm_last_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time ndj_last_time_{0, 0, RCL_ROS_TIME};
        std::vector<std::string> rc_priority_{"vtm", "ndj"};
        double rc_connection_timeout_s_{0.5};
        double rc_analog_zero_epsilon_{1e-6};

        // ========== 订阅者 ==========
        rclcpp::Subscription<msg::UnifiedInput>::SharedPtr sub_vtm_;
        rclcpp::Subscription<msg::UnifiedInput>::SharedPtr sub_ndj_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_vel_;
        rclcpp::Subscription<sp_msgs::msg::AutoAimCommandMsg>::SharedPtr sub_autoaim_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_referee_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_game_status_;

        // ========== 定时器 ==========
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_update_time_;

        // ========== QoS ==========
        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
