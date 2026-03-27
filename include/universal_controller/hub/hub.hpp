/**
 * @file hub.hpp
 * @brief 中心决策 Hub
 *
 * Hub 是 universal_controller 的核心组件，负责：
 * - 接收并聚合所有输入源（RC、自瞄、导航）
 * - 模式仲裁（急停 > 导航 > 自瞄 > 手动）
 * - 分发指令到各子控制器
 * - 提供 Action Server 供导航/行为树调用
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "universal_controller/common/types.hpp"
#include "universal_controller/common/config.hpp"
#include "universal_controller/controllers/chassis_controller.hpp"
#include "universal_controller/controllers/gimbal_controller.hpp"
#include "universal_controller/controllers/fire_controller.hpp"

#include "universal_controller/msg/unified_input.hpp"
#include "universal_controller/action/gimbal_control.hpp"
#include "universal_controller/action/fire_control.hpp"
#include "sp_msgs/msg/auto_aim_command_msg.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace universal_controller
{

    /**
     * @brief 中心决策 Hub
     */
    class Hub : public rclcpp::Node
    {
    public:
        using GimbalControl = universal_controller::action::GimbalControl;
        using GimbalControlGoal = GimbalControl::Goal;
        using GimbalControlResult = GimbalControl::Result;
        using GimbalControlFeedback = GimbalControl::Feedback;

        using FireControl = universal_controller::action::FireControl;
        using FireControlGoal = FireControl::Goal;
        using FireControlResult = FireControl::Result;
        using FireControlFeedback = FireControl::Feedback;

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

        // ========== 输入订阅回调 ==========
        void cb_unified_input(const msg::UnifiedInput::SharedPtr msg);
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

        // ========== Action Server 回调 ==========
        rclcpp_action::GoalResponse handle_gimbal_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const GimbalControlGoal> goal);

        rclcpp_action::CancelResponse handle_gimbal_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>> goal_handle);

        void execute_gimbal_goal(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>> goal_handle);

        rclcpp_action::GoalResponse handle_fire_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FireControlGoal> goal);

        rclcpp_action::CancelResponse handle_fire_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>> goal_handle);

        void execute_fire_goal(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>> goal_handle);

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
        rclcpp::Time last_unified_input_time_{0, 0, RCL_ROS_TIME};
        double unified_input_timeout_s_{0.2};
        sp_msgs::msg::AutoAimCommandMsg::SharedPtr autoaim_cmd_;
        bool autoaim_valid_{false};
        double autoaim_last_time_{0.0};

        RefereeConstraints referee_;
        bool game_started_{false};

        bool gimbal_action_active_{false};
        bool fire_action_active_{false};
        bool chassis_action_active_{false};

        // ========== 订阅者 ==========
        rclcpp::Subscription<msg::UnifiedInput>::SharedPtr sub_unified_;
        rclcpp::Subscription<sp_msgs::msg::AutoAimCommandMsg>::SharedPtr sub_autoaim_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_referee_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_game_status_;

        // ========== Action Servers ==========
        rclcpp_action::Server<GimbalControl>::SharedPtr gimbal_action_server_;
        rclcpp_action::Server<FireControl>::SharedPtr fire_action_server_;

        // ========== 定时器 ==========
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_update_time_;

        // ========== QoS ==========
        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
