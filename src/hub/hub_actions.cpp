/**
 * @file hub_actions.cpp
 * @brief Hub Action Server 回调
 */

#include "universal_controller/hub/hub.hpp"

namespace universal_controller
{

    rclcpp_action::GoalResponse Hub::handle_gimbal_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GimbalControlGoal> goal)
    {

        RCLCPP_INFO(this->get_logger(), "Gimbal goal received: yaw=%.2f, pitch=%.2f",
                    goal->target_yaw_rad, goal->target_pitch_deg);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse Hub::handle_gimbal_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>>)
    {
        RCLCPP_INFO(this->get_logger(), "Gimbal goal canceled");
        gimbal_action_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Hub::execute_gimbal_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>> goal_handle)
    {

        gimbal_action_active_ = true;
        auto goal = goal_handle->get_goal();

        GimbalCommand cmd;
        cmd.yaw_rad = goal->target_yaw_rad;
        cmd.pitch_deg = goal->target_pitch_deg;
        cmd.from_action = true;
        cmd.absolute = !goal->relative_mode;
        gimbal_->set_command(cmd);

        auto result = std::make_shared<GimbalControlResult>();
        rclcpp::Rate rate(100);

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                gimbal_action_active_ = false;
                return;
            }

            // 发布反馈
            auto feedback = std::make_shared<GimbalControlFeedback>();
            // TODO: 填充当前角度
            goal_handle->publish_feedback(feedback);

            // 检查是否到达目标
            // TODO: 实现到达检测

            rate.sleep();
        }

        result->success = true;
        goal_handle->succeed(result);
        gimbal_action_active_ = false;
    }

    rclcpp_action::GoalResponse Hub::handle_fire_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FireControlGoal> goal)
    {

        RCLCPP_INFO(this->get_logger(), "Fire goal received: fire=%d, mode=%d",
                    goal->fire, goal->fire_mode);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse Hub::handle_fire_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>>)
    {
        RCLCPP_INFO(this->get_logger(), "Fire goal canceled");
        fire_action_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Hub::execute_fire_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>> goal_handle)
    {

        fire_action_active_ = true;
        auto goal = goal_handle->get_goal();

        FireCommand cmd;
        cmd.friction_on = true;
        cmd.trigger_fire = goal->fire;
        cmd.burst_mode = (goal->fire_mode == 1 || goal->fire_mode == 2);
        cmd.from_action = true;
        fire_->set_command(cmd);

        auto result = std::make_shared<FireControlResult>();
        // TODO: 实现完整的发射 Action 执行逻辑

        result->success = true;
        goal_handle->succeed(result);
        fire_action_active_ = false;
    }

} // namespace universal_controller
