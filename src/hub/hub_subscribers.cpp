/**
 * @file hub_subscribers.cpp
 * @brief Hub 外部输入订阅回调（自瞄、裁判系统、导航速度）
 */

#include "universal_controller/hub/hub.hpp"

namespace universal_controller {

void Hub::cb_nav_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    nav_cmd_vel_ = msg;
    nav_vel_last_time_ = this->now();
}

void Hub::cb_autoaim(const sp_msgs::msg::AutoAimCommandMsg::SharedPtr msg) {
    autoaim_cmd_ = msg;
    autoaim_last_time_ = this->now().seconds();
    autoaim_valid_ = true;
}

void Hub::cb_gimbal_scan(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
    gimbal_scan_cmd_ = msg;
    gimbal_scan_last_time_ = this->now();
}

void Hub::cb_cmd_spin(const example_interfaces::msg::Float32::SharedPtr msg) {
    nav_spin_speed_ = msg->data;
}

void Hub::cb_referee_constraints(const dji_referee_protocol::msg::Constraints::SharedPtr msg) {
    referee_.heat = msg->shooter_heat;
    referee_.heat_limit = msg->heat_limit;
    referee_.power = msg->chassis_power;
    referee_.power_limit = msg->chassis_power_limit;
    referee_.fire_allowed = msg->fire_allowed;
    referee_.speed_scale = msg->speed_scale;
    referee_.timestamp = this->now().seconds();

    fire_->update_referee_constraints(referee_);
}

void Hub::cb_referee_game_status(const dji_referee_protocol::msg::GameStatus::SharedPtr msg) {
    // 使用常量判断比赛阶段
    using namespace dji_referee_protocol::msg;
    game_started_ = (msg->game_progress == Constants::GAME_STAGE_IN_GAME);
}

} // namespace universal_controller
