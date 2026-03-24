/**
 * @file hub.cpp
 * @brief 中心决策 Hub 实现
 */

#include "universal_controller_framework/hub/hub.hpp"
#include <nlohmann/json.hpp>
#include <cmath>

namespace universal_controller {

Hub::Hub() : Node("universal_controller_hub") {
    declare_parameters();

    // 初始化配置加载器
    config_loader_ = ConfigLoader(this);

    // 加载各控制器配置
    chassis_config_.load(config_loader_);
    gimbal_config_.load(config_loader_);
    fire_config_.load(config_loader_);

    // 初始化控制器
    chassis_ = std::make_unique<ChassisController>();
    chassis_->init(this, config_loader_);

    gimbal_ = std::make_unique<GimbalController>();
    gimbal_->init(this, config_loader_);

    fire_ = std::make_unique<FireController>();
    fire_->init(this, config_loader_);

    // 订阅统一输入
    std::string topic_unified = config_loader_.get_string("topic_unified_input", "/universal_controller/unified_input");
    sub_unified_ = this->create_subscription<msg::UnifiedInput>(
        topic_unified, qos_best_effort_,
        std::bind(&Hub::cb_unified_input, this, std::placeholders::_1));

    // 订阅自瞄指令
    std::string topic_autoaim = config_loader_.get_string("topic_autoaim_cmd", "/sp_vision/autoaim_command");
    sub_autoaim_ = this->create_subscription<sp_msgs::msg::AutoAimCommandMsg>(
        topic_autoaim, qos_best_effort_,
        std::bind(&Hub::cb_autoaim, this, std::placeholders::_1));

    // 订阅裁判系统
    std::string topic_referee = config_loader_.get_string("topic_referee_constraints", "/referee/constraints");
    sub_referee_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        topic_referee, qos_best_effort_,
        std::bind(&Hub::cb_referee, this, std::placeholders::_1));

    std::string topic_game_status = config_loader_.get_string("topic_referee_game_status", "/referee/game_status");
    sub_game_status_ = this->create_subscription<std_msgs::msg::String>(
        topic_game_status, qos_best_effort_,
        std::bind(&Hub::cb_game_status, this, std::placeholders::_1));

    // Action Servers
    gimbal_action_server_ = rclcpp_action::create_server<GimbalControl>(
        this, "/universal_controller/gimbal_control",
        std::bind(&Hub::handle_gimbal_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Hub::handle_gimbal_cancel, this, std::placeholders::_1),
        std::bind(&Hub::execute_gimbal_goal, this, std::placeholders::_1));

    fire_action_server_ = rclcpp_action::create_server<FireControl>(
        this, "/universal_controller/fire_control",
        std::bind(&Hub::handle_fire_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Hub::handle_fire_cancel, this, std::placeholders::_1),
        std::bind(&Hub::execute_fire_goal, this, std::placeholders::_1));

    // 控制定时器（1000Hz）
    int freq = config_loader_.get_int("control_frequency", 1000);
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / freq),
        std::bind(&Hub::control_loop, this));

    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Hub started @ %d Hz", freq);
}

void Hub::declare_parameters() {
    this->declare_parameter("control_frequency", 1000);
    this->declare_parameter("topic_unified_input", "/universal_controller/unified_input");
    this->declare_parameter("topic_autoaim_cmd", "/sp_vision/autoaim_command");
    this->declare_parameter("topic_referee_constraints", "/referee/constraints");
    this->declare_parameter("topic_referee_game_status", "/referee/game_status");
    this->declare_parameter("autoaim_timeout_s", 0.2);
    this->declare_parameter("referee_timeout_s", 0.5);
}

void Hub::control_loop() {
    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // 模式仲裁
    current_mode_ = arbitrate_mode();

    // 分发指令
    dispatch_commands();

    // 更新各控制器
    chassis_->update(dt);
    gimbal_->update(dt);
    fire_->update(dt);
}

void Hub::cb_unified_input(const msg::UnifiedInput::SharedPtr msg) {
    unified_input_ = msg;
}

void Hub::cb_autoaim(const sp_msgs::msg::AutoAimCommandMsg::SharedPtr msg) {
    autoaim_cmd_ = msg;
    autoaim_last_time_ = this->now().seconds();
    autoaim_valid_ = true;
}

void Hub::cb_referee(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) return;

    referee_.heat = msg->data[0];
    referee_.heat_limit = msg->data[1];
    referee_.power = msg->data[2];
    referee_.power_limit = msg->data[3];
    referee_.fire_allowed = (msg->data[4] > 0.5);
    referee_.speed_scale = msg->data[5];
    referee_.timestamp = this->now().seconds();

    fire_->update_referee_constraints(referee_);
}

void Hub::cb_game_status(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto parsed = nlohmann::json::parse(msg->data);
        if (parsed.contains("data") && parsed["data"].contains("game_progress")) {
            int progress = parsed["data"]["game_progress"];
            game_started_ = (progress == 4);  // 4 = 比赛中
        }
    } catch (...) {}
}

ControlMode Hub::arbitrate_mode() {
    if (!unified_input_) {
        return ControlMode::EMERGENCY_STOP;
    }

    // 1. 急停最高优先级
    if (unified_input_->emergency_stop || !unified_input_->rc_connected) {
        return ControlMode::EMERGENCY_STOP;
    }

    // 2. 导航/行为树 Action 优先
    if ((gimbal_action_active_ || fire_action_active_) && unified_input_->navigation_enabled) {
        return ControlMode::NAVIGATION;
    }

    // 3. 自瞄模式（左拨杆上档 + 自瞄有效 + 超时未过）
    double autoaim_timeout = this->get_parameter("autoaim_timeout_s").as_double();
    bool autoaim_fresh = (this->now().seconds() - autoaim_last_time_) < autoaim_timeout;

    if (unified_input_->autoaim_enabled && autoaim_cmd_ && autoaim_cmd_->control && autoaim_fresh) {
        return ControlMode::AUTOAIM;
    }

    return ControlMode::MANUAL;
}

void Hub::dispatch_commands() {
    if (!unified_input_) return;

    switch (current_mode_) {
        case ControlMode::EMERGENCY_STOP:
            chassis_->stop();
            gimbal_->stop();
            fire_->stop();
            break;

        case ControlMode::MANUAL:
        case ControlMode::AUTOAIM:
            dispatch_manual_or_autoaim();
            break;

        case ControlMode::NAVIGATION:
            dispatch_navigation();
            break;

        default:
            break;
    }
}

void Hub::dispatch_manual_or_autoaim() {
    // 底盘指令
    ChassisCommand chassis_cmd;
    chassis_cmd.vx_gimbal = unified_input_->vx;
    chassis_cmd.vy_gimbal = unified_input_->vy;
    chassis_cmd.wz = unified_input_->wz;
    chassis_cmd.spin_mode = unified_input_->spin_mode;
    chassis_cmd.spin_speed = unified_input_->spin_speed;
    chassis_->set_command(chassis_cmd);

    // 云台指令
    GimbalCommand gimbal_cmd;

    if (current_mode_ == ControlMode::AUTOAIM && autoaim_cmd_) {
        // 自瞄模式：使用自瞄输出
        gimbal_cmd.yaw_rad = autoaim_cmd_->yaw;
        gimbal_cmd.pitch_deg = -autoaim_cmd_->pitch * (180.0 / M_PI);
        gimbal_cmd.autoaim_enabled = true;
        gimbal_cmd.absolute = true;
    } else {
        // 手动模式：使用遥控器输入
        gimbal_cmd.pitch_deg = unified_input_->target_pitch;
        gimbal_cmd.yaw_rad = unified_input_->target_yaw;
        gimbal_cmd.autoaim_enabled = false;
        gimbal_cmd.absolute = !unified_input_->gimbal_absolute;
    }
    gimbal_->set_command(gimbal_cmd);

    // 发射指令
    FireCommand fire_cmd;
    fire_cmd.friction_on = unified_input_->friction_on;
    fire_cmd.trigger_fire = unified_input_->fire_trigger;
    fire_cmd.burst_mode = unified_input_->burst_mode;
    fire_cmd.friction_speed = unified_input_->friction_speed;
    fire_->set_command(fire_cmd);
}

void Hub::dispatch_navigation() {
    // 导航模式下，指令通过 Action 设置，这里不做额外处理
}

rclcpp_action::GoalResponse Hub::handle_gimbal_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const GimbalControlGoal> goal) {

    RCLCPP_INFO(this->get_logger(), "Gimbal goal received: yaw=%.2f, pitch=%.2f",
                goal->target_yaw_rad, goal->target_pitch_deg);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Hub::handle_gimbal_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>>) {
    RCLCPP_INFO(this->get_logger(), "Gimbal goal canceled");
    gimbal_action_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Hub::execute_gimbal_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GimbalControl>> goal_handle) {

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

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
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
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FireControlGoal> goal) {

    RCLCPP_INFO(this->get_logger(), "Fire goal received: fire=%d, mode=%d",
                goal->fire, goal->fire_mode);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Hub::handle_fire_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>>) {
    RCLCPP_INFO(this->get_logger(), "Fire goal canceled");
    fire_action_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Hub::execute_fire_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FireControl>> goal_handle) {

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

}  // namespace universal_controller
