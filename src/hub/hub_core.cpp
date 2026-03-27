/**
 * @file hub_core.cpp
 * @brief Hub 构造、参数声明、控制循环
 */

#include "universal_controller/hub/hub.hpp"
#include <stdexcept>

namespace universal_controller
{

    Hub::Hub(std::shared_ptr<ChassisController> chassis,
             std::shared_ptr<GimbalController> gimbal,
             std::shared_ptr<FireController> fire)
        : Node("universal_controller_hub"),
          chassis_(std::move(chassis)),
          gimbal_(std::move(gimbal)),
          fire_(std::move(fire)),
          config_loader_(this)
    {
        declare_parameters();

        if (!chassis_ || !gimbal_ || !fire_)
        {
            throw std::invalid_argument("Hub requires non-null chassis, gimbal, and fire controllers");
        }

        // 控制器节点独立出来，便于在 main 中作为单独节点统一注册。
        chassis_node_ = std::make_shared<rclcpp::Node>("chassis_controller_node");
        gimbal_node_ = std::make_shared<rclcpp::Node>("gimbal_controller_node");
        fire_node_ = std::make_shared<rclcpp::Node>("fire_controller_node");

        // 加载各控制器配置
        chassis_config_.load(config_loader_);
        gimbal_config_.load(config_loader_);
        fire_config_.load(config_loader_);

        // 初始化控制器
        chassis_->init(chassis_node_.get(), config_loader_);

        gimbal_->init(gimbal_node_.get(), config_loader_);

        fire_->init(fire_node_.get(), config_loader_);

        // 订阅 RC Hub 输出的统一输入
        std::string topic_unified = config_loader_.get_string("topics.unified_input", "/universal_controller/hub/rc_unified_input");
        sub_unified_ = this->create_subscription<msg::UnifiedInput>(
            topic_unified, qos_best_effort_,
            std::bind(&Hub::cb_unified_input, this, std::placeholders::_1));

        // 订阅自瞄指令
        std::string topic_autoaim = config_loader_.get_string("topics.autoaim_cmd", "/sp_vision/autoaim_command");
        sub_autoaim_ = this->create_subscription<sp_msgs::msg::AutoAimCommandMsg>(
            topic_autoaim, qos_best_effort_,
            std::bind(&Hub::cb_autoaim, this, std::placeholders::_1));

        // 订阅裁判系统
        std::string topic_referee = config_loader_.get_string("topics.referee_constraints", "/referee/constraints");
        sub_referee_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            topic_referee, qos_best_effort_,
            std::bind(&Hub::cb_referee, this, std::placeholders::_1));

        std::string topic_game_status = config_loader_.get_string("topics.referee_game_status", "/referee/game_status");
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

        RCLCPP_INFO(this->get_logger(), "Hub started @ %d Hz, unified_input=%s", freq, topic_unified.c_str());
    }

    void Hub::declare_parameters()
    {
        this->declare_parameter("control_frequency", 1000);
        this->declare_parameter("topics.unified_input", "/universal_controller/hub/rc_unified_input");
        this->declare_parameter("topics.unified_input_timeout_s", 0.2);
        this->declare_parameter("topics.autoaim_cmd", "/sp_vision/autoaim_command");
        this->declare_parameter("topics.referee_constraints", "/referee/constraints");
        this->declare_parameter("topics.referee_game_status", "/referee/game_status");
        this->declare_parameter("autoaim_timeout_s", 0.2);
        this->declare_parameter("referee_timeout_s", 0.5);

        unified_input_timeout_s_ = this->get_parameter("topics.unified_input_timeout_s").as_double();
    }

    void Hub::control_loop()
    {
        auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        if (last_unified_input_time_.nanoseconds() == 0 ||
            (now - last_unified_input_time_).seconds() > unified_input_timeout_s_)
        {
            unified_input_.reset();
        }

        // 模式仲裁
        arbitration_ = arbitrate();

        // 分发指令
        dispatch_commands();

        // 更新各控制器
        chassis_->update(dt);
        gimbal_->update(dt);
        fire_->update(dt);
    }

} // namespace universal_controller
