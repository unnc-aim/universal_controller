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

        // 订阅 RC 输入源（VTM / NDJ）
        std::string topic_vtm = config_loader_.get_string("rc_hub.topic_vtm_input", "/universal_controller/input/vtm");
        std::string topic_ndj = config_loader_.get_string("rc_hub.topic_ndj_input", "/universal_controller/input/ndj");
        sub_vtm_ = this->create_subscription<msg::UnifiedInput>(
            topic_vtm, qos_best_effort_,
            std::bind(&Hub::cb_vtm_input, this, std::placeholders::_1));
        sub_ndj_ = this->create_subscription<msg::UnifiedInput>(
            topic_ndj, qos_best_effort_,
            std::bind(&Hub::cb_ndj_input, this, std::placeholders::_1));

        // 订阅自瞄指令
        std::string topic_autoaim = config_loader_.get_string("topics.autoaim_cmd", "/sp_vision/autoaim_command");
        sub_autoaim_ = this->create_subscription<sp_msgs::msg::AutoAimCommandMsg>(
            topic_autoaim, qos_best_effort_,
            std::bind(&Hub::cb_autoaim, this, std::placeholders::_1));

        // 订阅导航速度指令
        std::string topic_nav_vel = config_loader_.get_string("topics.nav_cmd_vel", "/cmd_vel");
        sub_nav_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            topic_nav_vel, qos_best_effort_,
            std::bind(&Hub::cb_nav_vel, this, std::placeholders::_1));

        // 订阅裁判系统
        std::string topic_referee = config_loader_.get_string("topics.referee_constraints", "/referee/parsed/common/constraints");
        sub_referee_ = this->create_subscription<dji_referee_protocol::msg::Constraints>(
            topic_referee, qos_best_effort_,
            std::bind(&Hub::cb_referee_constraints, this, std::placeholders::_1));

        std::string topic_game_status = config_loader_.get_string("topics.referee_game_status", "/referee/common/game_status");
        sub_game_status_ = this->create_subscription<dji_referee_protocol::msg::GameStatus>(
            topic_game_status, qos_best_effort_,
            std::bind(&Hub::cb_referee_game_status, this, std::placeholders::_1));

        // 控制定时器（1000Hz）
        int freq = config_loader_.get_int("control_frequency", 1000);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000000 / freq),
            std::bind(&Hub::control_loop, this));

        last_update_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Hub started @ %d Hz, vtm=%s, ndj=%s", freq, topic_vtm.c_str(), topic_ndj.c_str());
    }

    void Hub::declare_parameters()
    {
        // 控制参数
        this->declare_parameter("control_frequency", 1000);
        this->declare_parameter("topics.autoaim_cmd", "/sp_vision/autoaim_command");
        this->declare_parameter("topics.referee_constraints", "/referee/parsed/common/constraints");
        this->declare_parameter("topics.referee_game_status", "/referee/common/game_status");
        this->declare_parameter("autoaim_timeout_s", 0.2);
        this->declare_parameter("referee_timeout_s", 0.5);
        this->declare_parameter("nav_vel_timeout_s", 0.2);
        this->declare_parameter("topics.nav_cmd_vel", "/cmd_vel");

        // RC 融合参数
        this->declare_parameter("rc_hub.topic_vtm_input", "/universal_controller/input/vtm");
        this->declare_parameter("rc_hub.topic_ndj_input", "/universal_controller/input/ndj");
        this->declare_parameter("rc_hub.connection_timeout_s", 0.5);
        this->declare_parameter("rc_hub.priority", std::vector<std::string>{"vtm", "ndj"});
        this->declare_parameter("rc_hub.analog_zero_epsilon", 1e-6);

        // 加载 RC 融合参数
        rc_connection_timeout_s_ = this->get_parameter("rc_hub.connection_timeout_s").as_double();
        rc_priority_ = this->get_parameter("rc_hub.priority").as_string_array();
        rc_analog_zero_epsilon_ = this->get_parameter("rc_hub.analog_zero_epsilon").as_double();
        if (rc_priority_.empty())
        {
            rc_priority_ = {"vtm", "ndj"};
        }
    }

    void Hub::control_loop()
    {
        auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        // RC 融合（直接在控制循环中执行）
        rc_fuse();

        // 模式仲裁
        arbitration_ = arbitrate();

        // 分发指令
        dispatch_commands();

        if (arbitration_.emergency_stop)
        {
            return;
        }
        // 更新各控制器
        chassis_->update(dt);
        gimbal_->update(dt);
        fire_->update(dt);
    }

} // namespace universal_controller
