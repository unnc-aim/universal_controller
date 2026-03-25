/**
 * @file rc_hub.hpp
 * @brief 遥控输入融合节点
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "universal_controller/msg/unified_input.hpp"

#include <optional>
#include <string>
#include <vector>

namespace universal_controller
{

    class RCHub : public rclcpp::Node
    {
    public:
        explicit RCHub(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RCHub() override = default;

    private:
        void declare_parameters();
        void load_parameters();

        void cb_vtm_input(const msg::UnifiedInput::SharedPtr msg);
        void cb_ndj_input(const msg::UnifiedInput::SharedPtr msg);

        void timer_publish();
        bool is_source_connected(const msg::UnifiedInput::SharedPtr &msg, const rclcpp::Time &stamp) const;
        std::optional<msg::UnifiedInput::SharedPtr> get_source_by_name(const std::string &name) const;

        // 订阅/发布
        rclcpp::Subscription<msg::UnifiedInput>::SharedPtr sub_vtm_;
        rclcpp::Subscription<msg::UnifiedInput>::SharedPtr sub_ndj_;
        rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;
        rclcpp::TimerBase::SharedPtr timer_;

        // 输入缓存
        msg::UnifiedInput::SharedPtr vtm_input_;
        msg::UnifiedInput::SharedPtr ndj_input_;
        rclcpp::Time vtm_last_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time ndj_last_time_{0, 0, RCL_ROS_TIME};

        // 参数
        std::string topic_vtm_input_;
        std::string topic_ndj_input_;
        std::string topic_unified_output_;
        std::vector<std::string> priority_{"vtm", "ndj"};
        double connection_timeout_s_{0.5};
        double analog_zero_epsilon_{1e-6};
        int publish_frequency_{200};

        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
