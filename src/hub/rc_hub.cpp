/**
 * @file rc_hub.cpp
 * @brief 遥控输入融合节点实现
 */

#include "universal_controller/hub/rc_hub.hpp"

#include <cmath>

namespace universal_controller
{

    RCHub::RCHub(const rclcpp::NodeOptions &options)
        : Node("rc_hub", options)
    {
        declare_parameters();
        load_parameters();

        sub_vtm_ = this->create_subscription<msg::UnifiedInput>(
            topic_vtm_input_, qos_best_effort_,
            std::bind(&RCHub::cb_vtm_input, this, std::placeholders::_1));

        sub_ndj_ = this->create_subscription<msg::UnifiedInput>(
            topic_ndj_input_, qos_best_effort_,
            std::bind(&RCHub::cb_ndj_input, this, std::placeholders::_1));

        pub_unified_ = this->create_publisher<msg::UnifiedInput>(
            topic_unified_output_, qos_best_effort_);

        const int period_us = std::max(1, 1000000 / std::max(1, publish_frequency_));
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(period_us),
            std::bind(&RCHub::timer_publish, this));

        RCLCPP_INFO(this->get_logger(),
                    "RC Hub started: vtm_in=%s, ndj_in=%s, unified_out=%s, freq=%d Hz",
                    topic_vtm_input_.c_str(), topic_ndj_input_.c_str(),
                    topic_unified_output_.c_str(), publish_frequency_);
    }

    void RCHub::declare_parameters()
    {
        this->declare_parameter("rc_hub.topic_vtm_input", "/universal_controller/rc_hub/vtm");
        this->declare_parameter("rc_hub.topic_ndj_input", "/universal_controller/rc_hub/ndj");
        this->declare_parameter("rc_hub.topic_unified_output", "/universal_controller/hub/rc_unified_input");
        this->declare_parameter("rc_hub.connection_timeout_s", 0.5);
        this->declare_parameter("rc_hub.priority", std::vector<std::string>{"vtm", "ndj"});
        this->declare_parameter("rc_hub.analog_zero_epsilon", 1e-6);
        this->declare_parameter("rc_hub.publish_frequency", 200);
    }

    void RCHub::load_parameters()
    {
        topic_vtm_input_ = this->get_parameter("rc_hub.topic_vtm_input").as_string();
        topic_ndj_input_ = this->get_parameter("rc_hub.topic_ndj_input").as_string();
        topic_unified_output_ = this->get_parameter("rc_hub.topic_unified_output").as_string();
        connection_timeout_s_ = this->get_parameter("rc_hub.connection_timeout_s").as_double();
        priority_ = this->get_parameter("rc_hub.priority").as_string_array();
        analog_zero_epsilon_ = this->get_parameter("rc_hub.analog_zero_epsilon").as_double();
        publish_frequency_ = this->get_parameter("rc_hub.publish_frequency").as_int();

        if (priority_.empty())
        {
            priority_ = {"vtm", "ndj"};
        }
    }

    void RCHub::cb_vtm_input(const msg::UnifiedInput::SharedPtr msg)
    {
        vtm_input_ = msg;
        vtm_last_time_ = this->now();
    }

    void RCHub::cb_ndj_input(const msg::UnifiedInput::SharedPtr msg)
    {
        ndj_input_ = msg;
        ndj_last_time_ = this->now();
    }

    bool RCHub::is_source_connected(const msg::UnifiedInput::SharedPtr &msg, const rclcpp::Time &stamp) const
    {
        if (!msg || !msg->connected)
        {
            return false;
        }
        if (stamp.nanoseconds() == 0)
        {
            return false;
        }
        return (this->now() - stamp).seconds() <= connection_timeout_s_;
    }

    std::optional<msg::UnifiedInput::SharedPtr> RCHub::get_source_by_name(const std::string &name) const
    {
        if (name == "vtm" && is_source_connected(vtm_input_, vtm_last_time_))
        {
            return vtm_input_;
        }
        if (name == "ndj" && is_source_connected(ndj_input_, ndj_last_time_))
        {
            return ndj_input_;
        }
        return std::nullopt;
    }

    void RCHub::timer_publish()
    {
        std::optional<msg::UnifiedInput::SharedPtr> selected;
        std::string selected_name;

        for (const auto &name : priority_)
        {
            auto src = get_source_by_name(name);
            if (src.has_value())
            {
                selected = src;
                selected_name = name;
                break;
            }
        }

        msg::UnifiedInput out;
        out.header.stamp = this->now();

        if (!selected.has_value())
        {
            out.control_source = "none";
            out.connected = false;
            out.emergency_stop = true;
            pub_unified_->publish(out);
            return;
        }

        out = *selected.value();
        out.header.stamp = this->now();
        out.control_source = selected_name;

        auto choose_analog = [&](auto getter) -> double
        {
            for (const auto &name : priority_)
            {
                auto src = get_source_by_name(name);
                if (!src.has_value())
                {
                    continue;
                }
                const double value = getter(*src.value());
                if (std::fabs(value) > analog_zero_epsilon_)
                {
                    return value;
                }
            }
            return 0.0;
        };

        // 连续量允许在高优先级为 0 时由低优先级覆盖。
        out.vx = choose_analog([](const msg::UnifiedInput &m)
                               { return m.vx; });
        out.vy = choose_analog([](const msg::UnifiedInput &m)
                               { return m.vy; });
        out.wz = choose_analog([](const msg::UnifiedInput &m)
                               { return m.wz; });
        out.spin_speed = choose_analog([](const msg::UnifiedInput &m)
                                       { return m.spin_speed; });
        out.pitch_delta = choose_analog([](const msg::UnifiedInput &m)
                                        { return m.pitch_delta; });
        out.yaw_delta = choose_analog([](const msg::UnifiedInput &m)
                                      { return m.yaw_delta; });
        out.chassis_speed_scale = choose_analog([](const msg::UnifiedInput &m)
                                                { return m.chassis_speed_scale; });

        pub_unified_->publish(out);
    }

} // namespace universal_controller
