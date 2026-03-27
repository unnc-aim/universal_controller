/**
 * @file hub_subscribers.cpp
 * @brief Hub 外部输入订阅回调（自瞄、裁判系统）
 */

#include "universal_controller/hub/hub.hpp"
#include <nlohmann/json.hpp>

namespace universal_controller
{

    void Hub::cb_autoaim(const sp_msgs::msg::AutoAimCommandMsg::SharedPtr msg)
    {
        autoaim_cmd_ = msg;
        autoaim_last_time_ = this->now().seconds();
        autoaim_valid_ = true;
    }

    void Hub::cb_referee(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6)
            return;

        referee_.heat = msg->data[0];
        referee_.heat_limit = msg->data[1];
        referee_.power = msg->data[2];
        referee_.power_limit = msg->data[3];
        referee_.fire_allowed = (msg->data[4] > 0.5);
        referee_.speed_scale = msg->data[5];
        referee_.timestamp = this->now().seconds();

        fire_->update_referee_constraints(referee_);
    }

    void Hub::cb_game_status(const std_msgs::msg::String::SharedPtr msg)
    {
        try
        {
            auto parsed = nlohmann::json::parse(msg->data);
            if (parsed.contains("data") && parsed["data"].contains("game_progress"))
            {
                int progress = parsed["data"]["game_progress"];
                game_started_ = (progress == 4); // 4 = 比赛中
            }
        }
        catch (...)
        {
        }
    }

} // namespace universal_controller
