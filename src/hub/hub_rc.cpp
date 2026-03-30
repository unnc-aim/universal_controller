/**
 * @file hub_rc.cpp
 * @brief Hub RC 输入融合（原 RCHub，已合并入 Hub）
 */

#include "universal_controller/hub/hub.hpp"
#include <cmath>

namespace universal_controller {

void Hub::cb_vtm_input(const msg::UnifiedInput::SharedPtr msg) {
    vtm_input_ = msg;
    vtm_last_time_ = this->now();
}

void Hub::cb_ndj_input(const msg::UnifiedInput::SharedPtr msg) {
    ndj_input_ = msg;
    ndj_last_time_ = this->now();
}

bool Hub::is_rc_source_connected(const msg::UnifiedInput::SharedPtr &msg, const rclcpp::Time &stamp) const {
    if (!msg || !msg->connected) {
        return false;
    }
    if (stamp.nanoseconds() == 0) {
        return false;
    }
    return (this->now() - stamp).seconds() <= rc_connection_timeout_s_;
}

std::optional<msg::UnifiedInput::SharedPtr> Hub::get_rc_source_by_name(const std::string &name) const {
    if (name == "vtm" && is_rc_source_connected(vtm_input_, vtm_last_time_)) {
        return vtm_input_;
    }
    if (name == "ndj" && is_rc_source_connected(ndj_input_, ndj_last_time_)) {
        return ndj_input_;
    }
    return std::nullopt;
}

void Hub::rc_fuse() {
    // 按优先级选择主输入源
    std::optional<msg::UnifiedInput::SharedPtr> selected;
    std::string selected_name;

    for (const auto &name : rc_priority_) {
        auto src = get_rc_source_by_name(name);
        if (src.has_value()) {
            selected = src;
            selected_name = name;
            break;
        }
    }

    if (!selected.has_value()) {
        unified_input_.reset();
        return;
    }

    // 复制选中源作为基础
    auto out = std::make_shared<msg::UnifiedInput>(*selected.value());
    out->header.stamp = this->now();
    out->control_source = selected_name;

    // 连续量允许在高优先级为 0 时由低优先级覆盖
    auto choose_analog = [&](auto getter) -> double {
        for (const auto &name : rc_priority_) {
            auto src = get_rc_source_by_name(name);
            if (!src.has_value()) {
                continue;
            }
            const double value = getter(*src.value());
            if (std::fabs(value) > rc_analog_zero_epsilon_) {
                return value;
            }
        }
        return 0.0;
    };

    out->vx = choose_analog([](const msg::UnifiedInput &m) { return m.vx; });
    out->vy = choose_analog([](const msg::UnifiedInput &m) { return m.vy; });
    out->wz = choose_analog([](const msg::UnifiedInput &m) { return m.wz; });
    out->spin_speed = choose_analog([](const msg::UnifiedInput &m) { return m.spin_speed; });
    out->pitch_delta = choose_analog([](const msg::UnifiedInput &m) { return m.pitch_delta; });
    out->yaw_delta = choose_analog([](const msg::UnifiedInput &m) { return m.yaw_delta; });
    out->chassis_speed_scale = choose_analog([](const msg::UnifiedInput &m) { return m.chassis_speed_scale; });

    unified_input_ = out;
}

} // namespace universal_controller
