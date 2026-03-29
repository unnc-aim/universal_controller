/**
 * @file keyboard_mouse_parser.hpp
 * @brief 统一键鼠解析器
 *
 * 处理纯键盘和鼠标输入，生成统一的输出。
 * 不处理摇杆、拨轮等 RC 硬件控件（由各 interpreter 自行处理）。
 */

#pragma once

#include "universal_controller/common/types.hpp"
#include "universal_controller/tools/input_processor.hpp"

#include <algorithm>
#include <cmath>

namespace universal_controller {

class KeyboardMouseParser {
  public:
    explicit KeyboardMouseParser(const InputProcessorConfig &config = InputProcessorConfig{});

    /**
     * @brief 解析键鼠输入
     * @param input 统一键鼠输入（由 interpreter 的 mapper 填充）
     * @return 键鼠解析输出
     */
    KeyboardMouseOutput parse(const KeyboardMouseInput &input);

    /**
     * @brief 更新小陀螺速度（由 interpreter 的拨轮调用）
     */
    void update_spin_speed(double dial_value, bool shift, bool ctrl);

    void set_spin_mode(bool enabled);
    bool get_spin_mode() const;
    double get_spin_speed() const;
    double get_speed_scale() const;
    void update_config(const InputProcessorConfig &config);

  private:
    InputProcessorConfig config_;
    double current_spd_mode_;
    double spin_spd_;
    bool spin_mode_{false};

    static double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }
};

} // namespace universal_controller
