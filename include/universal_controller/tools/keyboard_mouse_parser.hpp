/**
 * @file keyboard_mouse_parser.hpp
 * @brief 统一键鼠解析器
 *
 * 处理纯键盘和鼠标输入，生成统一的输出。
 * 不处理摇杆、拨轮等 RC 硬件控件（由各 interpreter 自行处理）。
 *
 * 支持通过 YAML (km_definition.yaml) 配置按钮事件映射，
 * 包括 on_press/on_release/on_long_press 等，同 VTM 按钮模型。
 */

#pragma once

#include "universal_controller/common/types.hpp"
#include "universal_controller/tools/input_processor.hpp"
#include "universal_controller/tools/rc_action_types.hpp"

#include <algorithm>
#include <cmath>
#include <string>

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
     * @brief 处理按钮事件（边沿检测 + 长按），返回累积 ActionSet
     * @param input 当前帧键鼠输入
     * @param current_time_s 当前时间（秒），用于长按检测
     * @return 累积的 ActionSet（由 interpreter 执行）
     */
    ActionSet process_button_events(const KeyboardMouseInput &input, double current_time_s);

    /**
     * @brief 加载键鼠 YAML 定义文件
     * @param file_path YAML 文件路径
     * @return true 加载成功
     */
    bool load_definition(const std::string &file_path);

    /** @brief KM 定义是否已加载 */
    bool definition_loaded() const;

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

    // KM YAML 定义
    KMTriggerDefinition km_def_;

    // 按钮状态跟踪
    struct BtnState {
        bool last_pressed{false};
        double press_start_time_s{0.0};
        bool long_press_active{false};
    };

    BtnState btn_shift_;
    BtnState btn_ctrl_;
    BtnState btn_q_, btn_e_, btn_r_, btn_f_, btn_g_;
    BtnState btn_z_, btn_x_, btn_c_, btn_v_, btn_b_;
    BtnState btn_mouse_left_, btn_mouse_right_, btn_mouse_middle_;
    double last_mouse_wheel_{0.0};

    // 内部工具
    void process_one_button(bool current_pressed, BtnState &state,
                            const KMButtonDefinition &def,
                            double current_time_s, ActionSet &accum,
                            double &speed_out);
};

} // namespace universal_controller
