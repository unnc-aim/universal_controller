/**
 * @file rc_action_types.hpp
 * @brief 遥控器动作类型定义（抽象共用）
 *
 * 提供遥控器动作解析的通用数据结构：
 * - TriStateAction: 三态动作 (on/off/toggle)
 * - FeederAction: 拨弹盘动作
 * - SpinControlAction: 小陀螺控制动作
 * - ActionSet: 动作集合
 * - DialAction: 滚轮/拨轮动作
 * - NDJTriggerDefinition: 拨杆定义（支持两档拨杆）
 */

#pragma once

#include <array>

namespace universal_controller {

/**
 * @brief 三态动作 (on/off/toggle)
 */
struct TriStateAction {
    bool on{false};
    bool off{false};
    bool toggle{false};
};

/**
 * @brief 小陀螺控制动作
 */
struct SpinControlAction {
    bool accelerate{false}; // 加速
    bool decelerate{false}; // 减速
};

/**
 * @brief 动作集合
 */
struct ActionSet {
    TriStateAction emergency_stop;
    TriStateAction autoaim;
    TriStateAction nav_topic;
    TriStateAction behavior_tree_topic;
    TriStateAction friction_wheel;
    TriStateAction feeder;
    TriStateAction feeder_burst;
    TriStateAction spin_mode;
    SpinControlAction spin_control;
};

/**
 * @brief 滚轮/拨轮动作（带阈值）
 */
struct DialAction {
    double threshold{0.5};
    ActionSet actions;
};

/**
 * @brief 拨杆定义（支持三档拨杆：上/中/下）
 *
 * dock_points: 3个停靠点 [up, mid, down]
 * transitions: 4个切换瞬间 [up_to_mid, mid_to_up, down_to_mid, mid_to_down]
 */
struct NDJTriggerDefinition {
    bool loaded{false};
    std::array<ActionSet, 3> left_dock_points{};  // 左拨杆停靠点
    std::array<ActionSet, 4> left_transitions{};  // 左拨杆切换
    std::array<ActionSet, 3> right_dock_points{}; // 右拨杆停靠点
    std::array<ActionSet, 4> right_transitions{}; // 右拨杆切换
    DialAction dial_up;                           // 滚轮向上
    DialAction dial_down;                         // 滚轮向下
};

/**
 * @brief 按钮定义（支持长按/短按事件）
 *
 * 事件时序:
 * - on_press: 按下瞬间触发
 * - 如果保持时间 < long_press_threshold_s:
 *   - on_short_press_released: 短按释放时触发
 * - 如果保持时间 >= long_press_threshold_s:
 *   - on_long_press_reached: 长按阈值到达时触发
 *   - on_long_press_released: 长按释放时触发
 * - on_release: 任何释放时触发（不受长短按影响）
 */
struct ButtonDefinition {
    bool loaded{false};
    double long_press_threshold_s{0.2}; // 长按阈值（秒）

    ActionSet on_press;                // 按下时触发
    ActionSet on_short_press_released; // 短按释放时触发
    ActionSet on_long_press_reached;   // 长按阈值到达时触发
    ActionSet on_long_press_released;  // 长按释放时触发
    ActionSet on_release;              // 任何释放时触发
};

/**
 * @brief VT13 遥控器完整定义
 *
 * 包含：
 * - gear_switching: 三档拨杆（左急停，中正常，右导航）
 * - pause_button: 自瞄切换按钮
 * - left_custom_button: 小陀螺开关
 * - right_custom_button: 摩擦轮开关
 * - thumb_wheel: 小陀螺速度调节滚轮
 * - trigger: 发射扳机（短按单发，长按连发）
 */
struct VTMTriggerDefinition {
    bool loaded{false};

    // 三档拨杆：左(1)=急停，中(2)=正常，右(3)=导航+行为树
    std::array<ActionSet, 3> gear_dock_points{}; // [left, mid, right]
    std::array<ActionSet, 4> gear_transitions{}; // [left_to_mid, mid_to_left, mid_to_right, right_to_mid]

    // 自定义按钮
    ButtonDefinition pause_button;        // 自瞄切换
    ButtonDefinition left_custom_button;  // 小陀螺开关
    ButtonDefinition right_custom_button; // 摩擦轮开关

    // 滚轮（小陀螺速度）
    DialAction thumb_wheel_up;   // 向上拨（加速）
    DialAction thumb_wheel_down; // 向下拨（减速）

    // 发射扳机
    ButtonDefinition trigger;
};

// ========== 共享按钮事件检测 ==========

/**
 * @brief 合并 ActionSet（src 的任何 true 标志叠加到 dst）
 */
inline void merge_action_set(ActionSet &dst, const ActionSet &src) {
    if (src.emergency_stop.on)
        dst.emergency_stop.on = true;
    if (src.emergency_stop.off)
        dst.emergency_stop.off = true;
    if (src.emergency_stop.toggle)
        dst.emergency_stop.toggle = true;

    if (src.autoaim.on)
        dst.autoaim.on = true;
    if (src.autoaim.off)
        dst.autoaim.off = true;
    if (src.autoaim.toggle)
        dst.autoaim.toggle = true;

    if (src.nav_topic.on)
        dst.nav_topic.on = true;
    if (src.nav_topic.off)
        dst.nav_topic.off = true;
    if (src.nav_topic.toggle)
        dst.nav_topic.toggle = true;

    if (src.behavior_tree_topic.on)
        dst.behavior_tree_topic.on = true;
    if (src.behavior_tree_topic.off)
        dst.behavior_tree_topic.off = true;
    if (src.behavior_tree_topic.toggle)
        dst.behavior_tree_topic.toggle = true;

    if (src.friction_wheel.on)
        dst.friction_wheel.on = true;
    if (src.friction_wheel.off)
        dst.friction_wheel.off = true;
    if (src.friction_wheel.toggle)
        dst.friction_wheel.toggle = true;

    if (src.feeder.on)
        dst.feeder.on = true;
    if (src.feeder.off)
        dst.feeder.off = true;
    if (src.feeder.toggle)
        dst.feeder.toggle = true;

    if (src.feeder_burst.on)
        dst.feeder_burst.on = true;
    if (src.feeder_burst.off)
        dst.feeder_burst.off = true;
    if (src.feeder_burst.toggle)
        dst.feeder_burst.toggle = true;

    if (src.spin_mode.on)
        dst.spin_mode.on = true;
    if (src.spin_mode.off)
        dst.spin_mode.off = true;
    if (src.spin_mode.toggle)
        dst.spin_mode.toggle = true;

    if (src.spin_control.accelerate)
        dst.spin_control.accelerate = true;
    if (src.spin_control.decelerate)
        dst.spin_control.decelerate = true;
}

// ========== 共享按钮事件检测 ==========

/**
 * @brief 按钮事件检测结果（边沿 + 长按状态）
 */
struct ButtonEventFlags {
    bool press_edge{false};    // 按下边沿
    bool release_edge{false};  // 释放边沿
    bool short_release{false}; // 短按释放
    bool long_reached{false};  // 长按阈值到达
    bool long_release{false};  // 长按释放
};

/**
 * @brief 通用按钮事件检测（从状态变化中检测边沿和长按）
 *
 * 输入/输出参数通过引用传递，函数会更新 last_pressed / press_start_time_s / long_press_active。
 * 可被 VTMInterpreter、NDJInterpreter、KeyboardMouseParser 共用。
 *
 * @param current_pressed   当前帧按钮是否按下
 * @param last_pressed      上一帧状态（会被更新）
 * @param press_start_time_s 按下时刻（秒，会被更新）
 * @param long_press_active 是否已触发长按（会被更新）
 * @param current_time_s    当前时刻（秒）
 * @param long_press_threshold_s 长按阈值（秒）
 * @return ButtonEventFlags 本次检测到的事件
 */
inline ButtonEventFlags detect_button_events(
    bool current_pressed,
    bool &last_pressed,
    double &press_start_time_s,
    bool &long_press_active,
    double current_time_s,
    double long_press_threshold_s) {
    ButtonEventFlags flags;

    // 按下边沿
    if (current_pressed && !last_pressed) {
        press_start_time_s = current_time_s;
        long_press_active = false;
        flags.press_edge = true;
    }

    // 长按检测（持续按下状态）
    if (current_pressed && !long_press_active) {
        if (current_time_s - press_start_time_s >= long_press_threshold_s) {
            long_press_active = true;
            flags.long_reached = true;
        }
    }

    // 释放边沿
    if (!current_pressed && last_pressed) {
        flags.release_edge = true;
        if (!long_press_active) {
            flags.short_release = true;
        } else {
            flags.long_release = true;
        }
    }

    last_pressed = current_pressed;
    return flags;
}

// ========== 键鼠动作定义 ==========

/**
 * @brief 键鼠按钮事件（带 speed 字段）
 *
 * speed >= 0: 直接设置 current_spd_mode_
 * speed < 0:  不改变速度
 */
struct KMButtonEvent {
    double speed{-1.0};
    ActionSet actions;
};

/**
 * @brief 键鼠按钮定义（同 ButtonDefinition 模式，但使用 KMButtonEvent）
 */
struct KMButtonDefinition {
    bool loaded{false};
    double long_press_threshold_s{0.2};

    KMButtonEvent on_press;
    KMButtonEvent on_short_press_released;
    KMButtonEvent on_long_press_reached;
    KMButtonEvent on_long_press_released;
    KMButtonEvent on_release;
};

/**
 * @brief 键鼠完整触发定义
 *
 * 包含：
 * - key_shift/key_ctrl: 速度控制键
 * - key_q~key_b: 功能键（WASD 不参与动作映射）
 * - mouse_left/right/middle: 鼠标按钮
 * - mouse_wheel_up/down: 鼠标滚轮（DialAction 模式）
 */
struct KMTriggerDefinition {
    bool loaded{false};
    double speed_default{5000.0};

    // 速度控制键
    KMButtonDefinition key_shift;
    KMButtonDefinition key_ctrl;

    // 功能键
    KMButtonDefinition key_q;
    KMButtonDefinition key_e;
    KMButtonDefinition key_r;
    KMButtonDefinition key_f;
    KMButtonDefinition key_g;
    KMButtonDefinition key_z;
    KMButtonDefinition key_x;
    KMButtonDefinition key_c;
    KMButtonDefinition key_v;
    KMButtonDefinition key_b;

    // 鼠标按钮
    KMButtonDefinition mouse_left;
    KMButtonDefinition mouse_right;
    KMButtonDefinition mouse_middle;

    // 鼠标滚轮（同 DialAction 模式）
    DialAction mouse_wheel_up;
    DialAction mouse_wheel_down;
};

} // namespace universal_controller
