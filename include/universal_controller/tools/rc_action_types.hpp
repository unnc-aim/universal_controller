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

} // namespace universal_controller
