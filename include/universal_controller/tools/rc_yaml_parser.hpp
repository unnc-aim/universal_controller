/**
 * @file rc_yaml_parser.hpp
 * @brief 遥控器 YAML 配置解析工具（抽象共用）
 *
 * 提供遥控器 YAML 配置的通用解析函数：
 * - yaml_bool: 安全读取 YAML 布尔值
 * - parse_tristate: 解析三态动作
 * - parse_actions: 解析动作集
 * - parse_dock_points: 解析停靠点
 * - parse_transitions: 解析切换瞬间
 * - parse_dial_action: 解析滚轮动作
 * - parse_button_definition: 解析按钮定义
 * - parse_trigger_button_definition: 解析扳机按钮定义
 */

#pragma once

#include "universal_controller/tools/rc_action_types.hpp"
#include <yaml-cpp/yaml.h>
#include <string>

namespace universal_controller
{

    /**
     * @brief 安全读取 YAML 布尔值
     * @param node YAML 节点
     * @param key 键名
     * @return 布尔值，如果不存在则返回 false
     */
    bool yaml_bool(const YAML::Node &node, const char *key);

    /**
     * @brief 安全读取 YAML 双精度值
     * @param node YAML 节点
     * @param key 键名
     * @param default_value 默认值
     * @return 双精度值
     */
    double yaml_double(const YAML::Node &node, const char *key, double default_value = 0.0);

    /**
     * @brief 解析三态动作
     * @param actions_node YAML 动作节点
     * @param key 键名
     * @param out 输出三态动作
     */
    void parse_tristate(const YAML::Node &actions_node, const char *key, TriStateAction &out);

    /**
     * @brief 解析小陀螺控制动作
     * @param spin_control_node YAML 小陀螺控制节点
     * @param out 输出小陀螺控制动作
     */
    void parse_spin_control_action(const YAML::Node &spin_control_node, SpinControlAction &out);

    /**
     * @brief 解析动作集
     * @param actions_node YAML 动作节点
     * @param out 输出动作集
     */
    void parse_actions(const YAML::Node &actions_node, ActionSet &out);

    /**
     * @brief 解析停靠点（三档）
     * @param dock_points_node YAML 停靠点节点
     * @param out 输出停靠点数组 [up, mid, down]
     */
    void parse_dock_points(const YAML::Node &dock_points_node, std::array<ActionSet, 3> &out);

    /**
     * @brief 解析停靠点（两档）
     * @param dock_points_node YAML 停靠点节点
     * @param out 输出停靠点数组 [left, right]
     */
    void parse_dock_points_two(const YAML::Node &dock_points_node, std::array<ActionSet, 2> &out);

    /**
     * @brief 解析切换瞬间（四方向：up_to_mid, mid_to_up, down_to_mid, mid_to_down）
     * @param transitions_node YAML 切换节点
     * @param out 输出切换数组
     * @param names 切换名称数组
     */
    void parse_transitions(const YAML::Node &transitions_node,
                           std::array<ActionSet, 4> &out,
                           const std::array<std::string, 4> &names);

    /**
     * @brief 解析滚轮动作
     * @param dial_node YAML 滚轮节点
     * @param out 输出滚轮动作
     */
    void parse_dial_action(const YAML::Node &dial_node, DialAction &out);

    /**
     * @brief 解析按钮定义（支持长按/短按事件）
     * @param button_node YAML 按钮节点
     * @param out 输出按钮定义
     */
    void parse_button_definition(const YAML::Node &button_node, ButtonDefinition &out);

    // ========== 动作执行工具 ==========

    /**
     * @brief 应用三态动作
     * @param action 三态动作
     * @param state 状态变量（会被修改）
     */
    void apply_tri_state_action(const TriStateAction &action, bool &state);

    /**
     * @brief 应用小陀螺控制动作
     * @param action 小陀螺控制动作
     * @param spin_speed_delta 小陀螺速度增量输出（会被修改）
     */
    void apply_spin_control_action(const SpinControlAction &action, double &spin_speed_delta);

    /**
     * @brief 执行动作集
     * @param actions 动作集
     * @param states 状态变量集合（会被修改）
     */
    struct ActionStates
    {
        bool emergency_state{false};
        bool autoaim_state{false};
        bool nav_topic_state{false};
        bool behavior_tree_state{false};
        bool friction_state{false};
        bool feeder_state{false};      // 拨弹盘开关状态
        bool burst_mode{false};        // 连发模式状态
        bool spin_mode{false};         // 小陀螺模式状态
        double spin_speed_delta{0.0};
    };

    void execute_action_set(const ActionSet &actions, ActionStates &states);

} // namespace universal_controller
