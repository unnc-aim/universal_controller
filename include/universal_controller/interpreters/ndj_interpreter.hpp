/**
 * @file ndj_interpreter.hpp
 * @brief NDJ 遥控器解释器（大疆新图传遥控器）
 *
 * 订阅 ReadDJIRC 消息，包含：
 * - 摇杆（底盘控制）
 * - 拨杆（模式切换）
 * - 键盘（WASD移动、QE旋转等）
 * - 鼠标（云台控制、发射）
 * - 拨轮（小陀螺速度调节）
 *
 * 直接发布 UnifiedInput.msg 到 Hub
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/read_djirc.hpp"
#include "universal_controller/msg/unified_input.hpp"
#include "universal_controller/tools/input_processor.hpp"

#include <array>
#include <memory>
#include <string>

namespace universal_controller
{

    class NDJInterpreter : public rclcpp::Node
    {
    public:
        explicit NDJInterpreter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~NDJInterpreter() override = default;

        struct TriStateAction
        {
            bool on{false};
            bool off{false};
            bool toggle{false};
        };

        struct FeederAction
        {
            bool single_shot_once{false};
            bool continuous_start{false};
            bool continuous_stop{false};
            bool continuous_toggle{false};
        };

        struct SpinControlAction
        {
            bool switch_toggle{false};
            bool accelerate{false};
            bool decelerate{false};
        };

        struct ActionSet
        {
            TriStateAction emergency_stop;
            TriStateAction autoaim;
            TriStateAction nav_topic;
            TriStateAction behavior_tree_topic;
            TriStateAction friction_wheel;
            FeederAction feeder;
            TriStateAction spin_mode;
            SpinControlAction spin_control;
        };

        struct DialAction
        {
            double threshold{0.5};
            ActionSet actions;
        };

        struct TriggerDefinition
        {
            bool loaded{false};
            std::array<ActionSet, 3> left_dock_points{};
            std::array<ActionSet, 4> left_transitions{};
            std::array<ActionSet, 3> right_dock_points{};
            std::array<ActionSet, 4> right_transitions{};
            DialAction dial_up;
            DialAction dial_down;
        };

    private:
        void declare_parameters();
        void load_parameters();
        void load_trigger_definition(const std::string &file_path);
        void cb_rc(const custom_msgs::msg::ReadDJIRC::SharedPtr msg);
        void process_input();
        void publish_unified();
        void execute_trigger_actions(const custom_msgs::msg::ReadDJIRC &rc);
        void execute_action_set(const ActionSet &actions);
        void apply_tri_state_action(const TriStateAction &action, bool &state);
        void apply_feeder_action(const FeederAction &action);
        void apply_spin_control_action(const SpinControlAction &action);
        static int switch_to_dock(uint8_t switch_value);
        static int transition_index(int from_dock, int to_dock);

        // 订阅
        rclcpp::Subscription<custom_msgs::msg::ReadDJIRC>::SharedPtr sub_rc_;

        // 发布
        rclcpp::Publisher<msg::UnifiedInput>::SharedPtr pub_unified_;

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 原始数据
        custom_msgs::msg::ReadDJIRC::SharedPtr raw_rc_data_;
        bool connected_{false};
        rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
        double connection_timeout_s_{0.5};

        // 输出
        msg::UnifiedInput unified_output_;

        // 输入处理器
        InputProcessor input_processor_;
        InputProcessorConfig input_config_;

        // 小陀螺模式
        bool spin_mode_enabled_{false};
        uint8_t last_gear_switch_{0};
        uint8_t last_pause_button_{0};
        bool last_v_pressed_{false};
        bool nav_mode_enabled_{false};

        // 参数
        std::string topic_rc_read_;
        std::string topic_unified_output_;
        std::string ndj_definition_file_;

        // 定义驱动状态
        TriggerDefinition trigger_definition_;
        uint8_t last_left_switch_{0};
        uint8_t last_right_switch_{0};
        double last_dial_{0.0};

        bool autoaim_state_{false};
        bool emergency_state_{false};
        bool nav_topic_state_{false};
        bool behavior_tree_state_{false};
        bool friction_state_{false};
        bool feeder_continuous_state_{false};
        bool fire_single_pulse_{false};

        rclcpp::QoS qos_best_effort_{rclcpp::QoS(1).best_effort()};
    };

} // namespace universal_controller
