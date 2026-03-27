/**
 * @file config.hpp
 * @brief 配置加载工具
 *
 * 提供统一的参数加载和配置管理
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <array>

namespace universal_controller
{

    /**
     * @brief 配置加载器类
     *
     * 封装 ROS2 参数读取，提供类型安全的配置访问
     */
    class ConfigLoader
    {
    public:
        explicit ConfigLoader(rclcpp::Node *node) : node_(node) {}

        // ========== 基础类型参数 ==========

        int get_int(const std::string &name, int default_val = 0) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_int();
        }

        double get_double(const std::string &name, double default_val = 0.0) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_double();
        }

        std::string get_string(const std::string &name, const std::string &default_val = "") const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_string();
        }

        bool get_bool(const std::string &name, bool default_val = false) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_bool();
        }

        // ========== 带前缀的参数 ==========

        int get_prefixed_int(const std::string &prefix, const std::string &name, int default_val = 0) const
        {
            return get_int(prefix + "." + name, default_val);
        }

        double get_prefixed_double(const std::string &prefix, const std::string &name, double default_val = 0.0) const
        {
            return get_double(prefix + "." + name, default_val);
        }

        std::string get_prefixed_string(const std::string &prefix, const std::string &name, const std::string &default_val = "") const
        {
            return get_string(prefix + "." + name, default_val);
        }

        bool get_prefixed_bool(const std::string &prefix, const std::string &name, bool default_val = false) const
        {
            return get_bool(prefix + "." + name, default_val);
        }

        // ========== 数组参数 ==========

        std::vector<int64_t> get_int_array(const std::string &name, const std::vector<int64_t> &default_val = {}) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_integer_array();
        }

        std::vector<double> get_double_array(const std::string &name, const std::vector<double> &default_val = {}) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_double_array();
        }

        std::vector<std::string> get_string_array(const std::string &name, const std::vector<std::string> &default_val = {}) const
        {
            if (!node_->has_parameter(name))
            {
                node_->declare_parameter(name, default_val);
            }
            return node_->get_parameter(name).as_string_array();
        }

        // ========== 特用数组转换 ==========

        /**
         * @brief 获取 4 元素数组
         */
        template <typename T>
        std::array<T, 4> get_array4(const std::string &name, const std::array<T, 4> &default_val = {}) const
        {
            std::vector<T> vec;
            if constexpr (std::is_same_v<T, int>)
            {
                std::vector<int64_t> default_vec(default_val.begin(), default_val.end());
                auto int_vec = get_int_array(name, default_vec);
                vec.reserve(int_vec.size());
                for (auto v : int_vec)
                {
                    vec.push_back(static_cast<int>(v));
                }
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                vec = get_double_array(name, std::vector<T>(default_val.begin(), default_val.end()));
            }
            std::array<T, 4> arr{};
            for (size_t i = 0; i < 4 && i < vec.size(); ++i)
            {
                arr[i] = vec[i];
            }
            return arr;
        }

        /**
         * @brief 获取 PID 参数 [kp, ki, kd, max_out, max_iout]
         */
        struct PIDParams
        {
            double kp, ki, kd, max_out, max_iout;
        };
        PIDParams get_pid_params(const std::string &name, const PIDParams &default_val = {0, 0, 0, 0, 0}) const
        {
            auto vec = get_double_array(name, {default_val.kp, default_val.ki, default_val.kd, default_val.max_out, default_val.max_iout});
            if (vec.size() >= 5)
            {
                return {vec[0], vec[1], vec[2], vec[3], vec[4]};
            }
            return default_val;
        }

        /**
         * @brief 获取电机类型
         */
        MotorType get_motor_type(const std::string &name, MotorType default_val = MotorType::DJI) const
        {
            std::string type_str = get_string(name, std::string(default_val == MotorType::DJI ? "DJI" : "LK"));
            return (type_str == "LK") ? MotorType::LK : MotorType::DJI;
        }

    private:
        rclcpp::Node *node_;
    };

    /**
     * @brief 底盘配置结构体
     */
    struct ChassisConfig
    {
        // 几何参数
        double wheel_track{0.372};
        double wheel_base{0.372};

        // 编码器零位
        std::array<int, 4> ecd_zeros{0, 0, 0, 0};
        int yaw_center_ecd{39200};

        // 电机类型
        MotorType motor_type{MotorType::DJI};

        // 话题
        std::string topic_drive_write;
        std::string topic_drive_read;
        std::string topic_steer_write;
        std::string topic_steer_read;
        std::string topic_yaw_read;

        // 小陀螺参数
        double spin_speed_default{3000.0};
        double spin_speed_min{800.0};
        double spin_speed_max{6500.0};

        // PID 参数（LK 电机用）
        ConfigLoader::PIDParams steer_angle_pid{0.3, 0.0, 0.0, 150.0, 300.0};
        ConfigLoader::PIDParams steer_speed_pid{5.5, 0.0, 3.3, 850.0, 500.0};
        ConfigLoader::PIDParams drive_speed_pid{6.0, 0.3, 0.0, 2000.0, 250.0};

        // 功率限制参数
        bool power_limit_enabled{false};
        double power_limit_default{80.0};
        double power_R{0.7708};
        double power_K{0.0131};
        double power_P0{2.4565};
        double power_K_min{0.001};
        double power_K_max{0.1};
        double power_filter_alpha{0.1};
        std::string topic_supercap;

        /**
         * @brief 从参数服务器加载
         */
        void load(const ConfigLoader &cfg)
        {
            wheel_track = cfg.get_prefixed_double("controllers.chassis", "wheel_track", wheel_track);
            wheel_base = cfg.get_prefixed_double("controllers.chassis", "wheel_base", wheel_base);
            ecd_zeros = cfg.get_array4<int>("controllers.chassis.ecd_zeros", ecd_zeros);
            yaw_center_ecd = cfg.get_prefixed_int("controllers.chassis", "yaw_center_ecd", yaw_center_ecd);

            motor_type = cfg.get_motor_type("motor_type.chassis", motor_type);

            topic_drive_write = cfg.get_prefixed_string("controllers.chassis", "topic_drive_write", topic_drive_write);
            topic_drive_read = cfg.get_prefixed_string("controllers.chassis", "topic_drive_read", topic_drive_read);
            topic_steer_write = cfg.get_prefixed_string("controllers.chassis", "topic_steer_write", topic_steer_write);
            topic_steer_read = cfg.get_prefixed_string("controllers.chassis", "topic_steer_read", topic_steer_read);
            topic_yaw_read = cfg.get_prefixed_string("controllers.chassis", "topic_yaw_read", topic_yaw_read);

            spin_speed_default = cfg.get_prefixed_double("controllers.chassis", "spin_speed_default", spin_speed_default);
            spin_speed_min = cfg.get_prefixed_double("controllers.chassis", "spin_speed_min", spin_speed_min);
            spin_speed_max = cfg.get_prefixed_double("controllers.chassis", "spin_speed_max", spin_speed_max);

            steer_angle_pid = cfg.get_pid_params("controllers.chassis.steer_angle_pid", steer_angle_pid);
            steer_speed_pid = cfg.get_pid_params("controllers.chassis.steer_speed_pid", steer_speed_pid);
            drive_speed_pid = cfg.get_pid_params("controllers.chassis.drive_speed_pid", drive_speed_pid);

            // 功率限制
            power_limit_enabled = cfg.get_prefixed_bool("controllers.chassis", "power_limit_enabled", power_limit_enabled);
            power_limit_default = cfg.get_prefixed_double("controllers.chassis", "power_limit_default", power_limit_default);
            power_R = cfg.get_prefixed_double("controllers.chassis", "power_R", power_R);
            power_K = cfg.get_prefixed_double("controllers.chassis", "power_K", power_K);
            power_P0 = cfg.get_prefixed_double("controllers.chassis", "power_P0", power_P0);
            power_K_min = cfg.get_prefixed_double("controllers.chassis", "power_K_min", power_K_min);
            power_K_max = cfg.get_prefixed_double("controllers.chassis", "power_K_max", power_K_max);
            power_filter_alpha = cfg.get_prefixed_double("controllers.chassis", "power_filter_alpha", power_filter_alpha);
            topic_supercap = cfg.get_prefixed_string("controllers.chassis", "topic_supercap", topic_supercap);
        }
    };

    /**
     * @brief 云台配置结构体
     */
    struct GimbalConfig
    {
        // Pitch 限位
        int pitch_center_ecd{4600};
        double pitch_min_deg{-25.0};
        double pitch_max_deg{40.0};

        // 鼠标灵敏度
        double mouse_sensitivity{1.0};

        // Yaw 电机类型
        MotorType yaw_motor_type{MotorType::LK};

        // 话题
        std::string topic_pitch_write;
        std::string topic_pitch_read;
        std::string topic_yaw_write;
        std::string topic_yaw_read;
        std::string topic_imu_read;
        std::string topic_autoaim_cmd;

        // PID 参数
        ConfigLoader::PIDParams yaw_pos_pid{20.0, 0.0, 0.5, 50.0, 50.0};
        ConfigLoader::PIDParams yaw_spd_pid{220.0, 0.3, 4600.0, 2048.0, 200.0};

        // 自瞄超时
        double autoaim_timeout_s{0.2};

        void load(const ConfigLoader &cfg)
        {
            pitch_center_ecd = cfg.get_prefixed_int("controllers.gimbal", "pitch_center_ecd", pitch_center_ecd);
            pitch_min_deg = cfg.get_prefixed_double("controllers.gimbal", "pitch_min_deg", pitch_min_deg);
            pitch_max_deg = cfg.get_prefixed_double("controllers.gimbal", "pitch_max_deg", pitch_max_deg);
            mouse_sensitivity = cfg.get_prefixed_double("controllers.gimbal", "mouse_sensitivity", mouse_sensitivity);

            yaw_motor_type = cfg.get_motor_type("motor_type.gimbal_yaw", yaw_motor_type);

            topic_pitch_write = cfg.get_prefixed_string("controllers.gimbal", "topic_pitch_write", topic_pitch_write);
            topic_pitch_read = cfg.get_prefixed_string("controllers.gimbal", "topic_pitch_read", topic_pitch_read);
            topic_yaw_write = cfg.get_prefixed_string("controllers.gimbal", "topic_yaw_write", topic_yaw_write);
            topic_yaw_read = cfg.get_prefixed_string("controllers.gimbal", "topic_yaw_read", topic_yaw_read);
            topic_imu_read = cfg.get_prefixed_string("controllers.gimbal", "topic_imu_read", topic_imu_read);
            topic_autoaim_cmd = cfg.get_prefixed_string("controllers.gimbal", "topic_autoaim_cmd", topic_autoaim_cmd);

            yaw_pos_pid = cfg.get_pid_params("controllers.gimbal.yaw_pos_pid", yaw_pos_pid);
            yaw_spd_pid = cfg.get_pid_params("controllers.gimbal.yaw_spd_pid", yaw_spd_pid);

            autoaim_timeout_s = cfg.get_prefixed_double("controllers.gimbal", "autoaim_timeout_s", autoaim_timeout_s);
        }
    };

    /**
     * @brief 发射配置结构体
     */
    struct FireConfig
    {
        // 是否启用
        bool enabled{true};

        // 摩擦轮
        double friction_speed_default{6500.0};

        // 发射参数
        double shot_period_ms{30.0};
        int load_current_threshold{500};
        double load_speed_ecd{2.5};

        // 话题
        std::string topic_fire_write;
        std::string topic_fire_read;

        // PID 参数
        ConfigLoader::PIDParams trigger_pos_pid{0.4, 0.0, 7.0, 13000.0, 1500.0};
        ConfigLoader::PIDParams trigger_spd_pid{5.0, 0.01, 0.0, 10000.0, 1000.0};

        // 裁判系统超时
        double referee_timeout_s{0.5};

        void load(const ConfigLoader &cfg)
        {
            enabled = cfg.get_prefixed_bool("controllers.fire", "enabled", enabled);
            friction_speed_default = cfg.get_prefixed_double("controllers.fire", "friction_speed_default", friction_speed_default);

            shot_period_ms = cfg.get_prefixed_double("controllers.fire", "shot_period_ms", shot_period_ms);
            load_current_threshold = cfg.get_prefixed_int("controllers.fire", "load_current_threshold", load_current_threshold);
            load_speed_ecd = cfg.get_prefixed_double("controllers.fire", "load_speed_ecd", load_speed_ecd);

            topic_fire_write = cfg.get_prefixed_string("controllers.fire", "topic_fire_write", topic_fire_write);
            topic_fire_read = cfg.get_prefixed_string("controllers.fire", "topic_fire_read", topic_fire_read);

            trigger_pos_pid = cfg.get_pid_params("controllers.fire.trigger_pos_pid", trigger_pos_pid);
            trigger_spd_pid = cfg.get_pid_params("controllers.fire.trigger_spd_pid", trigger_spd_pid);

            referee_timeout_s = cfg.get_prefixed_double("controllers.fire", "referee_timeout_s", referee_timeout_s);
        }
    };

} // namespace universal_controller
