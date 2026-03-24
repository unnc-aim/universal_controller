/**
 * @file swerve_kinematics.cpp
 * @brief 舵轮底盘运动学解算实现
 */

#include "universal_controller/tools/swerve_kinematics.hpp"
#include <cmath>

namespace universal_controller
{

    SwerveKinematics::SwerveKinematics(double wheel_track, double wheel_base, uint16_t ecd_range)
        : wheel_track_(wheel_track),
          wheel_base_(wheel_base),
          ecd_range_(ecd_range),
          half_range_(ecd_range / 2)
    {
        // 计算几何中心到轮子的距离系数
        geometry_factor_ = std::sqrt(wheel_track * wheel_track + wheel_base * wheel_base) / 2.0;
        // 归一化比例系数 (假设长宽相等)
        k_ = 0.7071068;
    }

    std::pair<std::array<double, 4>, std::array<uint16_t, 4>> SwerveKinematics::calculate_motion(
        double vx, double vy, double wz,
        const std::array<uint16_t, 4> &current_steer_ecds,
        const std::array<int, 4> &ecd_zeros) const
    {

        std::array<double, 4> drive_speeds{};
        std::array<uint16_t, 4> steer_targets{};

        // 计算旋转产生的线速度分量 v_w
        double v_w = wz;

        // 计算每个轮子的速度矢量 (vx_i, vy_i)
        // 符号参考原C++代码:
        // FL: (+, +), FR: (+, -), BL: (-, +), BR: (-, -)
        std::array<std::pair<double, double>, 4> vectors = {
            std::make_pair(vx + k_ * v_w, vy + k_ * v_w), // Front Left
            std::make_pair(vx - k_ * v_w, vy + k_ * v_w), // Front Right
            std::make_pair(vx + k_ * v_w, vy - k_ * v_w), // Back Left
            std::make_pair(vx - k_ * v_w, vy - k_ * v_w)  // Back Right
        };

        for (size_t i = 0; i < 4; ++i)
        {
            double vx_i = vectors[i].first;
            double vy_i = vectors[i].second;

            // 1. 计算目标线速度模长
            double speed = std::sqrt(vx_i * vx_i + vy_i * vy_i);

            // 2. 计算目标角度 (atan2 返回 -pi 到 pi)
            double angle_rad = std::atan2(vy_i, vx_i);

            // 3. 转换为目标编码器值 (0 到 ecd_range)
            if (angle_rad < 0)
            {
                angle_rad += 2.0 * M_PI;
            }

            uint16_t raw_target_ecd = static_cast<uint16_t>((angle_rad / (2.0 * M_PI)) * ecd_range_);
            // 加上零位偏移
            uint16_t target_ecd_with_offset = (static_cast<int>(raw_target_ecd) + ecd_zeros[i]) % ecd_range_;

            // 4. 最短路径优化
            auto [final_ecd, direction_mult] = calc_shortest_path(current_steer_ecds[i], target_ecd_with_offset);

            // 5. 限幅 (参考 limitMM 8000)
            speed = clamp(speed, -8000.0, 8000.0);

            drive_speeds[i] = speed * direction_mult;
            steer_targets[i] = final_ecd;
        }

        return {drive_speeds, steer_targets};
    }

    std::pair<uint16_t, double> SwerveKinematics::calc_shortest_path(
        uint16_t current_ecd, uint16_t target_ecd) const
    {

        // 方案A: 直接转到目标
        int32_t diff_a = static_cast<int32_t>(target_ecd) - static_cast<int32_t>(current_ecd);
        if (diff_a > half_range_)
        {
            diff_a -= ecd_range_;
        }
        else if (diff_a < -static_cast<int32_t>(half_range_))
        {
            diff_a += ecd_range_;
        }

        // 方案B: 转到目标对面 (+180度)，同时电机反转
        uint16_t target_flipped = (static_cast<int>(target_ecd) + half_range_) % ecd_range_;
        int32_t diff_b = static_cast<int32_t>(target_flipped) - static_cast<int32_t>(current_ecd);
        if (diff_b > half_range_)
        {
            diff_b -= ecd_range_;
        }
        else if (diff_b < -static_cast<int32_t>(half_range_))
        {
            diff_b += ecd_range_;
        }

        // 比较绝对距离，选择转动角度最小的方案
        if (std::abs(diff_a) <= std::abs(diff_b))
        {
            return {target_ecd, 1.0};
        }
        else
        {
            return {target_flipped, -1.0};
        }
    }

} // namespace universal_controller
