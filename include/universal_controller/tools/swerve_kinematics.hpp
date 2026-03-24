/**
 * @file swerve_kinematics.hpp
 * @brief 舵轮底盘运动学解算
 *
 * 参考 infantry_controller/chassis_kinematics.py 实现
 * 支持四轮舵轮底盘的全向移动和旋转
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

namespace universal_controller {

/**
 * @brief 舵轮底盘运动学解算器
 *
 * 实现四轮舵轮底盘的运动学正解算
 * 轮子顺序: FL(前左), FR(前右), BL(后左), BR(后右)
 */
class SwerveKinematics {
public:
    /**
     * @brief 构造函数
     * @param wheel_track 轮距（宽度，单位：米）
     * @param wheel_base 轴距（长度，单位：米）
     * @param ecd_range 编码器范围（DJI=8192, LK=65536）
     */
    SwerveKinematics(double wheel_track, double wheel_base, uint16_t ecd_range = 8192);

    /**
     * @brief 计算四个轮子的速度和舵向角度
     * @param vx X方向速度（前进）
     * @param vy Y方向速度（横移）
     * @param wz 旋转角速度
     * @param current_steer_ecds 当前舵向电机编码器值 [FL, FR, BL, BR]
     * @param ecd_zeros 舵向电机零位偏移 [FL, FR, BL, BR]
     * @return pair<驱动速度列表, 舵向目标编码器值列表>
     */
    std::pair<std::array<double, 4>, std::array<uint16_t, 4>> calculate_motion(
        double vx, double vy, double wz,
        const std::array<uint16_t, 4>& current_steer_ecds,
        const std::array<int, 4>& ecd_zeros) const;

    /**
     * @brief 计算最短旋转路径
     * @param current_ecd 当前编码器值
     * @param target_ecd 目标编码器值
     * @return pair<最优目标编码器值, 速度方向系数>
     */
    std::pair<uint16_t, double> calc_shortest_path(uint16_t current_ecd, uint16_t target_ecd) const;

    // Getters
    double wheel_track() const { return wheel_track_; }
    double wheel_base() const { return wheel_base_; }
    uint16_t ecd_range() const { return ecd_range_; }

private:
    double wheel_track_;         ///< 轮距 (米)
    double wheel_base_;          ///< 轴距 (米)
    uint16_t ecd_range_;         ///< 编码器范围
    uint16_t half_range_;        ///< 编码器半范围
    double geometry_factor_;     ///< 几何中心到轮子的距离系数
    double k_;                   ///< 归一化比例系数

    static double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }
};

}  // namespace universal_controller
