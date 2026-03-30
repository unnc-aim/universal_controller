/**
 * @file pid.hpp
 * @brief PID 控制器
 *
 * 实现比例-积分-微分控制器，支持：
 * - 积分限幅
 * - 输出限幅
 * - D 项低通滤波
 * - 动态参数更新
 */

#pragma once

#include <cmath>
#include <string>

namespace universal_controller {

/**
 * @brief PID 控制器类
 *
 * 参考 infantry_controller/pid.py 实现
 */
class PID {
  public:
    /**
     * @brief 构造函数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param max_out 最大输出限幅
     * @param max_iout 最大积分限幅
     */
    PID(double kp, double ki, double kd, double max_out, double max_iout)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          max_out_(max_out),
          max_iout_(max_iout),
          integral_(0.0),
          last_error_(0.0),
          last_d_out_(0.0) {
    }

    /**
     * @brief 默认构造函数（参数为0）
     */
    PID()
        : PID(0.0, 0.0, 0.0, 0.0, 0.0) {
    }

    /**
     * @brief 计算 PID 输出
     * @param error 当前误差
     * @param d_input 可选的 D 项输入（如果提供，D 项 = kd * d_input）
     * @param alpha D 项低通滤波系数 [0, 1]，值越大滤波越强
     * @return PID 控制器输出
     */
    double update(double error, double d_input = NAN, double alpha = 0.01) {
        // P 项
        double p_out = kp_ * error;

        // I 项
        integral_ += error;
        // 积分限幅
        if (max_iout_ > 0.0) {
            integral_ = clamp(integral_, -max_iout_, max_iout_);
        }
        double i_out = ki_ * integral_;

        // D 项
        double raw_d;
        if (!std::isnan(d_input)) {
            // 使用自定义的 D 输入（例如：期望速度 - 测量速度）
            raw_d = kd_ * d_input;
        } else {
            // 标准微分（误差差分）
            raw_d = kd_ * (error - last_error_);
        }

        // D 项低通滤波
        double d_out = alpha * raw_d + (1.0 - alpha) * last_d_out_;
        last_d_out_ = d_out;

        last_error_ = error;

        double output = p_out + i_out + d_out;

        // 总输出限幅
        if (max_out_ > 0.0) {
            output = clamp(output, -max_out_, max_out_);
        }

        return output;
    }

    /**
     * @brief 重置 PID 控制器状态
     */
    void reset() {
        integral_ = 0.0;
        last_error_ = 0.0;
        last_d_out_ = 0.0;
    }

    // ========== 参数更新 ==========

    void set_kp(double kp) {
        kp_ = kp;
    }
    void set_ki(double ki) {
        ki_ = ki;
    }
    void set_kd(double kd) {
        kd_ = kd;
    }
    void set_max_out(double max_out) {
        max_out_ = max_out;
    }
    void set_max_iout(double max_iout) {
        max_iout_ = max_iout;
    }

    /**
     * @brief 批量更新参数
     */
    void update_params(double kp = NAN, double ki = NAN, double kd = NAN, double max_out = NAN, double max_iout = NAN) {
        if (!std::isnan(kp))
            kp_ = kp;
        if (!std::isnan(ki))
            ki_ = ki;
        if (!std::isnan(kd))
            kd_ = kd;
        if (!std::isnan(max_out))
            max_out_ = max_out;
        if (!std::isnan(max_iout))
            max_iout_ = max_iout;
    }

    // ========== 参数获取 ==========

    double kp() const {
        return kp_;
    }
    double ki() const {
        return ki_;
    }
    double kd() const {
        return kd_;
    }
    double max_out() const {
        return max_out_;
    }
    double max_iout() const {
        return max_iout_;
    }
    double integral() const {
        return integral_;
    }

  private:
    double kp_;       ///< 比例增益
    double ki_;       ///< 积分增益
    double kd_;       ///< 微分增益
    double max_out_;  ///< 最大输出限幅
    double max_iout_; ///< 最大积分限幅

    double integral_;   ///< 积分累积值
    double last_error_; ///< 上一次误差值
    double last_d_out_; ///< 上一次 D 项输出值（用于滤波）

    /**
     * @brief 数值限幅
     */
    static double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }
};

/**
 * @brief 级联 PID 控制器（位置环 + 速度环）
 */
class CascadePID {
  public:
    PID pos_pid; ///< 位置环 PID
    PID spd_pid; ///< 速度环 PID

    CascadePID() = default;

    CascadePID(const PID &pos, const PID &spd)
        : pos_pid(pos),
          spd_pid(spd) {
    }

    /**
     * @brief 级联更新
     * @param pos_error 位置误差
     * @param spd_error 速度误差（目标速度 - 实际速度）
     * @param d_input D 项输入（可选）
     * @return 最终输出
     */
    double update(double pos_error, double spd_error, double d_input = NAN) {
        double target_spd = pos_pid.update(pos_error, d_input);
        return spd_pid.update(target_spd - spd_error);
    }

    void reset() {
        pos_pid.reset();
        spd_pid.reset();
    }
};

} // namespace universal_controller
