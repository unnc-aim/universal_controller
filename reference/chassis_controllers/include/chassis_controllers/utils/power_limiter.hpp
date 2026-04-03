#ifndef BUILD_POWER_LIMITER_HPP
#define BUILD_POWER_LIMITER_HPP

#include <cmath>

#include "cstdint"
#include "rls.hpp"

#define STEER 1
#define WHEEL 2

// #define POWER_FITTING WHEEL
// #define POWER_FITTING STEER

class PowerLimiter {
public:
    PowerLimiter(float k0_,
                 float k1_,
                 float k2_,
                 float k3_,
                 uint8_t motor_count_) : k0_(k0_), k1_(k1_), k2_(k2_), k3_(k3_), motor_count_(motor_count_) {
        rls_.setParamVector((Eigen::Vector2f() << k1_, k2_).finished());
    }

    float *get_decay_power(const float rpm[], const float outputs[]);

    void update_status(const float rpm[],
                       const float current[],
                       const float outputs[],
                       float updated_max_power);

    [[nodiscard]] float get_command_power() const {
        return this->command_power_;
    }

    [[nodiscard]] float get_estimated_power() const {
        return this->estimated_power_;
    }

    void update_rls(const float rpm[],
                    const float current[],
                    float measured_power);

private:
    // P = τΩ + k1|Ω| + k2τ^2 + k3
    float k0_, k1_, k2_, k3_;
    // The model params
    // k0: Torque constant, Nm/A
    // k1: Friction constant
    // k2: Joules constant
    // k3: Constant heat loss

    const uint8_t motor_count_;

    float max_power_ = 0;
    float estimated_power_ = 0;
    float command_power_ = 0;

    float command_powers_[256]{};
    float estimated_powers_[256]{};

    float actual_outputs_[256]{};

#ifdef POWER_FITTING
    RLS<2> rls_{1e5, 0.99999f};
#else
    RLS<2> rls_{1e-5, 0.99999f};
#endif

    [[nodiscard]] float current_to_torque(const float current) const {
        return current * k0_;
    }

    constexpr static float RPM_TO_RAD = 0.10471975511965977f;

    static float rpm_to_rad(const float rpm) {
        return rpm * RPM_TO_RAD;
    }

    static float rad_to_rpm(const float rad) {
        return rad / RPM_TO_RAD;
    }

    static bool float_equal(const float a, const float b) {
        return std::fabs(a - b) < 1e-6f;
    }
};

#endif //BUILD_POWER_LIMITER_HPP
