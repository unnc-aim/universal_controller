#include "chassis_controllers/utils/power_limiter.hpp"

#include "cmath"
#include "chassis_controllers/utils/rls.hpp"

void PowerLimiter::update_status(
    const float rpm[],
    const float current[],
    const float outputs[],
    const float updated_max_power) {
    max_power_ = updated_max_power;
    float estimated_powers = 0, command_powers = 0;

    for (int i = 0; i < motor_count_; i++) {
        const float w = rpm_to_rad(rpm[i]);
        const float t = current_to_torque(current[i]);

        estimated_powers_[i] = t * w +
                               k1_ * std::fabs(w) +
                               k2_ * t * t +
                               k3_ / static_cast<float>(motor_count_);
        estimated_powers += estimated_powers_[i];

        const float command_torque = current_to_torque(outputs[i]);
        command_powers_[i] = command_torque * w +
                             k1_ * std::fabs(w) +
                             command_torque * command_torque * k2_ +
                             k3_ / static_cast<float>(motor_count_);
        command_powers += command_powers_[i];
    }

    estimated_power_ = estimated_powers;
    command_power_ = command_powers;
}

float *PowerLimiter::get_decay_power(
    const float rpm[],
    const float outputs[]) {
    float allocatable_power = this->max_power_;
    float total_required_power = 0;

    for (int i = 0; i < motor_count_; i++) {
        actual_outputs_[i] = outputs[i];
        if (this->command_powers_[i] > 0.0f) {
            total_required_power += this->command_powers_[i];
        } else {
            allocatable_power -= this->command_powers_[i];
        }
    }

    // start to calculate the decay current, if the power is over the limit
    if (this->command_power_ > this->max_power_) {
        for (int i = 0; i < motor_count_; i++) {
            if (float_equal(command_powers_[i], 0.0f) || command_powers_[i] < 0.0f) {
                continue;
            }

            const float curAv = rpm_to_rad(rpm[i]);
            const float powerWeight = command_powers_[i] / total_required_power;
            const float delta = curAv * curAv -
                                4.f * k2_ *
                                (k1_ * std::fabs(curAv) + k3_ / static_cast<float>(motor_count_) -
                                 powerWeight * allocatable_power);

            if (delta > 0.0f) {
                // distinct roots
                actual_outputs_[i] = actual_outputs_[i] > 0.0f
                                         ? (-curAv + std::sqrt(delta)) / (2.0f * k2_) / k0_
                                         : (-curAv - std::sqrt(delta)) / (2.0f * k2_) / k0_;
            } else {
                // repeat roots
                // imaginary roots
                actual_outputs_[i] = -curAv / (2.0f * k2_) / k0_;
            }
        }
    }

    return actual_outputs_;
}

void PowerLimiter::update_rls(
    const float rpm[],
    const float current[],
    const float measured_power
) {
    Eigen::Vector2f samples = Eigen::Vector2f::Zero();
    float effectivePower = 0.0f;

    for (int i = 0; i < motor_count_; i++) {
        effectivePower += current_to_torque(current[i]) * rpm_to_rad(rpm[i]);
        samples(0) += std::fabs(rpm_to_rad(rpm[i]));
        samples(1) += current_to_torque(current[i]) * current_to_torque(current[i]);
    }

    const Eigen::Vector2f &params = rls_.update(samples, measured_power - effectivePower - k3_);
    k1_ = std::fmax(params(0), 1e-5f); // In case the k1 diverge to negative number
    k2_ = std::fmax(params(1), 1e-5f); // In case the k2 diverge to negative number

    std::cout << "ks:\t" << k1_ << "\t" << k2_ << std::endl;
}
