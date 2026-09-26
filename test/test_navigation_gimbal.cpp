// Exercise the controller directly with synthetic samples and local state.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "universal_controller/controllers/gimbal_controller.hpp"

namespace universal_controller {

struct NavigationGimbalTest {
    static constexpr int64_t START = 10000000000LL;

    static void sample(GimbalController &controller, double yaw, int64_t stamp) {
        auto imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu->header.stamp = rclcpp::Time(stamp);
        imu->orientation.w = std::cos(yaw / 2.0);
        imu->orientation.z = std::sin(yaw / 2.0);
        controller.cb_imu(imu);
    }

    static void command(GimbalController &controller, double heading, double speed, int64_t stamp) {
        GimbalCommand cmd;
        cmd.follow_navigation = true;
        const double relative = normalize_angle(heading - controller.imu_yaw_rad_);
        cmd.navigation_vx = speed * std::cos(relative);
        cmd.navigation_vy = speed * std::sin(relative);
        cmd.navigation_stamp_ns = stamp;
        controller.set_command(cmd);
    }

    static void tick(GimbalController &controller, int64_t stamp) {
        assert(controller.navigation_inputs_ready(stamp));
        const double previous = controller.target_yaw_rad_;
        const double rate = controller.navigation_yaw_rate_;
        const bool active = controller.navigation_follow_active_;
        controller.compute_pitch_control();
        controller.compute_yaw_control();
        assert(std::abs(controller.navigation_yaw_rate_) <= controller.config_.follow_max_yaw_rate + 1e-12);
        assert(std::abs(controller.navigation_yaw_rate_ - rate) <=
               controller.config_.follow_max_yaw_accel * controller.dt_ + 1e-12);
        if (active) {
            assert(std::abs(normalize_angle(controller.target_yaw_rad_ - previous)) <=
                   controller.config_.follow_max_yaw_rate * controller.dt_ + 1e-12);
        }
    }

    static void run() {
        GimbalController controller;
        controller.config_.follow_navigation = true;
        controller.dt_ = 0.001;
        for (const auto &angles : {std::make_pair(0.0, 1.0), std::make_pair(3.13, -3.0),
                                  std::make_pair(-3.13, 3.0), std::make_pair(0.0, M_PI)}) {
            controller.reset_navigation_follow();
            controller.imu_stamp_ns_ = 0;
            sample(controller, angles.first, START);
            controller.imu_pitch_rad_ = 0.1;
            double travel = 0.0;
            double previous = angles.first;
            for (int i = 0; i < 16000; ++i) {
                const int64_t stamp = START + i * 1000000LL;
                sample(controller, previous, stamp);
                if (i % 50 == 0) command(controller, angles.second, 0.2, stamp);
                tick(controller, stamp);
                travel += normalize_angle(controller.target_yaw_rad_ - previous);
                previous = controller.target_yaw_rad_;
            }
            assert(std::abs(normalize_angle(previous - angles.second)) <= controller.config_.follow_yaw_tolerance);
            assert(std::abs(travel - normalize_angle(angles.second - angles.first)) <= controller.config_.follow_yaw_tolerance);
            assert(std::abs(controller.navigation_yaw_rate_) < 1e-12);
        }

        // A small turn uses the minimum target rate, then settles within tolerance.
        controller.reset_navigation_follow();
        controller.imu_stamp_ns_ = 0;
        sample(controller, 0.0, START);
        command(controller, 0.03, 0.2, START);
        tick(controller, START);
        controller.navigation_yaw_rate_ = controller.config_.follow_min_yaw_rate;
        tick(controller, START);
        assert(controller.navigation_yaw_rate_ >= controller.config_.follow_min_yaw_rate - 1e-12);
        for (int i = 1; i < 1000; ++i) {
            const int64_t stamp = START + i * 1000000LL;
            sample(controller, controller.target_yaw_rad_, stamp);
            if (i % 50 == 0) command(controller, 0.03, 0.2, stamp);
            tick(controller, stamp);
        }
        assert(std::abs(controller.target_yaw_rad_ - 0.03) <= controller.config_.follow_yaw_tolerance);
        assert(std::abs(controller.navigation_yaw_rate_) < 1e-12);

        // A direction reversal and a stop keep the target rate and acceleration bounded.
        controller.reset_navigation_follow();
        controller.imu_stamp_ns_ = 0;
        sample(controller, 0.0, START);
        for (int i = 0; i < 2200; ++i) {
            const int64_t stamp = START + i * 1000000LL;
            sample(controller, controller.imu_yaw_rad_, stamp);
            if (i % 50 == 0) command(controller, i < 600 ? 1.0 : -1.0, i < 1600 ? 0.2 : 0.0, stamp);
            tick(controller, stamp);
            controller.imu_yaw_rad_ = controller.target_yaw_rad_;
        }
        assert(std::abs(controller.navigation_yaw_rate_) < 1e-12);
        const double held = controller.target_yaw_rad_;
        command(controller, 2.0, 0.01, START + 2200000000LL);
        sample(controller, held, START + 2200000000LL);
        tick(controller, START + 2200000000LL);
        assert(controller.target_yaw_rad_ == held);

        // One navigation sample keeps its heading while the measured gimbal rotates.
        const double direction = controller.navigation_target_yaw_;
        sample(controller, held + 0.1, START + 2201000000LL);
        tick(controller, START + 2201000000LL);
        assert(controller.navigation_target_yaw_ == direction);

        // Absolute control takes over, then following restarts at the measured pose.
        GimbalCommand aim;
        aim.yaw_rad = 2.0;
        aim.pitch_deg = 7.0;
        controller.set_command(aim);
        assert(!controller.navigation_follow_active_);
        controller.compute_pitch_control();
        controller.compute_yaw_control();
        assert(controller.target_yaw_rad_ == 2.0 && controller.target_pitch_deg_ == 7.0);
        sample(controller, 1.8, START + 2300000000LL);
        controller.imu_pitch_rad_ = -0.1;
        command(controller, 0.0, 0.2, START + 2300000000LL);
        tick(controller, START + 2300000000LL);
        assert(std::abs(controller.target_yaw_rad_ - 1.8) <=
               controller.config_.follow_max_yaw_accel * controller.dt_ * controller.dt_ + 1e-12);
        assert(std::abs(controller.target_pitch_deg_ - 0.1 * 180.0 / M_PI) < 1e-12);

        // Existing scan boundary behavior remains available after following.
        GimbalCommand scan;
        scan.scan_mode = true;
        scan.scan_vel_yaw = 0.5;
        controller.set_command(scan);
        controller.target_yaw_rad_ = M_PI - 0.0001;
        controller.compute_yaw_control();
        assert(controller.target_yaw_rad_ == M_PI && controller.scan_direction_yaw_ == -1.0);

        controller.reset_navigation_follow();
        controller.imu_stamp_ns_ = 0;
        sample(controller, 0.0, START);
        command(controller, 0.0, 0.2, START);
        assert(controller.navigation_inputs_ready(START));
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        assert(!controller.navigation_inputs_ready(START - 100000000LL));
        sample(controller, 0.0, START + 300000000LL);
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        command(controller, 0.0, 0.2, START + 300000000LL);
        assert(controller.navigation_inputs_ready(START + 300000000LL));
        controller.navigation_stamp_ns_ = START + 400000000LL;
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        controller.navigation_stamp_ns_ = 0;
        controller.command_.navigation_vx = std::numeric_limits<double>::quiet_NaN();
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        command(controller, 0.0, 0.2, START + 300000000LL);
        controller.dt_ = 0.0;
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        controller.dt_ = 0.001;
        controller.config_.follow_navigation = false;
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        controller.config_.follow_navigation = true;
        auto bad_imu = std::make_shared<sensor_msgs::msg::Imu>();
        bad_imu->header.stamp = rclcpp::Time(START + 300000000LL);
        bad_imu->orientation.w = 0.0;
        controller.cb_imu(bad_imu);
        assert(!controller.navigation_inputs_ready(START + 300000000LL));
        std::cout << "PASS: minimum turn rate, target settling, heading wrap, smooth turns, stop hold, takeover recovery, scan behavior and input validity\n";
    }
};

} // namespace universal_controller

int main() {
    universal_controller::NavigationGimbalTest::run();
}
