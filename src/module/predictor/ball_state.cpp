#include "ball_state.hpp"

#include <algorithm>
#include <numbers>
#include <utility>

#include "utility/time/delta_time.hpp"

namespace pingpong_tracker::predictor {

BallState::BallState() : BallState(UKFParameters{}, BallModelParameters{}) {
}

BallState::BallState(UKFParameters ukf_params, BallModelParameters ball_params)
    : ukf_{ukf_params.alpha, ukf_params.beta, ukf_params.kappa},
      ball_params_{std::move(ball_params)},
      ukf_params_{std::move(ukf_params)} {
}

auto BallState::predict(Clock::time_point stamp) -> void {
    if (!initialized_) {
        time_stamp_ = stamp;
        return;
    }

    const auto dt = util::delta_time(stamp, time_stamp_);
    if (dt <= std::chrono::duration<double>::zero()) {
        return;
    }

    if (dt > reset_interval_) {
        reset();
        time_stamp_ = stamp;
        return;
    }

    const double dt_seconds = dt.count();
    if (dt_seconds <= kMinTimeStepSeconds) {
        return;
    }

    const auto transition = [this, dt_seconds](const XVec& x) {
        return propagate_state(x, dt_seconds);
    };

    ukf_.predict(transition, process_noise(dt_seconds));
    time_stamp_ = stamp;
}

auto BallState::update(const ZVec& measurement, Clock::time_point stamp) -> bool {
    if (!initialized_) {
        initialize_from_measurement(measurement, stamp);
        return true;
    }

    predict(stamp);

    if (!initialized_) {
        initialize_from_measurement(measurement, stamp);
        return true;
    }

    const auto observe = [](const XVec& x) -> ZVec { return x.head<3>(); };
    const bool success = ukf_.update(measurement, observe, measurement_noise(),
                                     pingpong_tracker::util::DefaultAdd,
                                     pingpong_tracker::util::DefaultSubtract);

    if (!success) {
        return false;
    }

    ++update_count_;
    time_stamp_ = stamp;
    return true;
}

auto BallState::update(const ZVec& measurement) -> bool {
    return update(measurement, Clock::now());
}

auto BallState::reset() noexcept -> void {
    initialized_  = false;
    update_count_ = 0;
    ukf_          = UKF{ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa};
}

auto BallState::set_reset_interval(std::chrono::duration<double> interval) noexcept -> void {
    reset_interval_ = std::max(interval, std::chrono::duration<double>::zero());
}

auto BallState::initialized() const noexcept -> bool {
    return initialized_;
}

auto BallState::update_count() const noexcept -> std::size_t {
    return update_count_;
}

auto BallState::reset_interval() const noexcept -> std::chrono::duration<double> {
    return reset_interval_;
}

auto BallState::state() const -> XVec {
    return ukf_.x;
}

auto BallState::covariance() const -> PMat {
    return ukf_.covariance();
}

auto BallState::position() const -> Vector {
    return ukf_.x.head<3>();
}

auto BallState::velocity() const -> Vector {
    return ukf_.x.segment<3>(3);
}

auto BallState::spin() const -> Vector {
    return ukf_.x.tail<3>();
}

auto BallState::predict_trajectory(std::chrono::duration<double> horizon,
                                   std::chrono::duration<double> step) const
    -> std::vector<Vector> {
    if (!initialized_ || horizon <= std::chrono::duration<double>::zero() ||
        step <= std::chrono::duration<double>::zero()) {
        return {};
    }

    std::vector<Vector> trajectory;
    const auto steps = static_cast<std::size_t>(horizon / step);
    trajectory.reserve(steps);

    auto x = ukf_.x;
    for (std::size_t i = 0; i < steps; ++i) {
        x = propagate_state(x, step.count());
        trajectory.push_back(x.head<3>());
    }

    return trajectory;
}

auto BallState::initialize_from_measurement(const ZVec& measurement, Clock::time_point stamp) -> void {
    XVec x0 = XVec::Zero();
    x0.head<3>() = measurement;

    PMat p0 = PMat::Zero();
    for (int i = 0; i < UKFParameters::StateDim; ++i) {
        p0(i, i) = ukf_params_.initial_covariance[i];
    }

    ukf_         = UKF{x0, p0, ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa};
    initialized_ = true;
    update_count_ = 1;
    time_stamp_   = stamp;
}

auto BallState::process_noise(double dt_seconds) const -> UKF::QMat {
    UKF::QMat q = UKF::QMat::Zero();

    for (int i = 0; i < UKFParameters::StateDim; ++i) {
        const double sigma = ukf_params_.process_noise_std[i];
        q(i, i)            = sigma * sigma * dt_seconds;
    }

    return q;
}

auto BallState::measurement_noise() const -> UKF::RMat {
    UKF::RMat r = UKF::RMat::Zero();

    for (int i = 0; i < UKFParameters::ObsDim; ++i) {
        const double sigma = ukf_params_.measurement_noise_std[i];
        r(i, i)            = sigma * sigma;
    }

    return r;
}

auto BallState::propagate_state(const XVec& x, double dt_seconds) const -> XVec {
    const double area = std::numbers::pi_v<double> * ball_params_.radius * ball_params_.radius;
    const double inv_mass = 1.0 / ball_params_.mass;
    const double common = 0.5 * ball_params_.air_density * area * inv_mass;

    const double drag_factor = common * ball_params_.drag_coefficient;
    const double magnus_factor = common * ball_params_.lift_coefficient * ball_params_.radius;

    const auto derivative = [this, drag_factor, magnus_factor](const XVec& state) {
        const Vector velocity = state.segment<3>(3);
        const Vector spin     = state.segment<3>(6);
        const double speed    = velocity.norm();

        Vector acceleration(0.0, 0.0, -ball_params_.gravity);

        if (speed > 1e-6) {
            acceleration -= drag_factor * speed * velocity;
            acceleration += magnus_factor * spin.cross(velocity);
        }

        XVec delta = XVec::Zero();
        delta.head<3>() = velocity;
        delta.segment<3>(3) = acceleration;
        return delta;
    };

    const XVec k1 = derivative(x);
    const XVec k2 = derivative(x + 0.5 * dt_seconds * k1);
    const XVec k3 = derivative(x + 0.5 * dt_seconds * k2);
    const XVec k4 = derivative(x + dt_seconds * k3);

    return x + (dt_seconds / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace pingpong_tracker::predictor
