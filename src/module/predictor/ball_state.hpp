#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include <eigen3/Eigen/Core>

#include "module/predictor/ball_model_parameters.hpp"
#include "module/predictor/ukf_parameters.hpp"
#include "utility/time/clock.hpp"

namespace pingpong_tracker::predictor {

class BallState {
public:
    using BallModelParameters = predictor::BallModelParameters;
    using UKF    = UKFParameters::UKF;
    using XVec   = UKF::XVec;
    using ZVec   = UKF::ZVec;
    using PMat   = UKF::PMat;
    using Clock  = util::Clock;
    using Vector = Eigen::Vector3d;

    BallState();
    explicit BallState(UKFParameters ukf_params, BallModelParameters ball_params);

    auto predict(Clock::time_point stamp) -> void;
    auto update(const ZVec& measurement, Clock::time_point stamp) -> bool;
    auto update(const ZVec& measurement) -> bool;

    auto reset() noexcept -> void;
    auto set_reset_interval(std::chrono::duration<double> interval) noexcept -> void;

    [[nodiscard]] auto initialized() const noexcept -> bool;
    [[nodiscard]] auto update_count() const noexcept -> std::size_t;
    [[nodiscard]] auto reset_interval() const noexcept -> std::chrono::duration<double>;

    [[nodiscard]] auto state() const -> XVec;
    [[nodiscard]] auto covariance() const -> PMat;
    [[nodiscard]] auto position() const -> Vector;
    [[nodiscard]] auto velocity() const -> Vector;
    [[nodiscard]] auto spin() const -> Vector;

    [[nodiscard]] auto predict_trajectory(std::chrono::duration<double> horizon,
                                          std::chrono::duration<double> step) const
        -> std::vector<Vector>;

private:
    static constexpr double kMinTimeStepSeconds = 1e-6;

    auto initialize_from_measurement(const ZVec& measurement, Clock::time_point stamp) -> void;
    [[nodiscard]] auto process_noise(double dt_seconds) const -> UKF::QMat;
    [[nodiscard]] auto measurement_noise() const -> UKF::RMat;
    [[nodiscard]] auto propagate_state(const XVec& x, double dt_seconds) const -> XVec;

    bool initialized_ = false;
    std::size_t update_count_ = 0;
    std::chrono::duration<double> reset_interval_{1.0};
    Clock::time_point time_stamp_{};

    UKF ukf_;
    BallModelParameters ball_params_;
    UKFParameters ukf_params_;
};

}  // namespace pingpong_tracker::predictor
