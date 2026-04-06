#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include <eigen3/Eigen/Core>

#include "module/predictor/ball_model_parameters.hpp"
#include "module/predictor/ukf_parameters.hpp"
#include "utility/time/clock.hpp"

namespace pingpong_tracker::predictor {

class ImmBallState {
public:
    using BallModelParameters = predictor::BallModelParameters;
    using UKF    = UKFParameters::UKF;
    using XVec   = UKF::XVec;
    using ZVec   = UKF::ZVec;
    using PMat   = UKF::PMat;
    using Clock  = util::Clock;
    using Vector = Eigen::Vector3d;

    struct ModeParameters {
        BallModelParameters ball_model{};
        double process_noise_scale  = 1.0;
        bool enable_table_bounce    = false;
        double table_height         = 0.0;
        double restitution          = 0.88;
        double tangential_damping   = 0.90;
        double spin_damping_on_hit  = 0.95;
    };

    struct ImmParameters {
        std::vector<ModeParameters> modes;
        Eigen::MatrixXd transition_matrix;
        std::size_t bounce_mode_index = 0;
        double bounce_height_gate     = 0.04;
        double bounce_prior_boost     = 3.0;
    };

    ImmBallState();
    explicit ImmBallState(UKFParameters ukf_params, ImmParameters imm_params);

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

    [[nodiscard]] auto mode_probabilities() const -> std::vector<double>;
    [[nodiscard]] auto dominant_mode() const -> std::size_t;

    [[nodiscard]] auto predict_trajectory(std::chrono::duration<double> horizon,
                                          std::chrono::duration<double> step) const
        -> std::vector<Vector>;

private:
    struct ModeState {
        UKF ukf;
        ModeParameters params{};
    };

    struct FusedEstimate {
        XVec x = XVec::Zero();
        PMat p = PMat::Identity();
    };

    static constexpr double kMinTimeStepSeconds = 1e-6;
    static constexpr double kProbabilityFloor   = 1e-12;

    [[nodiscard]] static auto default_imm_parameters() -> ImmParameters;
    [[nodiscard]] static auto default_transition_matrix(std::size_t mode_count) -> Eigen::MatrixXd;
    [[nodiscard]] static auto normalize_probabilities(const Eigen::VectorXd& raw) -> Eigen::VectorXd;
    [[nodiscard]] static auto make_positive_definite(const PMat& matrix) -> PMat;

    auto initialize_from_measurement(const ZVec& measurement, Clock::time_point stamp) -> void;
    auto imm_predict(double dt_seconds) -> void;
    auto imm_predict_step(std::vector<ModeState>& modes, Eigen::VectorXd& mode_probabilities,
                          double dt_seconds) const -> void;
    auto imm_update(const ZVec& measurement) -> bool;
    auto fuse_modes() -> void;
    [[nodiscard]] static auto fuse_modes_snapshot(const std::vector<ModeState>& modes,
                                                  const Eigen::VectorXd& mode_probabilities)
        -> FusedEstimate;

    [[nodiscard]] auto process_noise(double dt_seconds, const ModeParameters& mode) const -> UKF::QMat;
    [[nodiscard]] auto measurement_noise() const -> UKF::RMat;
    [[nodiscard]] auto propagate_state(const XVec& x, double dt_seconds, const ModeParameters& mode) const
        -> XVec;

    [[nodiscard]] auto should_boost_bounce_prior(const ZVec& measurement) const -> bool;
    [[nodiscard]] auto log_likelihood_from_innovation(const ZVec& innovation,
                                                      const Eigen::Matrix3d& innovation_covariance) const
        -> double;

    bool initialized_ = false;
    std::size_t update_count_ = 0;
    std::chrono::duration<double> reset_interval_{1.0};
    Clock::time_point time_stamp_{};

    std::vector<ModeState> modes_;
    Eigen::MatrixXd transition_matrix_;
    Eigen::VectorXd mode_probabilities_;

    UKFParameters ukf_params_;
    std::size_t bounce_mode_index_ = 0;
    double bounce_height_gate_ = 0.04;
    double bounce_prior_boost_ = 3.0;

    XVec fused_x_ = XVec::Zero();
    PMat fused_p_ = PMat::Identity();
};

}  // namespace pingpong_tracker::predictor
