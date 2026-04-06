#include "imm_ball_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Cholesky>

#include "utility/time/delta_time.hpp"

namespace pingpong_tracker::predictor {

namespace {

auto uniform_vector(std::size_t n) -> Eigen::VectorXd {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));
    if (n == 0) {
        return out;
    }
    out.array() = 1.0 / static_cast<double>(n);
    return out;
}

}  // namespace

ImmBallState::ImmBallState() : ImmBallState(UKFParameters{}, default_imm_parameters()) {
}

ImmBallState::ImmBallState(UKFParameters ukf_params, ImmParameters imm_params)
    : ukf_params_{std::move(ukf_params)},
      bounce_mode_index_{imm_params.bounce_mode_index},
      bounce_height_gate_{std::max(imm_params.bounce_height_gate, 0.0)},
      bounce_prior_boost_{std::max(imm_params.bounce_prior_boost, 1.0)} {
    if (imm_params.modes.empty()) {
        imm_params = default_imm_parameters();
        bounce_mode_index_ = imm_params.bounce_mode_index;
    }

    modes_.reserve(imm_params.modes.size());
    for (const auto& mode : imm_params.modes) {
        modes_.push_back(ModeState{
            .ukf = UKF{ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa},
            .params = mode,
        });
    }

    transition_matrix_ = imm_params.transition_matrix;
    if (transition_matrix_.rows() != static_cast<Eigen::Index>(modes_.size()) ||
        transition_matrix_.cols() != static_cast<Eigen::Index>(modes_.size())) {
        transition_matrix_ = default_transition_matrix(modes_.size());
    }

    for (Eigen::Index i = 0; i < transition_matrix_.rows(); ++i) {
        transition_matrix_.row(i) = transition_matrix_.row(i).cwiseMax(0.0);
        const double row_sum = transition_matrix_.row(i).sum();
        if (row_sum <= kProbabilityFloor) {
            transition_matrix_.row(i).setConstant(
                1.0 / static_cast<double>(std::max<std::size_t>(modes_.size(), 1U)));
        } else {
            transition_matrix_.row(i) /= row_sum;
        }
    }

    if (bounce_mode_index_ >= modes_.size()) {
        bounce_mode_index_ = modes_.size() - 1;
    }

    mode_probabilities_ = uniform_vector(modes_.size());
}

auto ImmBallState::predict(Clock::time_point stamp) -> void {
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

    imm_predict(dt_seconds);
    fuse_modes();
    time_stamp_ = stamp;
}

auto ImmBallState::update(const ZVec& measurement, Clock::time_point stamp) -> bool {
    if (!initialized_) {
        initialize_from_measurement(measurement, stamp);
        return true;
    }

    predict(stamp);
    if (!initialized_) {
        initialize_from_measurement(measurement, stamp);
        return true;
    }

    if (!imm_update(measurement)) {
        return false;
    }

    ++update_count_;
    time_stamp_ = stamp;
    return true;
}

auto ImmBallState::update(const ZVec& measurement) -> bool {
    return update(measurement, Clock::now());
}

auto ImmBallState::reset() noexcept -> void {
    initialized_  = false;
    update_count_ = 0;

    for (auto& mode : modes_) {
        mode.ukf = UKF{ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa};
    }

    mode_probabilities_ = uniform_vector(modes_.size());
    fused_x_.setZero();
    fused_p_.setIdentity();
}

auto ImmBallState::set_reset_interval(std::chrono::duration<double> interval) noexcept -> void {
    reset_interval_ = std::max(interval, std::chrono::duration<double>::zero());
}

auto ImmBallState::initialized() const noexcept -> bool {
    return initialized_;
}

auto ImmBallState::update_count() const noexcept -> std::size_t {
    return update_count_;
}

auto ImmBallState::reset_interval() const noexcept -> std::chrono::duration<double> {
    return reset_interval_;
}

auto ImmBallState::state() const -> XVec {
    return fused_x_;
}

auto ImmBallState::covariance() const -> PMat {
    return fused_p_;
}

auto ImmBallState::position() const -> Vector {
    return fused_x_.head<3>();
}

auto ImmBallState::velocity() const -> Vector {
    return fused_x_.segment<3>(3);
}

auto ImmBallState::spin() const -> Vector {
    return fused_x_.tail<3>();
}

auto ImmBallState::mode_probabilities() const -> std::vector<double> {
    std::vector<double> out(static_cast<std::size_t>(mode_probabilities_.size()), 0.0);
    for (Eigen::Index i = 0; i < mode_probabilities_.size(); ++i) {
        out[static_cast<std::size_t>(i)] = mode_probabilities_(i);
    }
    return out;
}

auto ImmBallState::dominant_mode() const -> std::size_t {
    if (mode_probabilities_.size() == 0) {
        return 0U;
    }
    Eigen::Index best = 0;
    mode_probabilities_.maxCoeff(&best);
    return static_cast<std::size_t>(best);
}

auto ImmBallState::predict_trajectory(std::chrono::duration<double> horizon,
                                      std::chrono::duration<double> step) const
    -> std::vector<Vector> {
    if (!initialized_ || horizon <= std::chrono::duration<double>::zero() ||
        step <= std::chrono::duration<double>::zero()) {
        return {};
    }

    std::vector<Vector> trajectory;
    const auto steps = static_cast<std::size_t>(horizon / step);
    trajectory.reserve(steps);

    auto rollout_modes = modes_;
    auto rollout_probabilities = mode_probabilities_;
    for (std::size_t i = 0; i < steps; ++i) {
        imm_predict_step(rollout_modes, rollout_probabilities, step.count());
        const auto fused = fuse_modes_snapshot(rollout_modes, rollout_probabilities);
        trajectory.push_back(fused.x.head<3>());
    }

    return trajectory;
}

auto ImmBallState::default_imm_parameters() -> ImmParameters {
    ImmParameters params;

    ModeParameters low_spin;
    low_spin.ball_model.drag_coefficient = 0.40;
    low_spin.ball_model.lift_coefficient = 0.06;
    low_spin.process_noise_scale = 1.0;

    ModeParameters high_spin;
    high_spin.ball_model.drag_coefficient = 0.52;
    high_spin.ball_model.lift_coefficient = 0.24;
    high_spin.process_noise_scale = 1.4;

    ModeParameters bounce;
    bounce.ball_model.drag_coefficient = 0.47;
    bounce.ball_model.lift_coefficient = 0.15;
    bounce.process_noise_scale = 2.0;
    bounce.enable_table_bounce = true;
    bounce.table_height = 0.0;
    bounce.restitution = 0.86;
    bounce.tangential_damping = 0.86;
    bounce.spin_damping_on_hit = 0.92;

    params.modes = {low_spin, high_spin, bounce};
    params.transition_matrix.resize(3, 3);
    params.transition_matrix << 0.94, 0.05, 0.01, 0.05, 0.93, 0.02, 0.12, 0.12, 0.76;
    params.bounce_mode_index = 2;
    params.bounce_height_gate = 0.04;
    params.bounce_prior_boost = 3.0;
    return params;
}

auto ImmBallState::default_transition_matrix(std::size_t mode_count) -> Eigen::MatrixXd {
    if (mode_count == 0) {
        return {};
    }

    if (mode_count == 1) {
        return Eigen::MatrixXd::Ones(1, 1);
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Constant(
        static_cast<Eigen::Index>(mode_count), static_cast<Eigen::Index>(mode_count),
        0.05 / static_cast<double>(mode_count - 1));
    matrix.diagonal().setConstant(0.95);
    return matrix;
}

auto ImmBallState::normalize_probabilities(const Eigen::VectorXd& raw) -> Eigen::VectorXd {
    if (raw.size() == 0) {
        return raw;
    }

    Eigen::VectorXd clamped = raw.cwiseMax(kProbabilityFloor);
    const double total = clamped.sum();
    if (total <= kProbabilityFloor) {
        clamped.setConstant(1.0 / static_cast<double>(clamped.size()));
        return clamped;
    }

    return clamped / total;
}

auto ImmBallState::make_positive_definite(const PMat& matrix) -> PMat {
    PMat sym = 0.5 * (matrix + matrix.transpose());
    double jitter = 1e-9;

    for (int i = 0; i < 6; ++i) {
        Eigen::LLT<PMat> llt(sym);
        if (llt.info() == Eigen::Success) {
            return sym;
        }
        sym += PMat::Identity() * jitter;
        jitter *= 10.0;
    }

    return sym;
}

auto ImmBallState::initialize_from_measurement(const ZVec& measurement, Clock::time_point stamp) -> void {
    XVec x0 = XVec::Zero();
    x0.head<3>() = measurement;

    PMat p0 = PMat::Zero();
    for (int i = 0; i < UKFParameters::StateDim; ++i) {
        p0(i, i) = ukf_params_.initial_covariance[i];
    }

    for (auto& mode : modes_) {
        mode.ukf = UKF{x0, p0, ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa};
    }

    mode_probabilities_ = uniform_vector(modes_.size());
    initialized_ = true;
    update_count_ = 1;
    time_stamp_ = stamp;
    fuse_modes();
}

auto ImmBallState::imm_predict(double dt_seconds) -> void {
    imm_predict_step(modes_, mode_probabilities_, dt_seconds);
}

auto ImmBallState::imm_predict_step(std::vector<ModeState>& modes, Eigen::VectorXd& mode_probabilities,
                                    double dt_seconds) const -> void {
    const auto mode_count = modes.size();
    if (mode_count == 0) {
        return;
    }

    std::vector<XVec> prev_x(mode_count, XVec::Zero());
    std::vector<PMat> prev_p(mode_count, PMat::Identity());
    for (std::size_t i = 0; i < mode_count; ++i) {
        prev_x[i] = modes[i].ukf.x;
        prev_p[i] = modes[i].ukf.covariance();
    }

    Eigen::VectorXd predicted_probs = normalize_probabilities(transition_matrix_.transpose() * mode_probabilities);

    std::vector<XVec> mixed_x(mode_count, XVec::Zero());
    std::vector<PMat> mixed_p(mode_count, PMat::Zero());

    for (std::size_t j = 0; j < mode_count; ++j) {
        const double c_j = std::max(predicted_probs(static_cast<Eigen::Index>(j)), kProbabilityFloor);

        for (std::size_t i = 0; i < mode_count; ++i) {
            const double mu_ij =
                transition_matrix_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) *
                mode_probabilities(static_cast<Eigen::Index>(i)) / c_j;
            mixed_x[j] += mu_ij * prev_x[i];
        }

        for (std::size_t i = 0; i < mode_count; ++i) {
            const double mu_ij =
                transition_matrix_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) *
                mode_probabilities(static_cast<Eigen::Index>(i)) / c_j;
            const XVec diff = prev_x[i] - mixed_x[j];
            mixed_p[j] += mu_ij * (prev_p[i] + diff * diff.transpose());
        }
        mixed_p[j] = make_positive_definite(mixed_p[j]);
    }

    for (std::size_t j = 0; j < mode_count; ++j) {
        modes[j].ukf =
            UKF{mixed_x[j], mixed_p[j], ukf_params_.alpha, ukf_params_.beta, ukf_params_.kappa};
        const auto transition = [this, &modes, dt_seconds, j](const XVec& x) {
            return propagate_state(x, dt_seconds, modes[j].params);
        };
        modes[j].ukf.predict(transition, process_noise(dt_seconds, modes[j].params));
    }

    mode_probabilities = predicted_probs;
}

auto ImmBallState::imm_update(const ZVec& measurement) -> bool {
    const auto mode_count = modes_.size();
    if (mode_count == 0) {
        return false;
    }

    Eigen::VectorXd prior_probs = mode_probabilities_;
    if (bounce_mode_index_ < mode_count && should_boost_bounce_prior(measurement)) {
        prior_probs(static_cast<Eigen::Index>(bounce_mode_index_)) *= bounce_prior_boost_;
        prior_probs = normalize_probabilities(prior_probs);
    }

    const auto observe = [](const XVec& x) -> ZVec { return x.head<3>(); };

    Eigen::VectorXd log_weights =
        Eigen::VectorXd::Constant(static_cast<Eigen::Index>(mode_count), -std::numeric_limits<double>::infinity());

    bool any_success = false;
    for (std::size_t j = 0; j < mode_count; ++j) {
        const ZVec innovation = measurement - modes_[j].ukf.x.head<3>();
        Eigen::Matrix3d innovation_covariance =
            modes_[j].ukf.covariance().topLeftCorner<3, 3>() + measurement_noise();

        const double log_likelihood = log_likelihood_from_innovation(innovation, innovation_covariance);
        if (!std::isfinite(log_likelihood)) {
            continue;
        }

        const bool updated = modes_[j].ukf.update(measurement, observe, measurement_noise(),
                                                  pingpong_tracker::util::DefaultAdd,
                                                  pingpong_tracker::util::DefaultSubtract);
        if (!updated) {
            continue;
        }

        any_success = true;
        const double prior = std::max(prior_probs(static_cast<Eigen::Index>(j)), kProbabilityFloor);
        log_weights(static_cast<Eigen::Index>(j)) = std::log(prior) + log_likelihood;
    }

    if (!any_success) {
        return false;
    }

    const double max_log = log_weights.maxCoeff();
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(log_weights.size());
    for (Eigen::Index i = 0; i < log_weights.size(); ++i) {
        if (std::isfinite(log_weights(i))) {
            weights(i) = std::exp(log_weights(i) - max_log);
        }
    }
    mode_probabilities_ = normalize_probabilities(weights);
    fuse_modes();
    return true;
}

auto ImmBallState::fuse_modes_snapshot(const std::vector<ModeState>& modes,
                                       const Eigen::VectorXd& mode_probabilities) -> FusedEstimate {
    FusedEstimate fused;

    if (modes.empty()) {
        return fused;
    }

    fused.x.setZero();
    for (std::size_t i = 0; i < modes.size(); ++i) {
        fused.x += mode_probabilities(static_cast<Eigen::Index>(i)) * modes[i].ukf.x;
    }

    fused.p.setZero();
    for (std::size_t i = 0; i < modes.size(); ++i) {
        const XVec diff = modes[i].ukf.x - fused.x;
        fused.p += mode_probabilities(static_cast<Eigen::Index>(i)) *
                   (modes[i].ukf.covariance() + diff * diff.transpose());
    }

    fused.p = 0.5 * (fused.p + fused.p.transpose());
    return fused;
}

auto ImmBallState::fuse_modes() -> void {
    if (modes_.empty()) {
        fused_x_.setZero();
        fused_p_.setIdentity();
        return;
    }
    const auto fused = fuse_modes_snapshot(modes_, mode_probabilities_);
    fused_x_ = fused.x;
    fused_p_ = fused.p;
}

auto ImmBallState::process_noise(double dt_seconds, const ModeParameters& mode) const -> UKF::QMat {
    UKF::QMat q = UKF::QMat::Zero();
    const double scale = std::max(mode.process_noise_scale, 1e-3);

    for (int i = 0; i < UKFParameters::StateDim; ++i) {
        const double sigma = ukf_params_.process_noise_std[i] * scale;
        q(i, i)            = sigma * sigma * dt_seconds;
    }

    return q;
}

auto ImmBallState::measurement_noise() const -> UKF::RMat {
    UKF::RMat r = UKF::RMat::Zero();
    for (int i = 0; i < UKFParameters::ObsDim; ++i) {
        const double sigma = ukf_params_.measurement_noise_std[i];
        r(i, i)            = sigma * sigma;
    }
    return r;
}

auto ImmBallState::propagate_state(const XVec& x, double dt_seconds, const ModeParameters& mode) const -> XVec {
    const auto& ball = mode.ball_model;

    const double area = std::numbers::pi_v<double> * ball.radius * ball.radius;
    const double inv_mass = 1.0 / ball.mass;
    const double common = 0.5 * ball.air_density * area * inv_mass;

    const double drag_factor = common * ball.drag_coefficient;
    const double magnus_factor = common * ball.lift_coefficient * ball.radius;

    const auto derivative = [drag_factor, magnus_factor, &ball](const XVec& state) {
        const Vector velocity = state.segment<3>(3);
        const Vector spin     = state.segment<3>(6);
        const double speed    = velocity.norm();

        Vector acceleration(0.0, 0.0, -ball.gravity);
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

    XVec next = x + (dt_seconds / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    if (mode.enable_table_bounce) {
        const double ball_bottom = next(2) - ball.radius;
        if (ball_bottom <= mode.table_height && next(5) < 0.0) {
            next(2) = mode.table_height + ball.radius;
            next(3) *= mode.tangential_damping;
            next(4) *= mode.tangential_damping;
            next(5) = -next(5) * mode.restitution;
            next.tail<3>() *= mode.spin_damping_on_hit;
        }
    }

    return next;
}

auto ImmBallState::should_boost_bounce_prior(const ZVec& measurement) const -> bool {
    if (bounce_mode_index_ >= modes_.size()) {
        return false;
    }

    const auto& bounce_mode = modes_[bounce_mode_index_].params;
    const double distance_to_table =
        std::abs(measurement(2) - (bounce_mode.table_height + bounce_mode.ball_model.radius));
    const bool descending = fused_x_(5) < -0.2;
    return distance_to_table <= bounce_height_gate_ && descending;
}

auto ImmBallState::log_likelihood_from_innovation(
    const ZVec& innovation, const Eigen::Matrix3d& innovation_covariance) const -> double {
    Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        return -std::numeric_limits<double>::infinity();
    }

    const auto diagonal = ldlt.vectorD();
    if ((diagonal.array() <= 0.0).any()) {
        return -std::numeric_limits<double>::infinity();
    }

    const double log_det = diagonal.array().log().sum();
    const double mahalanobis = innovation.dot(ldlt.solve(innovation));
    return -0.5 * (static_cast<double>(UKFParameters::ObsDim) * std::log(2.0 * std::numbers::pi) +
                   log_det + mahalanobis);
}

}  // namespace pingpong_tracker::predictor
