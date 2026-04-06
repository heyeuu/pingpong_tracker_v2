#include "module/predictor/imm_ball_state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <numeric>
#include <random>

namespace {

using pingpong_tracker::predictor::ImmBallState;
using pingpong_tracker::predictor::UKFParameters;
using pingpong_tracker::util::Clock;

struct KinematicState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

auto propagate_ballistic(KinematicState state, double dt, double gravity) -> KinematicState {
    const Eigen::Vector3d acceleration{0.0, 0.0, -gravity};
    state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;
    state.velocity += acceleration * dt;
    return state;
}

auto build_imm_parameters() -> ImmBallState::ImmParameters {
    ImmBallState::ImmParameters imm_params;

    ImmBallState::ModeParameters low_spin;
    low_spin.ball_model.drag_coefficient = 0.0;
    low_spin.ball_model.lift_coefficient = 0.0;
    low_spin.process_noise_scale = 1.0;

    ImmBallState::ModeParameters high_spin = low_spin;
    high_spin.process_noise_scale = 1.5;

    ImmBallState::ModeParameters bounce = low_spin;
    bounce.process_noise_scale = 2.0;
    bounce.enable_table_bounce = true;
    bounce.table_height = 0.0;
    bounce.restitution = 0.84;
    bounce.tangential_damping = 0.88;

    imm_params.modes = {low_spin, high_spin, bounce};
    imm_params.transition_matrix.resize(3, 3);
    imm_params.transition_matrix << 0.95, 0.04, 0.01, 0.06, 0.90, 0.04, 0.10, 0.10, 0.80;
    imm_params.bounce_mode_index = 2;
    imm_params.bounce_prior_boost = 4.0;
    imm_params.bounce_height_gate = 0.03;
    return imm_params;
}

auto build_switching_imm_parameters() -> ImmBallState::ImmParameters {
    ImmBallState::ImmParameters imm_params;

    ImmBallState::ModeParameters nominal;
    nominal.ball_model.drag_coefficient = 0.0;
    nominal.ball_model.lift_coefficient = 0.0;
    nominal.ball_model.gravity = 9.81;
    nominal.process_noise_scale = 1.0;

    ImmBallState::ModeParameters aggressive = nominal;
    aggressive.ball_model.drag_coefficient = 1.2;
    aggressive.ball_model.gravity = 14.0;
    aggressive.process_noise_scale = 1.6;

    imm_params.modes = {nominal, aggressive};
    imm_params.transition_matrix.resize(2, 2);
    imm_params.transition_matrix << 0.10, 0.90, 0.85, 0.15;
    imm_params.bounce_mode_index = 1;
    return imm_params;
}

}  // namespace

TEST(ImmBallState, TracksNoisy3DTrajectoryAndKeepsValidModeProbabilities) {
    UKFParameters ukf_params;
    ukf_params.process_noise_std = {0.01, 0.01, 0.01, 0.10, 0.10, 0.10, 1.0, 1.0, 1.0};
    ukf_params.measurement_noise_std = {0.01, 0.01, 0.01};
    ukf_params.initial_covariance = {0.05, 0.05, 0.05, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};

    ImmBallState tracker{ukf_params, build_imm_parameters()};

    KinematicState truth{
        .position = Eigen::Vector3d{0.0, 0.0, 1.0},
        .velocity = Eigen::Vector3d{3.5, -1.2, 3.4},
    };

    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.01);

    constexpr double dt_seconds = 1.0 / 120.0;
    const auto dt = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt_seconds));

    auto stamp = Clock::time_point{};
    constexpr int steps = 120;
    for (int i = 0; i < steps; ++i) {
        stamp += dt;
        truth = propagate_ballistic(truth, dt_seconds, 9.81);

        ImmBallState::ZVec measurement;
        measurement << truth.position.x() + noise(rng), truth.position.y() + noise(rng),
            truth.position.z() + noise(rng);
        ASSERT_TRUE(tracker.update(measurement, stamp));
    }

    ASSERT_TRUE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), static_cast<std::size_t>(steps));
    EXPECT_LT((tracker.position() - truth.position).norm(), 0.10);
    EXPECT_LT((tracker.velocity() - truth.velocity).norm(), 0.55);

    const auto probs = tracker.mode_probabilities();
    ASSERT_EQ(probs.size(), 3U);
    for (double prob : probs) {
        EXPECT_GE(prob, 0.0);
    }
    const double prob_sum = std::accumulate(probs.begin(), probs.end(), 0.0);
    EXPECT_NEAR(prob_sum, 1.0, 1e-9);
    EXPECT_LT(tracker.dominant_mode(), probs.size());

    const auto trajectory =
        tracker.predict_trajectory(std::chrono::milliseconds(200), std::chrono::milliseconds(10));
    ASSERT_EQ(trajectory.size(), 20U);
    for (const auto& point : trajectory) {
        EXPECT_TRUE(point.allFinite());
    }
    EXPECT_GT((trajectory.front() - tracker.position()).norm(), 1e-6);
    EXPECT_GT((trajectory.back() - trajectory.front()).norm(), 1e-6);
}

TEST(ImmBallState, ResetsAfterTimeoutAndReinitializes) {
    ImmBallState tracker;

    ImmBallState::ZVec measurement;
    measurement << 0.0, 0.0, 1.0;

    const auto t0 = Clock::time_point{};
    ASSERT_TRUE(tracker.update(measurement, t0));
    ASSERT_TRUE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), 1U);

    tracker.predict(t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(2)));
    EXPECT_FALSE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), 0U);

    ImmBallState::ZVec reinit_measurement;
    reinit_measurement << 1.0, -0.2, 0.9;

    ASSERT_TRUE(tracker.update(
        reinit_measurement,
        t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(2))));
    EXPECT_TRUE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), 1U);
    EXPECT_NEAR(tracker.position().x(), 1.0, 1e-12);
}

TEST(ImmBallState, PredictTrajectoryMatchesDirectPredictStep) {
    UKFParameters ukf_params;
    ukf_params.process_noise_std = {0.01, 0.01, 0.01, 0.20, 0.20, 0.20, 1.0, 1.0, 1.0};
    ukf_params.measurement_noise_std = {0.001, 0.001, 0.001};
    ukf_params.initial_covariance = {0.02, 0.02, 0.02, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};

    ImmBallState tracker{ukf_params, build_switching_imm_parameters()};

    const auto t0 = Clock::time_point{};
    const auto dt = std::chrono::milliseconds{20};

    ImmBallState::ZVec z0;
    z0 << 0.00, 0.00, 1.00;
    ASSERT_TRUE(tracker.update(z0, t0));

    ImmBallState::ZVec z1;
    z1 << 0.14, -0.02, 1.09;
    ASSERT_TRUE(tracker.update(z1, t0 + std::chrono::duration_cast<Clock::duration>(dt)));

    ImmBallState::ZVec z2;
    z2 << 0.29, -0.04, 1.14;
    ASSERT_TRUE(tracker.update(z2, t0 + std::chrono::duration_cast<Clock::duration>(2 * dt)));

    tracker.predict(t0 + std::chrono::duration_cast<Clock::duration>(3 * dt));

    const auto trajectory = tracker.predict_trajectory(dt, dt);
    ASSERT_EQ(trajectory.size(), 1U);

    auto predicted_tracker = tracker;
    predicted_tracker.predict(t0 + std::chrono::duration_cast<Clock::duration>(4 * dt));

    EXPECT_LT((trajectory.front() - predicted_tracker.position()).norm(), 1e-9);
}
