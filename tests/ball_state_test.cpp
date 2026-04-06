#include "module/predictor/ball_state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <random>

namespace {

using pingpong_tracker::predictor::BallState;
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

}  // namespace

TEST(BallStateUkf, TracksNoisy3DTrajectory) {
    UKFParameters ukf_params;
    ukf_params.process_noise_std = {0.01, 0.01, 0.01, 0.10, 0.10, 0.10, 1.0, 1.0, 1.0};
    ukf_params.measurement_noise_std = {0.01, 0.01, 0.01};
    ukf_params.initial_covariance = {0.05, 0.05, 0.05, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};

    BallState::BallModelParameters model_params;
    model_params.drag_coefficient = 0.0;
    model_params.lift_coefficient = 0.0;

    BallState tracker{ukf_params, model_params};

    KinematicState truth{
        .position = Eigen::Vector3d{0.0, 0.0, 1.0},
        .velocity = Eigen::Vector3d{4.0, -1.0, 3.0},
    };

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.01);

    constexpr double dt_seconds = 1.0 / 120.0;
    const auto dt = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt_seconds));

    auto stamp = Clock::time_point{};
    constexpr int steps = 120;

    for (int i = 0; i < steps; ++i) {
        stamp += dt;
        truth = propagate_ballistic(truth, dt_seconds, model_params.gravity);

        BallState::ZVec measurement;
        measurement << truth.position.x() + noise(rng), truth.position.y() + noise(rng),
            truth.position.z() + noise(rng);

        ASSERT_TRUE(tracker.update(measurement, stamp));
    }

    ASSERT_TRUE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), static_cast<std::size_t>(steps));
    EXPECT_LT((tracker.position() - truth.position).norm(), 0.08);
    EXPECT_LT((tracker.velocity() - truth.velocity).norm(), 0.40);

    const auto count_before_predict = tracker.update_count();
    const auto position_before = tracker.position();

    stamp += std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(20));
    tracker.predict(stamp);

    EXPECT_EQ(tracker.update_count(), count_before_predict);
    EXPECT_GT((tracker.position() - position_before).norm(), 1e-6);

    const auto trajectory =
        tracker.predict_trajectory(std::chrono::milliseconds(300), std::chrono::milliseconds(10));
    ASSERT_EQ(trajectory.size(), 30U);
    EXPECT_LT(trajectory.back().z(), trajectory.front().z());
}

TEST(BallStateUkf, ResetsAfterTimeoutAndReinitializes) {
    BallState tracker;

    BallState::ZVec measurement;
    measurement << 0.0, 0.0, 1.0;

    const auto t0 = Clock::time_point{};
    ASSERT_TRUE(tracker.update(measurement, t0));
    ASSERT_TRUE(tracker.initialized());

    tracker.predict(t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(2)));

    EXPECT_FALSE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), 0U);

    BallState::ZVec reinit_measurement;
    reinit_measurement << 1.0, 0.0, 1.0;
    ASSERT_TRUE(
        tracker.update(reinit_measurement,
                       t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(2))));

    EXPECT_TRUE(tracker.initialized());
    EXPECT_EQ(tracker.update_count(), 1U);
    EXPECT_NEAR(tracker.position().x(), 1.0, 1e-12);
}
