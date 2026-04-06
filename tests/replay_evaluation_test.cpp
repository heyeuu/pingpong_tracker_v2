#include "module/predictor/replay_evaluation.hpp"

#include <gtest/gtest.h>

namespace {

using pingpong_tracker::predictor::ReplaySample;
using pingpong_tracker::predictor::interpolate_position_at_time;

TEST(ReplayEvaluation, InterpolatesByTimestampInsteadOfSampleIndex) {
    const auto samples = std::vector<ReplaySample>{
        {.timestamp_seconds = 0.00, .position = Eigen::Vector3d{0.00, 0.0, 0.0}},
        {.timestamp_seconds = 0.04, .position = Eigen::Vector3d{0.04, 0.0, 0.0}},
        {.timestamp_seconds = 0.15, .position = Eigen::Vector3d{0.15, 0.0, 0.0}},
    };

    const auto interpolated = interpolate_position_at_time(samples, 0.10, 1);
    ASSERT_TRUE(interpolated.has_value());
    EXPECT_NEAR(interpolated->x(), 0.10, 1e-12);
    EXPECT_NEAR(interpolated->y(), 0.0, 1e-12);
    EXPECT_NEAR(interpolated->z(), 0.0, 1e-12);
}

TEST(ReplayEvaluation, ReturnsNulloptOutsideKnownTimeRange) {
    const auto samples = std::vector<ReplaySample>{
        {.timestamp_seconds = 0.10, .position = Eigen::Vector3d{1.0, 0.0, 0.0}},
        {.timestamp_seconds = 0.20, .position = Eigen::Vector3d{2.0, 0.0, 0.0}},
    };

    EXPECT_FALSE(interpolate_position_at_time(samples, 0.05).has_value());
    EXPECT_FALSE(interpolate_position_at_time(samples, 0.25).has_value());
}

}  // namespace
