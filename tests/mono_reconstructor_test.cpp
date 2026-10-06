#include "module/reconstruction/mono_reconstructor.hpp"

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace {

using pingpong_tracker::Ball2D;
using pingpong_tracker::reconstruction::MonoReconstructor;
using pingpong_tracker::reconstruction::load_mono_reconstructor_config;

auto make_reconstructor_from_yaml(const char* yaml_text) -> MonoReconstructor {
    const auto root = YAML::Load(yaml_text);
    const auto config = load_mono_reconstructor_config(root);
    if (!config) {
        throw std::runtime_error(config.error());
    }
    return MonoReconstructor{*config};
}

}  // namespace

TEST(MonoReconstructor, ReconstructsValidObservation) {
    auto reconstructor = make_reconstructor_from_yaml(R"yaml(
reconstruction:
  mono:
    fx: 1000.0
    fy: 1000.0
    cx: 320.0
    cy: 240.0
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
    ball_radius_m: 0.02
    min_confidence: 0.5
    min_radius_px: 2.0
    max_radius_px: 200.0
    min_depth_m: 0.2
    max_depth_m: 8.0
    image_width_px: 640
    image_height_px: 480
    expected_radius_px: 10.0
    radius_tolerance_px: 5.0
    edge_margin_px: 20.0
)yaml");

    const Ball2D ball{
        .center = {420.0F, 290.0F},
        .radius = 10.0F,
        .confidence = 0.90F,
    };

    const auto reconstructed = reconstructor.reconstruct(ball, 0.25);
    ASSERT_TRUE(reconstructed.has_value()) << reconstructed.error();

    EXPECT_NEAR(reconstructed->timestamp_seconds, 0.25, 1e-12);
    EXPECT_NEAR(reconstructed->position.x(), 0.2, 1e-6);
    EXPECT_NEAR(reconstructed->position.y(), 0.1, 1e-6);
    EXPECT_NEAR(reconstructed->position.z(), 2.0, 1e-6);
    EXPECT_GT(reconstructed->quality_score, 0.0);
    EXPECT_LE(reconstructed->quality_score, 1.0);
}

TEST(MonoReconstructor, RejectsOutOfImageMeasurement) {
    auto reconstructor = make_reconstructor_from_yaml(R"yaml(
reconstruction:
  mono:
    fx: 1000.0
    fy: 1000.0
    cx: 320.0
    cy: 240.0
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
    image_width_px: 640
    image_height_px: 480
)yaml");

    const Ball2D ball{
        .center = {-1.0F, 240.0F},
        .radius = 10.0F,
        .confidence = 0.95F,
    };

    const auto reconstructed = reconstructor.reconstruct(ball, 0.10);
    ASSERT_FALSE(reconstructed.has_value());
    EXPECT_NE(reconstructed.error().find("out of image bounds"), std::string::npos);
}

TEST(MonoReconstructor, RejectsLowConfidenceMeasurement) {
    auto reconstructor = make_reconstructor_from_yaml(R"yaml(
reconstruction:
  mono:
    fx: 1000.0
    fy: 1000.0
    cx: 320.0
    cy: 240.0
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
    min_confidence: 0.7
)yaml");

    const Ball2D ball{
        .center = {320.0F, 240.0F},
        .radius = 10.0F,
        .confidence = 0.50F,
    };

    const auto reconstructed = reconstructor.reconstruct(ball, 0.10);
    ASSERT_FALSE(reconstructed.has_value());
    EXPECT_NE(reconstructed.error().find("Low confidence"), std::string::npos);
}

TEST(MonoReconstructorConfig, FailsWhenRequiredFieldMissing) {
    const auto root = YAML::Load(R"yaml(
reconstruction:
  mono:
    fy: 1000.0
    cx: 320.0
    cy: 240.0
)yaml");

    const auto config = load_mono_reconstructor_config(root);
    ASSERT_FALSE(config.has_value());
    EXPECT_NE(config.error().find("reconstruction.mono.fx"), std::string::npos);
}
