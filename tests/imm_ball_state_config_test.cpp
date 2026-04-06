#include "module/predictor/predictor_config.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <numeric>

namespace {

using pingpong_tracker::predictor::BallState;
using pingpong_tracker::predictor::ImmBallState;
using pingpong_tracker::predictor::PredictorConfig;
using pingpong_tracker::predictor::PredictorFilterType;
using pingpong_tracker::predictor::load_predictor_config;
using pingpong_tracker::predictor::make_predictor;

TEST(PredictorConfig, UsesDefaultsWhenPredictorSectionMissing) {
    const auto root = YAML::Load("{}");

    const auto config = load_predictor_config(root);
    ASSERT_TRUE(config.has_value()) << config.error();

    const auto tracker = make_predictor(*config);
    ASSERT_TRUE(std::holds_alternative<ImmBallState>(tracker));
    const auto* imm_tracker = std::get_if<ImmBallState>(&tracker);
    ASSERT_NE(imm_tracker, nullptr);
    const auto probabilities = imm_tracker->mode_probabilities();

    ASSERT_EQ(probabilities.size(), 3U);
    const double sum = std::accumulate(probabilities.begin(), probabilities.end(), 0.0);
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

TEST(PredictorConfig, ParsesCustomUkfAndImmModes) {
    const auto root = YAML::Load(R"yaml(
predictor:
  filter_type: imm_ukf
  reset_interval_seconds: 2.5
  ukf:
    alpha: 0.2
    beta: 2.0
    kappa: 1.0
    process_noise_std: [0.1, 0.1, 0.1, 0.4, 0.4, 0.4, 4.0, 4.0, 4.0]
    measurement_noise_std: [0.02, 0.03, 0.04]
    initial_covariance: [0.2, 0.2, 0.2, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0]
  imm:
    bounce_mode_index: 1
    bounce_height_gate: 0.02
    bounce_prior_boost: 5.0
    modes:
      - process_noise_scale: 1.0
        ball_model:
          drag_coefficient: 0.3
          lift_coefficient: 0.1
      - process_noise_scale: 2.0
        enable_table_bounce: true
        table_height: 0.02
        restitution: 0.8
        tangential_damping: 0.85
        spin_damping_on_hit: 0.9
    transition_matrix:
      - [0.95, 0.05]
      - [0.10, 0.90]
)yaml");

    const auto config = load_predictor_config(root);
    ASSERT_TRUE(config.has_value()) << config.error();

    EXPECT_DOUBLE_EQ(config->ukf.alpha, 0.2);
    EXPECT_DOUBLE_EQ(config->ukf.kappa, 1.0);
    EXPECT_DOUBLE_EQ(config->ukf.measurement_noise_std[2], 0.04);
    EXPECT_DOUBLE_EQ(config->reset_interval.count(), 2.5);

    ASSERT_EQ(config->imm.modes.size(), 2U);
    EXPECT_TRUE(config->imm.modes[1].enable_table_bounce);
    EXPECT_DOUBLE_EQ(config->imm.modes[1].restitution, 0.8);
    EXPECT_EQ(config->imm.transition_matrix.rows(), 2);
    EXPECT_EQ(config->imm.transition_matrix.cols(), 2);

    const auto tracker = make_predictor(*config);
    ASSERT_TRUE(std::holds_alternative<ImmBallState>(tracker));
    const auto* imm_tracker = std::get_if<ImmBallState>(&tracker);
    ASSERT_NE(imm_tracker, nullptr);
    EXPECT_EQ(imm_tracker->mode_probabilities().size(), 2U);
}

TEST(PredictorConfig, RejectsInvalidTransitionMatrixShape) {
    const auto root = YAML::Load(R"yaml(
predictor:
  imm:
    modes:
      - {}
      - {}
    transition_matrix:
      - [0.9, 0.1]
)yaml");

    const auto config = load_predictor_config(root);
    ASSERT_FALSE(config.has_value());
    EXPECT_NE(config.error().find("row count mismatch"), std::string::npos);
}

TEST(PredictorConfig, BuildsSingleModelUkfWhenRequested) {
    const auto root = YAML::Load(R"yaml(
predictor:
  filter_type: ukf
  ball_model:
    drag_coefficient: 0.33
    lift_coefficient: 0.05
)yaml");

    const auto config = load_predictor_config(root);
    ASSERT_TRUE(config.has_value()) << config.error();
    EXPECT_EQ(config->filter_type, PredictorFilterType::Ukf);
    EXPECT_DOUBLE_EQ(config->ball_model.drag_coefficient, 0.33);

    const auto tracker = make_predictor(*config);
    EXPECT_TRUE(std::holds_alternative<BallState>(tracker));
}

TEST(PredictorConfig, IgnoresInvalidImmSectionWhenUkfIsSelected) {
    const auto root = YAML::Load(R"yaml(
predictor:
  filter_type: ukf
  imm:
    transition_matrix:
      - [1.0]
)yaml");

    const auto config = load_predictor_config(root);
    ASSERT_TRUE(config.has_value()) << config.error();
    EXPECT_EQ(config->filter_type, PredictorFilterType::Ukf);
}

TEST(PredictorConfig, BuildsImmTrackerForImmAlias) {
    const auto root = YAML::Load(R"yaml(
predictor:
  filter_type: imm
)yaml");

    const auto config = load_predictor_config(root);
    ASSERT_TRUE(config.has_value()) << config.error();
    EXPECT_EQ(config->filter_type, PredictorFilterType::ImmUkf);

    const auto tracker = make_predictor(*config);
    EXPECT_TRUE(std::holds_alternative<ImmBallState>(tracker));
}

}  // namespace
