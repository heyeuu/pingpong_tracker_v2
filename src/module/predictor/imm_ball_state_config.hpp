#pragma once

#include <chrono>
#include <expected>
#include <string>
#include <variant>

#include <yaml-cpp/yaml.h>

#include "module/predictor/ball_state.hpp"
#include "module/predictor/imm_ball_state.hpp"
#include "module/predictor/ukf_parameters.hpp"

namespace pingpong_tracker::predictor {

enum class PredictorFilterType {
    Ukf,
    ImmUkf,
};

using ConfiguredPredictor = std::variant<BallState, ImmBallState>;

struct PredictorConfig {
    PredictorFilterType filter_type = PredictorFilterType::ImmUkf;
    UKFParameters ukf{};
    BallModelParameters ball_model{};
    ImmBallState::ImmParameters imm{};
    std::chrono::duration<double> reset_interval{1.0};
};

auto load_predictor_config(const YAML::Node& root) -> std::expected<PredictorConfig, std::string>;
auto make_ball_state(const PredictorConfig& config) -> BallState;
auto make_imm_ball_state(const PredictorConfig& config) -> ImmBallState;
auto make_predictor(const PredictorConfig& config) -> ConfiguredPredictor;
[[nodiscard]] auto predictor_filter_type_name(PredictorFilterType filter_type) -> const char*;

using ConfiguredBallTracker = ConfiguredPredictor;
using ImmBallStateConfig = PredictorConfig;

inline auto load_imm_ball_state_config(const YAML::Node& root)
    -> std::expected<ImmBallStateConfig, std::string> {
    return load_predictor_config(root);
}

inline auto make_configured_ball_tracker(const ImmBallStateConfig& config) -> ConfiguredBallTracker {
    return make_predictor(config);
}

}  // namespace pingpong_tracker::predictor
