#include "imm_ball_state_config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <format>
#include <ranges>
#include <string_view>

namespace pingpong_tracker::predictor {

namespace {

auto to_string(const YAML::Mark& mark) -> std::string {
    if (mark.is_null()) {
        return "unknown location";
    }
    return std::format("line {}, column {}", mark.line + 1, mark.column + 1);
}

template <typename T>
auto read_optional_scalar(const YAML::Node& node, const char* key, T& target)
    -> std::expected<void, std::string> {
    const auto child = node[key];
    if (!child || child.IsNull()) {
        return {};
    }

    try {
        target = child.as<T>();
        return {};
    } catch (const YAML::BadConversion& e) {
        return std::unexpected{
            std::format("Failed to parse '{}' at {}: {}", key, to_string(child.Mark()), e.what()),
        };
    }
}

template <std::size_t N>
auto read_optional_array(const YAML::Node& node, const char* key, std::array<double, N>& target)
    -> std::expected<void, std::string> {
    const auto child = node[key];
    if (!child || child.IsNull()) {
        return {};
    }

    if (!child.IsSequence()) {
        return std::unexpected{
            std::format("'{}' must be a sequence at {}", key, to_string(child.Mark())),
        };
    }

    if (child.size() != N) {
        return std::unexpected{std::format(
            "'{}' size mismatch at {}: expected {}, got {}", key, to_string(child.Mark()), N,
            child.size())};
    }

    for (std::size_t i = 0; i < N; ++i) {
        try {
            target[i] = child[i].as<double>();
        } catch (const YAML::BadConversion& e) {
            return std::unexpected{std::format("Failed to parse '{}[{}]' at {}: {}", key, i,
                                               to_string(child[i].Mark()), e.what())};
        }
    }
    return {};
}

auto read_ball_model(const YAML::Node& ball_node, BallModelParameters& model)
    -> std::expected<void, std::string> {
    if (!ball_node || ball_node.IsNull()) {
        return {};
    }

    if (!ball_node.IsMap()) {
        return std::unexpected{
            std::format("'ball_model' must be a map at {}", to_string(ball_node.Mark())),
        };
    }

    if (auto ret = read_optional_scalar(ball_node, "gravity", model.gravity); !ret) {
        return ret;
    }
    if (auto ret = read_optional_scalar(ball_node, "drag_coefficient", model.drag_coefficient); !ret) {
        return ret;
    }
    if (auto ret = read_optional_scalar(ball_node, "mass", model.mass); !ret) {
        return ret;
    }
    if (auto ret = read_optional_scalar(ball_node, "radius", model.radius); !ret) {
        return ret;
    }
    if (auto ret = read_optional_scalar(ball_node, "air_density", model.air_density); !ret) {
        return ret;
    }
    if (auto ret = read_optional_scalar(ball_node, "lift_coefficient", model.lift_coefficient); !ret) {
        return ret;
    }
    return {};
}

auto validate_mode(const ImmBallState::ModeParameters& mode, std::size_t mode_index)
    -> std::expected<void, std::string> {
    if (mode.ball_model.mass <= 0.0) {
        return std::unexpected{std::format("Mode {} has non-positive mass", mode_index)};
    }
    if (mode.ball_model.radius <= 0.0) {
        return std::unexpected{std::format("Mode {} has non-positive radius", mode_index)};
    }
    if (mode.process_noise_scale <= 0.0) {
        return std::unexpected{std::format("Mode {} has non-positive process_noise_scale", mode_index)};
    }
    if (mode.restitution < 0.0 || mode.restitution > 1.2) {
        return std::unexpected{
            std::format("Mode {} has invalid restitution {}, expected [0, 1.2]", mode_index,
                        mode.restitution),
        };
    }
    if (mode.tangential_damping < 0.0 || mode.tangential_damping > 1.2) {
        return std::unexpected{
            std::format("Mode {} has invalid tangential_damping {}, expected [0, 1.2]", mode_index,
                        mode.tangential_damping),
        };
    }
    if (mode.spin_damping_on_hit < 0.0 || mode.spin_damping_on_hit > 1.2) {
        return std::unexpected{std::format(
            "Mode {} has invalid spin_damping_on_hit {}, expected [0, 1.2]", mode_index,
            mode.spin_damping_on_hit)};
    }
    return {};
}

auto parse_modes(const YAML::Node& imm_node, std::vector<ImmBallState::ModeParameters>& modes)
    -> std::expected<void, std::string> {
    const auto modes_node = imm_node["modes"];
    if (!modes_node || modes_node.IsNull()) {
        return {};
    }

    if (!modes_node.IsSequence() || modes_node.size() == 0) {
        return std::unexpected{"'predictor.imm.modes' must be a non-empty sequence"};
    }

    modes.clear();
    modes.reserve(modes_node.size());

    for (std::size_t i = 0; i < modes_node.size(); ++i) {
        const auto mode_node = modes_node[i];
        if (!mode_node.IsMap()) {
            return std::unexpected{
                std::format("'predictor.imm.modes[{}]' must be a map at {}", i,
                            to_string(mode_node.Mark())),
            };
        }

        ImmBallState::ModeParameters mode;
        if (auto ret = read_ball_model(mode_node["ball_model"], mode.ball_model); !ret) {
            return ret;
        }
        if (auto ret =
                read_optional_scalar(mode_node, "process_noise_scale", mode.process_noise_scale);
            !ret) {
            return ret;
        }
        if (auto ret = read_optional_scalar(mode_node, "enable_table_bounce", mode.enable_table_bounce);
            !ret) {
            return ret;
        }
        if (auto ret = read_optional_scalar(mode_node, "table_height", mode.table_height); !ret) {
            return ret;
        }
        if (auto ret = read_optional_scalar(mode_node, "restitution", mode.restitution); !ret) {
            return ret;
        }
        if (auto ret = read_optional_scalar(mode_node, "tangential_damping", mode.tangential_damping);
            !ret) {
            return ret;
        }
        if (auto ret = read_optional_scalar(mode_node, "spin_damping_on_hit", mode.spin_damping_on_hit);
            !ret) {
            return ret;
        }

        if (auto ret = validate_mode(mode, i); !ret) {
            return ret;
        }
        modes.push_back(mode);
    }

    return {};
}

auto parse_transition_matrix(const YAML::Node& imm_node, std::size_t mode_count, Eigen::MatrixXd& matrix)
    -> std::expected<void, std::string> {
    const auto matrix_node = imm_node["transition_matrix"];
    if (!matrix_node || matrix_node.IsNull()) {
        return {};
    }

    if (!matrix_node.IsSequence() || matrix_node.size() == 0) {
        return std::unexpected{"'predictor.imm.transition_matrix' must be a non-empty matrix"};
    }

    if (mode_count == 0) {
        return std::unexpected{
            "'predictor.imm.transition_matrix' requires predictor.imm.modes to be configured"};
    }

    if (matrix_node.size() != mode_count) {
        return std::unexpected{std::format(
            "transition_matrix row count mismatch: expected {}, got {}", mode_count,
            matrix_node.size())};
    }

    matrix.resize(static_cast<Eigen::Index>(mode_count), static_cast<Eigen::Index>(mode_count));
    for (std::size_t r = 0; r < mode_count; ++r) {
        const auto row = matrix_node[r];
        if (!row.IsSequence() || row.size() != mode_count) {
            return std::unexpected{std::format(
                "transition_matrix row {} must have {} columns", r, mode_count)};
        }

        for (std::size_t c = 0; c < mode_count; ++c) {
            try {
                matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = row[c].as<double>();
            } catch (const YAML::BadConversion& e) {
                return std::unexpected{std::format("transition_matrix[{}][{}] parse error at {}: {}",
                                                   r, c, to_string(row[c].Mark()), e.what())};
            }
        }
    }
    return {};
}

auto validate_transition_matrix(Eigen::MatrixXd& matrix) -> std::expected<void, std::string> {
    if (matrix.rows() == 0) {
        return {};
    }

    for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
        for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
            if (matrix(r, c) < 0.0) {
                return std::unexpected{
                    std::format("transition_matrix contains negative value at [{}, {}]", r, c),
                };
            }
        }

        const double row_sum = matrix.row(r).sum();
        if (row_sum <= 0.0) {
            return std::unexpected{
                std::format("transition_matrix row {} sum must be positive", r),
            };
        }
        matrix.row(r) /= row_sum;
    }

    return {};
}

auto lower_copy(std::string text) -> std::string {
    std::ranges::transform(text, text.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

auto parse_filter_type(std::string_view filter_type)
    -> std::expected<PredictorFilterType, std::string> {
    if (filter_type == "ukf") {
        return PredictorFilterType::Ukf;
    }
    if (filter_type == "imm_ukf" || filter_type == "imm") {
        return PredictorFilterType::ImmUkf;
    }
    return std::unexpected{
        std::format("Unsupported predictor.filter_type '{}', expected 'ukf' or 'imm_ukf'", filter_type),
    };
}

}  // namespace

auto load_predictor_config(const YAML::Node& root)
    -> std::expected<PredictorConfig, std::string> {
    auto config = PredictorConfig{};

    const auto predictor_node = root["predictor"];
    if (!predictor_node || predictor_node.IsNull()) {
        return config;
    }
    if (!predictor_node.IsMap()) {
        return std::unexpected{"'predictor' must be a map"};
    }

    if (const auto node = predictor_node["filter_type"]; node && !node.IsNull()) {
        try {
            const auto filter_type = lower_copy(node.as<std::string>());
            if (const auto parsed = parse_filter_type(filter_type); parsed) {
                config.filter_type = *parsed;
            } else {
                return std::unexpected{parsed.error()};
            }
        } catch (const YAML::BadConversion& e) {
            return std::unexpected{
                std::format("Failed to parse predictor.filter_type at {}: {}", to_string(node.Mark()),
                            e.what()),
            };
        }
    }

    auto reset_interval_seconds = config.reset_interval.count();
    if (auto ret =
            read_optional_scalar(predictor_node, "reset_interval_seconds", reset_interval_seconds);
        !ret) {
        return std::unexpected{ret.error()};
    }
    config.reset_interval = std::chrono::duration<double>{reset_interval_seconds};
    config.reset_interval = std::max(config.reset_interval, std::chrono::duration<double>::zero());

    const auto ukf_node = predictor_node["ukf"];
    if (ukf_node && !ukf_node.IsNull()) {
        if (!ukf_node.IsMap()) {
            return std::unexpected{"'predictor.ukf' must be a map"};
        }

        if (auto ret = read_optional_scalar(ukf_node, "alpha", config.ukf.alpha); !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret = read_optional_scalar(ukf_node, "beta", config.ukf.beta); !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret = read_optional_scalar(ukf_node, "kappa", config.ukf.kappa); !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret =
                read_optional_array(ukf_node, "process_noise_std", config.ukf.process_noise_std);
            !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret = read_optional_array(ukf_node, "measurement_noise_std",
                                           config.ukf.measurement_noise_std);
            !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret =
                read_optional_array(ukf_node, "initial_covariance", config.ukf.initial_covariance);
            !ret) {
            return std::unexpected{ret.error()};
        }
    }

    if (auto ret = read_ball_model(predictor_node["ball_model"], config.ball_model); !ret) {
        return std::unexpected{ret.error()};
    }

    if (config.filter_type == PredictorFilterType::Ukf) {
        return config;
    }

    const auto imm_node = predictor_node["imm"];
    if (imm_node && !imm_node.IsNull()) {
        if (!imm_node.IsMap()) {
            return std::unexpected{"'predictor.imm' must be a map"};
        }

        if (auto ret = parse_modes(imm_node, config.imm.modes); !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret = parse_transition_matrix(imm_node, config.imm.modes.size(),
                                               config.imm.transition_matrix);
            !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret = validate_transition_matrix(config.imm.transition_matrix); !ret) {
            return std::unexpected{ret.error()};
        }

        if (auto ret = read_optional_scalar(imm_node, "bounce_mode_index", config.imm.bounce_mode_index);
            !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret =
                read_optional_scalar(imm_node, "bounce_height_gate", config.imm.bounce_height_gate);
            !ret) {
            return std::unexpected{ret.error()};
        }
        if (auto ret =
                read_optional_scalar(imm_node, "bounce_prior_boost", config.imm.bounce_prior_boost);
            !ret) {
            return std::unexpected{ret.error()};
        }

        config.imm.bounce_height_gate = std::max(config.imm.bounce_height_gate, 0.0);
        config.imm.bounce_prior_boost = std::max(config.imm.bounce_prior_boost, 1.0);
    }

    return config;
}

auto make_ball_state(const PredictorConfig& config) -> BallState {
    auto tracker = BallState{config.ukf, config.ball_model};
    tracker.set_reset_interval(config.reset_interval);
    return tracker;
}

auto make_imm_ball_state(const PredictorConfig& config) -> ImmBallState {
    auto tracker = ImmBallState{config.ukf, config.imm};
    tracker.set_reset_interval(config.reset_interval);
    return tracker;
}

auto make_predictor(const PredictorConfig& config) -> ConfiguredPredictor {
    if (config.filter_type == PredictorFilterType::Ukf) {
        return make_ball_state(config);
    }
    return make_imm_ball_state(config);
}

auto predictor_filter_type_name(PredictorFilterType filter_type) -> const char* {
    switch (filter_type) {
    case PredictorFilterType::Ukf:
        return "ukf";
    case PredictorFilterType::ImmUkf:
        return "imm_ukf";
    }

    return "unknown";
}

}  // namespace pingpong_tracker::predictor
