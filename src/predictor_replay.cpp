#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <expected>
#include <filesystem>
#include <fstream>
#include <format>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <ranges>

#include "module/predictor/predictor_config.hpp"
#include "module/predictor/replay_evaluation.hpp"
#include "utility/configure/configuration.hpp"

namespace {

using pingpong_tracker::predictor::BallState;
using pingpong_tracker::predictor::ImmBallState;
using pingpong_tracker::predictor::ReplayMetrics;
using pingpong_tracker::predictor::ReplaySample;
using pingpong_tracker::predictor::evaluate_replay;
using pingpong_tracker::predictor::load_predictor_config;
using pingpong_tracker::predictor::make_predictor;
using pingpong_tracker::predictor::predictor_filter_type_name;

struct ReplayOptions {
    std::filesystem::path csv_path;
    std::optional<std::filesystem::path> config_path;
    int horizon_ms = 100;
    int step_ms    = 10;
    int warmup     = 10;
};

auto trim(std::string text) -> std::string {
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!text.empty() && is_space(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && is_space(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

auto split_csv_line(const std::string& line) -> std::vector<std::string> {
    auto cells = std::vector<std::string>{};
    std::string current;
    std::stringstream ss(line);
    while (std::getline(ss, current, ',')) {
        cells.push_back(trim(current));
    }
    if (!line.empty() && line.back() == ',') {
        cells.emplace_back("");
    }
    return cells;
}

auto parse_double(const std::string& text) -> std::optional<double> {
    if (text.empty()) {
        return std::nullopt;
    }
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

auto to_lower(std::string text) -> std::string {
    std::ranges::transform(text, text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

auto parse_options(int argc, char** argv) -> std::expected<ReplayOptions, std::string> {
    if (argc < 2) {
        return std::unexpected{
            "Usage: pingpong_tracker_predictor_replay <csv_path> "
            "[--config <path>] [--horizon-ms <int>] [--step-ms <int>] [--warmup <int>]",
        };
    }

    auto options = ReplayOptions{
        .csv_path = std::filesystem::path{argv[1]},
        .config_path = std::nullopt,
        .horizon_ms = 100,
        .step_ms = 10,
        .warmup = 10,
    };

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        auto read_int_arg = [&](const char* flag, int& target) -> std::expected<void, std::string> {
            if (i + 1 >= argc) {
                return std::unexpected{std::format("Missing value for {}", flag)};
            }
            const auto value = parse_double(argv[i + 1]);
            if (!value || *value < 0.0 || std::floor(*value) != *value) {
                return std::unexpected{
                    std::format("Invalid integer value '{}' for {}", argv[i + 1], flag),
                };
            }
            target = static_cast<int>(*value);
            ++i;
            return {};
        };

        if (arg == "--config") {
            if (i + 1 >= argc) {
                return std::unexpected{"Missing path value for --config"};
            }
            options.config_path = std::filesystem::path{argv[++i]};
            continue;
        }
        if (arg == "--horizon-ms") {
            if (auto ret = read_int_arg("--horizon-ms", options.horizon_ms); !ret) {
                return std::unexpected{ret.error()};
            }
            continue;
        }
        if (arg == "--step-ms") {
            if (auto ret = read_int_arg("--step-ms", options.step_ms); !ret) {
                return std::unexpected{ret.error()};
            }
            continue;
        }
        if (arg == "--warmup") {
            if (auto ret = read_int_arg("--warmup", options.warmup); !ret) {
                return std::unexpected{ret.error()};
            }
            continue;
        }

        return std::unexpected{std::format("Unknown argument '{}'", arg)};
    }

    if (options.horizon_ms <= 0) {
        return std::unexpected{"--horizon-ms must be positive"};
    }
    if (options.step_ms <= 0) {
        return std::unexpected{"--step-ms must be positive"};
    }
    if (options.horizon_ms < options.step_ms) {
        return std::unexpected{"--horizon-ms must be >= --step-ms"};
    }
    if (options.horizon_ms % options.step_ms != 0) {
        return std::unexpected{"--horizon-ms must be a multiple of --step-ms"};
    }

    return options;
}

auto find_header_index(const std::vector<std::string>& header, std::initializer_list<const char*> aliases)
    -> std::optional<std::size_t> {
    for (std::size_t i = 0; i < header.size(); ++i) {
        const auto field = to_lower(trim(header[i]));
        for (const auto* alias : aliases) {
            if (field == alias) {
                return i;
            }
        }
    }
    return std::nullopt;
}

auto parse_sample_from_cells(const std::vector<std::string>& cells, std::size_t line_no,
                             std::size_t t_idx, std::size_t x_idx, std::size_t y_idx,
                             std::size_t z_idx) -> std::expected<ReplaySample, std::string> {
    const auto max_index = std::max({t_idx, x_idx, y_idx, z_idx});
    if (cells.size() <= max_index) {
        return std::unexpected{
            std::format("Line {} does not have enough columns", line_no),
        };
    }

    const auto t = parse_double(cells[t_idx]);
    const auto x = parse_double(cells[x_idx]);
    const auto y = parse_double(cells[y_idx]);
    const auto z = parse_double(cells[z_idx]);
    if (!t || !x || !y || !z) {
        return std::unexpected{
            std::format("Line {} contains non-numeric data in required columns", line_no),
        };
    }

    if (*t < 0.0) {
        return std::unexpected{
            std::format("Line {} has negative timestamp {}", line_no, *t),
        };
    }

    return ReplaySample{
        .timestamp_seconds = *t,
        .position = Eigen::Vector3d{*x, *y, *z},
    };
}

auto load_samples_csv(const std::filesystem::path& csv_path)
    -> std::expected<std::vector<ReplaySample>, std::string> {
    auto file = std::ifstream{csv_path};
    if (!file.is_open()) {
        return std::unexpected{
            std::format("Failed to open CSV file '{}'", csv_path.string()),
        };
    }

    std::string line;
    std::size_t line_no = 0;
    std::optional<std::size_t> t_idx;
    std::optional<std::size_t> x_idx;
    std::optional<std::size_t> y_idx;
    std::optional<std::size_t> z_idx;
    bool header_checked = false;

    auto samples = std::vector<ReplaySample>{};
    while (std::getline(file, line)) {
        ++line_no;
        const auto trimmed = trim(line);
        if (trimmed.empty() || trimmed.starts_with('#')) {
            continue;
        }

        const auto cells = split_csv_line(trimmed);
        if (!header_checked) {
            header_checked = true;

            const bool first_line_all_numeric = std::ranges::all_of(cells, [](const auto& cell) {
                return parse_double(cell).has_value();
            });
            if (!first_line_all_numeric) {
                t_idx = find_header_index(cells, {"t", "time", "timestamp", "time_sec",
                                                  "timestamp_sec", "sec", "s"});
                x_idx = find_header_index(cells, {"x"});
                y_idx = find_header_index(cells, {"y"});
                z_idx = find_header_index(cells, {"z"});
                if (!t_idx || !x_idx || !y_idx || !z_idx) {
                    return std::unexpected{
                        "CSV header must contain columns for time,x,y,z (aliases supported for time)",
                    };
                }
                continue;
            }

            t_idx = 0;
            x_idx = 1;
            y_idx = 2;
            z_idx = 3;
        }

        const auto parsed =
            parse_sample_from_cells(cells, line_no, *t_idx, *x_idx, *y_idx, *z_idx);
        if (!parsed) {
            return std::unexpected{parsed.error()};
        }

        if (!samples.empty() && parsed->timestamp_seconds <= samples.back().timestamp_seconds) {
            return std::unexpected{
                std::format("Timestamp must be strictly increasing, line {}", line_no),
            };
        }
        samples.push_back(*parsed);
    }

    if (samples.size() < 2) {
        return std::unexpected{"CSV must contain at least 2 valid samples"};
    }
    return samples;
}

auto load_root_yaml(const ReplayOptions& options) -> std::expected<YAML::Node, std::string> {
    try {
        if (options.config_path.has_value()) {
            return YAML::LoadFile(options.config_path->string());
        }
        return pingpong_tracker::util::configuration();
    } catch (const std::exception& e) {
        return std::unexpected{
            std::format("Failed to load config YAML: {}", e.what()),
        };
    }
}

auto rmse(double sse, std::size_t count) -> double {
    if (count == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(sse / static_cast<double>(count));
}

}  // namespace

auto main(int argc, char** argv) -> int {
    const auto options = parse_options(argc, argv);
    if (!options) {
        std::cerr << options.error() << '\n';
        return 2;
    }

    const auto root = load_root_yaml(*options);
    if (!root) {
        std::cerr << root.error() << '\n';
        return 2;
    }

    const auto config = load_predictor_config(*root);
    if (!config) {
        std::cerr << std::format("Failed to parse predictor config: {}\n", config.error());
        return 2;
    }

    const auto samples = load_samples_csv(options->csv_path);
    if (!samples) {
        std::cerr << std::format("CSV parse error: {}\n", samples.error());
        return 2;
    }

    const auto horizon = std::chrono::milliseconds{options->horizon_ms};
    const auto step = std::chrono::milliseconds{options->step_ms};

    auto tracker = make_predictor(*config);
    const auto metrics = std::visit(
        [&](auto& typed_tracker) {
            return evaluate_replay(typed_tracker, std::span<const ReplaySample>{*samples}, horizon, step,
                                   static_cast<std::size_t>(options->warmup));
        },
        tracker);

    const auto filtered_rmse = rmse(metrics.filtered_sse, metrics.filtered_count);
    const auto horizon_rmse = rmse(metrics.horizon_sse, metrics.horizon_count);

    std::cout << std::format("Filter type: {}\n", predictor_filter_type_name(config->filter_type));
    std::cout << std::format("Replay samples: {}\n", samples->size());
    std::cout << std::format("Update success: {}/{}\n", metrics.update_success, metrics.update_attempts);
    std::cout << std::format("Filtered RMSE (m): {:.6f} over {} samples\n", filtered_rmse,
                             metrics.filtered_count);
    std::cout << std::format("{}ms Horizon RMSE (m): {:.6f} over {} samples\n", options->horizon_ms,
                             horizon_rmse, metrics.horizon_count);

    std::visit(
        [&](const auto& typed_tracker) {
            using Tracker = std::decay_t<decltype(typed_tracker)>;
            if constexpr (std::is_same_v<Tracker, ImmBallState>) {
                const auto probs = typed_tracker.mode_probabilities();
                std::cout << std::format("Dominant mode at end: {}\n", typed_tracker.dominant_mode());
                std::cout << "Final mode probabilities:";
                for (std::size_t i = 0; i < probs.size(); ++i) {
                    std::cout << std::format(" m{}={:.4f}", i, probs[i]);
                }
                std::cout << '\n';
            } else if constexpr (std::is_same_v<Tracker, BallState>) {
                std::cout << "Dominant mode at end: n/a (single-model UKF)\n";
            }
        },
        tracker);

    return 0;
}
