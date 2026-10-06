#pragma once

#include <chrono>
#include <cstddef>
#include <expected>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "module/predictor/predictor_config.hpp"
#include "utility/time/clock.hpp"

namespace pingpong_tracker::kernel {

class Predictor {
public:
    using Result = std::expected<void, std::string>;

    Predictor() = default;

    auto initialize(const YAML::Node& root) noexcept -> Result;

    auto predict(util::Clock::time_point stamp) noexcept -> void;
    auto reset() noexcept -> void;
    auto update(const Eigen::Vector3d& measurement, util::Clock::time_point stamp) noexcept -> bool;

    [[nodiscard]] auto initialized() const noexcept -> bool;
    [[nodiscard]] auto update_count() const noexcept -> std::size_t;
    [[nodiscard]] auto position() const noexcept -> Eigen::Vector3d;

    [[nodiscard]] auto predict_trajectory(std::chrono::milliseconds horizon,
                                          std::chrono::milliseconds step) const
        -> std::vector<Eigen::Vector3d>;

private:
    std::optional<predictor::ConfiguredPredictor> predictor_;

    bool observation_gate_enabled_ = true;
    double observation_gate_sigma_ = 3.0;
    Eigen::Matrix3d measurement_covariance_ = Eigen::Matrix3d::Identity() * 1e-6;
};

}  // namespace pingpong_tracker::kernel
