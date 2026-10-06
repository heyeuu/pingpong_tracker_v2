#include "predictor.hpp"

#include <cmath>
#include <cstddef>
#include <type_traits>
#include <variant>

#include <eigen3/Eigen/Cholesky>

namespace pingpong_tracker::kernel {

auto Predictor::initialize(const YAML::Node& root) noexcept -> Result {
    const auto config = predictor::load_predictor_config(root);
    if (!config) {
        return std::unexpected{config.error()};
    }

    observation_gate_enabled_ = config->observation_gate.enabled;
    observation_gate_sigma_   = config->observation_gate.sigma;
    for (Eigen::Index i = 0; i < measurement_covariance_.rows(); ++i) {
        const auto sigma = config->ukf.measurement_noise_std[static_cast<std::size_t>(i)];
        measurement_covariance_(i, i) = sigma * sigma;
    }

    predictor_ = predictor::make_predictor(*config);
    return {};
}

auto Predictor::predict(util::Clock::time_point stamp) noexcept -> void {
    if (!predictor_.has_value()) {
        return;
    }

    std::visit(
        [&](auto& tracker) {
            tracker.predict(stamp);
        },
        *predictor_);
}

auto Predictor::reset() noexcept -> void {
    if (!predictor_.has_value()) {
        return;
    }

    std::visit(
        [](auto& tracker) {
            tracker.reset();
        },
        *predictor_);
}

auto Predictor::update(const Eigen::Vector3d& measurement, util::Clock::time_point stamp) noexcept -> bool {
    if (!predictor_.has_value()) {
        return false;
    }

    return std::visit(
        [&](auto& tracker) {
            typename std::decay_t<decltype(tracker)>::ZVec z;
            z << measurement.x(), measurement.y(), measurement.z();

            if (observation_gate_enabled_ && tracker.initialized()) {
                if (!measurement.allFinite()) {
                    return false;
                }

                const auto predicted_position = tracker.position();
                const auto predicted_covariance =
                    tracker.covariance().template topLeftCorner<3, 3>();
                const Eigen::Matrix3d innovation_covariance =
                    predicted_covariance + measurement_covariance_;

                const Eigen::LDLT<Eigen::Matrix3d> ldlt{innovation_covariance};
                if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
                    return false;
                }

                const Eigen::Vector3d innovation = z - predicted_position;
                const double squared_distance = innovation.transpose() * ldlt.solve(innovation);
                if (!std::isfinite(squared_distance)
                    || std::sqrt(squared_distance) > observation_gate_sigma_) {
                    return false;
                }
            }

            return tracker.update(z, stamp);
        },
        *predictor_);
}

auto Predictor::initialized() const noexcept -> bool {
    if (!predictor_.has_value()) {
        return false;
    }

    return std::visit(
        [](const auto& tracker) {
            return tracker.initialized();
        },
        *predictor_);
}

auto Predictor::update_count() const noexcept -> std::size_t {
    if (!predictor_.has_value()) {
        return 0U;
    }

    return std::visit(
        [](const auto& tracker) {
            return tracker.update_count();
        },
        *predictor_);
}

auto Predictor::position() const noexcept -> Eigen::Vector3d {
    if (!predictor_.has_value()) {
        return Eigen::Vector3d::Zero();
    }

    return std::visit(
        [](const auto& tracker) {
            return tracker.position();
        },
        *predictor_);
}

auto Predictor::predict_trajectory(std::chrono::milliseconds horizon, std::chrono::milliseconds step) const
    -> std::vector<Eigen::Vector3d> {
    if (!predictor_.has_value()) {
        return {};
    }

    return std::visit(
        [&](const auto& tracker) {
            return tracker.predict_trajectory(horizon, step);
        },
        *predictor_);
}

}  // namespace pingpong_tracker::kernel
