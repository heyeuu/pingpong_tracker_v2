#pragma once

#include <expected>
#include <optional>

#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "module/reconstruction/mono_reconstructor.hpp"
#include "utility/ball/ball.hpp"
#include "utility/time/clock.hpp"

namespace pingpong_tracker::kernel {

class Reconstruction {
public:
    struct Observation {
        util::Clock::time_point stamp{};
        double timestamp_seconds = 0.0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        double quality_score = 0.0;
    };

    using Result = std::expected<void, std::string>;

    Reconstruction() = default;

    auto initialize(const YAML::Node& root) noexcept -> Result;
    auto reconstruct(const Ball2D& ball, util::Clock::time_point stamp) noexcept
        -> std::expected<Observation, std::string>;

    [[nodiscard]] auto initialized() const noexcept -> bool;

private:
    std::optional<reconstruction::MonoReconstructor> mono_reconstructor_;
    std::optional<util::Clock::time_point> first_stamp_;
};

}  // namespace pingpong_tracker::kernel
