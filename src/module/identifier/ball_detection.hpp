#pragma once

#include <yaml-cpp/node/node.h>

#include <expected>
#include <optional>
#include <vector>

#include "utility/ball/ball.hpp"
#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

namespace pingpong_tracker::identifier {

class BallDetection {
    PINGPONG_TRACKER_PIMPL_DEFINITION(BallDetection)

public:
    BallDetection();
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto sync_detect(const Image&) noexcept -> std::optional<std::vector<Ball2D>>;
};

}  // namespace pingpong_tracker::identifier
