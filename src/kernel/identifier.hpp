#pragma once

#include <yaml-cpp/node/node.h>

#include <expected>
#include <optional>
#include <vector>

#include "utility/ball/ball.hpp"
#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

namespace pingpong_tracker::kernel {

class Identifier {
    PINGPONG_TRACKER_PIMPL_DEFINITION(Identifier)

public:
    Identifier();
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_identify(const Image&) noexcept -> std::optional<std::vector<Ball2D>>;
};

}  // namespace pingpong_tracker::kernel
