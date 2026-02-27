#pragma once
#include <yaml-cpp/yaml.h>

#include <expected>

#include "utility/image/image.hpp"

namespace pingpong_tracker::kernel {

class Visualization {
    PINGPONG_TRACKER_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept {
        return "visualization";
    }

    auto operator<<(const Image& image) noexcept -> Visualization& {
        return send_image(image), *this;
    }

public:
    Visualization() noexcept;
    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto send_image(const Image& image) noexcept -> bool;
};

}  // namespace pingpong_tracker::kernel
