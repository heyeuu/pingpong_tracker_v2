#pragma once
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace pingpong_tracker {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    explicit Image();
    using Clock = util::Clock;

    struct Details;
    auto details() noexcept -> Details&;
    [[nodiscard]] auto details() const noexcept -> Details const&;

    [[nodiscard]] auto get_timestamp() const noexcept -> Clock::time_point;
    auto set_timestamp(Clock::time_point) noexcept -> void;
};

}  // namespace pingpong_tracker
