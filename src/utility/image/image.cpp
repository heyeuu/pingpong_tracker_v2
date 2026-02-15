#include "image.details.hpp"

namespace pingpong_tracker {

struct Image::Impl {
    Clock::time_point timestamp;
    Details details;
};

auto Image::details() noexcept -> Details& {
    return pimpl->details;
}
auto Image::details() const noexcept -> const Details& {
    return pimpl->details;
}

auto Image::get_timestamp() const noexcept -> Clock::time_point {
    return pimpl->timestamp;
}
auto Image::set_timestamp(Clock::time_point timestamp) noexcept -> void {
    pimpl->timestamp = timestamp;
}

Image::Image() : pimpl{std::make_unique<Impl>()} {
}

Image::~Image() noexcept = default;

}  // namespace pingpong_tracker
