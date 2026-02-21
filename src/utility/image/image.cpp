#include <cassert>

#include "image.details.hpp"

namespace pingpong_tracker {

struct Image::Impl {
    Clock::time_point timestamp{};
    Details details;
};

auto Image::details() noexcept -> Details& {
    assert(pimpl_ && "Image accessed after move");
    return pimpl_->details;
}
auto Image::details() const noexcept -> const Details& {
    assert(pimpl_ && "Image accessed after move");
    return pimpl_->details;
}

auto Image::get_timestamp() const noexcept -> Clock::time_point {
    assert(pimpl_ && "Image accessed after move");
    return pimpl_->timestamp;
}
auto Image::set_timestamp(Clock::time_point timestamp) noexcept -> void {
    assert(pimpl_ && "Image accessed after move");
    pimpl_->timestamp = timestamp;
}

Image::Image() : pimpl_{std::make_unique<Impl>()} {
}
Image::~Image() noexcept                  = default;
Image::Image(Image&&) noexcept            = default;
Image& Image::operator=(Image&&) noexcept = default;

}  // namespace pingpong_tracker
