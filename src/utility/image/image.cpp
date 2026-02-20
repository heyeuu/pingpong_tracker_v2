#include "image.details.hpp"

#include <cassert>

namespace pingpong_tracker {

struct Image::Impl {
    Clock::time_point timestamp{};
    Details details;
};

auto Image::details() noexcept -> Details& {
    assert(pimpl && "Image accessed after move");
    return pimpl->details;
}
auto Image::details() const noexcept -> const Details& {
    assert(pimpl && "Image accessed after move");
    return pimpl->details;
}

auto Image::get_timestamp() const noexcept -> Clock::time_point {
    assert(pimpl && "Image accessed after move");
    return pimpl->timestamp;
}
auto Image::set_timestamp(Clock::time_point timestamp) noexcept -> void {
    assert(pimpl && "Image accessed after move");
    pimpl->timestamp = timestamp;
}

Image::Image() : pimpl{std::make_unique<Impl>()} {
}
Image::~Image() noexcept                  = default;
Image::Image(Image&&) noexcept            = default;
Image& Image::operator=(Image&&) noexcept = default;

}  // namespace pingpong_tracker
