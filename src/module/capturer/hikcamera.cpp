#include "hikcamera.hpp"

#include "utility/image/image.details.hpp"

namespace pingpong_tracker::cap {

auto Hikcamera::wait_image() noexcept -> ImageResult {
    using pingpong_tracker::Image;
    auto captured = read_image_with_timestamp();
    if (!captured.has_value()) {
        return std::unexpected{captured.error()};
    }

    auto image = std::make_unique<Image>();
    image->details().set_mat(captured->mat);
    image->set_timestamp(captured->timestamp);

    return image;
}

}  // namespace pingpong_tracker::cap
