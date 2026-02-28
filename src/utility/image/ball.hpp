#pragma once
#include "utility/ball/ball.hpp"
#include "utility/image/image.hpp"

namespace pingpong_tracker::util {

auto draw(Image&, const Ball2D&) noexcept -> void;

}
