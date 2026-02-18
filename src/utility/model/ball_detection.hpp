#pragma once

#include <cassert>
#include <cstddef>
#include <cstring>
#include <span>
#include <type_traits>

#include "utility/math/point.hpp"

namespace pingpong_tracker {

template <typename precision_type = float>
struct BallInferResult {
    struct Point {
        precision_type x{0}, y{0};
    };

    Point point{};

    precision_type confidence{0};

    auto unsafe_from(std::span<const precision_type> raw) noexcept -> void {
        assert(raw.size_bytes() >= sizeof(BallInferResult)
               && "unsafe_from: input buffer too small for BallInferResult");
        static_assert(std::is_trivially_copyable_v<BallInferResult>);
        static_assert(sizeof(BallInferResult) == 3 * sizeof(precision_type), "Layout mismatch");
        std::memcpy(this, raw.data(), sizeof(BallInferResult));
    }

    auto scale_position(precision_type scaling) noexcept -> void {
        point.x *= scaling;
        point.y *= scaling;
    }

    template <typename T = Point2D>
    [[nodiscard]] auto to_point2d() const noexcept -> T {
        static_assert(std::is_constructible_v<T, precision_type, precision_type>,
                      "T must be constructible from precision_type");
        return T{point.x, point.y};
    }

    constexpr static auto length() noexcept -> std::size_t {
        static_assert(sizeof(BallInferResult) == 3 * sizeof(precision_type), "Layout mismatch");
        return sizeof(BallInferResult) / sizeof(precision_type);
    }
};
}  // namespace pingpong_tracker
