#pragma once

#include <compare>

namespace pingpong_tracker {

struct Point2D {
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    double x{0.0};
    double y{0.0};
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    auto operator<=>(const Point2D&) const -> std::partial_ordering = default;

    constexpr auto operator+=(const Point2D& rhs) noexcept -> Point2D& {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    constexpr auto operator-=(const Point2D& rhs) noexcept -> Point2D& {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    constexpr auto operator*=(double scalar) noexcept -> Point2D& {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    constexpr auto operator/=(double scalar) noexcept -> Point2D& {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

[[nodiscard]] constexpr auto operator+(Point2D lhs, const Point2D& rhs) noexcept -> Point2D {
    return lhs += rhs;
}
[[nodiscard]] constexpr auto operator-(Point2D lhs, const Point2D& rhs) noexcept -> Point2D {
    return lhs -= rhs;
}
[[nodiscard]] constexpr auto operator*(Point2D lhs, double scalar) noexcept -> Point2D {
    return lhs *= scalar;
}
[[nodiscard]] constexpr auto operator*(double scalar, Point2D rhs) noexcept -> Point2D {
    return rhs *= scalar;
}
[[nodiscard]] constexpr auto operator/(Point2D lhs, double scalar) noexcept -> Point2D {
    return lhs /= scalar;
}

}  // namespace pingpong_tracker