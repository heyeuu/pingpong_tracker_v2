#pragma once

#include <chrono>

namespace pingpong_tracker::util {

constexpr auto delta_time(std::chrono::steady_clock::time_point late,
                          std::chrono::steady_clock::time_point early) -> auto {
    return std::chrono::duration<double>(late - early);
}

}  // namespace pingpong_tracker::util