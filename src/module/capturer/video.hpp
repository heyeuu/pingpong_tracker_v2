#pragma once
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <string>

#include "utility/pimpl.hpp"

namespace pingpong_tracker::module {

class VideoCapturer {
    PINGPONG_TRACKER_PIMPL_DEFINITION(VideoCapturer)
public:
    VideoCapturer() noexcept;
    /// @throw std::runtime_error Read error, timeout, e.g
    explicit VideoCapturer(const std::string& path);

    /// @throw std::runtime_error Read error, timeout, e.g
    auto read(std::chrono::milliseconds timeout) const -> cv::Mat;

    auto set_framerate(double hz) noexcept -> void;
};

}  // namespace pingpong_tracker::module
