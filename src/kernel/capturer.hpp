#pragma once
#include <yaml-cpp/yaml.h>

#include <expected>

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

namespace pingpong_tracker::kernel {

class Capturer final {
    PINGPONG_TRACKER_PIMPL_DEFINITION(Capturer)
    using Image = pingpong_tracker::Image;

public:
    Capturer() noexcept;

    using ImageUnique = std::unique_ptr<Image>;

    using Result = std::expected<void, std::string>;

    auto initialize(const YAML::Node&) noexcept -> Result;

    /// @brief
    ///   Fetches an image from the background worker thread.
    /// @note
    ///   - Non-blocking: returns immediately with either a valid image or nullptr.
    ///   - Thread-safe: safe to call from multiple threads, but only one thread
    ///     should fetch at a time.
    auto fetch_image() noexcept -> ImageUnique;

    static constexpr auto get_prefix() noexcept {
        return "capturer";
    }
};

}  // namespace pingpong_tracker::kernel
