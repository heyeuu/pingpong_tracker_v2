#pragma once
#include <hikcamera/capturer.hpp>

#include "utility/image/image.hpp"
#include "utility/serializable.hpp"

namespace pingpong_tracker::cap {

namespace util = pingpong_tracker::util;

using NormalResult = std::expected<void, std::string>;
using ImageResult  = std::expected<std::unique_ptr<Image>, std::string>;

struct Hikcamera : public hikcamera::Camera {
    using Camera::Camera;

    struct Config : hikcamera::Config, util::SerializableMixin {
        static constexpr auto kMetas = std::tuple{
            "timeout_ms",
            &Config::timeout_ms,
            "exposure_us",
            &Config::exposure_us,
            "framerate",
            &Config::framerate,
            "gain",
            &Config::gain,
            "invert_image",
            &Config::invert_image,
            "software_sync",
            &Config::software_sync,
            "trigger_mode",
            &Config::trigger_mode,
            "fixed_framerate",
            &Config::fixed_framerate,
        };
    };

    auto wait_image() noexcept -> ImageResult;

    static constexpr auto get_prefix() noexcept {
        return "hikcamera";
    }
};

}  // namespace pingpong_tracker::cap
