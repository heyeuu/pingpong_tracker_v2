#pragma once
#include "utility/image/image.hpp"
#include "utility/serializable.hpp"
#include <expected>
#include <yaml-cpp/yaml.h>

namespace pingpong_tracker::cap {

using pingpong_tracker::Image;
namespace util = pingpong_tracker::util;

struct LocalVideo {
    PINGPONG_TRACKER_PIMPL_DEFINITION(LocalVideo)

private:
    struct ConfigDetail {
        std::string location;
        double frame_rate;
        bool loop_play;
        bool allow_skipping;
    };

public:
    struct Config : ConfigDetail, util::SerializableMixin {
        constexpr static auto kMetas = std::tuple {
            "location",
            &ConfigDetail::location, // 视频文件路径
            "frame_rate",
            &ConfigDetail::frame_rate, // 帧率
            "loop_play",
            &ConfigDetail::loop_play, // 循环播放
            "allow_skipping",
            &ConfigDetail::allow_skipping // 允许跳帧以保证实时性
        };
    };

    LocalVideo() noexcept;

    auto configure(Config const&) -> std::expected<void, std::string>;

    auto connect() noexcept -> std::expected<void, std::string>;

    auto connected() const noexcept -> bool;

    auto disconnect() noexcept -> void;

    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string>;
};
}
