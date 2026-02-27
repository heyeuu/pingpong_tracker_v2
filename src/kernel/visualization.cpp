#include "visualization.hpp"

#include <spdlog/spdlog.h>

#include <fstream>

#include "module/debug/visualization/stream_session.hpp"
#include "utility/image/image.details.hpp"
#include "utility/serializable.hpp"

using namespace pingpong_tracker::kernel;
using namespace pingpong_tracker::util;

constexpr std::array kVideoTypes{
    "RTP_JPEG",
    "RTP_H264",
};

struct Visualization::Impl {
    using SessionConfig = debug::StreamSession::Config;
    using NormalResult  = std::expected<void, std::string>;

    struct Config : util::SerializableMixin {
        int framerate = 80;

        std::string monitor_host = "localhost";
        std::string monitor_port = "5000";

        std::string stream_type = "RTP_JPEG";

        static constexpr auto kMetas = std::tuple{
            "framerate",    &Config::framerate,    "monitor_host", &Config::monitor_host,
            "monitor_port", &Config::monitor_port, "stream_type",  &Config::stream_type,
        };
    };

    std::unique_ptr<debug::StreamSession> session;
    SessionConfig session_config;

    bool is_initialized  = false;
    bool size_determined = false;

    Impl() noexcept {
        session = std::make_unique<debug::StreamSession>();
    }

    auto initialize(const YAML::Node& yaml) noexcept -> NormalResult {
        auto config = Config{};
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected{result.error()};
        }

        session_config.target.host = config.monitor_host;
        session_config.target.port = config.monitor_port;
        session_config.format.hz   = static_cast<int>(config.framerate);

        if (config.stream_type == kVideoTypes[0]) {
            session_config.type = debug::StreamType::RTP_JPEG;
        } else if (config.stream_type == kVideoTypes[1]) {
            session_config.type = debug::StreamType::RTP_H264;
        } else {
            return std::unexpected{"Unknown video type: " + config.stream_type};
        }

        is_initialized = true;
        return {};
    }

    auto initialized() const noexcept {
        return is_initialized;
    }

    auto send_image(const Image& image) noexcept -> bool {
        if (!is_initialized)
            return false;

        const auto& mat = image.details().get_mat();

        if (!size_determined) {
            session_config.format.w = mat.cols;
            session_config.format.h = mat.rows;

            {  // open session
                auto ret = session->open(session_config);
                if (!ret) {
                    spdlog::error("Failed to open visualization session");
                    spdlog::error("  e: {}", ret.error());
                    return false;
                }
                spdlog::info("Visualization session is opened");
            }
            {  // write SDP
                auto ret = session->session_description_protocol();
                if (!ret) {
                    spdlog::error("Failed to get description protocol");
                    spdlog::error("  e: {}", ret.error());
                    return false;
                }

                const auto output_location = "/tmp/pingpong_tracker.sdp";
                if (auto ofstream = std::ofstream{output_location}) {
                    ofstream << ret.value();
                    spdlog::info("Sdp has been written to: {}", output_location);
                } else {
                    spdlog::error("Failed to write sdp: {}", output_location);
                    return false;
                }
            }
            size_determined = true;
        }
        if (!session->opened())
            return false;

        return session->push_frame(mat);
    }
};

auto Visualization::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl_->initialize(yaml);
}

auto Visualization::initialized() const noexcept -> bool {
    return pimpl_->initialized();
}

auto Visualization::send_image(const Image& image) noexcept -> bool {
    return pimpl_->send_image(image);
}

Visualization::Visualization() noexcept : pimpl_{std::make_unique<Impl>()} {
}

Visualization::~Visualization() noexcept = default;
