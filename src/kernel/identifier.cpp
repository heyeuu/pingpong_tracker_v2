#include "identifier.hpp"

#include "module/identifier/ball_detection.hpp"

namespace pingpong_tracker::kernel {
namespace identifier = pingpong_tracker::identifier;

struct Identifier::Impl {
    identifier::BallDetection ball_detection;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        return ball_detection.initialize(yaml);
    }

    auto identify(const Image& src) noexcept -> std::optional<std::vector<Ball2D>> {
        return ball_detection.sync_detect(src);
    }
};

Identifier::Identifier() : pimpl{std::make_unique<Impl>()} {
}

Identifier::~Identifier() noexcept = default;

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image& src) noexcept -> std::optional<std::vector<Ball2D>> {
    return pimpl->identify(src);
}

}  // namespace pingpong_tracker::kernel
