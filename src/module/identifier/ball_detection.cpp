#include "ball_detection.hpp"

#include "module/identifier/model.hpp"

namespace pingpong_tracker::identifier {

struct BallDetection::Impl {
    OpenVinoNet openvino_net;

    auto initialize(const YAML::Node& yaml) noexcept {
        return openvino_net.configure(yaml);  //
    }

    auto sync_detect(const Image& image) noexcept -> std::optional<std::vector<Ball2D>> {
        return openvino_net.sync_infer(image);
    }
};

BallDetection::BallDetection() : pimpl{std::make_unique<Impl>()} {
}

BallDetection::~BallDetection() = default;

auto BallDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto BallDetection::sync_detect(const Image& image) noexcept -> std::optional<std::vector<Ball2D>> {
    return pimpl->sync_detect(image);
}

}  // namespace pingpong_tracker::identifier