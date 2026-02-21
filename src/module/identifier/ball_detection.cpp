#include "ball_detection.hpp"

#include "module/identifier/model.hpp"

namespace pingpong_tracker::identifier {

struct BallDetection::Impl {
    OpenVinoNet openvino_net;

    auto initialize(const YAML::Node& yaml) noexcept {
        return openvino_net.configure(yaml);
    }

    auto sync_detect(const Image& image) noexcept
        -> std::expected<std::vector<Ball2D>, std::string> {
        return openvino_net.sync_infer(image);
    }
};

BallDetection::BallDetection() : pimpl_{std::make_unique<Impl>()} {
}

BallDetection::~BallDetection() noexcept                          = default;
BallDetection::BallDetection(BallDetection&&) noexcept            = default;
BallDetection& BallDetection::operator=(BallDetection&&) noexcept = default;

auto BallDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl_->initialize(yaml);
}

auto BallDetection::sync_detect(const Image& image) noexcept
    -> std::expected<std::vector<Ball2D>, std::string> {
    return pimpl_->sync_detect(image);
}

}  // namespace pingpong_tracker::identifier
