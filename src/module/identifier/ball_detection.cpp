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

}  // namespace pingpong_tracker::identifier