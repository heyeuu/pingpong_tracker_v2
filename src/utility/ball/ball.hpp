#pragma once

#include <opencv2/core/types.hpp>
namespace pingpong_tracker {
struct Ball2D {
    cv::Point2f center{0.0F, 0.0F};
    float radius{0.0F};
    float confidence{0.0F};
};

}  // namespace pingpong_tracker
