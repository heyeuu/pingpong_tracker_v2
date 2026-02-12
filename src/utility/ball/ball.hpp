#pragma once

#include <opencv2/core/types.hpp>
namespace pingpong_tracker {
struct Ball2D {
    cv::Point2f center;
    float radius;
    float confidence;
};

}  // namespace pingpong_tracker
