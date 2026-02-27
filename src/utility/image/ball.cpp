#include "ball.hpp"

#include <format>
#include <opencv2/imgproc.hpp>

#include "utility/image/image.details.hpp"

namespace pingpong_tracker::util {

auto draw(Image& canvas, const Ball2D& ball) noexcept -> void {
    auto& opencv_mat = const_cast<cv::Mat&>(canvas.details().get_mat());

    const auto color = cv::Scalar{0, 255, 0};

    cv::circle(opencv_mat, ball.center, static_cast<int>(ball.radius), color, 2);
    cv::circle(opencv_mat, ball.center, 2, cv::Scalar{0, 0, 255}, -1);

    const auto font      = cv::FONT_HERSHEY_SIMPLEX;
    const auto scale     = 0.6;
    const auto thickness = 1;
    const auto white     = cv::Scalar{255, 255, 255};

    auto info = std::format("{:.2f}", ball.confidence);
    cv::putText(opencv_mat, info, ball.center - cv::Point2f{0, ball.radius + 5}, font, scale, white,
                thickness, cv::LINE_AA);
}

}  // namespace pingpong_tracker::util
