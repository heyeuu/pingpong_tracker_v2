#pragma once

#include <opencv2/core/mat.hpp>

#include "image.hpp"

namespace pingpong_tracker {

struct Image::Details {
    auto set_mat(cv::Mat mat) noexcept -> void {
        mat_ = std::move(mat);
    }
    [[nodiscard]] auto get_mat() const noexcept -> const cv::Mat& {
        return mat_;
    }

    [[nodiscard]] auto get_cols() const noexcept {
        return mat_.cols;
    }
    [[nodiscard]] auto get_rows() const noexcept {
        return mat_.rows;
    }

private:
    cv::Mat mat_;
};

}  // namespace pingpong_tracker
