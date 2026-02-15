#pragma once

#include <opencv2/core/mat.hpp>

#include "image.hpp"

namespace pingpong_tracker {

struct Image::Details {
    auto set_mat(cv::Mat mat) noexcept -> void {
        this->mat = std::move(mat);
    }

    [[nodiscard]] auto get_cols() const noexcept {
        return mat.cols;
    }
    [[nodiscard]] auto get_rows() const noexcept {
        return mat.rows;
    }

    cv::Mat mat;
};

}  // namespace pingpong_tracker
