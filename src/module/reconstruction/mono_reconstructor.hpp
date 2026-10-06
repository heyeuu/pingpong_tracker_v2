#pragma once

#include <array>
#include <expected>
#include <string>

#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "utility/ball/ball.hpp"

namespace pingpong_tracker::reconstruction {

struct MonoReconstructorConfig {
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 640.0;
    double cy = 360.0;

    std::array<double, 5> distortion_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};

    double ball_radius_m = 0.02;
    double table_height_m = 0.0;

    double min_confidence = 0.4;
    double min_radius_px = 2.0;
    double max_radius_px = 200.0;
    double min_depth_m = 0.2;
    double max_depth_m = 8.0;

    double expected_radius_px = 12.0;
    double radius_tolerance_px = 8.0;

    int image_width_px = 1280;
    int image_height_px = 720;
    double edge_margin_px = 30.0;

    std::array<double, 9> rotation_wc = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
    std::array<double, 3> translation_wc = {0.0, 0.0, 0.0};
};

struct MonoReconstructionSample {
    double timestamp_seconds = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double quality_score = 0.0;
};

class MonoReconstructor {
public:
    MonoReconstructor();
    explicit MonoReconstructor(MonoReconstructorConfig config);

    [[nodiscard]] auto reconstruct(const Ball2D& measurement, double timestamp_seconds) const
        -> std::expected<MonoReconstructionSample, std::string>;

    [[nodiscard]] auto config() const -> const MonoReconstructorConfig&;

private:
    [[nodiscard]] auto undistort_normalized_point(const Ball2D& measurement) const
        -> std::expected<Eigen::Vector2d, std::string>;
    [[nodiscard]] auto estimate_depth(const Ball2D& measurement) const
        -> std::expected<double, std::string>;
    [[nodiscard]] auto compute_quality_score(const Ball2D& measurement) const -> double;
    [[nodiscard]] auto camera_to_world(const Eigen::Vector3d& camera_point) const -> Eigen::Vector3d;

    MonoReconstructorConfig config_;
};

[[nodiscard]] auto load_mono_reconstructor_config(const YAML::Node& root)
    -> std::expected<MonoReconstructorConfig, std::string>;

}  // namespace pingpong_tracker::reconstruction
