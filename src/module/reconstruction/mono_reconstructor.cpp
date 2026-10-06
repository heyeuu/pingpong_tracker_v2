#include "mono_reconstructor.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

namespace pingpong_tracker::reconstruction {

namespace {

template <typename T>
auto read_required_scalar(const YAML::Node& node, const char* key, T& out)
    -> std::expected<void, std::string> {
    const auto value = node[key];
    if (!value || value.IsNull()) {
        return std::unexpected{std::format("Missing required key 'reconstruction.mono.{}'", key)};
    }

    try {
        out = value.as<T>();
    } catch (const YAML::BadConversion& e) {
        return std::unexpected{
            std::format("Failed to parse key 'reconstruction.mono.{}': {}", key, e.what()),
        };
    }

    return {};
}

template <typename T>
auto read_optional_scalar(const YAML::Node& node, const char* key, T& out)
    -> std::expected<void, std::string> {
    const auto value = node[key];
    if (!value || value.IsNull()) {
        return {};
    }

    try {
        out = value.as<T>();
    } catch (const YAML::BadConversion& e) {
        return std::unexpected{
            std::format("Failed to parse key 'reconstruction.mono.{}': {}", key, e.what()),
        };
    }

    return {};
}

template <std::size_t N>
auto read_optional_array(const YAML::Node& node, const char* key, std::array<double, N>& out)
    -> std::expected<void, std::string> {
    const auto value = node[key];
    if (!value || value.IsNull()) {
        return {};
    }

    if (!value.IsSequence() || value.size() != N) {
        return std::unexpected{std::format("'reconstruction.mono.{}' must be an array of size {}", key,
                                           N)};
    }

    for (std::size_t i = 0; i < N; ++i) {
        try {
            out[i] = value[i].as<double>();
        } catch (const YAML::BadConversion& e) {
            return std::unexpected{std::format("Failed to parse key 'reconstruction.mono.{}[{}]': {}",
                                               key, i, e.what())};
        }
    }

    return {};
}

auto clamp01(double value) -> double {
    return std::clamp(value, 0.0, 1.0);
}

}  // namespace

MonoReconstructor::MonoReconstructor() : MonoReconstructor(MonoReconstructorConfig{}) {
}

MonoReconstructor::MonoReconstructor(MonoReconstructorConfig config) : config_{std::move(config)} {
}

auto MonoReconstructor::reconstruct(const Ball2D& measurement, double timestamp_seconds) const
    -> std::expected<MonoReconstructionSample, std::string> {
    if (!std::isfinite(timestamp_seconds) || timestamp_seconds < 0.0) {
        return std::unexpected{std::format("Invalid timestamp_seconds {}", timestamp_seconds)};
    }

    if (measurement.confidence < config_.min_confidence) {
        return std::unexpected{std::format(
            "Low confidence {:.3f}, expected >= {:.3f}", measurement.confidence,
            config_.min_confidence)};
    }

    if (measurement.radius < config_.min_radius_px || measurement.radius > config_.max_radius_px) {
        return std::unexpected{std::format(
            "Radius {:.3f}px out of range [{:.3f}, {:.3f}]", measurement.radius,
            config_.min_radius_px, config_.max_radius_px)};
    }

    const bool in_width = measurement.center.x >= 0.0F
                          && measurement.center.x < static_cast<float>(config_.image_width_px);
    const bool in_height = measurement.center.y >= 0.0F
                           && measurement.center.y < static_cast<float>(config_.image_height_px);
    if (!in_width || !in_height) {
        return std::unexpected{std::format(
            "Ball center ({:.3f}, {:.3f}) out of image bounds [{}x{}]", measurement.center.x,
            measurement.center.y, config_.image_width_px, config_.image_height_px)};
    }

    const auto normalized = undistort_normalized_point(measurement);
    if (!normalized) {
        return std::unexpected{normalized.error()};
    }

    const auto depth = estimate_depth(measurement);
    if (!depth) {
        return std::unexpected{depth.error()};
    }

    const Eigen::Vector3d camera_point{
        normalized->x() * *depth,
        normalized->y() * *depth,
        *depth,
    };

    const auto world_point = camera_to_world(camera_point);
    if (!world_point.allFinite()) {
        return std::unexpected{"Reconstructed world coordinate is not finite"};
    }

    if (world_point.z() < config_.table_height_m - 0.5) {
        return std::unexpected{std::format(
            "Reconstructed z {:.3f}m is too far below table height {:.3f}m", world_point.z(),
            config_.table_height_m)};
    }

    return MonoReconstructionSample{
        .timestamp_seconds = timestamp_seconds,
        .position = world_point,
        .quality_score = compute_quality_score(measurement),
    };
}

auto MonoReconstructor::config() const -> const MonoReconstructorConfig& {
    return config_;
}

auto MonoReconstructor::undistort_normalized_point(const Ball2D& measurement) const
    -> std::expected<Eigen::Vector2d, std::string> {
    const cv::Matx33d camera_matrix{
        config_.fx, 0.0, config_.cx,
        0.0, config_.fy, config_.cy,
        0.0, 0.0, 1.0,
    };

    const cv::Vec<double, 5> dist_coeffs{
        config_.distortion_coeffs[0],
        config_.distortion_coeffs[1],
        config_.distortion_coeffs[2],
        config_.distortion_coeffs[3],
        config_.distortion_coeffs[4],
    };

    const auto src = std::vector<cv::Point2f>{measurement.center};
    auto dst = std::vector<cv::Point2f>{};

    try {
        cv::undistortPoints(src, dst, camera_matrix, dist_coeffs);
    } catch (const cv::Exception& e) {
        return std::unexpected{std::format("cv::undistortPoints failed: {}", e.what())};
    }

    if (dst.empty()) {
        return std::unexpected{"Undistortion returned empty result"};
    }

    return Eigen::Vector2d{dst.front().x, dst.front().y};
}

auto MonoReconstructor::estimate_depth(const Ball2D& measurement) const
    -> std::expected<double, std::string> {
    if (measurement.radius <= 0.0F) {
        return std::unexpected{"Radius must be positive"};
    }

    const double focal = 0.5 * (config_.fx + config_.fy);
    if (focal <= 0.0) {
        return std::unexpected{"Invalid focal length (fx/fy must be positive)"};
    }

    const double depth = focal * config_.ball_radius_m / static_cast<double>(measurement.radius);
    if (!std::isfinite(depth)) {
        return std::unexpected{"Estimated depth is not finite"};
    }

    if (depth < config_.min_depth_m || depth > config_.max_depth_m) {
        return std::unexpected{std::format("Estimated depth {:.3f}m out of range [{:.3f}, {:.3f}]",
                                           depth, config_.min_depth_m, config_.max_depth_m)};
    }

    return depth;
}

auto MonoReconstructor::compute_quality_score(const Ball2D& measurement) const -> double {
    const double confidence_component =
        clamp01((measurement.confidence - config_.min_confidence)
                / std::max(1.0 - config_.min_confidence, 1e-6));

    const double tolerance = std::max(config_.radius_tolerance_px, 1e-6);
    const double radius_delta =
        (static_cast<double>(measurement.radius) - config_.expected_radius_px) / tolerance;
    const double radius_component = std::exp(-0.5 * radius_delta * radius_delta);

    const double left = static_cast<double>(measurement.center.x);
    const double right = static_cast<double>(config_.image_width_px) - static_cast<double>(measurement.center.x);
    const double top = static_cast<double>(measurement.center.y);
    const double bottom = static_cast<double>(config_.image_height_px) - static_cast<double>(measurement.center.y);
    const double boundary_distance = std::max(0.0, std::min({left, right, top, bottom}));
    const double boundary_component = clamp01(boundary_distance / std::max(config_.edge_margin_px, 1e-6));

    return clamp01(0.60 * confidence_component + 0.25 * radius_component + 0.15 * boundary_component);
}

auto MonoReconstructor::camera_to_world(const Eigen::Vector3d& camera_point) const -> Eigen::Vector3d {
    Eigen::Matrix3d rotation;
    rotation << config_.rotation_wc[0], config_.rotation_wc[1], config_.rotation_wc[2],
        config_.rotation_wc[3], config_.rotation_wc[4], config_.rotation_wc[5],
        config_.rotation_wc[6], config_.rotation_wc[7], config_.rotation_wc[8];

    const Eigen::Vector3d translation{
        config_.translation_wc[0],
        config_.translation_wc[1],
        config_.translation_wc[2],
    };

    return rotation * camera_point + translation;
}

auto load_mono_reconstructor_config(const YAML::Node& root)
    -> std::expected<MonoReconstructorConfig, std::string> {
    auto config = MonoReconstructorConfig{};

    const auto reconstruction_node = root["reconstruction"];
    if (!reconstruction_node || reconstruction_node.IsNull()) {
        return config;
    }

    const auto mono_node = reconstruction_node["mono"];
    if (!mono_node || mono_node.IsNull()) {
        return config;
    }

    if (!mono_node.IsMap()) {
        return std::unexpected{"'reconstruction.mono' must be a map"};
    }

    if (auto ret = read_required_scalar(mono_node, "fx", config.fx); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_required_scalar(mono_node, "fy", config.fy); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_required_scalar(mono_node, "cx", config.cx); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_required_scalar(mono_node, "cy", config.cy); !ret) {
        return std::unexpected{ret.error()};
    }

    if (auto ret = read_optional_array(mono_node, "distortion_coeffs", config.distortion_coeffs); !ret) {
        return std::unexpected{ret.error()};
    }

    if (auto ret = read_optional_scalar(mono_node, "ball_radius_m", config.ball_radius_m); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "table_height_m", config.table_height_m); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "min_confidence", config.min_confidence); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "min_radius_px", config.min_radius_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "max_radius_px", config.max_radius_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "min_depth_m", config.min_depth_m); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "max_depth_m", config.max_depth_m); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "expected_radius_px", config.expected_radius_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "radius_tolerance_px", config.radius_tolerance_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "image_width_px", config.image_width_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "image_height_px", config.image_height_px); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_scalar(mono_node, "edge_margin_px", config.edge_margin_px); !ret) {
        return std::unexpected{ret.error()};
    }

    if (auto ret = read_optional_array(mono_node, "rotation_wc", config.rotation_wc); !ret) {
        return std::unexpected{ret.error()};
    }
    if (auto ret = read_optional_array(mono_node, "translation_wc", config.translation_wc); !ret) {
        return std::unexpected{ret.error()};
    }

    if (config.fx <= 0.0 || config.fy <= 0.0) {
        return std::unexpected{"'reconstruction.mono.fx/fy' must be positive"};
    }
    if (config.ball_radius_m <= 0.0) {
        return std::unexpected{"'reconstruction.mono.ball_radius_m' must be positive"};
    }
    if (config.min_radius_px <= 0.0 || config.max_radius_px <= config.min_radius_px) {
        return std::unexpected{"'reconstruction.mono.min_radius_px/max_radius_px' range is invalid"};
    }
    if (config.min_depth_m <= 0.0 || config.max_depth_m <= config.min_depth_m) {
        return std::unexpected{"'reconstruction.mono.min_depth_m/max_depth_m' range is invalid"};
    }
    if (config.image_width_px <= 0 || config.image_height_px <= 0) {
        return std::unexpected{"'reconstruction.mono.image_width_px/image_height_px' must be positive"};
    }

    config.min_confidence = clamp01(config.min_confidence);
    config.edge_margin_px = std::max(config.edge_margin_px, 1.0);
    config.radius_tolerance_px = std::max(config.radius_tolerance_px, 1e-3);

    return config;
}

}  // namespace pingpong_tracker::reconstruction
