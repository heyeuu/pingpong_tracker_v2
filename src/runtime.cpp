
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <format>
#include <limits>
#include <numeric>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "kernel/capturer.hpp"
#include "kernel/identifier.hpp"
#include "kernel/predictor.hpp"
#include "kernel/reconstruction.hpp"
#include "kernel/visualization.hpp"
#include "module/debug/action_throttler.hpp"
#include "utility/configure/configuration.hpp"
#include "utility/image/ball.hpp"
#include "utility/image/image.details.hpp"
#include "utility/panic.hpp"
#include "utility/singleton/running.hpp"

using namespace pingpong_tracker;

namespace {

struct CandidateMotionTracker {
    struct Anchor {
        cv::Point2f origin;
        cv::Point2f last;
        util::Clock::time_point last_seen;
        int hits = 0;
    };

    static constexpr float kMatchRadiusPx = 15.0F;
    static constexpr float kMaxDriftPx    = 30.0F;
    static constexpr int kStaticHits      = 12;

    static constexpr auto kAnchorLifetime = std::chrono::duration<double>{0.5};

    std::vector<Anchor> anchors;

    auto update(const std::vector<Ball2D>& balls, util::Clock::time_point stamp)
        -> std::vector<bool> {
        auto is_static = std::vector<bool>(balls.size(), false);
        auto matched   = std::vector<bool>(anchors.size(), false);

        for (std::size_t i = 0; i < balls.size(); ++i) {
            const auto& center = balls[i].center;

            auto best_index    = std::optional<std::size_t>{};
            auto best_distance = kMatchRadiusPx;

            for (std::size_t j = 0; j < anchors.size(); ++j) {
                if (matched[j]) {
                    continue;
                }
                const auto distance = cv::norm(center - anchors[j].last);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index    = j;
                }
            }

            if (!best_index.has_value()) {
                anchors.push_back(
                    Anchor{.origin = center, .last = center, .last_seen = stamp, .hits = 1});
                matched.push_back(true);
                continue;
            }

            auto& anchor         = anchors[*best_index];
            matched[*best_index] = true;

            if (cv::norm(center - anchor.origin) > kMaxDriftPx) {
                anchor = Anchor{.origin = center, .last = center, .last_seen = stamp, .hits = 1};
            } else {
                anchor.last      = center;
                anchor.last_seen = stamp;
                ++anchor.hits;
            }

            is_static[i] = anchor.hits >= kStaticHits;
        }

        std::erase_if(anchors, [&](const Anchor& anchor) {
            return std::chrono::duration<double>(stamp - anchor.last_seen) > kAnchorLifetime;
        });

        return is_static;
    }
};

}  // namespace

int main() {
    using namespace std::chrono_literals;

    auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            spdlog::error("Failed to init '{}'", runtime_name);
            spdlog::error("  {}", result.error());
            util::panic(std::format("Failed to initialize {}", runtime_name));
        }
    };

    /// Runtime
    auto capturer   = kernel::Capturer{};
    auto identifier = kernel::Identifier{};
    auto reconstruction = kernel::Reconstruction{};
    auto predictor = kernel::Predictor{};

    auto visualization    = kernel::Visualization{};
    auto action_throttler = util::ActionThrottler{1s, 233};

    /// Configure
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();

    // Overlay projection intrinsics (world == camera while R_wc = I, t_wc = 0)
    auto mono_configuration = configuration["reconstruction"]["mono"];
    const auto overlay_fx   = mono_configuration["fx"].as<double>();
    const auto overlay_fy   = mono_configuration["fy"].as<double>();
    const auto overlay_cx   = mono_configuration["cx"].as<double>();
    const auto overlay_cy   = mono_configuration["cy"].as<double>();

    const auto reset_interval_node = configuration["predictor"]["reset_interval_seconds"];
    const auto predictor_reset_interval =
        std::chrono::duration<double>{reset_interval_node && !reset_interval_node.IsNull()
                                          ? reset_interval_node.as<double>()
                                          : 1.0};

    // CAPTURER
    {
        auto config = configuration["capturer"];
        auto result = capturer.initialize(config);
        handle_result("capturer", result);
    }
    // IDENTIFIER
    {
        auto config = configuration["identifier"];

        const auto model_location =
            std::filesystem::path{util::Parameters::share_location()}
            / std::filesystem::path{config["model_location"].as<std::string>()};
        config["model_location"] = model_location.string();

        auto result = identifier.initialize(config);
        handle_result("identifier", result);
    }
    // RECONSTRUCTION
    {
        auto result = reconstruction.initialize(configuration);
        handle_result("reconstruction", result);
    }
    // PREDICTOR
    {
        auto result = predictor.initialize(configuration);
        handle_result("predictor", result);
    }

    // VISUALIZATION
    if (use_visualization) {
        auto config = configuration["visualization"];
        auto result = visualization.initialize(config);
        handle_result("visualization", result);
    }

    // DEBUG
    {
        action_throttler.register_action("no_balls_detected", 3);
        action_throttler.register_action("identify_error", 1);
        action_throttler.register_action("balls_detected", 10);
        action_throttler.register_action("reconstruction_error", 2);
        action_throttler.register_action("predictor_update_error", 2);
    }

    auto detect_balls = [&](const auto& image) {
        auto result = identifier.sync_identify(*image);
        if (!result) {
            action_throttler.dispatch("identify_error", [&] {
                spdlog::warn("Failed to identify balls: {}", result.error());
            });
            return typename decltype(result)::value_type{};
        }

        const auto& balls = *result;
        if (balls.empty()) {
            action_throttler.dispatch("no_balls_detected",
                                      [&] { spdlog::info("Detected {} balls", balls.size()); });
        } else {
            action_throttler.dispatch("balls_detected",
                                      [&] { spdlog::info("Detected {} balls", balls.size()); });
            action_throttler.reset("no_balls_detected");
        }

        return std::move(*result);
    };

    struct SelectedObservation {
        std::size_t index = 0;
        kernel::Reconstruction::Observation observation;
    };

    auto select_observation = [&](const auto& balls_2d, const std::vector<bool>& is_static,
                                  const auto& stamp) -> std::optional<SelectedObservation> {
        if (balls_2d.empty()) {
            return std::nullopt;
        }

        auto order = std::vector<std::size_t>(balls_2d.size());
        std::iota(order.begin(), order.end(), 0U);
        std::ranges::sort(order, [&](std::size_t lhs, std::size_t rhs) {
            return balls_2d[lhs].confidence > balls_2d[rhs].confidence;
        });

        auto best          = std::optional<SelectedObservation>{};
        auto best_distance = std::numeric_limits<double>::infinity();

        for (const auto index : order) {
            if (index < is_static.size() && is_static[index]) {
                continue;
            }

            const auto reconstructed = reconstruction.reconstruct(balls_2d[index], stamp);
            if (!reconstructed) {
                continue;
            }

            if (!predictor.initialized()) {
                return SelectedObservation{.index = index, .observation = *reconstructed};
            }

            const auto distance = (reconstructed->position - predictor.position()).norm();
            if (distance < best_distance) {
                best_distance = distance;
                best = SelectedObservation{.index = index, .observation = *reconstructed};
            }
        }

        return best;
    };

    auto visualize_detection = [&](auto& image, const auto& balls_2d,
                                   const std::vector<bool>& is_static,
                                   const std::optional<std::size_t>& selected_index,
                                   bool update_success,
                                   const std::optional<kernel::Reconstruction::Observation>&
                                       observation) {
        if (use_painted_image) {
            auto& opencv_mat = const_cast<cv::Mat&>(image.details().get_mat());

            for (std::size_t i = 0; i < balls_2d.size(); ++i) {
                util::draw(image, balls_2d[i]);

                if (i < is_static.size() && is_static[i]) {
                    cv::circle(opencv_mat, balls_2d[i].center,
                               static_cast<int>(balls_2d[i].radius) + 6, cv::Scalar{128, 128, 128},
                               2);
                    cv::putText(opencv_mat, "static",
                                balls_2d[i].center + cv::Point2f{0.0F, -34.0F},
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{128, 128, 128}, 1,
                                cv::LINE_AA);
                }
            }

            if (selected_index.has_value() && *selected_index < balls_2d.size()) {
                const auto& selected = balls_2d[*selected_index];
                cv::circle(opencv_mat, selected.center, static_cast<int>(selected.radius) + 4,
                           cv::Scalar{255, 255, 0}, 2);
                cv::putText(opencv_mat, "used", selected.center + cv::Point2f{0.0F, -20.0F},
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{255, 255, 0}, 1, cv::LINE_AA);
            }
            auto y_offset = 24;

            auto draw_text = [&](const std::string& text, const cv::Scalar& color) {
                cv::putText(opencv_mat, text, cv::Point{10, y_offset}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            color, 1, cv::LINE_AA);
                y_offset += 18;
            };

            draw_text(std::format("Predictor initialized: {}", predictor.initialized() ? "yes" : "no"),
                      cv::Scalar{230, 230, 230});
            draw_text(std::format("Update count: {}", predictor.update_count()),
                      cv::Scalar{230, 230, 230});
            draw_text(std::format("Last update: {}", update_success ? "success" : "no update"),
                      update_success ? cv::Scalar{0, 220, 0} : cv::Scalar{0, 180, 255});

            if (observation.has_value()) {
                draw_text(std::format("Obs xyz: {:.3f}, {:.3f}, {:.3f}", observation->position.x(),
                                      observation->position.y(), observation->position.z()),
                          cv::Scalar{255, 210, 120});
                draw_text(std::format("Obs quality: {:.2f}", observation->quality_score),
                          cv::Scalar{255, 210, 120});
            }

            const auto project = [&](const Eigen::Vector3d& point) -> std::optional<cv::Point> {
                if (!point.allFinite() || point.z() <= 1e-6) {
                    return std::nullopt;
                }
                return cv::Point{
                    static_cast<int>(overlay_fx * point.x() / point.z() + overlay_cx),
                    static_cast<int>(overlay_fy * point.y() / point.z() + overlay_cy),
                };
            };

            const auto draw_marker = [&](const Eigen::Vector3d& point, const cv::Scalar& color,
                                         const char* label) {
                const auto pixel = project(point);
                if (!pixel.has_value()) {
                    return;
                }
                cv::drawMarker(opencv_mat, *pixel, color, cv::MARKER_CROSS, 18, 2);
                cv::putText(opencv_mat, label, *pixel + cv::Point{10, -10}, cv::FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1, cv::LINE_AA);
            };

            if (observation.has_value()) {
                draw_marker(observation->position, cv::Scalar{255, 160, 0}, "obs");
            }

            if (predictor.initialized()) {
                const auto estimate = predictor.position();
                draw_text(std::format("Est xyz: {:.3f}, {:.3f}, {:.3f}", estimate.x(), estimate.y(),
                                      estimate.z()),
                          cv::Scalar{140, 255, 140});
                draw_marker(estimate, cv::Scalar{0, 255, 0}, "est");

                const auto trajectory = predictor.predict_trajectory(100ms, 10ms);
                auto previous_pixel   = std::optional<cv::Point>{};
                for (const auto& point : trajectory) {
                    const auto pixel = project(point);
                    if (!pixel.has_value()) {
                        previous_pixel.reset();
                        continue;
                    }
                    if (previous_pixel.has_value()) {
                        cv::line(opencv_mat, *previous_pixel, *pixel, cv::Scalar{0, 220, 255}, 2,
                                 cv::LINE_AA);
                    }
                    previous_pixel = pixel;
                }

                if (!trajectory.empty()) {
                    const auto& prediction = trajectory.back();
                    draw_text(std::format("Pred@100ms: {:.3f}, {:.3f}, {:.3f}", prediction.x(),
                                          prediction.y(), prediction.z()),
                              cv::Scalar{140, 255, 255});
                    draw_marker(prediction, cv::Scalar{0, 0, 255}, "pred@100ms");
                }
            }
        }

        if (visualization.initialized()) {
            visualization.send_image(image);
        }
    };

    auto last_update_stamp = std::optional<util::Clock::time_point>{};
    auto motion_tracker    = CandidateMotionTracker{};
    auto rejected_count    = 0;

    constexpr auto kMaxRejectedUpdates = 3;

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        if (auto image = capturer.fetch_image()) {
            auto balls           = detect_balls(image);
            const auto stamp     = image->get_timestamp();
            const auto is_static = motion_tracker.update(balls, stamp);

            if (predictor.initialized() && last_update_stamp.has_value()
                && std::chrono::duration<double>(stamp - *last_update_stamp)
                       > predictor_reset_interval) {
                predictor.reset();
                last_update_stamp.reset();
                rejected_count = 0;
            }

            auto update_success = false;
            auto observation    = std::optional<kernel::Reconstruction::Observation>{};
            auto selected_index = std::optional<std::size_t>{};

            if (const auto selected = select_observation(balls, is_static, stamp);
                selected.has_value()) {
                selected_index = selected->index;
                observation    = selected->observation;
                update_success = predictor.update(observation->position, stamp);

                if (!update_success && predictor.initialized()
                    && ++rejected_count >= kMaxRejectedUpdates) {
                    predictor.reset();
                    update_success = predictor.update(observation->position, stamp);
                    rejected_count = 0;
                }

                if (update_success) {
                    last_update_stamp = stamp;
                    rejected_count    = 0;
                } else {
                    action_throttler.dispatch("predictor_update_error", [&] {
                        spdlog::warn(
                            "Predictor update rejected or failed for reconstructed observation");
                    });
                }
            }

            visualize_detection(*image, balls, is_static, selected_index, update_success,
                                observation);

        } else {
            std::this_thread::sleep_for(1ms);
        }
    }

    return 0;
}
