#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <eigen3/Eigen/Core>

#include "utility/time/clock.hpp"

namespace pingpong_tracker::predictor {

struct ReplaySample {
    double timestamp_seconds = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct ReplayMetrics {
    std::size_t update_attempts = 0;
    std::size_t update_success = 0;
    std::size_t filtered_count = 0;
    std::size_t horizon_count = 0;
    double filtered_sse = 0.0;
    double horizon_sse = 0.0;
};

[[nodiscard]] inline auto to_clock_stamp(double timestamp_seconds) -> util::Clock::time_point {
    const auto duration = std::chrono::duration<double>{timestamp_seconds};
    return util::Clock::time_point{
        std::chrono::duration_cast<util::Clock::duration>(duration),
    };
}

[[nodiscard]] inline auto interpolate_position_at_time(std::span<const ReplaySample> samples,
                                                       double timestamp_seconds,
                                                       std::size_t search_from = 0)
    -> std::optional<Eigen::Vector3d> {
    if (samples.empty()) {
        return std::nullopt;
    }

    const auto start_offset = std::min(search_from, samples.size());
    const auto begin = samples.begin() + static_cast<std::ptrdiff_t>(start_offset);
    const auto it =
        std::lower_bound(begin, samples.end(), timestamp_seconds,
                         [](const ReplaySample& sample, double value) {
                             return sample.timestamp_seconds < value;
                         });

    if (it != samples.end() && it->timestamp_seconds == timestamp_seconds) {
        return it->position;
    }
    if (it == begin || it == samples.end()) {
        return std::nullopt;
    }

    const auto& early = *(it - 1);
    const auto& late = *it;
    const double span_seconds = late.timestamp_seconds - early.timestamp_seconds;
    if (span_seconds <= 0.0) {
        return std::nullopt;
    }

    const double ratio = (timestamp_seconds - early.timestamp_seconds) / span_seconds;
    return early.position + ratio * (late.position - early.position);
}

template <typename Tracker>
auto evaluate_replay(Tracker& tracker, std::span<const ReplaySample> samples,
                     std::chrono::milliseconds horizon, std::chrono::milliseconds step,
                     std::size_t warmup) -> ReplayMetrics {
    auto metrics = ReplayMetrics{};
    if (samples.empty()) {
        return metrics;
    }

    const auto horizon_steps = static_cast<std::size_t>(horizon / step);
    const double horizon_seconds = std::chrono::duration<double>{horizon}.count();

    for (std::size_t i = 0; i < samples.size(); ++i) {
        metrics.update_attempts += 1;

        typename Tracker::ZVec measurement;
        measurement << samples[i].position.x(), samples[i].position.y(), samples[i].position.z();

        if (!tracker.update(measurement, to_clock_stamp(samples[i].timestamp_seconds))) {
            continue;
        }
        metrics.update_success += 1;

        if (tracker.update_count() > 1) {
            const Eigen::Vector3d filtered_error = tracker.position() - samples[i].position;
            metrics.filtered_sse += filtered_error.squaredNorm();
            metrics.filtered_count += 1;
        }

        if (i < warmup || horizon_steps == 0) {
            continue;
        }

        const double target_timestamp = samples[i].timestamp_seconds + horizon_seconds;
        if (target_timestamp > samples.back().timestamp_seconds) {
            continue;
        }

        const auto target_position = interpolate_position_at_time(samples, target_timestamp, i + 1);
        if (!target_position) {
            continue;
        }

        const auto trajectory = tracker.predict_trajectory(horizon, step);
        if (trajectory.size() < horizon_steps) {
            continue;
        }

        const Eigen::Vector3d horizon_error = trajectory[horizon_steps - 1] - *target_position;
        metrics.horizon_sse += horizon_error.squaredNorm();
        metrics.horizon_count += 1;
    }

    return metrics;
}

}  // namespace pingpong_tracker::predictor
