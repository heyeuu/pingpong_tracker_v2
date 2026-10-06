#include "reconstruction.hpp"

#include <chrono>

namespace pingpong_tracker::kernel {

auto Reconstruction::initialize(const YAML::Node& root) noexcept -> Result {
    const auto config = reconstruction::load_mono_reconstructor_config(root);
    if (!config) {
        return std::unexpected{config.error()};
    }

    mono_reconstructor_.emplace(*config);
    first_stamp_.reset();
    return {};
}

auto Reconstruction::reconstruct(const Ball2D& ball, util::Clock::time_point stamp) noexcept
    -> std::expected<Observation, std::string> {
    if (!mono_reconstructor_.has_value()) {
        return std::unexpected{"Reconstruction is not initialized"};
    }

    if (!first_stamp_.has_value()) {
        first_stamp_ = stamp;
    }

    if (stamp < *first_stamp_) {
        first_stamp_ = stamp;
    }

    const auto elapsed = std::chrono::duration<double>(stamp - *first_stamp_).count();
    const auto reconstructed = mono_reconstructor_->reconstruct(ball, elapsed);
    if (!reconstructed) {
        return std::unexpected{reconstructed.error()};
    }

    return Observation{
        .stamp = stamp,
        .timestamp_seconds = reconstructed->timestamp_seconds,
        .position = reconstructed->position,
        .quality_score = reconstructed->quality_score,
    };
}

auto Reconstruction::initialized() const noexcept -> bool {
    return mono_reconstructor_.has_value();
}

}  // namespace pingpong_tracker::kernel
