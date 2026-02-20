#pragma once

#include <yaml-cpp/yaml.h>

#include <coroutine>
#include <expected>
#include <functional>
#include <optional>
#include <string>

#include "utility/ball/ball.hpp"
#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

namespace pingpong_tracker::identifier {

class OpenVinoNet {
    struct Impl;
    std::shared_ptr<Impl> pimpl;

public:
    OpenVinoNet();
    ~OpenVinoNet();

    OpenVinoNet(const OpenVinoNet&)                = delete;
    OpenVinoNet& operator=(const OpenVinoNet&)     = delete;
    OpenVinoNet(OpenVinoNet&&) noexcept            = default;
    OpenVinoNet& operator=(OpenVinoNet&&) noexcept = default;

    auto configure(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto sync_infer(const Image&) noexcept -> std::expected<std::vector<Ball2D>, std::string>;
    auto async_infer(const Image&,
                     std::function<void(std::expected<std::vector<Ball2D>, std::string>)>) noexcept
        -> void;

    struct AsyncResult final {
        using handle_type = std::coroutine_handle<>;
        OpenVinoNet& network;
        Image image;
        std::expected<std::vector<Ball2D>, std::string> result{std::unexpected{"Nothing here"}};
        auto await_resume() noexcept {
            return std::move(result);
        }
        auto await_suspend(handle_type coroutine) noexcept {
            network.async_infer(image, [coroutine, this](auto result) {
                this->result = std::move(result);
                // coroutine.resume() executes on the OpenVINO callback thread.
                // Callers must ensure thread-safety or marshal resume back to the original/executor
                // thread.
                coroutine.resume();
            });
        }
        static constexpr auto await_ready() noexcept {
            return false;
        }
    };

    [[nodiscard]] auto await_infer(Image image) -> AsyncResult {
        return AsyncResult{.network = *this, .image = std::move(image)};
    }
};

}  // namespace pingpong_tracker::identifier
