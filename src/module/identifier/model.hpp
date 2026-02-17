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
    auto configure(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto sync_infer(const Image&) noexcept -> std::optional<std::vector<Ball2D>>;
    auto async_infer(const Image&,
                     std::function<void(std::expected<std::vector<Ball2D>, std::string>)>) noexcept
        -> void;

    struct AsyncResult final {
        using handle_type = std::coroutine_handle<>;
        OpenVinoNet& network;
        const Image& readonly;
        std::expected<std::vector<Ball2D>, std::string> result{std::unexpected{"Nothing here"}};
        auto await_resume() noexcept {
            return std::move(result);
        }
        auto await_suspend(handle_type coroutine) noexcept {
            network.async_infer(readonly, [coroutine, this](auto result) {
                this->result = std::move(result);
                coroutine.resume();
            });
        }
        static constexpr auto await_ready() noexcept {
            return false;
        }
    };

    auto await_infer(const Image& image) noexcept -> AsyncResult {
        return AsyncResult{.network = *this, .readonly = image};
    }
};

}  // namespace pingpong_tracker::identifier
