
#include <spdlog/spdlog.h>

#include <format>
#include <thread>

#include "kernel/capturer.hpp"
#include "kernel/identifier.hpp"
#include "module/debug/action_throttler.hpp"
#include "module/debug/framerate.hpp"
#include "utility/configure/configuration.hpp"
#include "utility/panic.hpp"
#include "utility/singleton/running.hpp"

using namespace pingpong_tracker;

int main() {
    using namespace std::chrono_literals;

    auto handle_result = [&](auto runtime_name, const auto& result) {
        if (!result.has_value()) {
            spdlog::error("Failed to init '{}'", runtime_name);
            spdlog::error("  {}", result.error());
            util::panic(std::format("Failed to initialize {}", runtime_name));
        }
    };

    auto framerate = FramerateCounter{};
    framerate.set_interval(5s);

    /// Runtime
    auto capturer   = kernel::Capturer{};
    auto identifier = kernel::Identifier{};

    auto action_throttler = util::ActionThrottler{1s, 233};

    /// Configure
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();

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

    auto detect_balls = [&](const auto& image) {
        auto result = identifier.sync_identify(*image);
        if (!result) {
            action_throttler.dispatch("identify_error", [&] {
                spdlog::warn("Failed to identify balls: {}", result.error());
            });
            return typename decltype(result)::value_type{};
        }
        return std::move(*result);
    };

    // DEBUG
    {
        action_throttler.register_action("no_balls_detected", 3);
        action_throttler.register_action("identify_error", 1);
        action_throttler.register_action("balls_detected", 10);
    }

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        if (auto image = capturer.fetch_image()) {
            auto balls = detect_balls(image);

            if (use_visualization) {
                if (balls.empty()) {
                    action_throttler.dispatch("no_balls_detected", [&] {
                        spdlog::info("Detected {} balls", balls.size());
                    });
                } else {
                    action_throttler.dispatch(
                        "balls_detected", [&] { spdlog::info("Detected {} balls", balls.size()); });
                    action_throttler.reset("no_balls_detected");
                }
            }

        } else {
            std::this_thread::sleep_for(1ms);
        }
    }

    return 0;
}
