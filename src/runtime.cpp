
#include <spdlog/spdlog.h>

#include <format>
#include <print>

#include "kernel/capturer.hpp"
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
    auto capturer = kernel::Capturer{};

    /// Configure
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();
    auto use_painted_image = configuration["use_painted_image"].as<bool>();
    auto is_local_runtime  = configuration["is_local_runtime"].as<bool>();

    // CAPTURER
    {
        auto config = configuration["capturer"];
        auto result = capturer.initialize(config);
        handle_result("capturer", result);
    }

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        if (auto image = capturer.fetch_image()) {
        }
    }

    return 0;
}
