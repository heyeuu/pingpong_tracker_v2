#pragma once
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <format>

#include "utility/configure/parameters.hpp"
#include "utility/panic.hpp"

namespace pingpong_tracker::util {

inline auto configuration() {
    static auto root = [] {
        auto location = std::filesystem::path{
            Parameters::share_location(),
        };
        auto path = location / "config.yaml";
        if (!std::filesystem::exists(path)) {
            util::panic(std::format("Config file not found: {}", std::filesystem::absolute(path).string()));
        }
        try {
            return YAML::LoadFile(path.string());
        } catch (const std::exception& e) {
            util::panic(std::format("Failed to load config file: {}", e.what()));
        }
    }();
    return root;
}

}  // namespace pingpong_tracker::util
