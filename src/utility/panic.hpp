#pragma once
#include <source_location>
#include <string>

namespace pingpong_tracker::util {

[[noreturn]] auto panic(const std::string& message,
    const std::source_location& loc = std::source_location::current()) -> void;

}
