#pragma once

namespace pingpong_tracker::util {

auto get_running() noexcept -> bool;

auto set_running(bool) noexcept -> void;

}
