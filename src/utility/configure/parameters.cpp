#include "parameters.hpp"

using namespace pingpong_tracker::util;

[[maybe_unused]] static auto get_share_location() noexcept {
#ifdef PINGPONG_TRACKER_SOURCE
    return std::string(PINGPONG_TRACKER_SOURCE);
#else
    return std::string("");
#endif
}

struct Parameters::Impl { };

auto Parameters::share_location() noexcept -> std::string { return get_share_location(); }

Parameters::Parameters() noexcept
    : pimpl_ { std::make_unique<Impl>() } { }

Parameters::~Parameters() noexcept = default;
