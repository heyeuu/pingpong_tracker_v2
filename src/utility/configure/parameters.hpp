#pragma once
#include <string>
#include <vector>

#include "utility/pimpl.hpp"

namespace pingpong_tracker::util {

struct Parameters {
    PINGPONG_TRACKER_PIMPL_DEFINITION(Parameters)

public:
    Parameters() noexcept;

    static auto share_location() noexcept -> std::string;
};

struct IParams {
    virtual ~IParams() = default;

    virtual auto get_string(const std::string&) -> std::string                    = 0;
    virtual auto get_string_array(const std::string&) -> std::vector<std::string> = 0;

    virtual auto get_int64(const std::string&) -> std::int64_t                    = 0;
    virtual auto get_int64_array(const std::string&) -> std::vector<std::int64_t> = 0;

    virtual auto get_bool(const std::string&) -> bool                    = 0;
    virtual auto get_bool_array(const std::string&) -> std::vector<bool> = 0;

    virtual auto get_double(const std::string&) -> double                    = 0;
    virtual auto get_double_array(const std::string&) -> std::vector<double> = 0;

    virtual auto get_uint8_array(const std::string&) -> std::vector<std::uint8_t> = 0;
};

}  // namespace pingpong_tracker::util
