#pragma once
#include <memory>

// NOLINTBEGIN(cppcoreguidelines-macro-usage, bugprone-macro-parentheses)
#define PINGPONG_TRACKER_PIMPL_DEFINITION(CLASS) \
public:                                          \
    ~CLASS() noexcept;                           \
    CLASS(const CLASS&)            = delete;     \
    CLASS& operator=(const CLASS&) = delete;     \
    CLASS(CLASS&&) noexcept;                     \
    CLASS& operator=(CLASS&&) noexcept;          \
                                                 \
private:                                         \
    struct Impl;                                 \
    std::unique_ptr<Impl> pimpl;
// NOLINTEND(cppcoreguidelines-macro-usage, bugprone-macro-parentheses)
