#pragma once
#include <memory>

namespace pimpl::internal {
inline auto use_memory_header() {
    // Remove warning from unused include
    std::ignore = std::unique_ptr<int>();
}
} // namespace internal

#define RMCS_PIMPL_DEFINITION(CLASS)                                                               \
public:                                                                                            \
    explicit CLASS();                                                                              \
    ~CLASS() noexcept;                                                                             \
    CLASS(const CLASS&)            = delete;                                                       \
    CLASS& operator=(const CLASS&) = delete;                                                       \
    CLASS(CLASS&&) noexcept;                                                                       \
    CLASS& operator=(CLASS&&) noexcept;                                                            \
                                                                                                   \
private:                                                                                           \
    struct Impl;                                                                                   \
    std::unique_ptr<Impl> pimpl;
