#include "workers.hpp"
#include <queue>

using namespace pingpong_tracker;

struct WorkersContext::Impl {
    std::queue<std::unique_ptr<WorkersContext::InternalTask>> tasks;
};

WorkersContext::WorkersContext() noexcept
    : pimpl_ { std::make_unique<Impl>() } { }

WorkersContext::~WorkersContext() noexcept = default;

auto WorkersContext::internal_enqueue(std::unique_ptr<InternalTask> task) noexcept -> void {
    pimpl_->tasks.push(std::move(task));
}
