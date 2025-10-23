#include <format>

#include <wujihandcpp/device/latch.hpp>
#include <wujihandcpp/utility/api.hpp>

#include "utility/final_action.hpp"

namespace wujihandcpp::device {

WUJIHANDCPP_API void Latch::wait() {
    utility::FinalAction reset_error{[this]() { error_count_ = 0; }};

    int current = waiting_count_.load(std::memory_order_acquire);
    while (current != 0) {
        waiting_count_.wait(current, std::memory_order_acquire);
        current = waiting_count_.load(std::memory_order_acquire);
    }

    if (error_count_) {
        if (error_count_ == 1)
            throw TimeoutError("Operation timed out while waiting for completion");
        else
            throw TimeoutError(
                std::format("{} operations timed out while waiting for completion", error_count_));
    }
}

WUJIHANDCPP_API bool Latch::try_wait() noexcept {
    utility::FinalAction reset_error{[this]() { error_count_ = 0; }};

    int current = waiting_count_.load(std::memory_order_acquire);
    while (current != 0) {
        waiting_count_.wait(current, std::memory_order_acquire);
        current = waiting_count_.load(std::memory_order_acquire);
    }

    return error_count_ == 0;
}

WUJIHANDCPP_API void Latch::count_up() noexcept {
    waiting_count_.fetch_add(1, std::memory_order_relaxed);
}
WUJIHANDCPP_API void Latch::count_down(bool success) noexcept {
    if (!success)
        error_count_++;

    auto old = waiting_count_.fetch_sub(1, std::memory_order_release);
    if (old - 1 == 0)
        waiting_count_.notify_one();
}

} // namespace wujihandcpp::device