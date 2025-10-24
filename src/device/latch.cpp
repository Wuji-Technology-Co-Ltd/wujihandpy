#include <wujihandcpp/device/latch.hpp>
#include <wujihandcpp/utility/api.hpp>

namespace wujihandcpp::device {

WUJIHANDCPP_API int Latch::try_wait_internal() noexcept {
    int current = waiting_count_.load(std::memory_order_acquire);
    while (current != 0) {
        waiting_count_.wait(current, std::memory_order_acquire);
        current = waiting_count_.load(std::memory_order_acquire);
    }

    return error_count_.exchange(0, std::memory_order_relaxed);
}

WUJIHANDCPP_API void Latch::count_up() noexcept {
    waiting_count_.fetch_add(1, std::memory_order_relaxed);
}

WUJIHANDCPP_API void Latch::count_down(bool success) noexcept {
    if (!success)
        error_count_.fetch_add(1, std::memory_order_relaxed);

    const int old = waiting_count_.fetch_sub(1, std::memory_order_release);
    if (old - 1 == 0)
        waiting_count_.notify_one();
}

} // namespace wujihandcpp::device