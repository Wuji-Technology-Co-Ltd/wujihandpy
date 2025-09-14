#include <wujihandcpp/device/latch.hpp>
#include <wujihandcpp/utility/api.hpp>

namespace wujihandcpp::device {

WUJIHANDCPP_API void Latch::wait() {
    int current = waiting_count_.load(std::memory_order_acquire);
    while (current != 0) {
        waiting_count_.wait(current, std::memory_order_acquire);
        current = waiting_count_.load(std::memory_order_acquire);
    }
}

WUJIHANDCPP_API void Latch::count_up() { waiting_count_.fetch_add(1, std::memory_order_relaxed); }
WUJIHANDCPP_API void Latch::count_down() {
    auto old = waiting_count_.fetch_sub(1, std::memory_order_release);
    if (old - 1 == 0)
        waiting_count_.notify_one();
}

} // namespace wujihandcpp::device