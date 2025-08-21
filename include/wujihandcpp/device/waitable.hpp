#pragma once

#include <atomic>

namespace wujihandcpp::device {

class Waitable {
public:
    template <typename T>
    friend class DataOperator;

    void wait() {
        int current = waiting_count_.load(std::memory_order_acquire);
        while (current != 0) {
            waiting_count_.wait(current, std::memory_order_acquire);
            current = waiting_count_.load(std::memory_order_acquire);
        }
    }

private:
    void acquire() { waiting_count_.fetch_add(1, std::memory_order_relaxed); }
    void release() {
        if (waiting_count_.fetch_sub(1, std::memory_order_release) - 1 == 0)
            waiting_count_.notify_one();
    }

    std::atomic<int> waiting_count_ = 0;
};

} // namespace wujihandcpp::device