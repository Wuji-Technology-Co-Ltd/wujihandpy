#pragma once

#include <atomic>

namespace wujihandcpp::device {

class Latch {
public:
    template <typename T>
    friend class DataOperator;

    void wait();

private:
    void count_up();
    void count_down();

    std::atomic<int> waiting_count_ = 0;
};

} // namespace wujihandcpp::device