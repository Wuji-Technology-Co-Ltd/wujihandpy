#pragma once

#include <atomic>

#include "utility/cross_os.hpp"

namespace wujihandcpp {
namespace device {

class Latch {
public:
    template <typename T>
    friend class DataOperator;

    API void wait();

private:
    API void count_up();
    API void count_down();

    std::atomic<int> waiting_count_{0};
};

} // namespace device
} // namespace wujihandcpp