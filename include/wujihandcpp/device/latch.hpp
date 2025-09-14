#pragma once

#include <atomic>

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp {
namespace device {

class Latch {
public:
    template <typename T>
    friend class DataOperator;

    WUJIHANDCPP_API void wait();

private:
    WUJIHANDCPP_API void count_up();
    WUJIHANDCPP_API void count_down();

    std::atomic<int> waiting_count_{0};
};

} // namespace device
} // namespace wujihandcpp