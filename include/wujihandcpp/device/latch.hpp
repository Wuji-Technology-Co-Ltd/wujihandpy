#pragma once

#include <atomic>
#include <stdexcept>

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp {
namespace device {

class TimeoutError : public std::runtime_error {
public:
    using runtime_error::runtime_error;
};

class Latch {
public:
    template <typename T>
    friend class DataOperator;

    WUJIHANDCPP_API void wait();
    WUJIHANDCPP_API bool try_wait() noexcept;

private:
    WUJIHANDCPP_API void count_up() noexcept;
    WUJIHANDCPP_API void count_down(bool success) noexcept;

    std::atomic<int> waiting_count_{0};
    int error_count_{0};
};

} // namespace device
} // namespace wujihandcpp