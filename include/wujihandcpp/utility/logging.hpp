#pragma once

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp {
namespace logging {

enum class Level : int {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERR = 4,
    CRITICAL = 5,
    OFF = 6,
};

WUJIHANDCPP_API void set_log_to_console(bool value) noexcept;

WUJIHANDCPP_API void set_log_to_file(bool value) noexcept;

WUJIHANDCPP_API void set_log_level(Level value) noexcept;

WUJIHANDCPP_API void set_log_path(const char* value);

WUJIHANDCPP_API void flush() noexcept;

} // namespace logging
} // namespace wujihandcpp