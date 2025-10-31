#include <wujihandcpp/utility/api.hpp>

#include "utility/logging.hpp"

namespace wujihandcpp::logging {

WUJIHANDCPP_API void set_log_to_console(bool value) noexcept {
    get_config().set_log_to_console(value);
}

WUJIHANDCPP_API void set_log_to_file(bool value) noexcept { get_config().set_log_to_file(value); }

WUJIHANDCPP_API void set_log_level(Level value) noexcept { get_config().set_log_level(value); }

WUJIHANDCPP_API void set_log_path(const char* value) {
    if (!value)
        throw std::invalid_argument("Log path cannot be null");
    get_config().set_log_path(value);
}

} // namespace wujihandcpp::logging
