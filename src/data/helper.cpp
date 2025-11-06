
#include <cstddef>

#include <format>

#include <wujihandcpp/data/helper.hpp>

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp::data {

WUJIHANDCPP_API size_t FirmwareVersionData::string_length() const {
    if (pre == '\0')
        return std::formatted_size("{}.{}.{}", major, minor, patch);
    else
        return std::formatted_size("{}.{}.{}-{}", major, minor, patch, pre);
}

WUJIHANDCPP_API void FirmwareVersionData::write_to_string(char* dst) const {
    if (pre == '\0')
        std::format_to(dst, "{}.{}.{}", major, minor, patch);
    else
        std::format_to(dst, "{}.{}.{}-{}", major, minor, patch, pre);
}

} // namespace wujihandcpp::data