
#include <cctype>
#include <cstddef>

#include <format>

#include <wujihandcpp/data/helper.hpp>

#include "wujihandcpp/utility/api.hpp"

namespace wujihandcpp::data {

WUJIHANDCPP_API size_t FirmwareVersionData::string_length() const {
    if (pre == '~')
        return std::formatted_size("{}.{}.{}", major, minor, patch);
    else if (auto upper = std::toupper(static_cast<unsigned char>(pre));
             'A' <= upper && upper <= 'Z')
        return std::formatted_size("{}.{}.{}-rc{}", major, minor, patch, int(upper - 'A'));
    else
        return std::formatted_size("{}.{}.{}-{}", major, minor, patch, int(pre));
}

WUJIHANDCPP_API void FirmwareVersionData::write_to_string(char* dst) const {
    if (pre == '~')
        std::format_to(dst, "{}.{}.{}", major, minor, patch);
    else if (auto upper = std::toupper(static_cast<unsigned char>(pre));
             'A' <= upper && upper <= 'Z')
        std::format_to(dst, "{}.{}.{}-rc{}", major, minor, patch, int(upper - 'A'));
    else
        std::format_to(dst, "{}.{}.{}-{}", major, minor, patch, int(pre));
}

} // namespace wujihandcpp::data
