#pragma once

#include <cstdint>
#include <cstring>

namespace wujihandcpp {
namespace data {

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct RemoteData {
    using Base = Base_;

    RemoteData() = delete;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
};

} // namespace data
} // namespace wujihandcpp