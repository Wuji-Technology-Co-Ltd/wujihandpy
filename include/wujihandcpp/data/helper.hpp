#pragma once

#include <cstdint>
#include <cstring>

namespace wujihandcpp {
namespace data {

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct ReadOnlyData {
    using Base = Base_;

    ReadOnlyData() = delete;

    static constexpr bool readable = true;
    static constexpr bool writable = false;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
};

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct WriteOnlyData {
    using Base = Base_;

    WriteOnlyData() = delete;

    static constexpr bool readable = false;
    static constexpr bool writable = true;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
};

} // namespace data
} // namespace wujihandcpp