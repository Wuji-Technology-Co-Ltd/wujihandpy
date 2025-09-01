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
    static constexpr size_t value_size = sizeof(ValueType);

    static constexpr uint32_t policy(uint64_t) { return 0; }
}; // namespace data

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct WriteOnlyData {
    using Base = Base_;

    WriteOnlyData() = delete;

    static constexpr bool readable = false;
    static constexpr bool writable = true;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
    static constexpr size_t value_size = sizeof(ValueType);

    static constexpr uint32_t policy(uint64_t) { return 0; }
};

} // namespace data
} // namespace wujihandcpp