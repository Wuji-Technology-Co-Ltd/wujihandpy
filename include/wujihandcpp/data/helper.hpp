#pragma once

#include <cstdint>
#include <cstring>

#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {
namespace data {

using StorageInfo = protocol::Handler::StorageInfo;

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct ReadOnlyData {
    using Base = Base_;

    ReadOnlyData() = delete;

    static constexpr bool readable = true;
    static constexpr bool writable = false;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;

    static constexpr StorageInfo info(uint32_t) {
        return StorageInfo{sizeof(ValueType), index, sub_index, 0};
    }
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

    static constexpr StorageInfo info(uint32_t) {
        return StorageInfo{sizeof(ValueType), index, sub_index, 0};
    }
};

} // namespace data
} // namespace wujihandcpp