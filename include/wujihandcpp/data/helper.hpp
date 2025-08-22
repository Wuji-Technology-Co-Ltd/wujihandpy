#pragma once

#include <cstdint>
#include <cstring>

namespace wujihandcpp::data {

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct TestData {
    using Base = Base_;

    TestData() = delete;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
};

} // namespace wujihandcpp::data