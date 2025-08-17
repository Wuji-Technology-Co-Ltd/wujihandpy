#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "utility/cross_os.hpp"

namespace data {

constexpr size_t data_length_max = 4;

PACKED_STRUCT(Data {
    uint8_t control;
    uint16_t index;
    uint8_t sub_index;
    uint8_t data[data_length_max];

    Data() = default;

    // The length defined for the array data[] often exceeds the actual length of the data,
    // thus we disabled all default copy and move functions.
    // Please manually extract the data of the correct length.
    Data(const Data&) = delete;
    Data& operator=(const Data&) = delete;
    Data(Data&&) = delete;
    Data& operator=(Data&&) = delete;
});

template <typename Base_, uint16_t index_, uint8_t sub_index_, typename ValueType_>
struct TestData {
    using Base = Base_;

    TestData() = delete;

    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using ValueType = ValueType_;
};

template <uint16_t index_, uint8_t sub_index_, typename DataType_, bool writable_ = false>
struct SpecializedData {
    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using DataType = DataType_;
    static constexpr uint8_t read_control = 0x30;
    static constexpr uint8_t read_result_control = []() constexpr {
        if constexpr (sizeof(DataType) == 1)
            return 0x35;
        else if constexpr (sizeof(DataType) == 2)
            return 0x37;
        else if constexpr (sizeof(DataType) == 4)
            return 0x39;
        else if constexpr (sizeof(DataType) == 8)
            return 0x3D;
        else
            return 0x00;
    }();

    static constexpr bool writable = writable_;
};

} // namespace data