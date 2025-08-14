#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "utility/cross_os.hpp"

namespace data {

constexpr size_t data_length_max = 8;

PACKED_STRUCT(Data) {
    uint16_t index;
    uint8_t sub_index;
    uint8_t data_length;
    uint8_t data[data_length_max];

    Data() = default;

    // The length defined for the array data[] often exceeds the actual length of the data,
    // thus we disabled all default copy and move functions.
    // Please manually extract the data of the correct length.
    Data(const Data&) = delete;
    Data& operator=(const Data&) = delete;
    Data(Data&&) = delete;
    Data& operator=(Data&&) = delete;
};

template <uint16_t index_, uint8_t sub_index_, typename DataType_, bool writable_ = false>
struct SpecializedData {
    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using DataType = DataType_;

    static constexpr bool writable = writable_;

    // NOLINTNEXTLINE(google-explicit-constructor)
    ALWAYS_INLINE SpecializedData(const Data& data) {
        if (data.index == index && data.sub_index == sub_index) {
            std::memcpy(&data_, data.data, sizeof(DataType));
            available_ = true;
        } else {
            available_ = false;
        }
    }

    ALWAYS_INLINE explicit operator bool() const { return available_; }

    ALWAYS_INLINE DataType& operator*() { return data_; }
    ALWAYS_INLINE const DataType& operator*() const { return data_; }

    ALWAYS_INLINE DataType* operator->() { return &data_; }
    ALWAYS_INLINE const DataType* operator->() const { return &data_; }

private:
    DataType data_;
    bool available_;
};

} // namespace data