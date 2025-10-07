#pragma once

#include <cstdint>
#include <type_traits>

#include "utility/cross_os.hpp"
#include "utility/endian_promise.hpp"

namespace wujihandcpp::protocol {

PACKED_STRUCT(Header {
    uint16_t header = 0x55aa;

    uint8_t source = 0x00;
    uint8_t destination = 0xa0;

    utility::be_uint16_t description;

    uint8_t type = 0;
    uint8_t prefix = 0x00;
});

template <typename T>
concept is_type_erased_integral = std::is_integral_v<T> && std::is_unsigned_v<T>;

namespace sdo {

PACKED_STRUCT(Read {
    uint8_t control = 0x30;
    utility::be_uint16_t index;
    uint8_t sub_index;
});

template <is_type_erased_integral T>
PACKED_STRUCT(ReadResultSuccess {
    PACKED_STRUCT({
        uint8_t control;
        utility::be_uint16_t index;
        uint8_t sub_index;
    })
    header;
    T value;
});

PACKED_STRUCT(ReadResultError {
    PACKED_STRUCT({
        uint8_t control;
        utility::be_uint16_t index;
        uint8_t sub_index;
    })
    header;
    uint32_t err_code;
});

template <is_type_erased_integral T>
PACKED_STRUCT(Write {
    uint8_t control = []() constexpr {
        if constexpr (sizeof(T) == 1)
            return uint8_t(0x20);
        else if constexpr (sizeof(T) == 2)
            return uint8_t(0x22);
        else if constexpr (sizeof(T) == 4)
            return uint8_t(0x24);
        else if constexpr (sizeof(T) == 8)
            return uint8_t(0x28);
    }();
    utility::be_uint16_t index;
    uint8_t sub_index;
    T value;
});

PACKED_STRUCT(WriteResultSuccess {
    PACKED_STRUCT({
        uint8_t control;
        utility::be_uint16_t index;
        uint8_t sub_index;
    })
    header;
});

PACKED_STRUCT(WriteResultError {
    PACKED_STRUCT({
        uint8_t control;
        utility::be_uint16_t index;
        uint8_t sub_index;
    })
    header;
    uint32_t err_code;
});

} // namespace sdo

namespace pdo {

PACKED_STRUCT(Read { uint16_t pdo_id = 0x0100; });

PACKED_STRUCT(ReadResult {
    uint16_t pdo_id;
    int32_t positions[5][4];
});

PACKED_STRUCT(Write {
    uint16_t pdo_id = 0x0001;
    int32_t target_positions[5][4];
    uint32_t timestamp;
});

}; // namespace pdo

PACKED_STRUCT(CrcCheck { uint16_t value; });

} // namespace wujihandcpp::protocol