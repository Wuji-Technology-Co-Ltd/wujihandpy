#pragma once

#include <cstdint>
#include <type_traits>

#include "utility/cross_os.hpp"
#include "utility/endian_promise.hpp"

namespace device::protocol {

PACKED_STRUCT(Header {
    uint16_t header = 0x55aa;

    uint8_t source = 0x00;
    uint8_t destination = 0xa0;

    utility::be_uint16_t description;

    uint8_t type = 0;
    uint8_t prefix = 0x00;
});

namespace sdo {

PACKED_STRUCT(Read {
    uint8_t control;
    uint16_t index;
    uint8_t sub_index;
});

template <typename T>
requires(std::is_integral_v<T> && std::is_unsigned_v<T>) PACKED_STRUCT(ReadResult {
    PACKED_STRUCT({
        uint8_t control;
        uint16_t index;
        uint8_t sub_index;
    })
    header;
    T data;
});

} // namespace sdo

PACKED_STRUCT(CrcCheck { uint16_t value; });

} // namespace device::protocol