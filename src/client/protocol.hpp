#pragma once

#include <cstdint>

#include "utility/cross_os.hpp"
#include "utility/endian_promise.hpp"

namespace client::protocol {

PACKED_STRUCT(Header) {
    uint16_t header = 0x55aa;

    uint8_t source = 0x00;
    uint8_t destination = 0xa0;

    utility::be_uint16_t description;

    uint8_t type = 0;
    uint8_t prefix = 0x00;
};

PACKED_STRUCT(SdoPayload) {
    uint8_t control;
    utility::be_uint16_t index;
    uint8_t sub_index;
};

PACKED_STRUCT(CrcCheck) { uint16_t value; };

} // namespace client::protocol