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
};

} // namespace client::protocol