#pragma once

#include <cstdint>

namespace data::helper {

template <uint16_t index_, uint8_t sub_index_, typename DataType_>
struct DataStruct {
    static constexpr uint16_t index = index_;
    static constexpr uint8_t sub_index = sub_index_;

    using DataType = DataType_;

    static constexpr bool writable = false;
};

} // namespace data::helper