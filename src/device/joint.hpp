#pragma once

#include <cstdint>

#include "data/hand.hpp"
#include "device/data_operator.hpp"
#include "device/data_tuple.hpp"
#include "protocol/handler.hpp"

namespace device {

class Joint : public DataOperator<Joint> {
    friend class DataOperator;
    friend class Finger;

private:
    Joint(protocol::Handler& handler, uint16_t index_offset, int storage_offset)
        : handler_(handler)
        , index_offset_(index_offset)
        , storage_offset_(storage_offset) {}

    using Datas = DataTuple<
        data::hand::finger::joint::ControlWord, data::hand::finger::joint::Position,
        data::hand::finger::joint::ControlPosition>;

    protocol::Handler& handler_;
    uint16_t index_offset_;
    int storage_offset_;

    constexpr static int index_to_storage_id(uint16_t index, uint8_t sub_index) {
        return Datas::match_index(index, sub_index);
    }
};

} // namespace device