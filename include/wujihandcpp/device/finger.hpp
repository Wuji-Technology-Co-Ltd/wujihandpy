#pragma once

#include <cstdint>

#include <stdexcept>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/device/joint.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {
namespace device {

class Finger : public DataOperator<Finger> {
    friend class DataOperator;
    friend class Hand;

public:
    Joint joint(int index) {
        if (index < 0 || index >= sub_count_)
            throw std::runtime_error("Index out of bounds! Possible values: 0, 1, 2, 3.");
        return sub(index);
    }

private:
    Finger(protocol::Handler& handler, uint16_t index_offset, int storage_offset)
        : handler_(handler)
        , index_offset_(index_offset)
        , storage_offset_(storage_offset) {}

    using Datas = DataTuple<>;

    protocol::Handler& handler_;
    uint16_t index_offset_;
    int storage_offset_;

    using Sub = Joint;
    static constexpr int sub_count_ = 4;
    Sub sub(int index) {
        return {
            handler_, uint16_t(index_offset_ + index * 0x100),
            int(storage_offset_ + Datas::count + index * Sub::data_count())};
    }
};

} // namespace device
} // namespace wujihandcpp