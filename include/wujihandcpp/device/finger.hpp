#pragma once

#include <cstdint>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/device/joint.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp::device {

class Finger : public DataOperator<Finger> {
    friend class DataOperator;
    friend class Hand;

public:
    Joint joint(int index) { return sub(index); }

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

    constexpr static int index_to_storage_id(uint16_t index, uint8_t sub_index) {
        if (index >= 0x300)
            return Datas::count + 3 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x300, sub_index);
        else if (index >= 0x200)
            return Datas::count + 2 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x200, sub_index);
        else if (index >= 0x100)
            return Datas::count + 1 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x100, sub_index);
        else
            return Datas::count + 0 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x000, sub_index);
    }
};

} // namespace wujihandcpp::device