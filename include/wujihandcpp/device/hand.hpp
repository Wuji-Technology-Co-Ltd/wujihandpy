#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "../data/hand.hpp"
#include "../device/data_operator.hpp"
#include "../device/data_tuple.hpp"
#include "../device/finger.hpp"
#include "../protocol/handler.hpp"

namespace device {

class Hand : public DataOperator<Hand> {
    friend class DataOperator;

public:
    Hand(uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count = 64)
        : handler_(usb_vid, usb_pid, buffer_transfer_count, data_count(), index_to_storage_id) {};

    Finger finger(int index) { return sub(index); }

private:
    using Datas = DataTuple<
        data::hand::FirmwareVersion, data::hand::FirmwareDate, data::hand::SystemTime,
        data::hand::McuTemperature, data::hand::InputVoltage>;

    protocol::Handler handler_;

    static constexpr uint16_t index_offset_ = 0x0000;
    static constexpr int storage_offset_ = 0;

    using Sub = Finger;
    static constexpr int sub_count_ = 5;
    Sub sub(int index) {
        return {
            handler_, uint16_t(0x2000 + index * 0x800),
            int(Datas::count + index * Sub::data_count())};
    }

    constexpr static int index_to_storage_id(uint16_t index, uint8_t sub_index) {
        if (index >= 0x5000)
            return Datas::match_index(index, sub_index);
        else if (index >= 0x4000)
            return Datas::count + 4 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x4000, sub_index);
        else if (index >= 0x3800)
            return Datas::count + 3 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x3800, sub_index);
        else if (index >= 0x3000)
            return Datas::count + 2 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x3000, sub_index);
        else if (index >= 0x2800)
            return Datas::count + 1 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x2800, sub_index);
        else if (index >= 0x2000)
            return Datas::count + 0 * Sub::data_count()
                 + Sub::index_to_storage_id(index - 0x2000, sub_index);
        else
            return std::numeric_limits<int>::min();
    }
};

} // namespace device