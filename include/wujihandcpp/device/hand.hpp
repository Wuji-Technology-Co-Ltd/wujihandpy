#pragma once

#include <cstddef>
#include <cstdint>

#include <stdexcept>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/device/finger.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {
namespace device {

class Hand : public DataOperator<Hand> {
    friend class DataOperator;

public:
    explicit Hand(
        const char* serial_number = nullptr, int32_t usb_pid = -1, uint16_t usb_vid = 0x0483,
        uint32_t mask = 0)
        : handler_(usb_vid, usb_pid, serial_number, 64, data_count()) {
        init_storage_info(mask);
        write<data::joint::CurrentLimit>(1000);
    };

    Finger finger_thumb() { return finger(0); }
    Finger finger_index() { return finger(1); }
    Finger finger_middle() { return finger(2); }
    Finger finger_ring() { return finger(3); }
    Finger finger_little() { return finger(4); }

    Finger finger(int index) {
        if (index < 0 || index >= sub_count_)
            throw std::runtime_error("Index out of bounds! Possible values: 0, 1, 2, 3, 4.");
        return sub(index);
    }

    void pdo_write_async_unchecked(const double (&control_positions)[5][4], uint32_t timestamp) {
        handler_.pdo_write_async_unchecked(control_positions, timestamp);
    }

    void disable_thread_safe_check() { handler_.disable_thread_safe_check(); }

private:
    using Datas = DataTuple<
        data::hand::Handedness, data::hand::FirmwareVersion, data::hand::FirmwareDate,
        data::hand::SystemTime, data::hand::Temperature, data::hand::InputVoltage,
        data::hand::PdoEnabled, data::hand::GlobalTpdoId, data::hand::JointPdoInterval>;

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
};

} // namespace device
} // namespace wujihandcpp