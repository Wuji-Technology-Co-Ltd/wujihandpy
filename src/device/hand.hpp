#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>

#include "data/hand.hpp"
#include "device/data_operator.hpp"
#include "device/data_tuple.hpp"
#include "protocol/handler.hpp"

namespace device {

class Joint : public DataOperator<Joint> {
    friend class DataOperator;

public:
    Joint(protocol::Handler& handler, uint16_t index_offset, int storage_offset)
        : handler_(handler)
        , index_offset_(index_offset)
        , storage_offset_(storage_offset) {}

    using Datas = DataTuple<data::hand::finger::joint::Position>;

    protocol::Handler& handler_;
    uint16_t index_offset_;
    int storage_offset_;
};

class Finger : public DataOperator<Finger> {
    friend class DataOperator;
    friend class Hand;

public:
    Joint joint(int index) { return sub(index); }

    // private:
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

class Hand
    : public DataOperator<Hand>
    , private protocol::Handler {
    friend class DataOperator;

public:
    Hand(uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count = 64)
        : Handler(usb_vid, usb_pid, buffer_transfer_count, 200) {};

    Finger finger(int index) { return sub(index); }

    // private:
    using Datas = DataTuple<
        data::hand::FirmwareVersion, data::hand::FirmwareDate, data::hand::SystemTime,
        data::hand::McuTemperature, data::hand::InputVoltage>;

    protocol::Handler& handler_ = *this;
    static constexpr uint16_t index_offset_ = 0x0000;
    static constexpr int storage_offset_ = 0;

    using Sub = Finger;
    static constexpr int sub_count_ = 5;
    Sub sub(int index) {
        return {
            *this, uint16_t(index_offset_ + 0x2000 + index * 0x800),
            int(storage_offset_ + Datas::count + index * Sub::data_count())};
    }

    static int index_to_unique_id(uint16_t index, uint8_t sub_index) {
        return Datas::match_index(index, sub_index);
    }
}; // namespace device

} // namespace device