#pragma once

#include <cstdint>

#include <cstring>
#include <format>
#include <iostream>

#include <libusb.h>

#include "device/protocol.hpp"
#include "driver/async_transmit_buffer.hpp"
#include "driver/driver.hpp"

namespace device {
class Hand : driver::Driver<Hand> {
    friend class Driver<Hand>;
    friend class AsyncTransmitBuffer<protocol::Header>;

public:
    explicit Hand(uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count = 32)
        : Driver(usb_vid, usb_pid)
        , default_transmit_buffer_(*this, buffer_transfer_count) {}

    using Driver::usb_transmit;

    using Driver::handle_events;
    using Driver::stop_handling_events;

    template <typename T>
    auto read_data() {}

    template <typename T>
    bool read_data_async() {
        std::byte* buffer = default_transmit_buffer_.try_fetch_buffer(
            [](int free_size, libusb_transfer* transfer) {
                if (free_size < (int)(sizeof(protocol::SdoPayload) + sizeof(protocol::CrcCheck)))
                    return false;

                auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
                if (header.type == 0)
                    header.type = 0x21;
                else if (header.type != 0x21)
                    return false;

                return true;
            },
            [](int) { return sizeof(protocol::SdoPayload); });
        if (!buffer)
            return false;

        new (buffer) protocol::SdoPayload{
            .control = 0x30,
            .index = T::index,
            .sub_index = T::sub_index,
        };
        return true;
    }

    bool trigger_transmission() { return default_transmit_buffer_.trigger_transmission(); }

private:
    static void before_submitting_transmit_transfer(libusb_transfer* transfer) {
        auto compressed_frame_length = static_cast<uint16_t>(
            (transfer->length + (int)sizeof(protocol::CrcCheck) - 1) / 16 + 1);
        auto padded_length = 16 * compressed_frame_length;
        std::memset(&transfer->buffer[transfer->length], 0, padded_length - transfer->length);
        transfer->length = padded_length;

        auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
        struct {
            uint16_t max_receive_window : 10;
            uint16_t frame_length       : 6;
        } description{
            .max_receive_window = 0xA0, .frame_length = (uint8_t)(compressed_frame_length - 1)};
        header.description = std::bit_cast<int16_t>(description);
    }

    static void transmit_transfer_completed_callback(libusb_transfer* transfer) {
        auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
        header.type = 0;
        std::cout << std::format("Transmitted {} bytes: ", transfer->actual_length);
        // for (int i = 0; i < transfer->length; ++i) {
        //     std::cout << std::format("{:02x} ", static_cast<unsigned char>(transfer->buffer[i]));
        // }
        std::cout << '\n';
    }

    static void receive_transfer_completed_callback(libusb_transfer* transfer) {
        std::cout << std::format("Received {} bytes: ", transfer->actual_length);
        // for (int i = 0; i < transfer->actual_length; ++i) {
        //     std::cout << std::format("{:02x} ", static_cast<unsigned char>(transfer->buffer[i]));
        // }
        std::cout << '\n';
    }

    AsyncTransmitBuffer<protocol::Header> default_transmit_buffer_;
};

} // namespace device