#pragma once

#include <cassert>
#include <cstdint>

#include <format>
#include <iostream>

#include "client/protocol.hpp"
#include "driver/async_transmit_buffer.hpp"
#include "driver/driver.hpp"

namespace client {
class Hand : driver::Driver<Hand> {
    friend class driver::Driver<Hand>;

public:
    explicit Hand(uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count = 32)
        : Driver(usb_vid, usb_pid)
        , transmit_buffer_(*this, buffer_transfer_count) {}

    using Driver::usb_transmit;

    using Driver::handle_events;
    using Driver::stop_handling_events;

    template <typename T>
    auto read_data() {}

private:
    static void usb_receive_callback(const std::byte* buffer, int length) {
        std::cout << std::format("Received {} bytes: ", length);
        for (int i = 0; i < length; ++i) {
            std::cout << std::format("{:02x} ", static_cast<unsigned char>(buffer[i]));
        }
        std::cout << '\n';
    }

    AsyncTransmitBuffer<protocol::Header> transmit_buffer_;
};

} // namespace client