#include <bit>
#include <cassert>
#include <cstdint>

#include <format>
#include <iostream>
#include <thread>

#include "driver/usb_bulk.hpp"
#include "utility/cross_os.hpp"
#include "utility/endian_promise.hpp"

class Hand : driver::UsbBulk {
public:
    explicit Hand(uint16_t usb_vid, int32_t usb_pid)
        : UsbBulk(usb_vid, usb_pid) {}

    using UsbBulk::usb_transmit;

    using UsbBulk::handle_events;
    using UsbBulk::stop_handling_events;

private:
    void usb_receive_callback(const std::byte* buffer, int length) override {
        std::cout << std::format("Received {} bytes: ", length);
        for (int i = 0; i < length; ++i) {
            std::cout << std::format("{:02x} ", static_cast<unsigned char>(buffer[i]));
        }
        std::cout << '\n';
    }
};

constexpr size_t calculate_compressed_frame_length(size_t payload_size) {
    return (8 + payload_size - 1) / 16 + 1;
}

constexpr size_t calculate_padding_size(size_t payload_size) {
    return 16 * calculate_compressed_frame_length(payload_size) - (8 + payload_size);
}

constexpr int16_t make_control_field(uint8_t frame_length, uint16_t max_receive_window) {
    struct {
        uint16_t max_receive_window : 10;
        uint16_t frame_length       : 6;
    } control{.max_receive_window = max_receive_window, .frame_length = frame_length};
    return std::bit_cast<int16_t>(control);
}

template <typename T>
PACKED_STRUCT(PaddedStruct)
    : public T {
    uint8_t padding[calculate_padding_size(sizeof(T))];
};

template <typename T>
requires(calculate_padding_size(sizeof(T)) == 0) PACKED_STRUCT(PaddedStruct<T>)
    : public T{};

PACKED_STRUCT(Header) {
    uint16_t header = 0x55aa;

    uint8_t source = 0x00;
    uint8_t destination = 0xa0;

    utility::be_uint16_t control;
};

PACKED_STRUCT(Payload) {
    uint8_t protocol = 0x21;
    uint8_t prefix = 0x00;

    uint8_t control = 0x30;
    utility::be_uint16_t index = 0x520a;
    uint8_t sub_index = 0x01;
};

PACKED_STRUCT(Frame) {
    Header header{.control = make_control_field(sizeof(Frame) / 16 - 1, 0xa0)};

    PaddedStruct<Payload> payload{};

    uint16_t crc = 0;
};

int main() {
    Hand hand{0x0483, 0x5740};

    Frame frame;

    const auto data = reinterpret_cast<std::byte*>(&frame);
    for (size_t i = 0; i < sizeof(frame); ++i) {
        std::cout << std::format("{:02x} ", static_cast<unsigned char>(data[i]));
    }
    std::cout << '\n';

    std::jthread thread{[&hand]() { hand.handle_events(); }};

    using namespace std::chrono_literals;
    while (true) {
        std::cout << hand.usb_transmit(reinterpret_cast<std::byte*>(&frame), sizeof(frame))
                  << std::endl;
        std::this_thread::sleep_for(500ms);
    }
}