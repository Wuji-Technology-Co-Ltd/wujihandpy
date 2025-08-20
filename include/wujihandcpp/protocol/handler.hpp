#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>
#include <format>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <libusb.h>

#include "../driver/async_transmit_buffer.hpp"
#include "../driver/driver.hpp"
#include "../protocol/protocol.hpp"
#include "../protocol/storage.hpp"

namespace protocol {

class Handler : driver::Driver<Handler> {
    friend class Driver<Handler>;
    friend class AsyncTransmitBuffer<protocol::Header>;

public:
    explicit Handler(
        uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count, size_t storage_unit_count,
        int (*index_to_storage_id)(uint16_t, uint8_t))
        : Driver(usb_vid, usb_pid)
        , default_transmit_buffer_(*this, buffer_transfer_count)
        , event_thread_([this]() { handle_events(); })
        , sync_read_thread_id_(std::this_thread::get_id())
        , storage_(std::make_unique<StorageUnit[]>(storage_unit_count))
        , index_to_storage_id_(index_to_storage_id) {}

    virtual ~Handler() { stop_handling_events(); };

    OperationToken data_operation_token(int storage_id) {
        return storage_[storage_id].data_token();
    }

    void read_data_async(uint16_t index, uint8_t sub_index) {
        std::byte* buffer = fetch_sdo_buffer(sizeof(protocol::sdo::Read));
        new (buffer) protocol::sdo::Read{
            .index = index,
            .sub_index = sub_index,
        };
    }

    template <protocol::is_type_erased_integral T>
    void write_data_async(uint16_t index, uint8_t sub_index, T data) {
        std::byte* buffer = fetch_sdo_buffer(sizeof(protocol::sdo::Write<T>));
        new (buffer) protocol::sdo::Write<T>{
            .index = index,
            .sub_index = sub_index,
            .data = data,
        };
    }

    void write_pdo_async(const int32_t (&control_positions)[5][4], uint32_t timestamp) {
        std::byte* buffer = fetch_pdo_buffer();
        auto payload = new (buffer) protocol::pdo::Write{};

        static_assert(sizeof(payload->control_positions) == sizeof(control_positions));
        payload->pdo_id = 0x100;
        std::memcpy(&payload->control_positions, &control_positions, sizeof(control_positions));
        payload->timestamp = timestamp;

        trigger_transmission();
    }

    bool trigger_transmission() { return default_transmit_buffer_.trigger_transmission(); }

    OperationToken wait_data_operation(int storage_id, OperationToken old_token) {
        return storage_[storage_id].wait(old_token);
    }

    template <protocol::is_type_erased_integral T>
    T data(int storage_id) {
        return storage_[storage_id].read<T>();
    }

private:
    std::byte* fetch_sdo_buffer(int size) {
        auto buffer = default_transmit_buffer_.try_fetch_buffer(
            [size](int free_size, libusb_transfer* transfer) {
                if (free_size < size + int(sizeof(protocol::CrcCheck)))
                    return false;

                auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
                if (header.type == 0)
                    header.type = 0x21;
                else if (header.type != 0x21)
                    return false;

                return true;
            },
            [size](int) { return size; });
        if (!buffer)
            throw std::runtime_error{"No buffer available!"};

        return buffer;
    }

    std::byte* fetch_pdo_buffer() {
        auto buffer = default_transmit_buffer_.try_fetch_buffer(
            [](int free_size, libusb_transfer* transfer) {
                if (free_size < int(sizeof(protocol::pdo::Write) + sizeof(protocol::CrcCheck)))
                    return false;

                auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
                if (header.type == 0)
                    header.type = 0x11;
                else if (header.type != 0x11)
                    return false;

                return true;
            },
            [](int) { return sizeof(protocol::pdo::Write); });
        if (!buffer)
            throw std::runtime_error{"No buffer available!"};

        return buffer;
    }

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
        // std::cout << "Transmitted: ";
        // for (int i = 0; i < transfer->actual_length; i++)
        //     std::cout << std::format("{:02X} ", transfer->buffer[i]);
        // std::cout << '\n';
        auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
        header.type = 0;
    }

    void receive_transfer_completed_callback(libusb_transfer* transfer) {
        // std::cout << "Received:    ";
        // for (int i = 0; i < transfer->actual_length; i++)
        //     std::cout << std::format("{:02X} ", transfer->buffer[i]);
        // std::cout << '\n';

        auto pointer = reinterpret_cast<std::byte*>(transfer->buffer);
        const auto sentinel = pointer + transfer->actual_length;

        auto& header = *reinterpret_cast<protocol::Header*>(pointer);
        pointer += sizeof(protocol::Header);

        if (header.type == 0x21)
            read_sdo_frame(pointer, sentinel);
    }

    void read_sdo_frame(std::byte*& pointer, const std::byte* sentinel) {
        while (pointer < sentinel) {
            auto control = static_cast<uint8_t>(*pointer);
            if (control == 0x35)
                read_sdo_data_success<uint8_t>(pointer);
            else if (control == 0x37)
                read_sdo_data_success<uint16_t>(pointer);
            else if (control == 0x39)
                read_sdo_data_success<uint32_t>(pointer);
            else if (control == 0x3D)
                read_sdo_data_success<uint64_t>(pointer);
            else if (control == 0x33)
                read_sdo_data_error(pointer);
            else if (control == 0x21)
                read_sdo_data_write_success(pointer);
            else if (control == 0x23)
                read_sdo_data_write_error(pointer);
            else
                break;
        }
    }

    template <typename T>
    void read_sdo_data_success(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::ReadResultSuccess<T>*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};

        storage_[storage_id].update(data.data);
    }

    void read_sdo_data_error(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::ReadResultError*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};

        storage_[storage_id].update(false, data.err_code);
    }

    void read_sdo_data_write_success(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::WriteResultSuccess*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};

        storage_[storage_id].update(false, 0);
    }

    void read_sdo_data_write_error(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::WriteResultError*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};

        storage_[storage_id].update(false, data.err_code);
    }

    AsyncTransmitBuffer<protocol::Header> default_transmit_buffer_;
    std::jthread event_thread_;

    const std::thread::id sync_read_thread_id_;

    std::unique_ptr<StorageUnit[]> storage_;
    int (*index_to_storage_id_)(uint16_t, uint8_t);
};

} // namespace protocol