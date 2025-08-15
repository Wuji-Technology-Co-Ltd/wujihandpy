#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>
#include <format>
#include <functional>
#include <stdexcept>
#include <thread>

#include <libusb.h>

#include "data/helper.hpp"
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
        , default_transmit_buffer_(*this, buffer_transfer_count)
        , sync_read_thread_id_(std::this_thread::get_id()) {}

    virtual ~Hand() = default;

    using Driver::usb_transmit;

    using Driver::handle_events;
    using Driver::stop_handling_events;

    template <typename T>
    T::DataType read_data() {
        if (std::this_thread::get_id() != sync_read_thread_id_) [[unlikely]]
            throw std::runtime_error(
                std::format(
                    "Thread safety violation: \n"
                    "  Synchronized methods must be called exclusively "
                    "from the original construction thread. \n"
                    "  (thread ID: {})",
                    std::hash<std::thread::id>{}(sync_read_thread_id_)));

        auto flag = std::bit_cast<uint32_t>(protocol::sdo::Read{
            .control = T::read_result_control,
            .index = T::index,
            .sub_index = T::sub_index,
        });
        sync_read_flag_.store(flag, std::memory_order::release);
        read_data_async<T>();
        trigger_transmission();
        sync_read_flag_.wait(flag);

        return sync_read_data_.load(std::memory_order::acquire).as<typename T::DataType>();
    }

    void set_data_read_complete_callback(auto&& callable) {
        data_read_complete_callback_ = callable;
    }

    template <typename T>
    bool read_data_async() {
        std::byte* buffer = default_transmit_buffer_.try_fetch_buffer(
            [](int free_size, libusb_transfer* transfer) {
                if (free_size < (int)(sizeof(protocol::sdo::Read) + sizeof(protocol::CrcCheck)))
                    return false;

                auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
                if (header.type == 0)
                    header.type = 0x21;
                else if (header.type != 0x21)
                    return false;

                return true;
            },
            [](int) { return sizeof(protocol::sdo::Read); });
        if (!buffer)
            return false;

        new (buffer) protocol::sdo::Read{
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
    }

    void receive_transfer_completed_callback(libusb_transfer* transfer) {
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
                read_sdo_data<uint8_t>(pointer);
            else if (control == 0x37)
                read_sdo_data<uint16_t>(pointer);
            else if (control == 0x39)
                read_sdo_data<uint32_t>(pointer);
            else if (control == 0x3D)
                read_sdo_data<uint64_t>(pointer);
            else
                break;
        }
    }

    template <typename T>
    void read_sdo_data(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::ReadResult<T>*>(pointer);
        pointer += sizeof(data);
        if (sync_read_flag_.load(std::memory_order::acquire)
            == std::bit_cast<uint32_t>(data.header)) {
            sync_read_data_.store(SyncReadData{data.data}, std::memory_order::release);
            sync_read_flag_.store(0, std::memory_order::release);
            sync_read_flag_.notify_one();
        }
    }

    AsyncTransmitBuffer<protocol::Header> default_transmit_buffer_;

    const std::thread::id sync_read_thread_id_;
    std::atomic<uint32_t> sync_read_flag_ = 0;
    static_assert(std::atomic<uint32_t>::is_always_lock_free);

    union SyncReadData {
        constexpr explicit SyncReadData() = default;
        constexpr explicit SyncReadData(const uint8_t& data)
            : data8(data) {}
        constexpr explicit SyncReadData(const uint16_t& data)
            : data16(data) {}
        constexpr explicit SyncReadData(const uint32_t& data)
            : data32(data) {}
        constexpr explicit SyncReadData(const uint64_t& data)
            : data64(data) {}

        template <typename T>
        constexpr auto as() {
            if constexpr (sizeof(T) == sizeof(uint8_t))
                return std::bit_cast<T>(data8);
            else if constexpr (sizeof(T) == sizeof(uint16_t))
                return std::bit_cast<T>(data16);
            else if constexpr (sizeof(T) == sizeof(uint32_t))
                return std::bit_cast<T>(data32);
            else if constexpr (sizeof(T) == sizeof(uint64_t))
                return std::bit_cast<T>(data64);
        };

        uint8_t data8;
        uint16_t data16;
        uint32_t data32;
        uint64_t data64;
    };
    std::atomic<SyncReadData> sync_read_data_;
    static_assert(std::atomic<SyncReadData>::is_always_lock_free);

    std::function<void(const data::Data&)> data_read_complete_callback_;
};

} // namespace device