#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <wujihandcpp/protocol/handler.hpp>

#include "driver/async_transmit_buffer.hpp"
#include "driver/driver.hpp"
#include "protocol/protocol.hpp"

namespace wujihandcpp::protocol {

class Handler::Impl : driver::Driver<Impl> {
    friend class Driver<Impl>;
    friend class AsyncTransmitBuffer<protocol::Header>;

public:
    explicit Impl(
        uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count, size_t storage_unit_count,
        int (*index_to_storage_id)(uint16_t, uint8_t))
        : Driver(usb_vid, usb_pid)
        , default_transmit_buffer_(*this, buffer_transfer_count)
        , event_thread_transmit_buffer_(*this, buffer_transfer_count, std::chrono::milliseconds(1))
        , event_thread_([this]() { handle_events(); })
        , operation_thread_id_(std::this_thread::get_id())
        , storage_(std::make_unique<StorageUnit[]>(storage_unit_count))
        , index_to_storage_id_(index_to_storage_id) {}

    ~Impl() { stop_handling_events(); };

    void read_async_unchecked(uint16_t index, uint8_t sub_index, int storage_id) {
        operation_thread_check();

        if (storage_[storage_id].operating_state.load(std::memory_order::relaxed)
            != OperatingState::NONE)
            return;

        read_async_unchecked_internal(default_transmit_buffer_, index, sub_index);
    }

    void read_async(
        uint16_t index, uint8_t sub_index, int storage_id,
        void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context) {
        operation_thread_check();

        if (storage_[storage_id].operating_state.load(std::memory_order::acquire)
            != OperatingState::NONE)
            throw std::runtime_error("Illegal checked read: Data is being operated!");

        storage_[storage_id].callback.store(callback, std::memory_order::relaxed);
        storage_[storage_id].callback_context.store(callback_context, std::memory_order::relaxed);
        storage_[storage_id].operating_state.store(
            OperatingState::READING, std::memory_order::release);

        read_async_unchecked_internal(default_transmit_buffer_, index, sub_index);
    }

    template <size_t data_size>
    void write_async_unchecked(Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id) {
        operation_thread_check();

        if (storage_[storage_id].operating_state.load(std::memory_order::relaxed)
            != OperatingState::NONE)
            return;

        using T = SizeToUIntType<data_size>;
        write_async_unchecked_internal(default_transmit_buffer_, data.as<T>(), index, sub_index);
    }

    template <size_t data_size>
    void write_async(
        Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id,
        void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context) {
        operation_thread_check();

        if (storage_[storage_id].operating_state.load(std::memory_order::acquire)
            != OperatingState::NONE)
            throw std::runtime_error("Illegal checked write: Data is being operated!");

        storage_[storage_id].value.store(data, std::memory_order::relaxed);
        storage_[storage_id].callback.store(callback, std::memory_order::relaxed);
        storage_[storage_id].callback_context.store(callback_context, std::memory_order::relaxed);
        storage_[storage_id].operating_state.store(
            OperatingState::WRITING, std::memory_order::release);

        using T = SizeToUIntType<data_size>;
        write_async_unchecked_internal(default_transmit_buffer_, data.as<T>(), index, sub_index);
    }

    void pdo_write_async_unchecked(const int32_t (&control_positions)[5][4], uint32_t timestamp) {
        operation_thread_check();

        std::byte* buffer = fetch_pdo_buffer(default_transmit_buffer_);
        auto payload = new (buffer) protocol::pdo::Write{};

        static_assert(sizeof(payload->control_positions) == sizeof(control_positions));
        payload->pdo_id = 0x100;
        std::memcpy(&payload->control_positions, &control_positions, sizeof(control_positions));
        payload->timestamp = timestamp;

        trigger_transmission();
    }

    bool trigger_transmission() {
        operation_thread_check();
        return default_transmit_buffer_.trigger_transmission();
    }

    Buffer8 get(int storage_id) {
        return storage_[storage_id].value.load(std::memory_order::relaxed);
    }

    void disable_thread_safe_check() { operation_thread_id_ = std::thread::id{}; }

private:
    enum class OperatingState : uint32_t {
        NONE = 0,

        READING = 2,

        WRITING = 4,
        WRITING_CONFIRMING = 5,
    };
    struct StorageUnit {
        std::atomic<Buffer8> value;
        std::atomic<uint32_t> version = 0;
        std::atomic<OperatingState> operating_state;
        std::atomic<void (*)(Buffer8 context, Buffer8 value)> callback;
        std::atomic<Buffer8> callback_context;
    };

    void operation_thread_check() const {
        if (operation_thread_id_ == std::thread::id{})
            return;
        if (operation_thread_id_ != std::this_thread::get_id()) [[unlikely]]
            throw std::runtime_error(
                "Thread safety violation: \n"
                "  Operation must be called from the construction thread by default. \n"
                "  If you want to perform operations in multiple threads, call:\n"
                "      disable_thread_safe_check();\n"
                "  And use mutex to ensure that ONLY ONE THREAD is operating at the same time.");
    }

    static std::byte*
        fetch_sdo_buffer(AsyncTransmitBuffer<protocol::Header>& transmit_buffer, int size) {
        auto buffer = transmit_buffer.try_fetch_buffer(
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

    static std::byte* fetch_pdo_buffer(AsyncTransmitBuffer<protocol::Header>& transmit_buffer) {
        auto buffer = transmit_buffer.try_fetch_buffer(
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
        // std::cout << std::format(
        //     "[{:.3f}] Tx: ", std::chrono::duration<double, std::milli>(
        //                          std::chrono::steady_clock::now().time_since_epoch())
        //                          .count());
        // for (int i = 0; i < transfer->actual_length; i++)
        //     std::cout << std::format("{:02X} ", transfer->buffer[i]);
        // std::cout << '\n';
        auto& header = *reinterpret_cast<protocol::Header*>(transfer->buffer);
        header.type = 0;
    }

    void receive_transfer_completed_callback(libusb_transfer* transfer) {
        // std::cout << std::format(
        //     "[{:.3f}] Rx: ", std::chrono::duration<double, std::milli>(
        //                          std::chrono::steady_clock::now().time_since_epoch())
        //                          .count());
        // for (int i = 0; i < transfer->actual_length; i++)
        //     std::cout << std::format("{:02X} ", transfer->buffer[i]);
        // std::cout << '\n';

        auto pointer = reinterpret_cast<std::byte*>(transfer->buffer);
        const auto sentinel = pointer + transfer->actual_length;

        auto& header = *reinterpret_cast<protocol::Header*>(pointer);
        pointer += sizeof(protocol::Header);

        if (header.type == 0x21)
            read_sdo_frame(pointer, sentinel);

        event_thread_transmit_buffer_.trigger_transmission();
    }

    void read_sdo_frame(std::byte*& pointer, const std::byte* sentinel) {
        while (pointer < sentinel) {
            auto control = static_cast<uint8_t>(*pointer);
            if (control == 0x35)
                read_sdo_operation_read_success<uint8_t>(pointer);
            else if (control == 0x37)
                read_sdo_operation_read_success<uint16_t>(pointer);
            else if (control == 0x39)
                read_sdo_operation_read_success<uint32_t>(pointer);
            else if (control == 0x3D)
                read_sdo_operation_read_success<uint64_t>(pointer);
            else if (control == 0x33)
                read_sdo_operation_read_failed(pointer);
            else if (control == 0x21)
                read_sdo_operation_write_success(pointer);
            else if (control == 0x23)
                read_sdo_operation_write_failed(pointer);
            else
                break;
        }
    }

    template <typename T>
    void read_sdo_operation_read_success(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::ReadResultSuccess<T>*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0) [[unlikely]]
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};
        auto& storage = storage_[storage_id];

        auto operating_state = storage.operating_state.load(std::memory_order::acquire);
        if (operating_state != OperatingState::NONE) {
            if (operating_state == OperatingState::READING) {
                complete_operation(storage, Buffer8{data.value});
            } else if (operating_state == OperatingState::WRITING_CONFIRMING) {
                if (data.value == storage.value.load(std::memory_order::relaxed).as<T>())
                    complete_operation(storage);
                else {
                    storage.operating_state.store(
                        OperatingState::WRITING, std::memory_order::relaxed);
                    write_async_unchecked_internal(
                        event_thread_transmit_buffer_,
                        storage.value.load(std::memory_order::relaxed).as<T>(), data.header.index,
                        data.header.sub_index);
                }
                return;
            }
        }

        storage.value.store(Buffer8{data.value}, std::memory_order::relaxed);
        auto new_version = storage.version.load(std::memory_order::relaxed) + 1;
        if (new_version == 0)
            new_version = 1;
        storage.version.store(new_version, std::memory_order::release);
    }

    void read_sdo_operation_read_failed(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::ReadResultError*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};
        auto& storage = storage_[storage_id];

        auto operating_state = storage.operating_state.load(std::memory_order::acquire);
        if (operating_state != OperatingState::NONE) {
            if (operating_state == OperatingState::READING
                || operating_state == OperatingState::WRITING_CONFIRMING)
                read_async_unchecked_internal(
                    event_thread_transmit_buffer_, data.header.index, data.header.sub_index);
        }
    }

    void read_sdo_operation_write_success(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::WriteResultSuccess*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};
        auto& storage = storage_[storage_id];

        auto operating_state = storage.operating_state.load(std::memory_order::acquire);
        if (operating_state != OperatingState::NONE) {
            if (operating_state == OperatingState::WRITING)
                complete_operation(storage);
        }
    }

    void read_sdo_operation_write_failed(std::byte*& pointer) {
        const auto& data = *reinterpret_cast<protocol::sdo::WriteResultError*>(pointer);
        pointer += sizeof(data);

        int storage_id = index_to_storage_id_(data.header.index, data.header.sub_index);
        if (storage_id < 0)
            throw std::runtime_error{"Unexpected sdo index & sub-index!"};
        auto& storage = storage_[storage_id];

        auto operating_state = storage.operating_state.load(std::memory_order::acquire);
        if (operating_state != OperatingState::NONE) {
            if (operating_state == OperatingState::WRITING) {
                storage.operating_state.store(
                    OperatingState::WRITING_CONFIRMING, std::memory_order::relaxed);
                read_async_unchecked_internal(
                    event_thread_transmit_buffer_, data.header.index, data.header.sub_index);
            }
        }
    }

    static void read_async_unchecked_internal(
        AsyncTransmitBuffer<protocol::Header>& transmit_buffer, uint16_t index, uint8_t sub_index) {
        std::byte* buffer = fetch_sdo_buffer(transmit_buffer, sizeof(protocol::sdo::Read));
        new (buffer) protocol::sdo::Read{
            .index = index,
            .sub_index = sub_index,
        };
    }

    template <protocol::is_type_erased_integral T>
    void write_async_unchecked_internal(
        AsyncTransmitBuffer<protocol::Header>& transmit_buffer, T value, uint16_t index,
        uint8_t sub_index) {

        std::byte* buffer = fetch_sdo_buffer(transmit_buffer, sizeof(protocol::sdo::Write<T>));
        new (buffer) protocol::sdo::Write<T>{
            .index = index,
            .sub_index = sub_index,
            .value = value,
        };
    }

    static void complete_operation(StorageUnit& storage, Buffer8 value = {}) {
        storage.operating_state.store(OperatingState::NONE, std::memory_order::release);
        if (auto callback = storage.callback.load(std::memory_order::relaxed))
            callback(storage.callback_context.load(std::memory_order::relaxed), value);
    }

    template <size_t size>
    using SizeToUIntType = std::conditional_t<
        size == 1, uint8_t,
        std::conditional_t<
            size == 2, uint16_t,
            std::conditional_t<
                size == 4, uint32_t, std::conditional_t<size == 8, uint64_t, void>>>>;

    AsyncTransmitBuffer<protocol::Header> default_transmit_buffer_;
    AsyncTransmitBuffer<protocol::Header> event_thread_transmit_buffer_;
    std::jthread event_thread_;

    std::thread::id operation_thread_id_;

    std::unique_ptr<StorageUnit[]> storage_;
    int (*index_to_storage_id_)(uint16_t, uint8_t);
};

API Handler::Handler(
    uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count, size_t storage_unit_count,
    int (*index_to_storage_id)(uint16_t, uint8_t)) {
    static_assert(impl_align == alignof(Handler::Impl));
    static_assert(sizeof(impl_) == sizeof(Handler::Impl));
    new (impl_)
        Impl{usb_vid, usb_pid, buffer_transfer_count, storage_unit_count, index_to_storage_id};
}

API Handler::~Handler() { std::destroy_at(reinterpret_cast<Impl*>(impl_)); }

API void Handler::read_async_unchecked(uint16_t index, uint8_t sub_index, int storage_id) {
    reinterpret_cast<Impl*>(impl_)->read_async_unchecked(index, sub_index, storage_id);
}

API void Handler::read_async(
    uint16_t index, uint8_t sub_index, int storage_id,
    void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context) {
    reinterpret_cast<Impl*>(impl_)->read_async(
        index, sub_index, storage_id, callback, callback_context);
}

template <size_t data_size>
void Handler::write_async_unchecked(
    Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id) {
    reinterpret_cast<Impl*>(impl_)->template write_async_unchecked<data_size>(
        data, index, sub_index, storage_id);
}

template API void Handler::write_async_unchecked<1>(Buffer8, uint16_t, uint8_t, int);
template API void Handler::write_async_unchecked<2>(Buffer8, uint16_t, uint8_t, int);
template API void Handler::write_async_unchecked<4>(Buffer8, uint16_t, uint8_t, int);
template API void Handler::write_async_unchecked<8>(Buffer8, uint16_t, uint8_t, int);

template <size_t data_size>
void Handler::write_async(
    Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id,
    void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context) {
    reinterpret_cast<Impl*>(impl_)->template write_async<data_size>(
        data, index, sub_index, storage_id, callback, callback_context);
}

template API void
    Handler::write_async<1>(Buffer8, uint16_t, uint8_t, int, void (*)(Buffer8, Buffer8), Buffer8);
template API void
    Handler::write_async<2>(Buffer8, uint16_t, uint8_t, int, void (*)(Buffer8, Buffer8), Buffer8);
template API void
    Handler::write_async<4>(Buffer8, uint16_t, uint8_t, int, void (*)(Buffer8, Buffer8), Buffer8);
template API void
    Handler::write_async<8>(Buffer8, uint16_t, uint8_t, int, void (*)(Buffer8, Buffer8), Buffer8);

API void Handler::pdo_write_async_unchecked(
    const int32_t (&control_positions)[5][4], uint32_t timestamp) {
    reinterpret_cast<Impl*>(impl_)->pdo_write_async_unchecked(control_positions, timestamp);
}

API bool Handler::trigger_transmission() {
    return reinterpret_cast<Impl*>(impl_)->trigger_transmission();
}

API Handler::Buffer8 Handler::get(int storage_id) {
    return reinterpret_cast<Impl*>(impl_)->get(storage_id);
}

API void Handler::disable_thread_safe_check() {
    return reinterpret_cast<Impl*>(impl_)->disable_thread_safe_check();
}

} // namespace wujihandcpp::protocol