#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <format>
#include <map>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <spdlog/fmt/bin_to_hex.h>
#include <wujihandcpp/protocol/handler.hpp>
#include <wujihandcpp/utility/api.hpp>

#include "logging/logging.hpp"
#include "protocol/protocol.hpp"
#include "transport/transport.hpp"

namespace wujihandcpp::protocol {

class Handler::Impl {
public:
    explicit Impl(std::unique_ptr<transport::ITransport> transport, size_t storage_unit_count)
        : logger_(logging::get_logger())
        , operation_thread_id_(std::this_thread::get_id())
        , storage_unit_count_(storage_unit_count)
        , storage_(std::make_unique<StorageUnit[]>(storage_unit_count))
        , transport_(std::move(transport))
        , sdo_helper_(*transport_, 0x21)
        , pdo_helper_(*transport_, 0x11)
        , sdo_thread_([this](const std::stop_token& stop_token) { sdo_thread_main(stop_token); }) {

        transport_->receive([this](const std::byte* buffer, size_t size) {
            receive_transfer_completed_callback(buffer, size);
        });
    }

    ~Impl() = default;

    void init_storage_info(int storage_id, StorageInfo info) {
        storage_[storage_id].info = info;
        IndexMapKey index{.index = info.index, .sub_index = info.sub_index};
        index_storage_map_[std::bit_cast<uint32_t>(index)] = &storage_[storage_id];
    }

    void read_async_unchecked(int storage_id, std::chrono::steady_clock::duration::rep timeout) {
        operation_thread_check();

        if (storage_[storage_id].operation.load(std::memory_order::relaxed).mode
            != Operation::Mode::NONE)
            return;

        storage_[storage_id].timeout = std::chrono::steady_clock::duration(timeout);
        storage_[storage_id].callback = nullptr;
        storage_[storage_id].operation.store(
            Operation{.mode = Operation::Mode::READ, .state = Operation::State::WAITING},
            std::memory_order::release);
    }

    void read_async(
        int storage_id, std::chrono::steady_clock::duration::rep timeout,
        void (*callback)(Buffer8 context, bool success), Buffer8 callback_context) {
        operation_thread_check();

        if (storage_[storage_id].operation.load(std::memory_order::relaxed).mode
            != Operation::Mode::NONE) [[unlikely]]
            throw std::runtime_error("Illegal checked read: Data is being operated!");

        storage_[storage_id].timeout = std::chrono::steady_clock::duration(timeout);
        storage_[storage_id].callback = callback;
        storage_[storage_id].callback_context = callback_context;
        storage_[storage_id].operation.store(
            Operation{.mode = Operation::Mode::READ, .state = Operation::State::WAITING},
            std::memory_order::release);
    }

    void write_async_unchecked(
        Buffer8 data, int storage_id, std::chrono::steady_clock::duration::rep timeout) {
        operation_thread_check();

        store_data(storage_[storage_id], data);

        if (storage_[storage_id].operation.load(std::memory_order::relaxed).mode
            != Operation::Mode::NONE)
            return;

        storage_[storage_id].timeout = std::chrono::steady_clock::duration(timeout);
        storage_[storage_id].callback = nullptr;
        storage_[storage_id].operation.store(
            Operation{.mode = Operation::Mode::WRITE, .state = Operation::State::WAITING},
            std::memory_order::release);
    }

    void write_async(
        Buffer8 data, int storage_id, std::chrono::steady_clock::duration::rep timeout,
        void (*callback)(Buffer8 context, bool success), Buffer8 callback_context) {
        operation_thread_check();

        if (storage_[storage_id].operation.load(std::memory_order::relaxed).mode
            != Operation::Mode::NONE) [[unlikely]]
            throw std::runtime_error("Illegal checked write: Data is being operated!");

        store_data(storage_[storage_id], data);
        storage_[storage_id].timeout = std::chrono::steady_clock::duration(timeout);
        storage_[storage_id].callback = callback;
        storage_[storage_id].callback_context = callback_context;
        storage_[storage_id].operation.store(
            Operation{.mode = Operation::Mode::WRITE, .state = Operation::State::WAITING},
            std::memory_order::release);
    }

    void attach_realtime_controller(device::IRealtimeController* controller, bool enable_upstream) {
        std::unique_ptr<device::IRealtimeController> guard(controller);
        if (realtime_controller_)
            throw std::runtime_error("A realtime controller is already attached.");

        realtime_controller_ = std::move(guard);
        pdo_thread_ = std::jthread{[this, enable_upstream](const std::stop_token& stop_token) {
            pdo_thread_main(stop_token, enable_upstream);
        }};
    }

    device::IRealtimeController* detach_realtime_controller() {
        if (!realtime_controller_)
            throw std::runtime_error("No realtime controller attached.");

        pdo_thread_.request_stop();
        pdo_thread_.join();

        return realtime_controller_.release();
    }

    Buffer8 get(int storage_id) { return load_data(storage_[storage_id]); }

    void disable_thread_safe_check() { operation_thread_id_ = std::thread::id{}; }

private:
    struct Operation {
        enum class Mode : uint16_t {
            NONE = 0,

            READ,
            WRITE
        } mode;
        enum class State : uint16_t {
            SUCCESS = 0,

            WAITING,

            READING,

            WRITING,
            WRITING_CONFIRMING,
        } state;
    };
    struct alignas(64) StorageUnit {
        constexpr StorageUnit()
            : version(0) {};

        StorageInfo info;

        std::atomic<Operation> operation =
            Operation{.mode = Operation::Mode::NONE, .state = Operation::State::SUCCESS};
        static_assert(decltype(StorageUnit::operation)::is_always_lock_free);

        std::atomic<uint32_t> version;
        std::atomic<Buffer8> value;
        static_assert(decltype(StorageUnit::version)::is_always_lock_free);
        static_assert(decltype(StorageUnit::value)::is_always_lock_free);

        union {
            std::chrono::steady_clock::duration timeout;
            std::chrono::steady_clock::time_point timeout_point;
            static_assert(std::is_trivially_destructible_v<decltype(timeout)>);
            static_assert(std::is_trivially_destructible_v<decltype(timeout_point)>);
        };

        void (*callback)(Buffer8 context, bool success);
        Buffer8 callback_context;
    };
    static_assert(sizeof(StorageUnit) == 64);

    class TransmitHelper {
    public:
        TransmitHelper(transport::ITransport& transport, uint8_t header_type)
            : logger_(logging::get_logger())
            , transport_(transport)
            , header_type_(header_type) {
            flush();
        };

        std::byte* fetch_buffer(int size) {
            if (current_ + size + sizeof(protocol::CrcCheck) >= end_)
                flush();

            auto current = current_;
            current_ += size;
            return current;
        }

        void flush() {
            if (buffer_) {
                auto begin = buffer_->data();
                auto size = current_ - begin;

                auto compressed_frame_length =
                    static_cast<uint16_t>((size + (int)sizeof(protocol::CrcCheck) - 1) / 16 + 1);
                auto padded_length = 16 * compressed_frame_length;
                std::memset(current_, 0, padded_length - size);

                auto& header = *reinterpret_cast<protocol::Header*>(begin);
                struct {
                    uint16_t max_receive_window : 10;
                    uint16_t frame_length       : 6;
                } description{
                    .max_receive_window = 0xA0,
                    .frame_length = (uint8_t)(compressed_frame_length - 1)};
                header.description = std::bit_cast<int16_t>(description);

                logger_.trace(
                    "TX [{} bytes] {:Xp}", padded_length,
                    spdlog::to_hex(begin, begin + padded_length));

                transport_.transmit(std::move(buffer_), padded_length);
            }

            buffer_ = transport_.request_transmit_buffer();
            if (!buffer_)
                throw std::runtime_error{"No buffer available!"};

            current_ = buffer_->data();
            end_ = current_ + buffer_->size();

            auto& header = *new (current_) protocol::Header{};
            header.type = header_type_;
            current_ += sizeof(header);
        }

        logging::Logger& logger_;

        transport::ITransport& transport_;
        const uint8_t header_type_;

        std::unique_ptr<transport::IBuffer> buffer_ = nullptr;
        std::byte *current_ = nullptr, *end_ = nullptr;
    };

private:
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

    static void store_data(StorageUnit& storage, Buffer8 data) {
        if (storage.info.policy & StorageInfo::CONTROL_WORD) {
            storage.value.store(
                Buffer8{static_cast<uint16_t>(data.as<bool>() ? 1 : 5)},
                std::memory_order::relaxed);
        } else if (storage.info.policy & StorageInfo::POSITION) {
            auto value = to_raw_position(data.as<double>());
            if (storage.info.policy & StorageInfo::POSITION_REVERSED)
                value = -value;
            storage.value.store(Buffer8{value}, std::memory_order::relaxed);
        } else
            storage.value.store(data, std::memory_order::relaxed);
    }

    static Buffer8 load_data(StorageUnit& storage) {
        Buffer8 data = storage.value.load(std::memory_order::relaxed);

        if (storage.info.policy & StorageInfo::CONTROL_WORD) {
            return Buffer8{data.as<uint16_t>() == 1};
        } else if (storage.info.policy & StorageInfo::POSITION) {
            auto value = extract_raw_position(data.as<int32_t>());
            if (storage.info.policy & StorageInfo::POSITION_REVERSED)
                value = -value;
            return Buffer8{value};
        }

        return data;
    }

    static int32_t to_raw_position(double angle) {
        return static_cast<int32_t>(std::round(
            std::clamp<double>(
                angle * (std::numeric_limits<int32_t>::max() / (2 * std::numbers::pi)),
                std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max())));
    }

    static constexpr double extract_raw_position(int32_t angle) {
        return angle * (2 * std::numbers::pi / std::numeric_limits<int32_t>::max());
    }

    void receive_transfer_completed_callback(const std::byte* buffer, size_t size) {
        if (logger_.should_log(logging::Level::TRACE))
            logger_.trace("RX [{} bytes] {:Xp}", size, spdlog::to_hex(buffer, buffer + size));

        auto pointer = buffer;
        auto sentinel = pointer + size;

        try {
            const auto& header = read_frame_struct<protocol::Header>(pointer, sentinel);
            if (header.type == 0x21)
                read_sdo_frame(pointer, sentinel);
            else if (header.type == 0x11)
                read_pdo_frame(pointer, sentinel);
            else
                throw std::runtime_error{std::format("Invalid header type: 0x{:02X}", header.type)};
        } catch (const std::runtime_error& ex) {
            logger_.error("RX Frame parsing failed at offset {}", pointer - buffer);
            logger_.error(ex.what());
            logger_.error(
                "RX Frame dump [{} bytes] {:Xp}", size, spdlog::to_hex(buffer, buffer + size));
        }
    }

    void read_sdo_frame(const std::byte*& pointer, const std::byte* sentinel) {
        while (pointer < sentinel) {
            auto control = static_cast<uint8_t>(*pointer);
            if (control == 0x35)
                read_sdo_operation_read_success<uint8_t>(pointer, sentinel);
            else if (control == 0x37)
                read_sdo_operation_read_success<uint16_t>(pointer, sentinel);
            else if (control == 0x39)
                read_sdo_operation_read_success<uint32_t>(pointer, sentinel);
            else if (control == 0x3D)
                read_sdo_operation_read_success<uint64_t>(pointer, sentinel);
            else if (control == 0x33)
                read_sdo_operation_read_failed(pointer, sentinel);
            else if (control == 0x21)
                read_sdo_operation_write_success(pointer, sentinel);
            else if (control == 0x23)
                read_sdo_operation_write_failed(pointer, sentinel);
            else if (control == 0x00)
                break;
            else
                throw std::runtime_error(
                    std::format("Invalid SDO command specifier: 0x{:02X}", control));
        }
    }

    template <typename T>
    void read_sdo_operation_read_success(const std::byte*& pointer, const std::byte* sentinel) {
        const auto& data =
            read_frame_struct<protocol::sdo::ReadResultSuccess<T>>(pointer, sentinel);

        StorageUnit& storage = find_storage_by_index(data.header.index, data.header.sub_index);

        auto operation = storage.operation.load(std::memory_order::acquire);
        if (operation.mode == Operation::Mode::NONE) [[unlikely]]
            return;

        if (operation.state == Operation::State::READING) {
            storage.value.store(Buffer8{data.value}, std::memory_order::relaxed);
            auto new_version = storage.version.load(std::memory_order::relaxed) + 1;
            if (new_version == 0)
                new_version = 1;
            storage.version.store(new_version, std::memory_order::release);

            operation.state = Operation::State::SUCCESS;
            storage.operation.store(operation, std::memory_order::release);
        } else if (operation.state == Operation::State::WRITING_CONFIRMING) {
            if (data.value == storage.value.load(std::memory_order::relaxed).as<T>()) {
                operation.state = Operation::State::SUCCESS;
                storage.operation.store(operation, std::memory_order::relaxed);
            } else {
                operation.state = Operation::State::WRITING;
                storage.operation.store(operation, std::memory_order::relaxed);
            }
        }
    }

    static void
        read_sdo_operation_read_failed(const std::byte*& pointer, const std::byte* sentinel) {
        read_frame_struct<protocol::sdo::ReadResultError>(pointer, sentinel);
    }

    void read_sdo_operation_write_success(const std::byte*& pointer, const std::byte* sentinel) {
        const auto& data = read_frame_struct<protocol::sdo::WriteResultSuccess>(pointer, sentinel);

        StorageUnit& storage = find_storage_by_index(data.header.index, data.header.sub_index);

        auto operation = storage.operation.load(std::memory_order::acquire);
        if (operation.mode == Operation::Mode::NONE) [[unlikely]]
            return;

        if (operation.state == Operation::State::WRITING) {
            operation.state = Operation::State::SUCCESS;
            storage.operation.store(operation, std::memory_order::relaxed);
        }
    }

    static void
        read_sdo_operation_write_failed(const std::byte*& pointer, const std::byte* sentinel) {
        read_frame_struct<protocol::sdo::WriteResultError>(pointer, sentinel);
    }

    StorageUnit& find_storage_by_index(uint16_t index, uint8_t sub_index) {
        auto it = index_storage_map_.find(
            std::bit_cast<uint32_t>(IndexMapKey{.index = index, .sub_index = sub_index}));
        if (it == index_storage_map_.end())
            throw std::runtime_error{std::format(
                "SDO object not found: index=0x{:04X}, sub-index=0x{:02X}", index, sub_index)};
        return *it->second;
    }

    void sdo_thread_main(const std::stop_token& token) {
        constexpr double update_rate = 199.0;
        constexpr auto update_period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / update_rate));

        while (!token.stop_requested()) {
            auto now = std::chrono::steady_clock::now();

            for (size_t i = 0; i < storage_unit_count_; i++) {
                auto& storage = storage_[i];

                auto operation = storage.operation.load(std::memory_order::acquire);
                if (operation.mode == Operation::Mode::NONE)
                    continue;

                if (storage.info.policy & Handler::StorageInfo::MASKED)
                    operation.state = Operation::State::SUCCESS;
                if (operation.state == Operation::State::SUCCESS) {
                    auto callback = storage.callback;
                    auto context = storage.callback_context;
                    operation.mode = Operation::Mode::NONE;
                    storage.operation.store(operation, std::memory_order::release);
                    if (callback)
                        callback(context, true);
                }

                if (operation.state == Operation::State::WAITING) {
                    if (storage.timeout < std::chrono::steady_clock::duration::zero()
                        || now > std::chrono::steady_clock::time_point::max() - storage.timeout)
                        // Treat negative or overflowed timeout as never expires
                        storage.timeout_point = std::chrono::steady_clock::time_point::max();
                    else
                        storage.timeout_point = now + storage.timeout;

                    operation.state =
                        (operation.mode == Operation::Mode::READ ? Operation::State::READING
                                                                 : Operation::State::WRITING);
                    storage.operation.store(operation, std::memory_order::relaxed);
                } else if (now >= storage.timeout_point) {
                    auto callback = storage.callback;
                    auto context = storage.callback_context;
                    operation.mode = Operation::Mode::NONE;
                    storage.operation.store(operation, std::memory_order::release);
                    if (callback)
                        callback(context, false);
                } else if (
                    operation.state == Operation::State::READING
                    || operation.state == Operation::State::WRITING_CONFIRMING) {
                    read_async_unchecked_internal(storage.info.index, storage.info.sub_index);
                } else if (operation.state == Operation::State::WRITING) {
                    operation.state = Operation::State::WRITING_CONFIRMING;
                    storage.operation.store(operation, std::memory_order::relaxed);
                    if (storage.info.size == StorageInfo::Size::_1)
                        write_async_unchecked_internal(
                            storage.value.load(std::memory_order::relaxed).as<uint8_t>(),
                            storage.info.index, storage.info.sub_index);
                    else if (storage.info.size == StorageInfo::Size::_2)
                        write_async_unchecked_internal(
                            storage.value.load(std::memory_order::relaxed).as<uint16_t>(),
                            storage.info.index, storage.info.sub_index);
                    else if (storage.info.size == StorageInfo::Size::_4)
                        write_async_unchecked_internal(
                            storage.value.load(std::memory_order::relaxed).as<uint32_t>(),
                            storage.info.index, storage.info.sub_index);
                    else if (storage.info.size == StorageInfo::Size::_8)
                        write_async_unchecked_internal(
                            storage.value.load(std::memory_order::relaxed).as<uint64_t>(),
                            storage.info.index, storage.info.sub_index);
                }
            }
            sdo_helper_.flush();

            std::this_thread::sleep_for(update_period);
        }
    }

    void read_pdo_frame(const std::byte*& pointer, const std::byte* sentinel) {
        const auto& data = read_frame_struct<protocol::pdo::CommandResult>(pointer, sentinel);

        if (data.read_executed != 1)
            throw std::runtime_error(
                std::format(
                    "PDO frame invalid: read_executed flag is 0x{:02X}", data.read_executed));

        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                pdo_read_result_[i][j].store(data.positions[i][j], std::memory_order::relaxed);

        pdo_read_result_version_.store(
            pdo_read_result_version_.load(std::memory_order::relaxed) + 1,
            std::memory_order::release);
    }

    void pdo_thread_main(const std::stop_token& token, bool upstream_enabled) {
        constexpr double update_rate = 500.0;
        constexpr auto update_period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / update_rate));

        auto begin = std::chrono::steady_clock::now();
        auto next_iteration_time = begin;

        realtime_controller_->setup(update_rate);
        if (upstream_enabled) {
            const uint64_t old_version = pdo_read_result_version_.load(std::memory_order::relaxed);
            while (!token.stop_requested()) {
                pdo_read_async_unchecked();
                next_iteration_time += update_period;
                std::this_thread::sleep_until(next_iteration_time);
                if (pdo_read_result_version_.load(std::memory_order::acquire) != old_version)
                    break;
            }

            while (!token.stop_requested()) {
                device::IRealtimeController::JointPositions positions;
                for (int i = 0; i < 5; i++)
                    for (int j = 0; j < 4; j++) {
                        auto& value = positions.value[i][j];
                        value = extract_raw_position(
                            pdo_read_result_[i][j].load(std::memory_order::relaxed));
                        if (j == 0 && i != 0)
                            value = -value;
                    }

                auto target_positions = realtime_controller_->step(&positions);

                pdo_write_async_unchecked(
                    true, target_positions.value,
                    static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                              next_iteration_time - begin)
                                              .count()));

                next_iteration_time += update_period;
                std::this_thread::sleep_until(next_iteration_time);
            }
        } else {
            while (!token.stop_requested()) {
                auto target_positions = realtime_controller_->step(nullptr);
                pdo_write_async_unchecked(
                    false, target_positions.value,
                    static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                              next_iteration_time - begin)
                                              .count()));

                next_iteration_time += update_period;
                std::this_thread::sleep_until(next_iteration_time);
            }
        }
    }

    template <typename Struct>
    static const Struct& read_frame_struct(const std::byte*& pointer, const std::byte* sentinel) {
        static_assert(alignof(Struct) == 1);
        const std::size_t required = sizeof(Struct);
        const std::ptrdiff_t remaining = sentinel - pointer;
        if (remaining < static_cast<std::ptrdiff_t>(required)) {
            throw std::runtime_error(
                std::format(
                    "{} truncated: requires {} bytes, but {} remain", typeid(Struct).name(),
                    required, remaining));
        }

        const auto& data = *reinterpret_cast<const Struct*>(pointer);
        pointer += required;
        return data;
    }

    void read_async_unchecked_internal(uint16_t index, uint8_t sub_index) {
        std::byte* buffer = sdo_helper_.fetch_buffer(sizeof(protocol::sdo::Read));
        new (buffer) protocol::sdo::Read{
            .index = index,
            .sub_index = sub_index,
        };
    }

    template <protocol::is_type_erased_integral T>
    void write_async_unchecked_internal(T value, uint16_t index, uint8_t sub_index) {
        std::byte* buffer = sdo_helper_.fetch_buffer(sizeof(protocol::sdo::Write<T>));
        new (buffer) protocol::sdo::Write<T>{
            .index = index,
            .sub_index = sub_index,
            .value = value,
        };
    }

    void pdo_read_async_unchecked() {
        std::byte* buffer = pdo_helper_.fetch_buffer(sizeof(protocol::pdo::Read));
        new (buffer) protocol::pdo::Read{};
        pdo_helper_.flush();
    }
    void pdo_write_async_unchecked(
        bool upstream_enabled, const double (&target_positions)[5][4], uint32_t timestamp) {
        std::byte* buffer = pdo_helper_.fetch_buffer(sizeof(protocol::pdo::Write));
        auto payload = new (buffer) protocol::pdo::Write{};
        payload->enable_read = upstream_enabled;

        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++) {
                payload->target_positions[i][j] = to_raw_position(target_positions[i][j]);
                if (j == 0 && i != 0)
                    payload->target_positions[i][j] = -payload->target_positions[i][j];
            }
        payload->timestamp = timestamp;

        pdo_helper_.flush();
    }

    template <size_t size>
    using SizeToUIntType = std::conditional_t<
        size == 1, uint8_t,
        std::conditional_t<
            size == 2, uint16_t,
            std::conditional_t<
                size == 4, uint32_t, std::conditional_t<size == 8, uint64_t, void>>>>;

    logging::Logger& logger_;

    std::thread::id operation_thread_id_;

    size_t storage_unit_count_;
    std::unique_ptr<StorageUnit[]> storage_;

    struct IndexMapKey {
        uint16_t index;
        uint8_t sub_index;
        const uint8_t padding = 0;
    };
    std::map<uint32_t, StorageUnit*> index_storage_map_;

    std::unique_ptr<transport::ITransport> transport_;
    TransmitHelper sdo_helper_;
    TransmitHelper pdo_helper_;

    std::jthread sdo_thread_;

    std::atomic<int32_t> pdo_read_result_[5][4];
    std::atomic<uint64_t> pdo_read_result_version_ = 0;

    std::unique_ptr<device::IRealtimeController> realtime_controller_;
    std::jthread pdo_thread_;
};

WUJIHANDCPP_API Handler::Handler(
    uint16_t usb_vid, int32_t usb_pid, const char* serial_number, size_t storage_unit_count) {
    impl_ = new Impl{
        transport::create_usb_transport(usb_vid, usb_pid, serial_number), storage_unit_count};
}

WUJIHANDCPP_API Handler::~Handler() { delete impl_; }

WUJIHANDCPP_API void Handler::init_storage_info(int storage_id, StorageInfo info) {
    impl_->init_storage_info(storage_id, info);
}

WUJIHANDCPP_API void Handler::read_async_unchecked(
    int storage_id, std::chrono::steady_clock::duration::rep timeout) {
    impl_->read_async_unchecked(storage_id, timeout);
}

WUJIHANDCPP_API void Handler::read_async(
    int storage_id, std::chrono::steady_clock::duration::rep timeout,
    void (*callback)(Buffer8 context, bool success), Buffer8 callback_context) {
    impl_->read_async(storage_id, timeout, callback, callback_context);
}

WUJIHANDCPP_API void Handler::write_async_unchecked(
    Buffer8 data, int storage_id, std::chrono::steady_clock::duration::rep timeout) {
    impl_->write_async_unchecked(data, storage_id, timeout);
}

WUJIHANDCPP_API void Handler::write_async(
    Buffer8 data, int storage_id, std::chrono::steady_clock::duration::rep timeout,
    void (*callback)(Buffer8 context, bool success), Buffer8 callback_context) {
    impl_->write_async(data, storage_id, timeout, callback, callback_context);
}

WUJIHANDCPP_API void Handler::attach_realtime_controller(
    device::IRealtimeController* controller, bool enable_upstream) {
    impl_->attach_realtime_controller(controller, enable_upstream);
}

WUJIHANDCPP_API device::IRealtimeController* Handler::detach_realtime_controller() {
    return impl_->detach_realtime_controller();
}

WUJIHANDCPP_API Handler::Buffer8 Handler::get(int storage_id) { return impl_->get(storage_id); }

WUJIHANDCPP_API void Handler::disable_thread_safe_check() {
    return impl_->disable_thread_safe_check();
}

} // namespace wujihandcpp::protocol
