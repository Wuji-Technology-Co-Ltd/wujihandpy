#pragma once

#include <cstdint>

#include <atomic>
#include <bit>
#include <type_traits>

namespace protocol {

class BasicStorageUnit {
public:
    BasicStorageUnit() = default;

    uint32_t data_token() { return data_token_.load(std::memory_order::acquire); }

    bool wait(uint32_t old_data_token) {
        waiting_count_.fetch_add(1, std::memory_order::relaxed);
        data_token_.wait(old_data_token, std::memory_order::acquire);
        waiting_count_.fetch_sub(1, std::memory_order::relaxed);

        return data_token_.load(std::memory_order::relaxed) & 1;
    }

    template <typename T>
    T read() {
        return data_.load(std::memory_order::relaxed).as<T>();
    }

    template <typename T, typename F>
    requires(
        sizeof(F) <= 8 && alignof(F) <= 8 && std::is_trivially_copyable_v<F>
        && std::is_trivially_destructible_v<F> && requires(T t, const F& f) { f(t); })
    void subscribe(const F& callback) {
        struct alignas(8) Wrapper {
            F callback;
        };

        callback_.store(nullptr, std::memory_order::release);
        callback_context_.store(
            std::bit_cast<uint64_t>(Wrapper(callback)), std::memory_order::release);
        callback_.store(
            [](DataUnion data, uint64_t context) {
                auto wrapper = std::bit_cast<Wrapper>(context);
                wrapper.callback(data.as<T>());
            },
            std::memory_order::release);
    }

    template <typename T>
    void write(const T& data) {
        auto data_union = DataUnion{data};
        data_.store(data_union, std::memory_order::relaxed);

        auto token = data_token_.load(std::memory_order::relaxed);
        token = (token & ~3) + 4 + 1;
        data_token_.store(token, std::memory_order::release);

        if (waiting_count_.load(std::memory_order::acquire))
            data_token_.notify_all();

        void (*callback)(DataUnion, uint64_t);
        uint64_t callback_context;
        do {
            callback = callback_.load(std::memory_order::acquire);
            if (!callback)
                return;
            callback_context = callback_context_.load(std::memory_order::acquire);
        } while (callback_.load(std::memory_order::acquire) != callback);
        callback(data_union, callback_context);
    }

    void error(uint32_t err_code) {
        (void)err_code;

        auto token = data_token_.load(std::memory_order::relaxed);
        token = (token & ~3) + 4 + 2;
        data_token_.store(token, std::memory_order::release);

        if (waiting_count_.load(std::memory_order::acquire))
            data_token_.notify_all();
    }

private:
    union DataUnion {
        constexpr explicit DataUnion() = default;
        constexpr explicit DataUnion(const uint8_t& data)
            : data8(data) {}
        constexpr explicit DataUnion(const uint16_t& data)
            : data16(data) {}
        constexpr explicit DataUnion(const uint32_t& data)
            : data32(data) {}
        constexpr explicit DataUnion(const uint64_t& data)
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

    std::atomic<DataUnion> data_;
    std::atomic<uint32_t> data_token_ = 0;
    std::atomic<uint32_t> waiting_count_ = 0;
    static_assert(decltype(data_)::is_always_lock_free);
    static_assert(decltype(data_token_)::is_always_lock_free);

    std::atomic<void (*)(DataUnion, uint64_t)> callback_ = nullptr;
    std::atomic<uint64_t> callback_context_;
    static_assert(decltype(callback_)::is_always_lock_free);
    static_assert(decltype(callback_context_)::is_always_lock_free);
};

struct StorageUnit {
    BasicStorageUnit read_part, write_part;
};

} // namespace protocol