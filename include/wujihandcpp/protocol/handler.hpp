#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <type_traits>

namespace wujihandcpp {
namespace protocol {

class Handler final {

public:
    struct Buffer8 {
        Buffer8() = default;

        template <typename T>
        using remove_cvref_t = typename std::remove_cv<typename std::remove_cv<T>::type>::type;

        template <typename T>
        explicit Buffer8(const T& value) {
            static_assert(sizeof(remove_cvref_t<T>) <= 8, "");
            static_assert(
                std::is_trivially_copyable<remove_cvref_t<T>>::value
                    && std::is_trivially_destructible<remove_cvref_t<T>>::value,
                "");
            new (storage) remove_cvref_t<T>{value};
        };

        template <typename T>
        remove_cvref_t<T> as() const {
            remove_cvref_t<T> out;
            std::memcpy(&out, storage, sizeof(out));
            return out;
        }

        alignas(8) uint8_t storage[8];
        static_assert(sizeof(void*) == 8, "");
    };

    explicit Handler(
        uint16_t usb_vid, int32_t usb_pid, size_t buffer_transfer_count, size_t storage_unit_count,
        int (*index_to_storage_id)(uint16_t, uint8_t));

    ~Handler();

    void read_async_unchecked(uint16_t index, uint8_t sub_index, int storage_id);

    void read_async(
        uint16_t index, uint8_t sub_index, int storage_id,
        void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context);

    template <size_t data_size>
    void write_async_unchecked(Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id);

    template <size_t data_size>
    void write_async(
        Buffer8 data, uint16_t index, uint8_t sub_index, int storage_id,
        void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context);

    void pdo_write_async_unchecked(const int32_t (&control_positions)[5][4], uint32_t timestamp);

    bool trigger_transmission();

    Buffer8 get(int storage_id);

    void disable_thread_safe_check();

private:
    class Impl;
    static constexpr size_t impl_align = 8;
    alignas(impl_align) uint8_t impl_[808];
};

} // namespace protocol
} // namespace wujihandcpp