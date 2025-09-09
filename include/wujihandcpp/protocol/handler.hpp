#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <type_traits>

namespace wujihandcpp {
namespace protocol {

class Handler final {

public:
    struct StorageInfo {
        StorageInfo() = default;

        constexpr explicit StorageInfo(
            size_t data_size, uint16_t index, uint8_t sub_index, uint32_t policy = NONE)
            : index(index)
            , sub_index(sub_index)
            , size(
                  data_size == 1
                      ? Size::_1
                      : (data_size == 2 ? Size::_2 : (data_size == 4 ? Size::_4 : Size::_8)))
            , policy(policy) {}

        uint16_t index;
        uint8_t sub_index;

        enum class Size : uint32_t { _1, _2, _4, _8 } size : 2;
        enum Policy : uint32_t {
            NONE = 0,
            POSITION_FLOATING = 0x01,
            POSITION_REVERSED = 0x02,
            VELOCITY_FLOATING = 0x04,
            VELOCITY_REVERSED = 0x08,
            CURRENT_FLOATING = 0x10,
            CURRENT_REVERSED = 0x20,
        };
        uint32_t policy : 30;
    };

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
        }

        template <typename T>
        remove_cvref_t<T> as() const {
            return *reinterpret_cast<const remove_cvref_t<T>*>(storage);
        }

        alignas(8) uint8_t storage[8];
        static_assert(sizeof(void*) == 8, "");
    };

    explicit Handler(
        uint16_t usb_vid, int32_t usb_pid, const char* serial_number, size_t buffer_transfer_count,
        size_t storage_unit_count);

    ~Handler();

    void init_storage_info(int storage_id, StorageInfo info);

    void read_async_unchecked(int storage_id);

    void read_async(
        int storage_id, void (*callback)(Buffer8 context, Buffer8 value), Buffer8 callback_context);

    void write_async_unchecked(Buffer8 data, int storage_id);

    void write_async(
        Buffer8 data, int storage_id, void (*callback)(Buffer8 context, Buffer8 value),
        Buffer8 callback_context);

    void pdo_write_async_unchecked(const double (&control_positions)[5][4], uint32_t timestamp);

    bool trigger_transmission();

    Buffer8 get(int storage_id);

    void disable_thread_safe_check();

private:
    class Impl;
    static constexpr size_t impl_align = 8;
    alignas(impl_align) uint8_t impl_[760];
};

} // namespace protocol
} // namespace wujihandcpp