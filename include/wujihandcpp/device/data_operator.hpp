#pragma once

#include <cstdint>

#include <type_traits>

#include "wujihandcpp/device/latch.hpp"
#include "wujihandcpp/protocol/handler.hpp"

#if __cplusplus >= 202002L
# define SDK_CPP20_REQUIRES(...) requires(__VA_ARGS__)
#else
# define SDK_CPP20_REQUIRES(...)
#endif

namespace wujihandcpp {
namespace device {

template <typename T>
class DataOperator {
    using Handler = protocol::Handler;
    using Buffer8 = protocol::Handler::Buffer8;

    template <typename U>
    friend class DataOperator;

    template <typename Data, typename F>
    typename std::enable_if<std::is_same<typename Data::Base, T>::value>::type iterate(F&& f) {
        T& self = *static_cast<T*>(this);
        f(self.index_offset_ + Data::index, Data::sub_index,
          self.storage_offset_ + T::Datas::template index<Data>());
    }

    template <typename Data, typename F>
    typename std::enable_if<!std::is_same<typename Data::Base, T>::value>::type iterate(F&& f) {
        T& self = *static_cast<T*>(this);
        for (int i = 0; i < T::sub_count_; i++)
            self.sub(i).template iterate<Data>(f);
    }

public:
    template <typename Data>
    SDK_CPP20_REQUIRES(Data::readable)
    auto read() -> typename std::enable_if<
        std::is_same<typename Data::Base, T>::value, typename Data::ValueType>::type {
        static_assert(Data::readable, "");

        Latch latch;
        read_async<Data>(latch);
        trigger_transmission();
        latch.wait();
        return get<Data>();
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::readable)
    auto read() ->
        typename std::enable_if<!std::is_same<typename Data::Base, T>::value, void>::type {
        static_assert(Data::readable, "");

        Latch latch;
        read_async<Data>(latch);
        trigger_transmission();
        latch.wait();
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::readable)
    void read_async(Latch& latch) {
        static_assert(Data::readable, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            latch.count_up();

            Buffer8 callback_context{&latch};
            handler.read_async(
                index, sub_index, storage_id,
                [](Buffer8 context, Buffer8) { (context.as<Latch*>())->count_down(); },
                callback_context);
        });
    }

    template <typename Data, typename F>
    SDK_CPP20_REQUIRES(
        Data::readable && sizeof(F) <= 8 && alignof(F) <= 8
        && std::is_trivially_copyable_v<F> && std::is_trivially_destructible_v<F>
        && requires(Data::ValueType value, const F& f) { f(value); })
    void read_async(const F& f) {
        static_assert(Data::readable, "");

        static_assert(sizeof(F) <= 8, "");
        static_assert(alignof(F) <= 8, "");
        static_assert(std::is_trivially_copyable<F>::value, "");
        static_assert(std::is_trivially_destructible<F>::value, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            Buffer8 callback_context{f};
            handler.read_async(
                index, sub_index, storage_id,
                [](Buffer8 context, Buffer8 value) {
                    context.as<F>()(value.as<typename Data::ValueType>());
                },
                callback_context);
        });
    }

    // TODO
    // template <typename Data>
    // void set_data_read_callback();

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::readable)
    void read_async_unchecked() {
        static_assert(Data::readable, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&handler](uint16_t index, uint8_t sub_index, int storage_id) {
            handler.read_async_unchecked(index, sub_index, storage_id);
        });
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::readable)
    auto get() -> typename std::enable_if<
        std::is_same<typename Data::Base, T>::value, typename Data::ValueType>::type {
        static_assert(Data::readable, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        typename Data::ValueType value;
        iterate<Data>([&](uint16_t, uint8_t, int storage_id) {
            value = handler.get(storage_id).as<typename Data::ValueType>();
        });
        return value;
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::writable)
    void write(typename Data::ValueType value) {
        static_assert(Data::writable, "");

        Latch latch;
        write_async<Data>(latch, value);
        trigger_transmission();
        latch.wait();
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::writable)
    void write_async(Latch& latch, typename Data::ValueType value) {
        static_assert(Data::writable, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            latch.count_up();

            Buffer8 callback_context{&latch};
            handler.write_async<sizeof(value)>(
                Buffer8{value}, index, sub_index, storage_id,
                [](Buffer8 context, Buffer8) { (context.as<Latch*>())->count_down(); },
                callback_context);
        });
    }

    template <typename Data, typename F>
    SDK_CPP20_REQUIRES(
        Data::writable && sizeof(F) <= 8 && alignof(F) <= 8 && std::is_trivially_copyable_v<F>
        && std::is_trivially_destructible_v<F> && requires(const F& f) { f(); })
    void write_async(const F& f, typename Data::ValueType value) {
        static_assert(Data::writable, "");

        static_assert(sizeof(F) <= 8, "");
        static_assert(alignof(F) <= 8, "");
        static_assert(std::is_trivially_copyable<F>::value, "");
        static_assert(std::is_trivially_destructible<F>::value, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            Buffer8 callback_context{f};
            handler.write_async<sizeof(value)>(
                Buffer8{value}, index, sub_index, storage_id,
                [](Buffer8 context, Buffer8) { context.as<F>()(); }, callback_context);
        });
    }

    template <typename Data>
    SDK_CPP20_REQUIRES(Data::writable)
    void write_async_unchecked(typename Data::ValueType value) {
        static_assert(Data::writable, "");

        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            handler.write_async_unchecked<sizeof(value)>(
                Buffer8{value}, index, sub_index, storage_id);
        });
    }

private:
    void trigger_transmission() {
        Handler& handler = static_cast<T*>(this)->handler_;
        handler.trigger_transmission();
    }

    template <typename U>
    constexpr static decltype(std::declval<typename U::Sub>(), int()) data_count_internal(int) {
        return T::Datas::count + T::sub_count_ * T::Sub::data_count();
    }

    template <typename>
    constexpr static int data_count_internal(...) {
        return T::Datas::count;
    }

protected:
    static constexpr int data_count() { return data_count_internal<T>(0); }
};

} // namespace device
} // namespace wujihandcpp