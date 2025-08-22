#pragma once

#include <cstdint>

#include <type_traits>

#include "wujihandcpp/device/latch.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp::device {

template <typename T>
class DataOperator {
    using Handler = protocol::Handler;
    using Buffer8 = protocol::Handler::Buffer8;

public:
    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) Data::ValueType read_data() {
        read<Data>();
        return get<Data>();
    }

    template <typename Data>
    void read() {
        Latch latch;
        read_async<Data>(latch);
        trigger_transmission();
        latch.wait();
    }

    template <typename Data>
    void read_async(Latch& latch) {
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
    requires(
        sizeof(F) <= 8 && alignof(F) <= 8
        && std::is_trivially_copyable_v<F> && std::is_trivially_destructible_v<F>
        && requires(Data::ValueType value, const F& f) { f(value); })
    void read_async(const F& f) {
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

    template <typename Data>
    void set_data_read_callback(); // TODO

    template <typename Data>
    void read_async_unchecked() {
        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&handler](uint16_t index, uint8_t sub_index, int storage_id) {
            handler.read_async_unchecked(index, sub_index, storage_id);
        });
    }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) Data::ValueType get() {
        Handler& handler = static_cast<T*>(this)->handler_;
        typename Data::ValueType value;
        iterate<Data>([&](uint16_t, uint8_t, int storage_id) {
            value = handler.get(storage_id).as<typename Data::ValueType>();
        });
        return value;
    }

    template <typename Data>
    void write(Data::ValueType value) {
        Latch latch;
        write_async<Data>(latch, value);
        trigger_transmission();
        latch.wait();
    }

    template <typename Data>
    void write_async(Latch& latch, Data::ValueType value) {
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
    requires(
        sizeof(F) <= 8 && alignof(F) <= 8 && std::is_trivially_copyable_v<F>
        && std::is_trivially_destructible_v<F> && requires(const F& f) { f(); })
    void write_async(const F& f, Data::ValueType value) {
        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            Buffer8 callback_context{f};
            handler.write_async<sizeof(value)>(
                Buffer8{value}, index, sub_index, storage_id,
                [](Buffer8 context, Buffer8) { context.as<F>()(); }, callback_context);
        });
    }

    template <typename Data>
    void write_async_unchecked(Data::ValueType value) {
        Handler& handler = static_cast<T*>(this)->handler_;
        iterate<Data>([&](uint16_t index, uint8_t sub_index, int storage_id) {
            handler.write_async_unchecked<sizeof(value)>(
                Buffer8{value}, index, sub_index, storage_id);
        });
    }

    void trigger_transmission() {
        Handler& handler = static_cast<T*>(this)->handler_;
        handler.trigger_transmission();
    }

    static constexpr int data_count() {
        if constexpr (requires { typename T::Sub; }) {
            return T::Datas::count + T::sub_count_ * T::Sub::data_count();
        } else {
            return T::Datas::count;
        }
    }

    template <typename Data>
    void iterate(auto&& f) {
        T& self = *static_cast<T*>(this);
        if constexpr (std::is_same_v<typename Data::Base, T>)
            f(self.index_offset_ + Data::index, Data::sub_index,
              self.storage_offset_ + T::Datas::template index<Data>);
        else {
            for (int i = 0; i < T::sub_count_; i++)
                self.sub(i).template iterate<Data>(f);
        }
    }
};

} // namespace wujihandcpp::device