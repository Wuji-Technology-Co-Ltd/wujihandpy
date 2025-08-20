#pragma once

#include <chrono>
#include <cstdint>

#include <thread>
#include <type_traits>

#include "wujihandcpp/protocol/handler.hpp"
#include "wujihandcpp/protocol/storage.hpp"

namespace wujihandcpp::device {

template <typename T>
class DataOperator {
public:
    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) Data::ValueType read_data() {
        int storage_id;
        iterate_storage<Data>([&](int storage) { storage_id = storage; });

        while (true) {
            auto token = handler().data_operation_token(storage_id);

            read_data_async<Data>();
            trigger_transmission();

            auto new_token = handler().wait_data_operation(storage_id, token);
            if (new_token.data_version != token.data_version)
                break;
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };

        return data_internal<Data>(storage_id);
    }

    template <typename Data>
    void read_data() {
        constexpr int count = iterate_count<Data>();

        protocol::OperationToken token[count];
        bool success[count] = {false};
        int success_count = 0;

        iterate_storage<Data>([&, i = count](int storage_id) mutable {
            token[--i] = handler().data_operation_token(storage_id);
        });

        while (true) {
            iterate_index<Data>([&, i = 0](uint16_t index, uint8_t sub_index) mutable {
                if (!success[i++])
                    handler().read_data_async(index, sub_index);
            });
            trigger_transmission();

            iterate_storage<Data>([&, i = count](int storage_id) mutable {
                if (!success[--i]) {
                    auto new_token = handler().wait_data_operation(storage_id, token[i]);
                    success[i] = (new_token.data_version != token[i].data_version);
                    success_count += success[i];
                    token[i] = new_token;
                }
            });

            if (success_count == count)
                break;
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };
    }

    template <typename Data>
    void read_data_async() {
        iterate_index<Data>([this](uint16_t index, uint8_t sub_index) {
            handler().read_data_async(index, sub_index);
        });
    }

    void trigger_transmission() { handler().trigger_transmission(); }

    // template <typename Data>
    // requires(std::is_same_v<typename Data::Base, T>) uint32_t data_read_token() {
    //     uint32_t token;
    //     iterate_storage([&](int storage_id) { token = handler().data_read_token(storage_id); });
    //     return token;
    // }

    // template <typename Data>
    // void wait_reading_data() {
    //     iterate_storage([&](int storage_id) {
    //         uint32_t token = handler().data_read_token(storage_id);
    //         handler().wait_reading_data(storage_id, token);
    //     });
    // }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) Data::ValueType data() {
        typename Data::ValueType value;
        iterate_storage<Data>([&](int storage_id) { value = data_internal<Data>(storage_id); });
        return value;
    }

    template <typename Data>
    void set_data_read_callback();

    template <typename Data>
    void write_data(Data::ValueType value) {
        constexpr int count = iterate_count<Data>();

        protocol::OperationToken token[count];
        bool success[count] = {false};
        int success_count = 0;

        iterate_storage<Data>([&, i = count](int storage_id) mutable {
            token[--i] = handler().data_operation_token(storage_id);
        });

        while (true) {
            // Write
            iterate_index<Data>([&, i = 0](uint16_t index, uint8_t sub_index) mutable {
                if (!success[i++])
                    handler().write_data_async(index, sub_index, value);
            });
            trigger_transmission();

            iterate_storage<Data>([&, i = count](int storage_id) mutable {
                if (!success[--i]) {
                    protocol::OperationToken new_token =
                        handler().wait_data_operation(storage_id, token[i]);
                    success[i] = (new_token.err_code == 0);
                    success_count += success[i];
                    token[i] = new_token;
                }
            });

            if (success_count == count)
                break;
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Read
            iterate_index<Data>([&, i = 0](uint16_t index, uint8_t sub_index) mutable {
                if (!success[i++])
                    handler().read_data_async(index, sub_index);
            });
            trigger_transmission();

            iterate_storage<Data>([&, i = count](int storage_id) mutable {
                if (!success[--i]) {
                    auto new_token = handler().wait_data_operation(storage_id, token[i]);
                    if (new_token.data_version != token[i].data_version)
                        success[i] = (value == data_internal<Data>(storage_id));
                    success_count += success[i];
                    token[i] = new_token;
                }
            });

            if (success_count == count)
                break;
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };
    }

    template <typename Data>
    void write_data_async(Data::ValueType value) {
        iterate_index<Data>([this, value](uint16_t index, uint8_t sub_index) {
            handler().write_data_async(index, sub_index, value);
        });
    }

    template <typename Data>
    uint32_t data_wrote_token();

    template <typename Data>
    void wait_writing_data();

    template <typename Data>
    void set_data_wrote_callback();

    // protected:
    template <typename Data>
    static constexpr int iterate_count() {
        if constexpr (std::is_same_v<typename Data::Base, T>)
            return 1;
        else {
            return T::sub_count_ * T::Sub::template iterate_count<Data>();
        }
    }

    static constexpr int data_count() {
        if constexpr (requires { typename T::Sub; }) {
            return T::Datas::count + T::sub_count_ * T::Sub::data_count();
        } else {
            return T::Datas::count;
        }
    }

    template <typename Data>
    void iterate_index(auto&& f) {
        T& self = *static_cast<T*>(this);
        if constexpr (std::is_same_v<typename Data::Base, T>)
            f(self.index_offset_ + Data::index, Data::sub_index);
        else {
            for (int i = 0; i < T::sub_count_; i++)
                self.sub(i).template iterate_index<Data>(f);
        }
    }

    template <typename Data>
    void iterate_storage(auto&& f) {
        T& self = *static_cast<T*>(this);
        if constexpr (std::is_same_v<typename Data::Base, T>)
            f(self.storage_offset_ + T::Datas::template index<Data>);
        else {
            for (int i = T::sub_count_; i-- > 0;)
                self.sub(i).template iterate_storage<Data>(f);
        }
    }

private:
    protocol::Handler& handler() { return static_cast<T*>(this)->handler_; }

    template <typename Data>
    Data::ValueType data_internal(int storage) {
        using ValueType = typename Data::ValueType;
        if constexpr (sizeof(ValueType) == 1)
            return std::bit_cast<ValueType>(handler().template data<uint8_t>(storage));
        else if constexpr (sizeof(ValueType) == 2)
            return std::bit_cast<ValueType>(handler().template data<uint16_t>(storage));
        else if constexpr (sizeof(ValueType) == 4)
            return std::bit_cast<ValueType>(handler().template data<uint32_t>(storage));
        else if constexpr (sizeof(ValueType) == 8)
            return std::bit_cast<ValueType>(handler().template data<uint64_t>(storage));
    }
};

} // namespace wujihandcpp::device