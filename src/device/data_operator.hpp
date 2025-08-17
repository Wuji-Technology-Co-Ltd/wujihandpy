#pragma once

#include <cstdint>

#include "protocol/handler.hpp"

namespace device {

template <typename T>
class DataOperator {
public:
    template <typename Data>
    Data::ValueType read_data();

    template <typename Data>
    void read_data_async() {}

    template <typename Data>
    uint32_t data_read_token() {
        uint32_t token;
        T& self = *static_cast<T*>(this);
        self.apply_unique_ids(
            [&self, &token](int unique_id) { token = self.data_read_token(unique_id); });
        return token;
    }

    template <typename Data>
    void wait_reading_data() {
        T& self = *static_cast<T*>(this);
        self.apply_unique_ids([&self](int unique_id) {
            uint32_t token = self.data_read_token(unique_id);
            self.wait_reading_data(unique_id, token);
        });
    }

    template <typename Data>
    Data::ValueType data() {
        // Data::ValueType
        // T& self = *static_cast<T*>(this);
        // self.apply_unique_ids([](int unique_id) {

        // if constexpr (sizeof(T) == 1)
        //     return std::bit_cast<T>(storage_[unique_id].read_part.read<uint8_t>());
        // else if constexpr (sizeof(T) == 1)
        //     return std::bit_cast<T>(storage_[unique_id].read_part.read<uint16_t>());
        // else if constexpr (sizeof(T) == 1)
        //     return std::bit_cast<T>(storage_[unique_id].read_part.read<uint32_t>());
        // else if constexpr (sizeof(T) == 1)
        //     return std::bit_cast<T>(storage_[unique_id].read_part.read<uint64_t>());
        // });
    }

    template <typename Data>
    void set_data_read_callback();

    template <typename Data>
    void write_data(Data::ValueType value);

    template <typename Data>
    void write_data_async(Data::ValueType value);

    template <typename Data>
    uint32_t data_wrote_token();

    template <typename Data>
    void wait_writing_data();

    template <typename Data>
    void set_data_wrote_callback();

    // protected:
    template <typename Data>
    static constexpr size_t iterate_count() {
        if constexpr (std::is_same_v<typename Data::Base, T>)
            return 1;
        else {
            return T::sub_count_ * T::Sub::template iterate_count<Data>();
        }
    }

    static constexpr int data_count() requires(requires { typename T::Sub; }) {
        return T::Datas::count + T::sub_count_ * T::Sub::data_count();
    }

    static constexpr int data_count() { return T::Datas::count; }

    template <typename Data>
    void iterate_index(const auto& f) {
        T& self = *static_cast<T*>(this);
        if constexpr (std::is_same_v<typename Data::Base, T>)
            f(self.index_offset_);
        else {
            for (int i = 0; i < T::sub_count_; i++)
                self.sub(i).template iterate_index<Data>(f);
        }
    }

    template <typename Data>
    void iterate_storage(const auto& f) {
        T& self = *static_cast<T*>(this);
        if constexpr (std::is_same_v<typename Data::Base, T>)
            f(self.storage_offset_);
        else {
            for (int i = 0; i < T::sub_count_; i++)
                self.sub(i).template iterate_storage<Data>(f);
        }
    }
};

} // namespace device