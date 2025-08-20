#pragma once

#include <cstdint>
#include <limits>

namespace wujihandcpp::device {

template <typename... Types>
class DataTuple {
    // https://stackoverflow.com/a/16594174
    template <int Index, typename Search, typename First, typename... Ts>
    struct get_index_internal {
        typedef typename get_index_internal<Index + 1, Search, Ts...>::type type;
        static constexpr int index = Index;
    };

    template <int Index, typename Search, typename... Ts>
    struct get_index_internal<Index, Search, Search, Ts...> {
        typedef get_index_internal type;
        static constexpr int index = Index;
    };

    template <int id, typename T, typename... Ts>
    static constexpr int match_index_internal(const uint16_t index, const uint8_t sub_index) {
        if (T::index == index && T::sub_index == sub_index)
            return id;
        else
            return match_index_internal<id + 1, Ts...>(index, sub_index);
    }

    template <int id>
    static constexpr int match_index_internal(const uint16_t, const uint8_t) {
        return std::numeric_limits<int>::min();
    }

public:
    DataTuple() = delete;

    static constexpr int count = sizeof...(Types);

    template <typename T>
    static constexpr int index = get_index_internal<0, T, Types...>::type::index;

    static constexpr int match_index(const uint16_t index, const uint8_t sub_index) {
        return match_index_internal<0, Types...>(index, sub_index);
    }
};

} // namespace wujihandcpp::device