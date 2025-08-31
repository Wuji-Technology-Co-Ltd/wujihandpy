#pragma once

#include <cstdint>
#include <limits>
#include <utility>

namespace wujihandcpp {
namespace device {

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
        return (T::index == index && T::sub_index == sub_index)
                 ? id
                 : match_index_internal<id + 1, Ts...>(index, sub_index);
    }

    template <int id>
    static constexpr int match_index_internal(const uint16_t, const uint8_t) {
        return std::numeric_limits<int>::min();
    }

    template <int index, typename F, typename T, typename Tnext, typename... Ts>
    static void iterate_internal(F&& f) {
        f.template operator()<index, T>();
        iterate_internal<index + 1, F, Tnext, Ts...>(std::forward<F>(f));
    }

    template <int index, typename F, typename T>
    static void iterate_internal(F&& f) {
        f.template operator()<index, T>();
    }

    template <int, typename F>
    static void iterate_internal(F&&) {}

public:
    DataTuple() = delete;

    static constexpr int count = sizeof...(Types);

    template <typename T>
    static constexpr int index() {
        return get_index_internal<0, T, Types...>::type::index;
    }

    static constexpr int match_index(const uint16_t index, const uint8_t sub_index) {
        return match_index_internal<0, Types...>(index, sub_index);
    }

    template <typename F>
    static void iterate(F&& f) {
        iterate_internal<0, F, Types...>(std::forward<F>(f));
    }
};

} // namespace device
} // namespace wujihandcpp