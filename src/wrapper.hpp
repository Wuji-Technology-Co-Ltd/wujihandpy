#pragma once

#include <cstdint>

#include <atomic>
#include <type_traits>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/data/joint.hpp>

namespace py = pybind11;

template <typename T>
class Wrapper : private T {
public:
    using T::T;

    explicit Wrapper(T&& t)
        : T(std::move(t)) {}

    auto finger(int index) -> std::unique_ptr<Wrapper<wujihandcpp::device::Finger>> {
        return std::make_unique<Wrapper<wujihandcpp::device::Finger>>(T::finger(index));
    }

    auto joint(int index) -> std::unique_ptr<Wrapper<wujihandcpp::device::Joint>> {
        return std::make_unique<Wrapper<wujihandcpp::device::Joint>>(T::joint(index));
    }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) auto read() {
        py::gil_scoped_release release;
        return py::numpy_scalar{T::template read<Data>()};
    }

    template <typename Data>
    requires(!std::is_same_v<typename Data::Base, T>) auto read() {
        {
            py::gil_scoped_release release;
            T::template read<Data>();
        }
        return get<Data>();
    }

    template <typename Data>
    py::object read_async() {
        auto context = new FutureContext{*this, data_count<Data>()};
        T::template read_async<Data>([context](Data::ValueType) {
            if (context->count_down()) {
                py::gil_scoped_acquire acquire;
                context->call_threadsafe(
                    context->future.attr("set_result"), context->hand.template get<Data>());
                delete context;
            }
        });
        T::trigger_transmission();

        return context->future;
    }

    template <typename Data>
    void read_async_unchecked() {
        T::template read_async_unchecked<Data>();
    }

    template <typename Data>
    auto write(py::numpy_scalar<typename Data::ValueType> value) {
        py::gil_scoped_release release;
        T::template write<Data>(value.value);
    }

    template <typename Data>
    py::object write_async(py::numpy_scalar<typename Data::ValueType> value) {
        auto context = new FutureContext{*this, data_count<Data>()};
        T::template write_async<Data>(
            [context]() {
                if (context->count_down()) {
                    py::gil_scoped_acquire acquire;
                    context->call_threadsafe(context->future.attr("set_result"), py::none());
                    delete context;
                }
            },
            value.value);
        T::trigger_transmission();

        return context->future;
    }

    template <typename Data>
    void write_async_unchecked(py::numpy_scalar<typename Data::ValueType> value) {
        T::template write_async_unchecked<Data>(value.value);
    }

    void trigger_transmission() { T::trigger_transmission(); }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) auto get() {
        return py::numpy_scalar{T::template get<Data>()};
    }

    template <typename Data>
    requires(
        std::is_same_v<T, wujihandcpp::device::Finger>
        && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
    auto get() {
        using ValueType = Data::ValueType;
        auto buffer = new ValueType[4];
        for (int j = 0; j < 4; j++)
            buffer[j] = T::joint(j).template get<Data>();

        py::capsule free(buffer, [](void* ptr) { delete[] static_cast<ValueType*>(ptr); });

        return py::array_t<ValueType>({4}, buffer, free);
    }

    template <typename Data>
    requires(
        std::is_same_v<T, wujihandcpp::device::Hand>
        && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
    auto get() {
        using ValueType = Data::ValueType;
        auto buffer = new ValueType[5 * 4];
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                buffer[4 * i + j] = T::finger(i).joint(j).template get<Data>();

        py::capsule free(buffer, [](void* ptr) { delete[] static_cast<ValueType*>(ptr); });

        return py::array_t<ValueType>({5, 4}, buffer, free);
    }

    template <typename Data>
    static void register_py_interface(py::class_<Wrapper>& py_class, const std::string& name) {
        if constexpr (Data::readable) {
            py_class.def(("read_" + name).c_str(), &Wrapper::read<Data>);
            py_class.def(("read_" + name + "_async").c_str(), &Wrapper::read_async<Data>);
            py_class.def(
                ("read_" + name + "_unchecked").c_str(), &Wrapper::read_async_unchecked<Data>);
            py_class.def(("get_" + name).c_str(), &Wrapper::get<Data>);
        }
        if constexpr (Data::writable) {
            py_class.def(("write_" + name).c_str(), &Wrapper::write<Data>);
            py_class.def(("write_" + name + "_async").c_str(), &Wrapper::write_async<Data>);
            py_class.def(
                ("write_" + name + "_unchecked").c_str(), &Wrapper::write_async_unchecked<Data>);
        }
    }

private:
    struct FutureContext {
        explicit FutureContext(Wrapper& hand, int count)
            : hand(hand)
            , count(count) {
            auto loop = py::module::import("asyncio").attr("get_event_loop")();
            call_threadsafe = loop.attr("call_soon_threadsafe");
            future = loop.attr("create_future")();
        }

        bool count_down() {
            auto new_count = count.load(std::memory_order::relaxed) - 1;
            count.store(new_count, std::memory_order::relaxed);
            return new_count == 0;
        }

        Wrapper& hand;
        std::atomic<int> count;

        py::object call_threadsafe;
        py::object future;
    };

    template <typename Data>
    static constexpr int data_count() {
        if constexpr (std::is_same_v<typename Data::Base, T>)
            return 1;
        else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
            return 4;
        else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
            return 5 * 4;
    }
};
