#pragma once

#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/data/joint.hpp>
#include <wujihandcpp/device/latch.hpp>

#include "filter.hpp"

namespace py = pybind11;

template <typename T>
class Wrapper : private T {
public:
    explicit Wrapper(
        std::optional<std::string> serial_number, int32_t usb_pid, uint16_t usb_vid,
        std::optional<py::array_t<bool>> mask)
        : T(serial_number ? serial_number->c_str() : nullptr, usb_pid, usb_vid,
            parse_array_mask(mask)) {};

    uint32_t parse_array_mask(std::optional<py::array_t<bool>> mask) {
        if (!mask)
            return 0;
        if (mask->ndim() != 2 || mask->shape()[0] != 5 || mask->shape()[1] != 4)
            throw std::runtime_error("Mask shape must be {5, 4}!");

        auto r = mask->unchecked<2>();
        int k = 0;
        uint32_t result = 0;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++) {
                if (r(i, j))
                    result |= 1ul << k;
                k++;
            }
        return result;
    }

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

        return context->future;
    }

    template <typename Data>
    void read_async_unchecked() {
        T::template read_async_unchecked<Data>();
    }

    template <typename Data>
    void write(typename Data::ValueType value) {
        py::gil_scoped_release release;
        T::template write<Data>(value);
    }

    template <typename Data>
    void write(py::array_t<typename Data::ValueType> array) {
        py::gil_scoped_release release;
        wujihandcpp::device::Latch latch;

        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async<Data>(latch, r(j));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async<Data>(latch, r(i, j));
        }

        latch.wait();
    }

    template <typename Data>
    py::object write_async(typename Data::ValueType value) {
        auto context = new FutureContext{*this, data_count<Data>()};
        T::template write_async<Data>(
            [context]() {
                if (context->count_down()) {
                    py::gil_scoped_acquire acquire;
                    context->call_threadsafe(context->future.attr("set_result"), py::none());
                    delete context;
                }
            },
            value);

        return context->future;
    }

    template <typename Data>
    py::object write_async(py::array_t<typename Data::ValueType> array) {
        auto context = new FutureContext{*this, data_count<Data>()};

        auto callback = [context]() {
            if (context->count_down()) {
                py::gil_scoped_acquire acquire;
                context->call_threadsafe(context->future.attr("set_result"), py::none());
                delete context;
            }
        };

        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async<Data>(callback, r(j));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async<Data>(callback, r(i, j));
        }

        return context->future;
    }

    template <typename Data>
    void write_async_unchecked(typename Data::ValueType value) {
        T::template write_async_unchecked<Data>(value);
    }

    template <typename Data>
    void write_async_unchecked(py::array_t<typename Data::ValueType> array) {
        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async_unchecked<Data>(r(j));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async_unchecked<Data>(r(i, j));
        }
    }

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

    std::unique_ptr<IController>
        realtime_controller(bool enable_upstream, const filter::IFilter& filter) {
        return filter.create_controller(*this, enable_upstream);
    }

    template <typename Data>
    static void register_py_interface(py::class_<Wrapper>& py_class, const std::string& name) {
        if constexpr (Data::readable) {
            py_class.def(("read_" + name).c_str(), &Wrapper::read<Data>);
            py_class.def(
                ("read_" + name + "_async").c_str(), &Wrapper::read_async<Data>,
                py::keep_alive<0, 1>());
            py_class.def(
                ("read_" + name + "_unchecked").c_str(), &Wrapper::read_async_unchecked<Data>);
            py_class.def(("get_" + name).c_str(), &Wrapper::get<Data>);
        }
        if constexpr (Data::writable) {
            using V = Data::ValueType;
            py_class.def(
                ("write_" + name).c_str(), py::overload_cast<V>(&Wrapper::write<Data>),
                py::arg("value"));
            py_class.def(
                ("write_" + name + "_async").c_str(),
                py::overload_cast<V>(&Wrapper::write_async<Data>), py::arg("value"));
            py_class.def(
                ("write_" + name + "_unchecked").c_str(),
                py::overload_cast<V>(&Wrapper::write_async_unchecked<Data>), py::arg("value"));
            if constexpr (!std::is_same_v<typename Data::Base, T>) {
                py_class.def(
                    ("write_" + name).c_str(),
                    py::overload_cast<py::array_t<V>>(&Wrapper::write<Data>),
                    py::arg("value_array"));
                py_class.def(
                    ("write_" + name + "_async").c_str(),
                    py::overload_cast<py::array_t<V>>(&Wrapper::write_async<Data>),
                    py::arg("value_array"), py::keep_alive<0, 1>());
                py_class.def(
                    ("write_" + name + "_unchecked").c_str(),
                    py::overload_cast<py::array_t<V>>(&Wrapper::write_async_unchecked<Data>),
                    py::arg("value_array"));
            }
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
