#pragma once

#include <cmath>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
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

    static constexpr double default_timeout = 0.5;

    constexpr std::chrono::steady_clock::duration seconds_to_duration(double seconds) {
        using duration_t = std::chrono::steady_clock::duration;

        if (std::isnan(seconds))
            return duration_t::max();

        constexpr double max_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(duration_t::max()).count();
        constexpr double min_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(duration_t::min()).count();
        seconds = std::clamp(seconds, min_seconds, max_seconds);

        return std::chrono::duration_cast<duration_t>(std::chrono::duration<double>(seconds));
    }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, T>) auto read(double timeout) {
        py::gil_scoped_release release;
        return py::numpy_scalar{T::template read<Data>(seconds_to_duration(timeout))};
    }

    template <typename Data>
    requires(!std::is_same_v<typename Data::Base, T>) auto read(double timeout) {
        {
            py::gil_scoped_release release;
            T::template read<Data>(seconds_to_duration(timeout));
        }
        return get<Data>();
    }

    template <typename Data>
    py::object read_async(double timeout) {
        FutureLatch* latch = FutureLatch::create(*this, data_count<Data>());
        T::template read_async<Data>(
            [latch](bool success) {
                latch->count_down(success, [](Wrapper& wrapper) { return wrapper.get<Data>(); });
            },
            seconds_to_duration(timeout));

        return latch->future();
    }

    template <typename Data>
    void read_async_unchecked(double timeout) {
        T::template read_async_unchecked<Data>(seconds_to_duration(timeout));
    }

    template <typename Data>
    void write(typename Data::ValueType value, double timeout) {
        py::gil_scoped_release release;
        T::template write<Data>(value, seconds_to_duration(timeout));
    }

    template <typename Data>
    void write(py::array_t<typename Data::ValueType> array, double timeout) {
        py::gil_scoped_release release;
        wujihandcpp::device::Latch latch;

        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async<Data>(latch, r(j), seconds_to_duration(timeout));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async<Data>(
                        latch, r(i, j), seconds_to_duration(timeout));
        }

        latch.wait();
    }

    template <typename Data>
    py::object write_async(typename Data::ValueType value, double timeout) {
        FutureLatch* latch = FutureLatch::create(*this, data_count<Data>());
        T::template write_async<Data>(
            [latch](bool success) {
                latch->count_down(success, [](Wrapper&) { return py::none(); });
            },
            value, seconds_to_duration(timeout));

        return latch->future();
    }

    template <typename Data>
    py::object write_async(py::array_t<typename Data::ValueType> array, double timeout) {
        FutureLatch* latch = FutureLatch::create(*this, data_count<Data>());
        auto callback = [latch](bool success) {
            latch->count_down(success, [](Wrapper&) { return py::none(); });
        };

        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async<Data>(
                    callback, r(j), seconds_to_duration(timeout));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async<Data>(
                        callback, r(i, j), seconds_to_duration(timeout));
        }

        return latch->future();
    }

    template <typename Data>
    void write_async_unchecked(typename Data::ValueType value, double timeout) {
        T::template write_async_unchecked<Data>(value, seconds_to_duration(timeout));
    }

    template <typename Data>
    void write_async_unchecked(py::array_t<typename Data::ValueType> array, double timeout) {
        if constexpr (
            std::is_same_v<T, wujihandcpp::device::Finger>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 1 || array.shape()[0] != 4)
                throw std::runtime_error("Array shape must be {4}!");
            auto r = array.template unchecked<1>();
            for (int j = 0; j < 4; j++)
                T::joint(j).template write_async_unchecked<Data>(
                    r(j), seconds_to_duration(timeout));
        } else if constexpr (
            std::is_same_v<T, wujihandcpp::device::Hand>
            && std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) {
            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");
            auto r = array.template unchecked<2>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 4; j++)
                    T::finger(i).joint(j).template write_async_unchecked<Data>(
                        r(i, j), seconds_to_duration(timeout));
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
            py_class.def(("read_" + name).c_str(), &Wrapper::read<Data>, py::arg("timeout") = 0.5);
            py_class.def(
                ("read_" + name + "_async").c_str(), &Wrapper::read_async<Data>,
                py::arg("timeout") = 0.5, py::keep_alive<0, 1>());
            py_class.def(
                ("read_" + name + "_unchecked").c_str(), &Wrapper::read_async_unchecked<Data>,
                py::arg("timeout") = 0.5);
            py_class.def(("get_" + name).c_str(), &Wrapper::get<Data>);
        }
        if constexpr (Data::writable) {
            using V = Data::ValueType;
            py_class.def(
                ("write_" + name).c_str(), py::overload_cast<V, double>(&Wrapper::write<Data>),
                py::arg("value"), py::arg("timeout") = 0.5);
            py_class.def(
                ("write_" + name + "_async").c_str(),
                py::overload_cast<V, double>(&Wrapper::write_async<Data>), py::arg("value"),
                py::arg("timeout") = 0.5);
            py_class.def(
                ("write_" + name + "_unchecked").c_str(),
                py::overload_cast<V, double>(&Wrapper::write_async_unchecked<Data>),
                py::arg("value"), py::arg("timeout") = 0.5);
            if constexpr (!std::is_same_v<typename Data::Base, T>) {
                py_class.def(
                    ("write_" + name).c_str(),
                    py::overload_cast<py::array_t<V>, double>(&Wrapper::write<Data>),
                    py::arg("value_array"), py::arg("timeout") = 0.5);
                py_class.def(
                    ("write_" + name + "_async").c_str(),
                    py::overload_cast<py::array_t<V>, double>(&Wrapper::write_async<Data>),
                    py::arg("value_array"), py::arg("timeout") = 0.5, py::keep_alive<0, 1>());
                py_class.def(
                    ("write_" + name + "_unchecked").c_str(),
                    py::overload_cast<py::array_t<V>, double>(
                        &Wrapper::write_async_unchecked<Data>),
                    py::arg("value_array"), py::arg("timeout") = 0.5);
            }
        }
    }

private:
    struct FutureLatch {
        // Call with GIL
        static FutureLatch* create(Wrapper& hand, int waiting_count) {
            return new FutureLatch(hand, waiting_count);
        }

        // Call without GIL
        template <typename F>
        void count_down(bool success, const F& make_result) {
            if (!success)
                error_count_.fetch_add(1, std::memory_order_relaxed);

            const int old = waiting_count_.fetch_sub(1, std::memory_order_acq_rel);
            if (old == 1) {
                // This is the last completion. Safely interact with Python.
                py::gil_scoped_acquire acquire;
                const int error_count = error_count_.load(std::memory_order_relaxed);
                if (error_count) {
                    py::object timeout_error_type =
                        py::reinterpret_borrow<py::object>(PyExc_TimeoutError);
                    call_threadsafe_(
                        future_.attr("set_exception"),
                        timeout_error_type(
                            error_count == 1
                                ? "Operation timed out while waiting for completion"
                                : std::format(
                                      "{} operations timed out while waiting for completion",
                                      error_count)));
                } else {
                    call_threadsafe_(future_.attr("set_result"), make_result(wrapper));
                }
                delete this;
            }
        }

        py::object& future() { return future_; }

    private:
        explicit FutureLatch(Wrapper& hand, int waiting_count)
            : wrapper(hand)
            , waiting_count_(waiting_count)
            , error_count_(0) {
            py::object loop = py::module::import("asyncio").attr("get_event_loop")();
            future_ = loop.attr("create_future")();
            call_threadsafe_ = loop.attr("call_soon_threadsafe");
        }

        Wrapper& wrapper;

        py::object future_;
        py::object call_threadsafe_;

        std::atomic<int> waiting_count_;
        std::atomic<int> error_count_;
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
