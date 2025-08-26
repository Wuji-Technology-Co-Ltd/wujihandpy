#include <atomic>
#include <cstdint>

#include <iostream>
#include <type_traits>

#include <pybind11/cast.h>
#include <pybind11/gil.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/data/joint.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/device/joint.hpp>

namespace py = pybind11;

class Hand {
public:
    Hand(uint16_t usb_vid, int32_t usb_pid)
        : hand_(usb_vid, usb_pid) {};

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, wujihandcpp::device::Hand>) auto read() {
        py::gil_scoped_release release;
        return py::numpy_scalar{hand_.read<Data>()};
    }

    template <typename Data>
    requires(!std::is_same_v<typename Data::Base, wujihandcpp::device::Hand>) auto read() {
        {
            py::gil_scoped_release release;
            hand_.read<Data>();
        }
        return get<Data>();
    }

    template <typename Data>
    py::object read_async() {
        constexpr auto data_count = []() constexpr {
            if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Hand>)
                return 1;
            else if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Finger>)
                return 4;
            else if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
                return 5 * 4;
        };

        auto context = new FutureContext{*this, data_count()};
        hand_.read_async<Data>([context](Data::ValueType) {
            if (context->count_down()) {
                py::gil_scoped_acquire acquire;
                context->call_threadsafe(
                    context->future.attr("set_result"), context->hand.get<Data>());
                delete context;
            }
        });
        hand_.trigger_transmission();

        return context->future;
    }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, wujihandcpp::device::Hand>) auto get() {
        return py::numpy_scalar{hand_.get<Data>()};
    }

    template <typename Data>
    requires(std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>) auto get() {
        using ValueType = Data::ValueType;
        auto buffer = new ValueType[5 * 4];
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                buffer[4 * i + j] = hand_.finger(i).joint(j).get<Data>();

        py::capsule free(buffer, [](void* ptr) { delete[] static_cast<ValueType*>(ptr); });

        return py::array_t<ValueType>({5, 4}, buffer, free);
    }

    template <typename Data>
    static void register_py_interface(py::class_<Hand>& py_class, const std::string& name) {
        py_class.def(("read_" + name).c_str(), &Hand::read<Data>);
        py_class.def(("read_" + name + "_async").c_str(), &Hand::read_async<Data>);
    }

private:
    struct FutureContext {
        explicit FutureContext(Hand& hand, int count)
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

        Hand& hand;
        std::atomic<int> count;

        py::object call_threadsafe;
        py::object future;
    };

    wujihandcpp::device::Hand hand_;
};

PYBIND11_MODULE(wujihandpy, m) {
    m.attr("__name__") = "wujihandpy";

    auto hand = py::class_<Hand>(m, "Hand");
    hand.def(
        py::init<uint16_t, int32_t>(), py::arg("usb_vid") = 0x0483, py::arg("usb_pid") = 0x5740);
        
    Hand::register_py_interface<wujihandcpp::data::hand::FirmwareVersion>(hand, "firmware_version");
    Hand::register_py_interface<wujihandcpp::data::hand::FirmwareDate>(hand, "firmware_date");
    Hand::register_py_interface<wujihandcpp::data::hand::SystemTime>(hand, "system_time");
    Hand::register_py_interface<wujihandcpp::data::hand::McuTemperature>(hand, "mcu_temperature");
    Hand::register_py_interface<wujihandcpp::data::hand::InputVoltage>(hand, "input_voltage");

    Hand::register_py_interface<wujihandcpp::data::joint::Position>(hand, "joint_position");

#define STRINGIFY(x) #x
#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
