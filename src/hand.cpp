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
        auto context = new FutureContext{*this, data_count<Data>()};
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
    void read_async_unchecked() {
        hand_.read_async_unchecked<Data>();
    }

    template <typename Data>
    auto write(py::numpy_scalar<typename Data::ValueType> value) {
        py::gil_scoped_release release;
        hand_.write<Data>(value.value);
    }

    template <typename Data>
    py::object write_async(py::numpy_scalar<typename Data::ValueType> value) {
        auto context = new FutureContext{*this, data_count<Data>()};
        hand_.write_async<Data>(
            [context]() {
                if (context->count_down()) {
                    py::gil_scoped_acquire acquire;
                    context->call_threadsafe(context->future.attr("set_result"), py::none());
                    delete context;
                }
            },
            value.value);
        hand_.trigger_transmission();

        return context->future;
    }

    template <typename Data>
    void write_async_unchecked(py::numpy_scalar<typename Data::ValueType> value) {
        hand_.write_async_unchecked<Data>(value.value);
    }

    void trigger_transmission() { hand_.trigger_transmission(); }

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
        if constexpr (Data::readable) {
            py_class.def(("read_" + name).c_str(), &Hand::read<Data>);
            py_class.def(("read_" + name + "_async").c_str(), &Hand::read_async<Data>);
            py_class.def(
                ("read_" + name + "_async_unchecked").c_str(), &Hand::read_async_unchecked<Data>);
            py_class.def(("get_" + name).c_str(), &Hand::get<Data>);
        }
        if constexpr (Data::writable) {
            py_class.def(("write_" + name).c_str(), &Hand::write<Data>);
            py_class.def(("write_" + name + "_async").c_str(), &Hand::write_async<Data>);
            py_class.def(
                ("write_" + name + "_async_unchecked").c_str(), &Hand::write_async_unchecked<Data>);
        }
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

    template <typename Data>
    static constexpr int data_count() {
        if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Hand>)
            return 1;
        else if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Finger>)
            return 4;
        else if constexpr (std::is_same_v<typename Data::Base, wujihandcpp::device::Joint>)
            return 5 * 4;
    };

    wujihandcpp::device::Hand hand_;
};

PYBIND11_MODULE(wujihandpy, m) {
    m.attr("__name__") = "wujihandpy";

    auto hand = py::class_<Hand>(m, "Hand");
    hand.def(
        py::init<uint16_t, int32_t>(), py::arg("usb_vid") = 0x0483, py::arg("usb_pid") = 0x5740);

    using namespace wujihandcpp;
    Hand::register_py_interface<data::hand::FirmwareVersion>(hand, "firmware_version");
    Hand::register_py_interface<data::hand::FirmwareDate>(hand, "firmware_date");
    Hand::register_py_interface<data::hand::SystemTime>(hand, "system_time");
    Hand::register_py_interface<data::hand::McuTemperature>(hand, "mcu_temperature");
    Hand::register_py_interface<data::hand::InputVoltage>(hand, "input_voltage");

    Hand::register_py_interface<data::hand::PdoEnabled>(hand, "pdo_enabled");
    Hand::register_py_interface<data::hand::GlobalTpdoId>(hand, "global_tpdo_id");
    Hand::register_py_interface<data::hand::JointPdoInterval>(hand, "joint_pdo_interval");

    Hand::register_py_interface<data::joint::ControlMode>(hand, "joint_control_mode");
    Hand::register_py_interface<data::joint::SinLevel>(hand, "joint_sin_level");
    Hand::register_py_interface<data::joint::ControlWord>(hand, "joint_control_word");
    Hand::register_py_interface<data::joint::Position>(hand, "joint_position");
    Hand::register_py_interface<data::joint::ControlPosition>(hand, "joint_control_position");

    hand.def("trigger_transmission", &Hand::trigger_transmission);

#define STRINGIFY(x) #x
#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
