#include <type_traits>

#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <wujihandcpp/device/finger.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/device/joint.hpp>

#include "controller.hpp"
#include "filter.hpp"
#include "wrapper.hpp"

namespace py = pybind11;

template <typename Data>
void register_py_interface(const std::string&) {}

template <typename Data, typename T, typename... Others>
void register_py_interface(const std::string& name, py::class_<T>& py_class, Others&... others) {
    T::template register_py_interface<Data>(
        py_class,
        std::is_same_v<typename Data::Base, wujihandcpp::device::Joint> ? "joint_" + name : name);
    register_py_interface<Data>(name, others...);
}

PYBIND11_MODULE(_core, m) {
    py::class_<IController>(m, "IController")
        .def("__enter__", [](IController& self) -> IController& { return self; })
        .def(
            "__exit__", [](IController& self, const py::object&, const py::object&,
                           const py::object&) { self.close(); })
        .def("close", &IController::close)
        .def("get_joint_actual_position", &IController::get_joint_actual_position)
        .def(
            "set_joint_target_position", &IController::set_joint_target_position,
            py::arg("value_array"));

    filter::init_module(m);

    using namespace wujihandcpp;

    using Hand = Wrapper<wujihandcpp::device::Hand>;
    auto hand = py::class_<Hand>(m, "Hand");
    hand.def(
        py::init<std::optional<std::string>, int32_t, uint16_t, std::optional<py::array_t<bool>>>(),
        py::arg("serial_number") = py::none(), py::arg("usb_pid") = -1, py::arg("usb_vid") = 0x0483,
        py::arg("mask") = py::none());

    register_py_interface<data::hand::Handedness>("handedness", hand);
    register_py_interface<data::hand::FirmwareVersion>("firmware_version", hand);
    register_py_interface<data::hand::FirmwareDate>("firmware_date", hand);
    register_py_interface<data::hand::SystemTime>("system_time", hand);
    register_py_interface<data::hand::Temperature>("temperature", hand);
    register_py_interface<data::hand::InputVoltage>("input_voltage", hand);

    hand.def(
        "realtime_controller", &Hand::realtime_controller, py::arg("enable_upstream"),
        py::arg("filter"), py::keep_alive<0, 1>());

    using Finger = Wrapper<wujihandcpp::device::Finger>;
    auto finger = py::class_<Finger>(m, "Finger");
    hand.def("finger", &Hand::finger, py::arg("index"), py::keep_alive<0, 1>());

    using Joint = Wrapper<wujihandcpp::device::Joint>;
    auto joint = py::class_<Joint>(m, "Joint");
    finger.def("joint", &Finger::joint, py::arg("index"), py::keep_alive<0, 1>());

    register_py_interface<data::joint::HardwareVersion>("hardware_version", hand, finger, joint);
    register_py_interface<data::joint::HardwareDate>("hardware_date", hand, finger, joint);
    register_py_interface<data::joint::ControlMode>("control_mode", hand, finger, joint);
    register_py_interface<data::joint::SinLevel>("sin_level", hand, finger, joint);
    register_py_interface<data::joint::CurrentLimit>("current_limit", hand, finger, joint);
    register_py_interface<data::joint::BusVoltage>("bus_voltage", hand, finger, joint);
    register_py_interface<data::joint::Temperature>("temperature", hand, finger, joint);
    register_py_interface<data::joint::ResetError>("reset_error", hand, finger, joint);
    register_py_interface<data::joint::ErrorCode>("error_code", hand, finger, joint);
    register_py_interface<data::joint::Enabled>("enabled", hand, finger, joint);
    register_py_interface<data::joint::ActualPosition>("actual_position", hand, finger, joint);
    register_py_interface<data::joint::TargetPosition>("target_position", hand, finger, joint);
    register_py_interface<data::joint::UpperLimit>("upper_limit", hand, finger, joint);
    register_py_interface<data::joint::LowerLimit>("lower_limit", hand, finger, joint);
}
