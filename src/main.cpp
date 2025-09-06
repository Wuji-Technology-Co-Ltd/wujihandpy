#include <pybind11/pybind11.h>
#include <wujihandcpp/device/finger.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/device/joint.hpp>

#include "wrapper.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    using namespace wujihandcpp;

    using Hand = Wrapper<wujihandcpp::device::Hand>;
    auto hand = py::class_<Hand>(m, "Hand");
    hand.def(
        py::init<uint16_t, int32_t>(), py::arg("usb_vid") = 0x0483, py::arg("usb_pid") = 0x7530);

    Hand::register_py_interface<data::hand::FirmwareVersion>(hand, "firmware_version");
    Hand::register_py_interface<data::hand::FirmwareDate>(hand, "firmware_date");
    Hand::register_py_interface<data::hand::SystemTime>(hand, "system_time");
    Hand::register_py_interface<data::hand::McuTemperature>(hand, "mcu_temperature");
    Hand::register_py_interface<data::hand::InputVoltage>(hand, "input_voltage");

    Hand::register_py_interface<data::hand::PdoEnabled>(hand, "pdo_enabled");
    Hand::register_py_interface<data::hand::GlobalTpdoId>(hand, "global_tpdo_id");
    Hand::register_py_interface<data::hand::JointPdoInterval>(hand, "pdo_interval");

    Hand::register_py_interface<data::joint::ControlMode>(hand, "joint_control_mode");
    Hand::register_py_interface<data::joint::SinLevel>(hand, "joint_sin_level");
    Hand::register_py_interface<data::joint::CurrentLimit>(hand, "joint_current_limit");
    Hand::register_py_interface<data::joint::ControlWord>(hand, "joint_control_word");
    Hand::register_py_interface<data::joint::Position>(hand, "joint_position");
    Hand::register_py_interface<data::joint::ControlPosition>(hand, "joint_control_position");
    Hand::register_py_interface<data::joint::UpperLimit>(hand, "joint_upper_limit");
    Hand::register_py_interface<data::joint::LowerLimit>(hand, "joint_lower_limit");

    hand.def(
        "pdo_write_unchecked",
        py::overload_cast<py::numpy_scalar<double>>(&Hand::pdo_write_unchecked));
    hand.def(
        "pdo_write_unchecked",
        py::overload_cast<const py::array_t<double>&>(&Hand::pdo_write_unchecked));

    using Finger = Wrapper<wujihandcpp::device::Finger>;
    auto finger = py::class_<Finger>(m, "Finger");
    hand.def("finger", &Hand::finger, py::arg("index"));

    Finger::register_py_interface<data::joint::ControlMode>(finger, "joint_control_mode");
    Finger::register_py_interface<data::joint::SinLevel>(finger, "joint_sin_level");
    Finger::register_py_interface<data::joint::CurrentLimit>(finger, "joint_current_limit");
    Finger::register_py_interface<data::joint::ControlWord>(finger, "joint_control_word");
    Finger::register_py_interface<data::joint::Position>(finger, "joint_position");
    Finger::register_py_interface<data::joint::ControlPosition>(finger, "joint_control_position");
    Finger::register_py_interface<data::joint::UpperLimit>(finger, "joint_upper_limit");
    Finger::register_py_interface<data::joint::LowerLimit>(finger, "joint_lower_limit");

    using Joint = Wrapper<wujihandcpp::device::Joint>;
    auto joint = py::class_<Joint>(m, "Joint");
    finger.def("joint", &Finger::joint, py::arg("index"));

    Joint::register_py_interface<data::joint::ControlMode>(joint, "joint_control_mode");
    Joint::register_py_interface<data::joint::SinLevel>(joint, "joint_sin_level");
    Joint::register_py_interface<data::joint::CurrentLimit>(joint, "joint_current_limit");
    Joint::register_py_interface<data::joint::ControlWord>(joint, "joint_control_word");
    Joint::register_py_interface<data::joint::Position>(joint, "joint_position");
    Joint::register_py_interface<data::joint::ControlPosition>(joint, "joint_control_position");
    Joint::register_py_interface<data::joint::UpperLimit>(joint, "joint_upper_limit");
    Joint::register_py_interface<data::joint::LowerLimit>(joint, "joint_lower_limit");
}
