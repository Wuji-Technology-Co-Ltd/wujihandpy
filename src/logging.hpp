#pragma once

#include <pybind11/pybind11.h>
#include <wujihandcpp/utility/logging.hpp>

namespace py = pybind11;

namespace logging {

inline void set_log_to_console(bool value) noexcept {
    wujihandcpp::logging::set_log_to_console(value);
}

inline void set_log_to_file(bool value) noexcept { wujihandcpp::logging::set_log_to_file(value); }

inline void set_log_level(wujihandcpp::logging::Level value) noexcept {
    wujihandcpp::logging::set_log_level(value);
}

inline void set_log_path(const char* value) { wujihandcpp::logging::set_log_path(value); }

inline void init_module(py::module_& m) {
    auto logging = m.def_submodule("logging");

    using wujihandcpp::logging::Level;
    py::enum_<Level>(logging, "Level")
        .value("Trace", Level::TRACE)
        .value("Debug", Level::DEBUG)
        .value("Info", Level::INFO)
        .value("Warn", Level::WARN)
        .value("Error", Level::ERR)
        .value("Critical", Level::CRITICAL)
        .value("Off", Level::OFF);

    logging.def("set_log_to_console", &set_log_to_console, py::arg("value"));
    logging.def("set_log_to_file", &set_log_to_file, py::arg("value"));
    logging.def("set_log_level", &set_log_level, py::arg("value"));
    logging.def("set_log_path", &set_log_path, py::arg("value"));

    logging.def("flush", []() { wujihandcpp::logging::flush(); });
    py::module_::import("atexit").attr("register")(logging.attr("flush"));
}

} // namespace logging