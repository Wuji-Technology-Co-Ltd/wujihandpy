#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <wujihandcpp/filter/low_pass.hpp>

#include "controller.hpp"

namespace py = pybind11;

namespace filter {

class IFilter {
public:
    virtual ~IFilter() = default;

    virtual std::unique_ptr<IController>
        create_controller(wujihandcpp::device::Hand& hand, bool enable_upstream) const = 0;
};

class LowPass : public IFilter {
public:
    explicit LowPass(double cutoff_freq) noexcept
        : cutoff_freq_(cutoff_freq) {}

    std::unique_ptr<IController>
        create_controller(wujihandcpp::device::Hand& hand, bool enable_upstream) const override {
        return create_controller_helper(
            hand, enable_upstream, wujihandcpp::filter::LowPass{cutoff_freq_});
    }

    const double cutoff_freq_;
};

inline void init_module(py::module_& m) {
    auto filter = m.def_submodule("filter");

    (void)py::class_<IFilter>(filter, "IFilter");

    py::class_<LowPass, IFilter>(filter, "LowPass")
        .def(py::init<double>(), py::arg("cutoff_freq") = 10.0);
}

} // namespace filter