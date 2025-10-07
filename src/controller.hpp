#pragma once

#include <optional>

#include <pybind11/numpy.h>
#include <wujihandcpp/device/hand.hpp>

namespace py = pybind11;

class IController {
public:
    IController() = default;

    IController(const IController&) = delete;
    IController& operator=(const IController&) = delete;
    IController(IController&&) = delete;
    IController& operator=(IController&&) = delete;

    virtual ~IController() = default;

    virtual py::array_t<double> get_joint_position() = 0;

    virtual void set_joint_control_position(const py::array_t<double>&) = 0;

    virtual void close() = 0;
};

namespace internal {
template <bool enable_upstream, typename T>
inline std::unique_ptr<IController>
    create_controller_helper_impl(wujihandcpp::device::Hand& hand, const T& filter) {
    class Controller : public IController {
        using ControllerT = decltype(hand.realtime_controller<enable_upstream>(std::declval<T>()));

    public:
        explicit Controller(ControllerT&& controller) noexcept
            : controller_(std::move(controller)) {}

        py::array_t<double> get_joint_position() override {
            if (!controller_)
                throw std::runtime_error("Controller is closed.");

            if constexpr (!enable_upstream)
                throw std::logic_error("Upstream is disabled.");
            else {
                const auto& positions = controller_->get_joint_position();

                auto buffer = new double[5 * 4];
                for (size_t i = 0; i < 5; i++)
                    for (size_t j = 0; j < 4; j++)
                        buffer[4 * i + j] = positions[i][j].load(std::memory_order::relaxed);
                py::capsule free(buffer, [](void* ptr) { delete[] static_cast<double*>(ptr); });

                return py::array_t<double>({5, 4}, buffer, free);
            }
        }

        void set_joint_control_position(const py::array_t<double>& array) override {
            if (!controller_)
                throw std::runtime_error("Controller is closed.");

            if (array.ndim() != 2 || array.shape()[0] != 5 || array.shape()[1] != 4)
                throw std::runtime_error("Array shape must be {5, 4}!");

            auto r = array.unchecked<2>();
            double control_positions[5][4];
            for (size_t i = 0; i < 5; ++i)
                for (size_t j = 0; j < 4; ++j)
                    control_positions[i][j] = r(i, j);

            controller_->set_joint_control_position(control_positions);
        }

        void close() override { controller_ = std::nullopt; }

    private:
        std::optional<ControllerT> controller_;
    };

    return std::make_unique<Controller>(hand.realtime_controller<enable_upstream>(filter));
}
} // namespace internal

template <typename T>
inline std::unique_ptr<IController> create_controller_helper(
    wujihandcpp::device::Hand& hand, bool enable_upstream, const T& filter) {
    if (enable_upstream)
        return internal::create_controller_helper_impl<true>(hand, filter);
    else {
        return internal::create_controller_helper_impl<false>(hand, filter);
    }
}