#pragma once

#include <cstdint>

#include <memory>
#include <stdexcept>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/device/controller.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/device/finger.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {
namespace device {

class Hand : public DataOperator<Hand> {
    friend class DataOperator;

    template <typename FilterT, bool upstream_enabled>
    class ControllerOperator;

    template <typename FilterT>
    class ControllerOperator<FilterT, false> final {
    public:
        explicit ControllerOperator(Hand& hand, FilteredController<FilterT, false>& controller)
            : hand_(hand)
            , controller_(&controller) {}

        ControllerOperator(const ControllerOperator&) = delete;
        ControllerOperator& operator=(const ControllerOperator&) = delete;

        ControllerOperator(ControllerOperator&& other) noexcept
            : hand_(other.hand_)
            , controller_(other.controller_) {
            other.controller_ = nullptr;
        }
        ControllerOperator& operator=(ControllerOperator&&) = delete;

        ~ControllerOperator() {
            if (controller_)
                hand_.detach_realtime_controller();
        }

        void set_joint_target_position(const double (&positions)[5][4]) {
            controller_->set(positions);
        }

    private:
        Hand& hand_;
        FilteredController<FilterT, false>* controller_;
    };

    template <typename FilterT>
    class ControllerOperator<FilterT, true> final {
    public:
        explicit ControllerOperator(Hand& hand, FilteredController<FilterT, true>& controller)
            : hand_(hand)
            , controller_(&controller) {}

        ControllerOperator(const ControllerOperator&) = delete;
        ControllerOperator& operator=(const ControllerOperator&) = delete;

        ControllerOperator(ControllerOperator&& other) noexcept
            : hand_(other.hand_)
            , controller_(other.controller_) {
            other.controller_ = nullptr;
        }
        ControllerOperator& operator=(ControllerOperator&&) = delete;

        ~ControllerOperator() {
            if (controller_)
                hand_.detach_realtime_controller();
        }

        auto get_joint_actual_position() -> const std::atomic<double> (&)[5][4] {
            return controller_->get();
        }

        void set_joint_target_position(const double (&positions)[5][4]) {
            controller_->set(positions);
        }

    private:
        Hand& hand_;
        FilteredController<FilterT, true>* controller_;
    };

public:
    explicit Hand(
        const char* serial_number = nullptr, int32_t usb_pid = -1, uint16_t usb_vid = 0x0483,
        uint32_t mask = 0)
        : handler_(usb_vid, usb_pid, serial_number, 64, data_count()) {

        init_storage_info(mask);

        write<data::joint::Enabled>(false);
        Latch latch;
        write_async<data::joint::ControlMode>(latch, 2);
        write_async<data::joint::CurrentLimit>(latch, 1000);
        latch.wait();
    };

    Finger finger_thumb() { return finger(0); }
    Finger finger_index() { return finger(1); }
    Finger finger_middle() { return finger(2); }
    Finger finger_ring() { return finger(3); }
    Finger finger_little() { return finger(4); }

    Finger finger(int index) {
        if (index < 0 || index >= sub_count_)
            throw std::runtime_error("Index out of bounds! Possible values: 0, 1, 2, 3, 4.");
        return sub(index);
    }

    template <bool enable_upstream, typename FilterT>
    SDK_CPP20_REQUIRES(requires(const FilterT& f, typename FilterT::Unit& u, double v) {
        { u.reset(f, v) };
        { u.input(f, v) };
        { u.step(f) } -> std::convertible_to<double>;
    })
    auto realtime_controller(const FilterT& filter)
        -> ControllerOperator<FilterT, enable_upstream> {
        bool last_enabled[5][4];
        save_and_enable_joints(last_enabled);
        read<data::joint::ActualPosition>();
        revert_enabled_joints(last_enabled);

        double positions[5][4];
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                positions[i][j] = finger(i).joint(j).get<data::joint::ActualPosition>();

        auto controller =
            std::make_unique<FilteredController<FilterT, enable_upstream>>(positions, filter);
        auto controller_operator = ControllerOperator<FilterT, enable_upstream>(*this, *controller);
        attach_realtime_controller(std::move(controller), enable_upstream);

        return controller_operator;
    }

    void attach_realtime_controller(
        std::unique_ptr<IRealtimeController> controller, bool enable_upstream) {
        if (!controller)
            throw std::invalid_argument("Controller pointer must not be null.");

        bool last_enabled[5][4];
        save_and_disable_joints(last_enabled);

        {
            Latch latch;
            write_async<data::joint::ControlMode>(latch, 4);
            if (enable_upstream) {
                write_async<data::hand::TPdoId>(latch, 257);
                write_async<data::hand::PdoInterval>(latch, 2000);
                write_async<data::hand::TPdoTriggerOffset>(latch, 1000);
            } else {
                write_async<data::hand::TPdoId>(latch, 1);
                write_async<data::hand::PdoInterval>(latch, 1000);
            }
            write_async<data::hand::PdoEnabled>(latch, 1);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);

        handler_.attach_realtime_controller(controller.release(), enable_upstream);
    }

    std::unique_ptr<IRealtimeController> detach_realtime_controller() {
        bool last_enabled[5][4];
        save_and_disable_joints(last_enabled);

        {
            Latch latch;
            write_async<data::joint::ControlMode>(latch, 2);
            write_async<data::hand::PdoEnabled>(latch, 0);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);

        return std::unique_ptr<IRealtimeController>{handler_.detach_realtime_controller()};
    }

    void disable_thread_safe_check() { handler_.disable_thread_safe_check(); }

private:
    void save_and_enable_joints(bool (&last_enabled)[5][4]) {
        Latch latch;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++) {
                auto joint = finger(i).joint(j);
                last_enabled[i][j] = joint.get<data::joint::Enabled>();
                if (!last_enabled[i][j])
                    joint.write_async<data::joint::Enabled>(latch, true);
            }
        latch.wait();
    }

    void revert_enabled_joints(const bool (&last_enabled)[5][4]) {
        Latch latch;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                if (!last_enabled[i][j])
                    finger(i).joint(j).write_async<data::joint::Enabled>(latch, false);
        latch.wait();
    }

    void save_and_disable_joints(bool (&last_enabled)[5][4]) {
        Latch latch;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++) {
                auto joint = finger(i).joint(j);
                last_enabled[i][j] = joint.get<data::joint::Enabled>();
                if (last_enabled[i][j])
                    joint.write_async<data::joint::Enabled>(latch, false);
            }
        latch.wait();
    }

    void revert_disabled_joints(const bool (&last_enabled)[5][4]) {
        Latch latch;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                if (last_enabled[i][j])
                    finger(i).joint(j).write_async<data::joint::Enabled>(latch, true);
        latch.wait();
    }

    using Datas = DataTuple<
        data::hand::Handedness, data::hand::FirmwareVersion, data::hand::FirmwareDate,
        data::hand::SystemTime, data::hand::Temperature, data::hand::InputVoltage,
        data::hand::PdoEnabled, data::hand::RPdoId, data::hand::TPdoId, data::hand::PdoInterval,
        data::hand::RPdoTriggerOffset, data::hand::TPdoTriggerOffset>;

    protocol::Handler handler_;

    static constexpr uint16_t index_offset_ = 0x0000;
    static constexpr int storage_offset_ = 0;

    using Sub = Finger;
    static constexpr int sub_count_ = 5;
    Sub sub(int index) {
        return {
            handler_, uint16_t(0x2000 + index * 0x800),
            int(Datas::count + index * Sub::data_count())};
    }
};

} // namespace device
} // namespace wujihandcpp