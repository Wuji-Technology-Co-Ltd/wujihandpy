#pragma once

#include <cstdint>

#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/device/controller.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/device/finger.hpp"
#include "wujihandcpp/protocol/handler.hpp"
#include "wujihandcpp/utility/logging.hpp"

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
            if (!controller_)
                return;
            try {
                hand_.detach_realtime_controller();
            } catch (...) {
                // TODO: Add log here
            }
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
            if (!controller_)
                return;
            try {
                hand_.detach_realtime_controller();
            } catch (...) {
                // TODO: Add log here
            }
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
        : handler_(usb_vid, usb_pid, serial_number, data_count()) {

        init_storage_info(mask);

        try {
            check_firmware_version();

            write<data::joint::Enabled>(false);
            Latch latch;
            write_async<data::joint::ControlMode>(latch, 6);
            write_async<data::joint::CurrentLimit>(latch, 1000);
            latch.wait();
        } catch (const TimeoutError&) {
            throw TimeoutError("Hand initialization timed out: joint configuration incomplete");
        }
    };

    void check_firmware_version() {
        Latch latch;
        read_async<data::hand::FirmwareVersion>(latch);
        read_async<data::joint::FirmwareVersion>(latch);
        latch.wait();

        auto hand_version = data::FirmwareVersionData{read<data::hand::FirmwareVersion>()};
        if (hand_version < data::FirmwareVersionData{3, 0, 0})
            throw std::runtime_error(
                "The firmware version (" + hand_version.to_string()
                + ") is outdated. Please contact after-sales service for an upgrade.");

        std::string firmware_msg =
            "Using firmware version: "
            + data::FirmwareVersionData{get<data::hand::FirmwareVersion>()}.to_string() + " & ";

        uint32_t joint_version = finger(0).joint(0).get<data::joint::FirmwareVersion>();
        bool joint_version_consistent = true;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                joint_version_consistent =
                    joint_version_consistent
                    && joint_version == finger(i).joint(j).get<data::joint::FirmwareVersion>();

        if (joint_version_consistent) {
            firmware_msg += data::FirmwareVersionData{joint_version}.to_string();
            logging::log(logging::Level::INFO, firmware_msg.c_str(), firmware_msg.size());
        } else {
            firmware_msg += "[Matrix]";
            logging::log(logging::Level::INFO, firmware_msg.c_str(), firmware_msg.size());

            std::string joint_firmware_msg;
            for (int i = 0; i < 5; i++) {
                joint_firmware_msg.clear();
                for (int j = 0; j < 4; j++) {
                    joint_firmware_msg += "  ";
                    joint_firmware_msg +=
                        data::FirmwareVersionData{
                            finger(i).joint(j).get<data::joint::FirmwareVersion>()}
                            .to_string();
                }
                logging::log(
                    logging::Level::INFO, joint_firmware_msg.c_str(), joint_firmware_msg.size());
            }

            const char warning_msg[] = "Inconsistent driver board firmware version detected";
            logging::log(logging::Level::WARN, warning_msg, sizeof(warning_msg) - 1);
        }
    }

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
            write_async<data::joint::ControlMode>(latch, 5);
            write_async<data::hand::RPdoId>(latch, 0x01);
            if (enable_upstream)
                write_async<data::hand::TPdoId>(latch, 0x01);
            else
                write_async<data::hand::TPdoId>(latch, 0x00);
            write_async<data::hand::PdoInterval>(latch, 2000);
            write_async<data::hand::PdoEnabled>(latch, 1);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);

        handler_.attach_realtime_controller(controller.get(), enable_upstream);
        auto ignore = controller.release();
        (void)ignore;
    }

    std::unique_ptr<IRealtimeController> detach_realtime_controller() {
        bool last_enabled[5][4];
        save_and_disable_joints(last_enabled);

        {
            Latch latch;
            write_async<data::joint::ControlMode>(latch, 6);
            write_async<data::hand::PdoEnabled>(latch, 0);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);

        return std::unique_ptr<IRealtimeController>{handler_.detach_realtime_controller()};
    }

    void start_latency_test() {
        bool last_enabled[5][4];
        save_and_disable_joints(last_enabled);

        {
            Latch latch;
            write_async<data::hand::RPdoId>(latch, 0xD0);
            write_async<data::hand::TPdoId>(latch, 0xD0);
            write_async<data::hand::PdoInterval>(latch, 2000);
            write_async<data::hand::PdoEnabled>(latch, 1);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);
        handler_.start_latency_test();
    }

    void stop_latency_test() {
        bool last_enabled[5][4];
        save_and_disable_joints(last_enabled);

        {
            Latch latch;
            write_async<data::hand::PdoEnabled>(latch, 0);
            latch.wait();
        }

        revert_disabled_joints(last_enabled);
        handler_.stop_latency_test();
    }

    void disable_thread_safe_check() { handler_.disable_thread_safe_check(); }

    // Raw SDO operations for debugging
    // finger_id: 0-4 for fingers, -1 for Hand level
    // joint_id: 0-3 for joints (ignored when finger_id=-1)
    std::vector<std::byte> raw_sdo_read(
        int finger_id, int joint_id, uint16_t index, uint8_t sub_index,
        std::chrono::steady_clock::duration timeout = default_timeout) {
        uint16_t full_index = index + calculate_index_offset(finger_id, joint_id);
        return handler_.raw_sdo_read(full_index, sub_index, timeout);
    }

    void raw_sdo_write(
        int finger_id, int joint_id, uint16_t index, uint8_t sub_index,
        std::span<const std::byte> data,
        std::chrono::steady_clock::duration timeout = default_timeout) {
        uint16_t full_index = index + calculate_index_offset(finger_id, joint_id);
        handler_.raw_sdo_write(full_index, sub_index, data, timeout);
    }

private:
    static uint16_t calculate_index_offset(int finger_id, int joint_id) {
        if (finger_id < 0)
            return 0x0000; // Hand level
        if (finger_id > 4)
            throw std::invalid_argument("finger_id must be -1 to 4");
        if (joint_id < 0 || joint_id > 3)
            throw std::invalid_argument("joint_id must be 0 to 3");
        return static_cast<uint16_t>(0x2000 + finger_id * 0x800 + joint_id * 0x100);
    }

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
