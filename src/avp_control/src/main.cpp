#include <cmath>
#include <csignal>

#include <algorithm>
#include <cstdint>
#include <numbers>

#include <rclcpp/utilities.hpp>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/device/hand.hpp>

#include "handtracking_client.hpp"

class Controller : HandtrackingClient {
public:
    explicit Controller(const std::string& target)
        : HandtrackingClient(target)
        , hand_{0x0483, 0x5740} {}

    void run() {
        hand_.finger(1).write_data<data::hand::finger::joint::ControlWord>(1);
        HandtrackingClient::stream_hand_updates();
    }

private:
    void joint_angles_update_callback(const double (&joint_angles)[5][4]) override {
        constexpr double deg2rad = std::numbers::pi / 180.0;
        hand_.finger(1).joint(0).write_data_async<data::hand::finger::joint::ControlPosition>(
            to_raw_angle(joint_angles[1][0], -15 * deg2rad, (-15 + 90) * deg2rad, true));
        hand_.finger(1).joint(2).write_data_async<data::hand::finger::joint::ControlPosition>(
            to_raw_angle(joint_angles[1][2], -15 * deg2rad, (-15 + 90) * deg2rad, false));
        hand_.finger(1).joint(3).write_data_async<data::hand::finger::joint::ControlPosition>(
            to_raw_angle(joint_angles[1][3], -10 * deg2rad, (-10 + 90) * deg2rad, false));
        hand_.trigger_transmission();
    }

    static constexpr uint32_t
        to_raw_angle(double angle, double lower, double upper, bool reversed) {
        constexpr uint32_t raw_angle_max = 0xFFFFFF;
        angle = (angle - lower) / (upper - lower) * double(raw_angle_max);
        angle = std::clamp(angle, 0.0, double(raw_angle_max));
        uint32_t raw_angle = static_cast<uint32_t>(std::round(angle));
        if (reversed)
            raw_angle = raw_angle_max - raw_angle;
        return raw_angle;
    }

    device::Hand hand_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
        std::terminate();
    });
    auto controller = Controller("192.168.10.123:12345");
    controller.run();
}