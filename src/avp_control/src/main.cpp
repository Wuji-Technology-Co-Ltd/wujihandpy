#include <atomic>
#include <cmath>
#include <csignal>

#include <algorithm>
#include <cstdint>
#include <numbers>

#include <rclcpp/utilities.hpp>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/device/hand.hpp>

#include "handtracking_client.hpp"
#include "low_pass_filter.hpp"
#include "wujihandcpp/utility/fps_counter.hpp"

class Controller {
public:
    explicit Controller(const std::string& target)
        : hand_{0x0483, 0x5740}
        , client_(target, false) {
        control_positions_[0][0] = 0x200000;
        control_positions_[0][1] = 0x200000;
        control_positions_[0][2] = 0x200000;
        control_positions_[0][3] = 0x200000;
    }

    void run() {
        client_.wait_data_ready();
        std::cout << "AVP Stream connected.\n";

        hand_.write_data<data::hand::finger::joint::ControlMode>(4);
        hand_.write_data<data::hand::GlobalTpdoId>(1);
        hand_.write_data<data::hand::JointPdoInterval>(950);
        hand_.write_data<data::hand::PdoEnabled>(1);
        hand_.write_data<data::hand::finger::joint::ControlWord>(1);

        hand_.finger(0).write_data<data::hand::finger::joint::ControlWord>(5);
        for (int i = 1; i < 5; i++)
            hand_.finger(i).joint(1).write_data<data::hand::finger::joint::ControlWord>(5);

        using namespace std::chrono_literals;
        constexpr double update_rate = 1000.0;
        constexpr auto update_period =
            std::chrono::round<std::chrono::steady_clock::duration>(1.0s / update_rate);

        auto next_iteration_time = std::chrono::steady_clock::now();
        running.store(true, std::memory_order::relaxed);
        while (running.load(std::memory_order::relaxed)) {
            if (fps_counter.count())
                std::cout << "AVP Control Actual Fps: " << fps_counter.fps() << '\n';
            update();
            next_iteration_time += update_period;
            std::this_thread::sleep_until(next_iteration_time);
        }

        hand_.write_data<data::hand::finger::joint::ControlWord>(5);
        std::cout << "AVP Control exited correctly." << std::endl;
    }

    void stop() { running.store(false, std::memory_order::relaxed); }

private:
    void update() {
        update_filter();

        constexpr double deg2rad = std::numbers::pi / 180.0;
        for (int i = 1; i < 5; i++) {
            control_positions_[i][0] =
                to_raw_angle(filter_[i][0].value(), -15 * deg2rad, (-15 + 90) * deg2rad, true);
            control_positions_[i][1] = 0x8FFFFF;
            control_positions_[i][2] =
                to_raw_angle(filter_[i][2].value(), -15 * deg2rad, (-15 + 90) * deg2rad, false);
            control_positions_[i][3] =
                to_raw_angle(filter_[i][3].value(), -10 * deg2rad, (-10 + 90) * deg2rad, false);
        }
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin_time_);
        hand_.write_pdo_async(control_positions_, static_cast<uint32_t>(duration.count()));
    }

    void update_filter() {
        const auto& joint_angles = client_.joint_angles();
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 4; j++)
                filter_[i][j].update(joint_angles[i][j].load(std::memory_order::relaxed), alpha_);
    }

    static constexpr int32_t to_raw_angle(double angle, double lower, double upper, bool reversed) {
        constexpr int32_t raw_angle_max = 0xFFFFFF;
        angle = (angle - lower) / (upper - lower) * double(raw_angle_max);
        angle = std::clamp(angle, 0.0, double(raw_angle_max));
        auto raw_angle = static_cast<int32_t>(std::round(angle));
        if (reversed)
            raw_angle = raw_angle_max - raw_angle;
        return raw_angle;
    }

    device::Hand hand_;
    HandtrackingClient client_;

    std::atomic<bool> running = false;
    utility::FpsCounter fps_counter;

    static constexpr double alpha_ = LowPassFilter::calculate_alpha(4, 1000);
    LowPassFilter filter_[5][4];

    std::chrono::steady_clock::time_point begin_time_;
    int32_t control_positions_[5][4];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    static auto controller = Controller("192.168.10.123:12345");
    std::signal(SIGINT, [](int) {
        controller.stop();

        static bool first = true;
        if (!first) {
            rclcpp::shutdown();
            std::terminate();
        }
        first = false;
    });

    controller.run();
}