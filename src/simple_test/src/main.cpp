#include <cmath>
#include <csignal>

#include <atomic>
#include <thread>

#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/device/waitable.hpp>
#include <wujihandcpp/utility/fps_counter.hpp>

using namespace wujihandcpp;

int main() {
    static std::atomic<bool> running = true;
    std::signal(SIGINT, [](int) { running.store(false, std::memory_order::relaxed); });

    using namespace std::chrono_literals;
    device::Hand hand{0x0483, 0x5740};

    // Set control mode
    hand.write_data<data::joint::ControlMode>(2);

    // Enable whole hand
    hand.write_data<data::joint::ControlWord>(1);

    // Some older firmware require this for enabling.
    if (false) {
        hand.write_data<data::joint::SinLevel>(5);
        hand.write_data<data::joint::SinLevel>(0);
    }

    // Return all joints to initial point
    device::Waitable waitable;
    using ControlPosition = data::joint::ControlPosition;
    hand.finger(0).joint(0).write_data_async<ControlPosition>(waitable, 0x200000);
    hand.finger(0).joint(1).write_data_async<ControlPosition>(waitable, 0x200000);
    hand.finger(0).joint(2).write_data_async<ControlPosition>(waitable, 0x200000);
    hand.finger(0).joint(3).write_data_async<ControlPosition>(waitable, 0x200000);
    hand.finger(1).joint(1).write_data_async<ControlPosition>(waitable, 0xBFFFFF);
    hand.finger(2).joint(1).write_data_async<ControlPosition>(waitable, 0x900000);
    hand.finger(3).joint(1).write_data_async<ControlPosition>(waitable, 0x600000);
    hand.finger(4).joint(1).write_data_async<ControlPosition>(waitable, 0x400000);
    for (int i = 1; i < 5; i++) {
        hand.finger(i).joint(0).write_data_async<ControlPosition>(waitable, 0xFFFFFF);
        hand.finger(i).joint(2).write_data_async<ControlPosition>(waitable, 0x000000);
        hand.finger(i).joint(3).write_data_async<ControlPosition>(waitable, 0x000000);
    }
    hand.trigger_transmission();
    waitable.wait();

    // Wait for joints to move into place
    std::this_thread::sleep_for(500ms);

    // Disable the thumb
    hand.finger(0).write_data<data::joint::ControlWord>(5);

    // 1kHz SDO Control
    using namespace std::chrono_literals;
    constexpr double update_rate = 1000.0;
    constexpr auto update_period =
        std::chrono::round<std::chrono::steady_clock::duration>(1.0s / update_rate);

    auto next_iteration_time = std::chrono::steady_clock::now();
    utility::FpsCounter fps_counter;
    double x = 0;
    while (running.load(std::memory_order::relaxed)) {
        if (fps_counter.count())
            std::cout << "SDO Control Actual Fps: " << fps_counter.fps() << '\n';

        x += 0.005;
        double y = (std::cos(x) + 1) / 2;
        uint32_t position = static_cast<uint32_t>(std::round(double(0xFFFFFF) * y));
        for (int i = 1; i < 5; i++) {
            hand.finger(i).joint(0).write_data_async_unchecked<ControlPosition>(position);
            hand.finger(i).joint(2).write_data_async_unchecked<ControlPosition>(
                0xFFFFFF - position);
            hand.finger(i).joint(3).write_data_async_unchecked<ControlPosition>(
                0xFFFFFF - position);
        }
        hand.trigger_transmission();

        next_iteration_time += update_period;
        std::this_thread::sleep_until(next_iteration_time);
    }

    // Disable the entire hand
    hand.write_data<data::joint::ControlWord>(5);
    std::cout << "Program exited correctly.\n";
}