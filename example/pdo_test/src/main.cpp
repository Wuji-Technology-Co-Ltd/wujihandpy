#include <cmath>
#include <csignal>
#include <cstdint>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/device/latch.hpp>
#include <wujihandcpp/utility/fps_counter.hpp>

using namespace wujihandcpp;

int main() {
    static std::atomic<bool> running{true};
    std::signal(SIGINT, [](int) { running.store(false, std::memory_order_relaxed); });

    device::Hand hand{0x0483, 0x5740};

    // Set control mode & enable whole hand
    hand.write<data::joint::ControlMode>(2);
    hand.write<data::joint::ControlWord>(1);

    // Calculate initial control position
    hand.read<data::joint::Position>();
    double sum = 0;
    for (int i = 1; i < 5; i++)
        sum += double(hand.finger(i).joint(0).get<data::joint::Position>());
    auto initial = static_cast<int32_t>(std::round(sum / 4));

    // Return all joints to initial point
    device::Latch latch;
    using ControlPosition = data::joint::ControlPosition;
    hand.finger(0).joint(0).write_async<ControlPosition>(latch, 0x200000);
    hand.finger(0).joint(1).write_async<ControlPosition>(latch, 0x200000);
    hand.finger(0).joint(2).write_async<ControlPosition>(latch, 0x200000);
    hand.finger(0).joint(3).write_async<ControlPosition>(latch, 0x200000);
    hand.finger(1).joint(1).write_async<ControlPosition>(latch, 0xBFFFFF);
    hand.finger(2).joint(1).write_async<ControlPosition>(latch, 0x900000);
    hand.finger(3).joint(1).write_async<ControlPosition>(latch, 0x600000);
    hand.finger(4).joint(1).write_async<ControlPosition>(latch, 0x400000);
    for (int i = 1; i < 5; i++) {
        hand.finger(i).joint(0).write_async<ControlPosition>(latch, initial);
        hand.finger(i).joint(2).write_async<ControlPosition>(latch, 0xFFFFFF - initial);
        hand.finger(i).joint(3).write_async<ControlPosition>(latch, 0xFFFFFF - initial);
    }
    latch.wait();

    // Wait for joints to move into place
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    hand.write<data::joint::ControlWord>(5);

    // Enable CSP & PDO Control
    hand.write<data::joint::ControlMode>(4);
    hand.write<data::hand::GlobalTpdoId>(1);
    hand.write<data::hand::JointPdoInterval>(950);
    hand.write<data::hand::PdoEnabled>(1);
    hand.write<data::joint::ControlWord>(1);

    // Disable the whole thumb & each J2
    hand.finger(0).write<data::joint::ControlWord>(5);
    for (int i = 1; i < 5; i++)
        hand.finger(i).joint(1).write<data::joint::ControlWord>(5);

    // 1kHz SDO Control
    constexpr double update_rate = 1000.0;
    constexpr auto update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / update_rate));

    auto begin = std::chrono::steady_clock::now();
    auto next_iteration_time = begin;

    utility::FpsCounter fps_counter;
    double x = std::acos((double(initial) / 0xFFFFFF * 2) - 1);

    int32_t control_positions[5][4];
    control_positions[0][0] = 0x200000;
    control_positions[0][1] = 0x200000;
    control_positions[0][2] = 0x200000;
    control_positions[0][3] = 0x200000;

    while (running.load(std::memory_order_relaxed)) {
        if (fps_counter.count())
            std::cout << "PDO Control Actual Fps: " << fps_counter.fps() << '\n';

        x += 0.005;
        double y = (std::cos(x) + 1) / 2;
        auto position = static_cast<int32_t>(std::round(double(0xFFFFFF) * y));
        for (int i = 1; i < 5; i++) {
            control_positions[i][0] = position;
            control_positions[i][1] = 0x8FFFFF;
            control_positions[i][2] = 0xFFFFFF - position;
            control_positions[i][3] = 0xFFFFFF - position;
        }
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin);
        hand.pdo_write_async_unchecked(control_positions, static_cast<uint32_t>(duration.count()));

        next_iteration_time += update_period;
        std::this_thread::sleep_until(next_iteration_time);
    }

    // Disable the entire hand
    hand.write<data::joint::ControlWord>(5);
    std::cout << "Program exited correctly.\n";
}