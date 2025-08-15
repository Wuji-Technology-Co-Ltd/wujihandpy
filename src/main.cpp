#include <thread>

#include "data/spinal.hpp"
#include "device/hand.hpp"

int main() {
    device::Hand hand{0x0483, 0x5740};
    std::jthread thread{[&hand]() { hand.handle_events(); }};

    using namespace std::chrono_literals;

    while (true) {
        std::cout << hand.read_data<data::spinal::monitor_infomation::SystemTime>() << '\n';
        std::cout << hand.read_data<data::spinal::monitor_infomation::McuTemperature>() << '\n';
        std::this_thread::sleep_for(500ms);
    }
}