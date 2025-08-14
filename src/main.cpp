#include <thread>

#include "client/hand.hpp"
#include "data/spinal.hpp"

int main() {
    client::Hand hand{0x0483, 0x5740};
    std::jthread thread{[&hand]() { hand.handle_events(); }};

    using namespace std::chrono_literals;
    while (true) {
        hand.read_data_async<data::spinal::monitor_infomation::SystemTime>();
        hand.read_data_async<data::spinal::monitor_infomation::McuTemperature>();
        hand.trigger_transmission();
        std::this_thread::sleep_for(500ms);
    }
}