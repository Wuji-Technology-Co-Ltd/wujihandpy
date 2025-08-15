#include <thread>

#include "data/spinal.hpp"
#include "device/hand.hpp"

int main() {
    device::Hand hand{0x0483, 0x5740};

    using namespace std::chrono_literals;
    for (int i = 0; i < 5; i++) {
        std::cout << hand.read_data<data::spinal::monitor_infomation::SystemTime>() << '\n';
        std::cout << hand.read_data<data::spinal::monitor_infomation::McuTemperature>() << '\n';
        std::this_thread::sleep_for(500ms);
    }
}