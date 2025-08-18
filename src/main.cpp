#include "data/hand.hpp"
#include "device/hand.hpp"

#include <cmath>

#include <thread>

int main() {
    using namespace std::chrono_literals;
    device::Hand hand{0x0483, 0x5740};

    hand.finger(1).write_data<data::hand::finger::joint::ControlWord>(1);
    for (double x = 0;; x += 0.01) {
        double y = (std::sin(x) + 1) / 2;
        uint32_t position = static_cast<uint32_t>(std::round(double(0xFFFFFF) * y));
        hand.finger(1).joint(0).write_data_async<data::hand::finger::joint::ControlPosition>(
            position);
        hand.trigger_transmission();
        std::this_thread::sleep_for(1ms);
    }
}