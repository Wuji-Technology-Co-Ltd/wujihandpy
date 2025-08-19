#include "data/hand.hpp"
#include "device/hand.hpp"

#include <cmath>

#include <thread>

int main() {
    using namespace std::chrono_literals;
    device::Hand hand{0x0483, 0x5740};

    hand.write_data<data::hand::finger::joint::ControlWord>(1);
    // hand.finger(1).write_data<data::hand::finger::joint::SinLevel>(5);
    // hand.finger(1).write_data<data::hand::finger::joint::SinLevel>(0);
    hand.finger(1).joint(1).write_data<data::hand::finger::joint::ControlPosition>(0x8FFFFF);
    for (double x = 0;; x += 0.005) {
        double y = (std::sin(x) + 1) / 2;
        uint32_t position = static_cast<uint32_t>(std::round(double(0xFFFFFF) * y));
        hand.finger(0).joint(3).write_data_async<data::hand::finger::joint::ControlPosition>(
            position);
        // for (int i = 0; i < 5; i++)
        //     for (int j = 0; j < 4; j++) {
        //         if (j == 0)
        //             hand.finger(i)
        //                 .joint(j)
        //                 .write_data_async<data::hand::finger::joint::ControlPosition>(position);
        //         else if (j > 1)
        //             hand.finger(i)
        //                 .joint(j)
        //                 .write_data_async<data::hand::finger::joint::ControlPosition>(
        //                     0xFFFFFF - position);
        //     }
        hand.trigger_transmission();
        std::this_thread::sleep_for(1ms);
    }
}