#include "data/hand.hpp"
#include "device/hand.hpp"

#include <format>

int main() {
    using namespace std::chrono_literals;
    device::Hand hand{0x0483, 0x5740};

    hand.finger(1).read_data<data::hand::finger::joint::Position>();
    for (int i = 0; i < 4; i++)
        std::cout << std::format(
            "0x{:06x} ", hand.finger(1).joint(i).data<data::hand::finger::joint::Position>());
    std::cout << '\n';
}