#include "data/hand.hpp"
#include "device/hand.hpp"
#include <cstdint>
// #include <atomic>
// #include <bit>
// #include <chrono>
// #include <cstddef>
// #include <cstdint>
// #include <format>
// #include <thread>
// #include <tuple>
// #include <type_traits>
// #include <utility>

// #include "data/hand.hpp"
// #include "device/hand.hpp"

// static_assert(sizeof(DataStorage) == 32);

// void read_data(auto& identity) {}

// DataStorage storage;

int main() {
    using namespace std::chrono_literals;

    // constexpr auto a = device::Hand::sub_count<data::hand::finger::joint::Position>();

    // data::HandAdapter hand;
    // hand.apply<data::Hand::Finger::Joint::Position>([](uint16_t index, uint8_t sub_index) {
    //     std::cout << std::format("0x{:04X}, 0x{:04X}\n", index, sub_index);
    // });

    // auto version = storage.data_version();
    // std::cout << version << '\n';
    // std::jthread thread{[]() {
    //     std::this_thread::sleep_for(1s);
    //     storage.write<uint32_t>(214748322);
    //     storage.write<uint32_t>(214748322);
    // }};
    // std::cout << storage.wait<uint32_t>(version) << '\n';
    // std::cout << storage.data_version() << '\n';

    device::Hand hand{0x0483, 0x5740};

    // hand.iterate_index<data::hand::finger::joint::Position>([](uint16_t index_offset){
    //     std::cout << std::format("0x{:04X}\n", index_offset);
    // });
    hand.iterate_storage<data::hand::finger::joint::Position>([](int storage_offset){
        std::cout << std::format("{}\n", storage_offset);
    });


    // for (int i = 0; i < 5; i++) {
    //     std::cout << hand.read_data<data::spinal::monitor_infomation::SystemTime>() << '\n';
    //     std::cout << hand.read_data<data::spinal::monitor_infomation::McuTemperature>() << '\n';
    //     std::this_thread::sleep_for(500ms);
    // }
}