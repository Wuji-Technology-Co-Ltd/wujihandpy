#pragma once

#include <cstdint>

#include "wujihandcpp/data/helper.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {

namespace device {
class Hand;
class Finger;
class Joint;
}; // namespace device

namespace data {
namespace joint {

struct ControlMode : WriteOnlyData<device::Joint, 0x02, 1, uint16_t> {};

struct SinLevel : WriteOnlyData<device::Joint, 0x05, 8, uint16_t> {};

struct ControlWord : WriteOnlyData<device::Joint, 0x40, 0, uint16_t> {};

struct Position : ReadOnlyData<device::Joint, 0x64, 0, double> {
    static constexpr size_t value_size = sizeof(uint32_t);

    static constexpr uint32_t policy(uint64_t i) {
        using StorageInfo = protocol::Handler::StorageInfo;
        // Reverse each J1 except thumb
        return ((i & 0xFF) == 0 && i != 0x0000)
                 ? (StorageInfo::POSITION_FLOATING | StorageInfo::POSITION_REVERSED)
                 : (StorageInfo::POSITION_FLOATING);
    }
};
struct ControlPosition : WriteOnlyData<device::Joint, 0x7A, 0, double> {
    static constexpr size_t value_size = sizeof(uint32_t);

    static constexpr uint32_t policy(uint64_t i) { return Position::policy(i); }
};

} // namespace joint
} // namespace data

} // namespace wujihandcpp