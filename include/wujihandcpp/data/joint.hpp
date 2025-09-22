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

struct CurrentLimit : WriteOnlyData<device::Joint, 0x07, 2, uint16_t> {};

struct ControlWord : WriteOnlyData<device::Joint, 0x40, 0, uint16_t> {};

namespace internal {

static constexpr bool is_reversed_joint(uint64_t i) {
    // Reverse each J1 except thumb
    return (i & 0xFF) == 0 && i != 0x0000;
}

static constexpr uint32_t position_policy(uint64_t i) {
    return is_reversed_joint(i) ? (StorageInfo::POSITION_FLOATING | StorageInfo::POSITION_REVERSED)
                                : (StorageInfo::POSITION_FLOATING);
}

} // namespace internal

struct Position : ReadOnlyData<device::Joint, 0x64, 0, double> {
    static constexpr StorageInfo info(uint32_t i) {
        return StorageInfo{sizeof(uint32_t), index, sub_index, internal::position_policy(i)};
    }
};
struct ControlPosition : WriteOnlyData<device::Joint, 0x7A, 0, double> {
    static constexpr StorageInfo info(uint32_t i) {
        return StorageInfo{sizeof(uint32_t), index, sub_index, internal::position_policy(i)};
    }
};

struct UpperLimit : ReadOnlyData<device::Joint, 0x0E, 27, double> {
    static constexpr StorageInfo info(uint32_t i) {
        return StorageInfo{
            sizeof(uint32_t), index, internal::is_reversed_joint(i) ? uint8_t(28) : sub_index,
            internal::position_policy(i)};
    }
};
struct LowerLimit : ReadOnlyData<device::Joint, 0x0E, 28, double> {
    static constexpr StorageInfo info(uint32_t i) {
        return StorageInfo{
            sizeof(uint32_t), index, internal::is_reversed_joint(i) ? uint8_t(27) : sub_index,
            internal::position_policy(i)};
    }
};

} // namespace joint
} // namespace data

} // namespace wujihandcpp