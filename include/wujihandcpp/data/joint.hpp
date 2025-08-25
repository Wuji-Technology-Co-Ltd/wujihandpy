#pragma once

#include <cstdint>

#include "wujihandcpp/data/helper.hpp"

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

struct Position : ReadOnlyData<device::Joint, 0x64, 0, int32_t> {};
struct ControlPosition : WriteOnlyData<device::Joint, 0x7A, 0, int32_t> {};
} // namespace joint
} // namespace data

} // namespace wujihandcpp