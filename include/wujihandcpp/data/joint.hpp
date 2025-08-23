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
struct ControlMode : RemoteData<device::Joint, 0x02, 1, uint16_t> {};

struct SinLevel : RemoteData<device::Joint, 0x05, 8, uint16_t> {};

struct ControlWord : RemoteData<device::Joint, 0x40, 0, uint16_t> {};

struct Position : RemoteData<device::Joint, 0x64, 0, int32_t> {};
struct ControlPosition : RemoteData<device::Joint, 0x7A, 0, int32_t> {};
} // namespace joint
} // namespace data

} // namespace wujihandcpp