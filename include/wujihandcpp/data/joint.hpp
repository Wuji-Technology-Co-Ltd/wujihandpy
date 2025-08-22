#pragma once

#include <cstdint>

#include "wujihandcpp/data/helper.hpp"

namespace wujihandcpp {

namespace device {
class Hand;
class Finger;
class Joint;
}; // namespace device

namespace data::joint {
using ControlMode = TestData<device::Joint, 0x02, 1, uint16_t>;

using SinLevel = TestData<device::Joint, 0x05, 8, uint16_t>;

using ControlWord = TestData<device::Joint, 0x40, 0, uint16_t>;

using Position = TestData<device::Joint, 0x64, 0, int32_t>;
using ControlPosition = TestData<device::Joint, 0x7A, 0, int32_t>;
} // namespace data::joint

} // namespace wujihandcpp