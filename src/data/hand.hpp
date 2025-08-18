#pragma once

#include <cstdint>

#include "data/helper.hpp"

namespace device {

class Hand;
class Finger;
class Joint;

}; // namespace device

namespace data::hand {

using FirmwareVersion = TestData<device::Hand, 0x5201, 1, uint32_t>;
using FirmwareDate = TestData<device::Hand, 0x5201, 2, uint32_t>;

using SystemTime = TestData<device::Hand, 0x520A, 1, uint32_t>;
using McuTemperature = TestData<device::Hand, 0x520A, 9, float>;
using InputVoltage = TestData<device::Hand, 0x520A, 10, float>;

namespace finger::joint {

using ControlWord = TestData<device::Joint, 0x40, 0, uint16_t>;

using Position = TestData<device::Joint, 0x64, 0, uint32_t>;
using ControlPosition = TestData<device::Joint, 0x7A, 0, uint32_t>;

} // namespace finger::joint

} // namespace data::hand
