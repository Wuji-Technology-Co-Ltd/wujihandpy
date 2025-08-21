#pragma once

#include <cstdint>

#include "wujihandcpp/data/helper.hpp"

namespace wujihandcpp {

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

using PdoEnabled = TestData<device::Hand, 0x52A0, 5, uint8_t>;

using GlobalTpdoId = TestData<device::Hand, 0x52A4, 2, uint16_t>;
using JointPdoInterval = TestData<device::Hand, 0x52A4, 5, uint32_t>;
} // namespace data::hand

} // namespace wujihandcpp