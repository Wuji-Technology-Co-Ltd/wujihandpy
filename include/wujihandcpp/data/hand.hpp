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
struct FirmwareVersion : RemoteData<device::Hand, 0x5201, 1, uint32_t> {};
struct FirmwareDate : RemoteData<device::Hand, 0x5201, 2, uint32_t> {};

struct SystemTime : RemoteData<device::Hand, 0x520A, 1, uint32_t> {};
struct McuTemperature : RemoteData<device::Hand, 0x520A, 9, float> {};
struct InputVoltage : RemoteData<device::Hand, 0x520A, 10, float> {};

struct PdoEnabled : RemoteData<device::Hand, 0x52A0, 5, uint8_t> {};

struct GlobalTpdoId : RemoteData<device::Hand, 0x52A4, 2, uint16_t> {};
struct JointPdoInterval : RemoteData<device::Hand, 0x52A4, 5, uint32_t> {};
} // namespace data::hand

} // namespace wujihandcpp