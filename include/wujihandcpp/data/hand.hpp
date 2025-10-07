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
namespace hand {
struct Handedness : ReadOnlyData<device::Hand, 0x5090, 0, uint8_t> {};

struct FirmwareVersion : ReadOnlyData<device::Hand, 0x5201, 1, uint32_t> {};
struct FirmwareDate : ReadOnlyData<device::Hand, 0x5201, 2, uint32_t> {};

struct SystemTime : ReadOnlyData<device::Hand, 0x520A, 1, uint32_t> {};
struct Temperature : ReadOnlyData<device::Hand, 0x520A, 9, float> {};
struct InputVoltage : ReadOnlyData<device::Hand, 0x520A, 10, float> {};

struct PdoEnabled : WriteOnlyData<device::Hand, 0x52A0, 5, uint8_t> {};
struct RPdoId : WriteOnlyData<device::Hand, 0x52A4, 2, uint16_t> {};
struct TPdoId : WriteOnlyData<device::Hand, 0x52A4, 2, uint16_t> {};

struct PdoInterval : WriteOnlyData<device::Hand, 0x52A4, 5, uint32_t> {};
struct RPdoTriggerOffset : WriteOnlyData<device::Hand, 0x52A4, 6, uint32_t> {};
struct TPdoTriggerOffset : WriteOnlyData<device::Hand, 0x52A4, 7, uint32_t> {};
} // namespace hand
} // namespace data

} // namespace wujihandcpp