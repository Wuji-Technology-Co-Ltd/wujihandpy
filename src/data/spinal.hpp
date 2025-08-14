#pragma once

#include <cstdint>

#include "data/helper.hpp"

namespace data::spinal {

namespace firmware_infomation {

struct FirmwareVersion : helper::DataStruct<0x5201, 1, uint32_t> {};
struct FirmwareDate : helper::DataStruct<0x5201, 2, uint32_t> {};

}; // namespace firmware_infomation

namespace monitor_infomation {

struct SystemTime : helper::DataStruct<0x520A, 1, uint32_t> {};
struct McuTemperature : helper::DataStruct<0x520A, 9, float> {};
struct InputVoltage : helper::DataStruct<0x520A, 10, float> {};

}; // namespace monitor_infomation

} // namespace data::spinal