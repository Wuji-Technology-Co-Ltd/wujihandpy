#pragma once

#include <cstdint>

#include "data/helper.hpp"

namespace data::spinal {

namespace firmware_infomation {
constexpr uint16_t index = 0x0152;

using FirmwareVersion = SpecializedData<index, 1, uint32_t>;
using FirmwareDate = SpecializedData<index, 2, uint32_t>;

}; // namespace firmware_infomation

namespace monitor_infomation {
constexpr uint16_t index = 0x0A52;

using SystemTime = SpecializedData<index, 1, uint32_t>;
using McuTemperature = SpecializedData<index, 9, float>;
using InputVoltage = SpecializedData<index, 10, float>;

}; // namespace monitor_infomation

} // namespace data::spinal