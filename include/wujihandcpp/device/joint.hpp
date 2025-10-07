#pragma once

#include <cstdint>

#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/device/data_operator.hpp"
#include "wujihandcpp/device/data_tuple.hpp"
#include "wujihandcpp/protocol/handler.hpp"

namespace wujihandcpp {
namespace device {

class Joint : public DataOperator<Joint> {
    friend class DataOperator;
    friend class Finger;

private:
    Joint(protocol::Handler& handler, uint16_t index_offset, int storage_offset)
        : handler_(handler)
        , index_offset_(index_offset)
        , storage_offset_(storage_offset) {}

    using Datas = DataTuple<
        data::joint::HardwareVersion, data::joint::HardwareDate, data::joint::ControlMode,
        data::joint::SinLevel, data::joint::CurrentLimit, data::joint::BusVoltage,
        data::joint::Temperature, data::joint::ResetError, data::joint::ErrorCode,
        data::joint::Enabled, data::joint::ActualPosition, data::joint::TargetPosition,
        data::joint::UpperLimit, data::joint::LowerLimit>;

    protocol::Handler& handler_;
    uint16_t index_offset_;
    int storage_offset_;
};

} // namespace device
} // namespace wujihandcpp