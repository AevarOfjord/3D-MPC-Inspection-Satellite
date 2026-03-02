#pragma once

#include "nonlinear_sqp_controller.hpp"

namespace satellite_control {
namespace nonlinear {

using MPCV2Params = satellite_control::v2::MPCV2Params;
using ControlResultV2 = satellite_control::v2::ControlResultV2;
using SatelliteParams = satellite_control::SatelliteParams;

class SQPController : public satellite_control::v2::SQPController {
public:
    using satellite_control::v2::SQPController::SQPController;
};

}  // namespace nonlinear
}  // namespace satellite_control
