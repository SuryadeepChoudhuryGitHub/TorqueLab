#pragma once
/**
 * Service Brake & Mechanical Handbrake Subsystem.
 */

#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct WheelBrakeTorques {
    double fl{0.0};
    double fr{0.0};
    double rl{0.0};
    double rr{0.0};
};

WheelBrakeTorques computeWheelBrakeTorques(
    double brakeInput,
    double handbrakeInput,
    const VehicleConfig& config
);

} // namespace torquelab
