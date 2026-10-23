#pragma once
/**
 * Steering Geometry & Speed-Sensitive Actuator System.
 */

#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct AckermannSteeringAngles {
    double centerAngle{0.0};
    double frontLeftAngle{0.0};
    double frontRightAngle{0.0};
};

double computeSpeedSensitiveMaxSteer(
    double forwardSpeedMps,
    const VehicleConfig& config
);

double updateCenterSteeringAngle(
    double currentAngle,
    double steerInput,
    double forwardSpeedMps,
    const VehicleConfig& config,
    double dt
);

AckermannSteeringAngles computeAckermannWheelAngles(
    double centerAngle,
    const VehicleConfig& config
);

} // namespace torquelab
