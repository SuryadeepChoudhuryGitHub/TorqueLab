#pragma once
/**
 * Aerodynamic Drag, Downforce, and Rolling Resistance Subsystem.
 */

#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct AeroForceOutput {
    Vec3 dragForceWorld{0.0, 0.0, 0.0};
    Vec3 downforceWorld{0.0, 0.0, 0.0};
    double dragMagnitude{0.0};
    double downforceMagnitude{0.0};
};

AeroForceOutput computeAerodynamicForces(
    const Vec3& velocityWorld,
    const Vec3& chassisForwardWorld,
    const Vec3& chassisUpWorld,
    const VehicleConfig& config
);

double computeWheelRollingResistanceTorque(
    double normalLoad,
    double wheelRadius,
    double wheelAngularVelocity,
    double surfaceRollingResistanceMultiplier,
    const VehicleConfig& config
);

} // namespace torquelab
