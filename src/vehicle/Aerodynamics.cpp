#include "vehicle/Aerodynamics.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

AeroForceOutput computeAerodynamicForces(
    const Vec3& velocityWorld,
    const Vec3& chassisForwardWorld,
    const Vec3& chassisUpWorld,
    const VehicleConfig& config
) {
    const double speedSq = vec3LengthSq(velocityWorld);
    const double speed = std::sqrt(speedSq);
    if (speed < 1e-4) {
        return AeroForceOutput{};
    }

    const double dynPressure = 0.5 * config.airDensity * speedSq;
    const double dragMag = dynPressure * config.dragCoefficient * config.frontalArea;
    const Vec3 dragForceWorld = vec3Scale(velocityWorld, -dragMag / speed);

    const double forwardSpeed = vec3Dot(velocityWorld, chassisForwardWorld);
    const double downforceMag =
        0.5 * config.airDensity * config.downforceCoefficient * config.frontalArea *
        (forwardSpeed * forwardSpeed);
    const Vec3 downforceWorld = vec3Scale(chassisUpWorld, -downforceMag);

    return AeroForceOutput{
        dragForceWorld,
        downforceWorld,
        dragMag,
        downforceMag
    };
}

double computeWheelRollingResistanceTorque(
    double normalLoad,
    double wheelRadius,
    double wheelAngularVelocity,
    double surfaceRollingResistanceMultiplier,
    const VehicleConfig& config
) {
    if (normalLoad <= 0.0) {
        return 0.0;
    }
    const double speedFactor = clamp(std::abs(wheelAngularVelocity * wheelRadius) / 0.5, 0.0, 1.0);
    return config.rollingResistanceCoeff * surfaceRollingResistanceMultiplier *
           normalLoad * wheelRadius * speedFactor;
}

} // namespace torquelab
