#pragma once
/**
 * Independent 4-Wheel Suspension & Anti-Roll Bar Dynamics.
 */

#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct SuspensionForceOutput {
    double clampedLength{0.0};
    double compression{0.0};       // Positive when compressed: restLength - clampedLength (m)
    double compressionRatio{0.0};  // Normalized [0..1]
    double springForce{0.0};       // -k * x (N)
    double damperForce{0.0};       // -c * v (N)
    double bumpStopForce{0.0};     // Progressive bottom-out force (N)
    bool isBottomedOut{false};
};

struct SpringDamperResult {
    double springForce{0.0};
    double damperForce{0.0};
    double totalForce{0.0};
};

struct AxleAntiRollResult {
    double leftForce{0.0};
    double rightForce{0.0};
};

SpringDamperResult computeSpringDamperForce(
    double displacementFromRest,
    double lengthVelocity,
    double springStiffness,
    double dampingCompression,
    double dampingRebound
);

double computeBumpStopForce(
    double rawLength,
    double minLength,
    double bumpStopStiffness
);

SuspensionForceOutput evaluateWheelSuspension(
    double rawLength,
    double lengthVelocity,
    bool isFront,
    const VehicleConfig& config
);

AxleAntiRollResult computeAxleAntiRollForce(
    double leftCompression,
    double rightCompression,
    bool leftInContact,
    bool rightInContact,
    double antiRollStiffness
);

} // namespace torquelab
