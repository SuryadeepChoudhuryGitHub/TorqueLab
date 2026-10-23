#include "vehicle/SuspensionSystem.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

SpringDamperResult computeSpringDamperForce(
    double displacementFromRest,
    double lengthVelocity,
    double springStiffness,
    double dampingCompression,
    double dampingRebound
) {
    const double effectiveDisplacement = std::min(0.0, displacementFromRest);
    const double springForce = -springStiffness * effectiveDisplacement;
    const double dampingCoeff = (lengthVelocity < 0.0) ? dampingCompression : dampingRebound;
    const double damperForce = -dampingCoeff * lengthVelocity;
    return SpringDamperResult{
        springForce,
        damperForce,
        springForce + damperForce
    };
}

double computeBumpStopForce(
    double rawLength,
    double minLength,
    double bumpStopStiffness
) {
    const double engageZone = 0.03;
    const double threshold = minLength + engageZone;
    if (rawLength >= threshold) {
        return 0.0;
    }
    const double penetration = threshold - rawLength;
    const double normalized = penetration / engageZone;
    return bumpStopStiffness * penetration * (0.4 + 0.6 * normalized);
}

SuspensionForceOutput evaluateWheelSuspension(
    double rawLength,
    double lengthVelocity,
    bool isFront,
    const VehicleConfig& config
) {
    const double clampedLength = clamp(
        rawLength,
        config.suspensionMinLength * 0.85,
        config.suspensionRestLength
    );
    const double displacementFromRest = clampedLength - config.suspensionRestLength;
    const double compression = std::max(0.0, -displacementFromRest);
    const double maxTravel = std::max(0.01, config.suspensionRestLength - config.suspensionMinLength);
    const double compressionRatio = clamp(compression / maxTravel, 0.0, 1.0);

    const double k = isFront ? config.frontSpringStiffness : config.rearSpringStiffness;
    const double cComp = isFront ? config.frontDampingCompression : config.rearDampingCompression;
    const double cReb = isFront ? config.frontDampingRebound : config.rearDampingRebound;

    const double clampedVel = clamp(lengthVelocity, -15.0, 15.0);
    const SpringDamperResult sd = computeSpringDamperForce(
        displacementFromRest,
        clampedVel,
        k,
        cComp,
        cReb
    );
    const double bumpStopForce = computeBumpStopForce(
        rawLength,
        config.suspensionMinLength,
        config.bumpStopStiffness
    );

    return SuspensionForceOutput{
        clampedLength,
        compression,
        compressionRatio,
        sd.springForce,
        sd.damperForce,
        bumpStopForce,
        rawLength <= config.suspensionMinLength
    };
}

AxleAntiRollResult computeAxleAntiRollForce(
    double leftCompression,
    double rightCompression,
    bool leftInContact,
    bool rightInContact,
    double antiRollStiffness
) {
    if (!leftInContact && !rightInContact) {
        return AxleAntiRollResult{0.0, 0.0};
    }
    const double diff = leftCompression - rightCompression;
    const double transferForce = antiRollStiffness * diff;
    return AxleAntiRollResult{
        leftInContact ? transferForce : 0.0,
        rightInContact ? -transferForce : 0.0
    };
}

} // namespace torquelab
