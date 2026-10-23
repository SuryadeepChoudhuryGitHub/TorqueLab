#include "vehicle/TireModel.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

double computeLongitudinalSlipRatio(
    double wheelAngularVelocity,
    double wheelRadius,
    double patchSpeedLongitudinal
) {
    const double wheelCircumferentialSpeed = wheelAngularVelocity * wheelRadius;
    const double slipSpeed = wheelCircumferentialSpeed - patchSpeedLongitudinal;
    const double denom = std::max(std::abs(patchSpeedLongitudinal), 1.5);
    return clamp(slipSpeed / denom, -20.0, 20.0);
}

double computeLateralSlipAngle(
    double patchSpeedLongitudinal,
    double patchSpeedLateral
) {
    const double denom = std::max(std::abs(patchSpeedLongitudinal), 1.15);
    const double lowSpeedTaper = clamp(std::hypot(patchSpeedLongitudinal, patchSpeedLateral) / 0.45, 0.0, 1.0);
    return -std::atan2(patchSpeedLateral, denom) * lowSpeedTaper;
}

double computeEffectiveFrictionCoeff(
    double normalLoad,
    double surfaceFrictionMultiplier,
    const VehicleConfig& config
) {
    if (normalLoad <= 0.0 || config.tireFrictionCoeff <= 0.0 || surfaceFrictionMultiplier <= 0.0) {
        return 0.0;
    }
    const double loadDeltaRatio = (normalLoad - config.nominalTireLoad) / std::max(500.0, config.nominalTireLoad);
    const double loadFactor = clamp(1.0 - config.tireLoadSensitivity * loadDeltaRatio, 0.68, 1.18);
    return std::max(0.02, config.tireFrictionCoeff * surfaceFrictionMultiplier * loadFactor);
}

double magicFormulaCurve(
    double slip,
    double B,
    double C,
    double D,
    double E
) {
    const double bs = B * slip;
    const double arg = bs - E * (bs - std::atan(bs));
    return D * std::sin(C * std::atan(arg));
}

TireForceOutput computeTireForcesFromSlip(
    double slipRatio,
    double slipAngle,
    double normalLoad,
    double surfaceFrictionMultiplier,
    double surfaceStiffnessMultiplier,
    const VehicleConfig& config
) {
    if (normalLoad <= 0.0) {
        return TireForceOutput{
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            false
        };
    }

    const double effectiveFrictionCoeff = computeEffectiveFrictionCoeff(
        normalLoad,
        surfaceFrictionMultiplier,
        config
    );
    const double maxAvailableGripForce = effectiveFrictionCoeff * normalLoad;

    const double bx = config.tireLongitudinalStiffness * surfaceStiffnessMultiplier;
    const double by = config.tireLateralStiffness * surfaceStiffnessMultiplier;

    const double normFx = magicFormulaCurve(slipRatio, bx, 1.32, 1.0, -0.08);
    const double normFy = magicFormulaCurve(std::tan(clamp(slipAngle, -1.25, 1.25)), by, 1.35, 1.0, -0.20);

    double fx = maxAvailableGripForce * normFx;
    double fy = maxAvailableGripForce * normFy;

    const double rawMag = std::hypot(fx, fy);
    double gripUsageRatio = (maxAvailableGripForce > 1e-5) ? (rawMag / maxAvailableGripForce) : 0.0;

    if (rawMag > maxAvailableGripForce && rawMag > 1e-6) {
        const double scale = maxAvailableGripForce / rawMag;
        fx *= scale;
        fy *= scale;
        gripUsageRatio = 1.0;
    }

    const bool isSkidding =
        (gripUsageRatio > 0.90) && (std::abs(slipRatio) > 0.16 || std::abs(slipAngle) > 0.14);

    return TireForceOutput{
        slipRatio,
        slipAngle,
        effectiveFrictionCoeff,
        maxAvailableGripForce,
        fx,
        fy,
        clamp(gripUsageRatio, 0.0, 1.0),
        isSkidding
    };
}

} // namespace torquelab
