#pragma once
/**
 * Combined-Slip Tire Force Model (Pacejka Magic Formula Lite + Load Sensitivity + Friction Circle).
 */

#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct TireForceOutput {
    double slipRatio{0.0};              // Longitudinal slip ratio kappa
    double slipAngle{0.0};              // Lateral slip angle alpha (rad)
    double effectiveFrictionCoeff{0.0}; // Load-sensitive + surface-adjusted mu_eff
    double maxAvailableGripForce{0.0};  // Friction circle radius mu_eff * F_z (N)
    double longitudinalForce{0.0};      // F_x in tire frame (N, +Forward)
    double lateralForce{0.0};           // F_y in tire frame (N, +Left)
    double gripUsageRatio{0.0};         // sqrt(F_x^2 + F_y^2) / maxAvailableGripForce in [0..1]
    bool isSkidding{false};
};

double computeLongitudinalSlipRatio(
    double wheelAngularVelocity,
    double wheelRadius,
    double patchSpeedLongitudinal
);

double computeLateralSlipAngle(
    double patchSpeedLongitudinal,
    double patchSpeedLateral
);

double computeEffectiveFrictionCoeff(
    double normalLoad,
    double surfaceFrictionMultiplier,
    const VehicleConfig& config
);

double magicFormulaCurve(
    double slip,
    double B,
    double C,
    double D,
    double E
);

TireForceOutput computeTireForcesFromSlip(
    double slipRatio,
    double slipAngle,
    double normalLoad,
    double surfaceFrictionMultiplier,
    double surfaceStiffnessMultiplier,
    const VehicleConfig& config
);

} // namespace torquelab
