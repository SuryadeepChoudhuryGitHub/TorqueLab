#pragma once
/**
 * Per-Wheel Simulation State & Telemetry Data Structure.
 */

#include <string>
#include "physics/Math3D.h"
#include "physics/SurfaceSystem.h"

namespace torquelab {

using WheelIndex = int; // 0 = FL, 1 = FR, 2 = RL, 3 = RR

struct WheelState {
    WheelIndex index{0};
    std::string id{"FL"};
    bool isFront{true};
    bool isLeft{true};

    // --- Geometry & Raycast Contact ---
    Vec3 mountPointLocal{0.0, 0.0, 0.0};    // Upper suspension mount in chassis frame (m)
    Vec3 mountPointWorld{0.0, 0.0, 0.0};    // Upper suspension mount in world frame (m)
    Vec3 wheelCenterWorld{0.0, 0.0, 0.0};   // Current 3D center of wheel hub in world frame (m)
    Vec3 contactPointWorld{0.0, 0.0, 0.0};  // Ground contact patch position in world frame (m)
    Vec3 contactNormalWorld{0.0, 1.0, 0.0}; // Ground surface unit normal in world frame
    bool inContact{false};
    SurfaceProperties surface{getSurfaceProperties(SurfaceType::DryAsphalt)};

    // --- Suspension State ---
    double suspensionLength{0.38};          // Current spring length l (m)
    double prevSuspensionLength{0.38};      // Previous step spring length (m)
    double compression{0.0};                // Compression distance from rest: l_rest - l (m, positive when compressed)
    double compressionRatio{0.0};           // Normalized compression [0..1] between restLength and minLength
    double suspensionVelocity{0.0};         // Rate of change of length dl/dt (m/s, negative when compressing)
    double springForce{0.0};                // Upward spring force -k * x (N)
    double damperForce{0.0};                // Upward damper force -c * v (N)
    double bumpStopForce{0.0};              // Upward bottoming-out bump stop force (N)
    double antiRollForce{0.0};              // Upward/downward force from axle anti-roll bar (N)
    double normalLoad{0.0};                 // Total vertical tire normal load F_z on ground (N, >= 0)
    bool isBottomedOut{false};

    // --- Steering & Wheel Rotation ---
    double steeringAngle{0.0};              // Steer angle around chassis up axis (rad, +Left / -Right)
    double angularVelocity{0.0};            // Wheel spin angular velocity omega (rad/s, +Forward roll)
    double rotationAngle{0.0};              // Accumulated wheel spin angle for visual rendering (rad)

    // --- Tire Slip & Forces ---
    double patchSpeedLongitudinal{0.0};     // Ground velocity along tire forward direction v_x (m/s)
    double patchSpeedLateral{0.0};          // Ground velocity along tire lateral left direction v_y (m/s)
    double slipRatio{0.0};                  // Longitudinal slip ratio kappa (dimensionless)
    double slipAngle{0.0};                  // Lateral slip angle alpha (rad)
    double effectiveFrictionCoeff{1.1};     // Load-sensitive + surface-adjusted friction coefficient mu_eff
    double maxAvailableGripForce{0.0};      // Friction circle radius: mu_eff * F_z (N)
    double longitudinalForce{0.0};          // Tire longitudinal force F_x in tire frame (N)
    double lateralForce{0.0};               // Tire lateral force F_y in tire frame (N)
    double gripUsageRatio{0.0};             // sqrt(F_x^2 + F_y^2) / (mu_eff * F_z) in [0..1]
    bool isSkidding{false};                 // True when gripUsageRatio > 0.88 and significant slip exists
    Vec3 tireForceWorld{0.0, 0.0, 0.0};     // Total horizontal tire force vector in world space (N)

    // --- Torques ---
    double driveTorque{0.0};                // Drivetrain torque applied to wheel hub (N*m)
    double brakeTorque{0.0};                // Total braking torque magnitude available this step (N*m)
    double rollingResistanceTorque{0.0};    // Rolling resistance torque magnitude (N*m)
};

inline WheelState createInitialWheelState(
    WheelIndex index,
    const Vec3& mountLocal,
    double restLength,
    double wheelRadius
) {
    static const char* const ids[4] = {"FL", "FR", "RL", "RR"};
    const bool isFront = (index == 0 || index == 1);
    const bool isLeft = (index == 0 || index == 2);

    WheelState w{};
    w.index = index;
    w.id = (index >= 0 && index < 4) ? ids[index] : "FL";
    w.isFront = isFront;
    w.isLeft = isLeft;
    w.mountPointLocal = mountLocal;
    w.mountPointWorld = mountLocal;
    w.wheelCenterWorld = Vec3{mountLocal.x, mountLocal.y - restLength, mountLocal.z};
    w.contactPointWorld = Vec3{mountLocal.x, mountLocal.y - restLength - wheelRadius, mountLocal.z};
    w.contactNormalWorld = Vec3{0.0, 1.0, 0.0};
    w.inContact = false;
    w.surface = getSurfaceProperties(SurfaceType::DryAsphalt);
    w.suspensionLength = restLength;
    w.prevSuspensionLength = restLength;
    return w;
}

} // namespace torquelab
