#pragma once
/**
 * Complete 4-Wheel Vehicle Dynamics Orchestrator.
 *
 * Integrates all physical subsystems on every fixed simulation step (dt = 1/120 s):
 * 1. Raycast wheel-ground contact & surface detection
 * 2. Independent spring-damper suspension + progressive bump stops
 * 3. Front and rear anti-roll bar load transfer
 * 4. Speed-sensitive Ackermann steering geometry
 * 5. Engine, transmission, clutch, and viscous LSD drive torques
 * 6. Service brakes & rear handbrake torques
 * 7. Sub-stepped wheel angular momentum integration + Pacejka/Friction-Circle tire forces
 * 8. Aerodynamic drag & downforce
 * 9. Application of 3D forces at world contact patches / hubs onto the 6-DOF Bullet chassis
 */

#include <array>
#include <string>
#include <btBulletDynamicsCommon.h>
#include "physics/Math3D.h"
#include "physics/SurfaceSystem.h"
#include "vehicle/Aerodynamics.h"
#include "vehicle/BrakeSystem.h"
#include "vehicle/Drivetrain.h"
#include "vehicle/SteeringSystem.h"
#include "vehicle/SuspensionSystem.h"
#include "vehicle/TireModel.h"
#include "vehicle/VehicleBody.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/WheelState.h"

namespace torquelab {

struct VehicleControlInputs {
    double throttle{0.0};    // [0..1]
    double brake{0.0};       // [0..1]
    double steering{0.0};    // [-1..1], +1 = Left, -1 = Right
    double handbrake{0.0};   // [0..1]
    bool disableAutoReverse{false};
};

struct DrivetrainTelemetry {
    double rpm{850.0};
    int gearIndex{2};
    std::string gearLabel{"1"};
    TransmissionMode transmissionMode{TransmissionMode::Automatic};
    double engineTorqueNm{0.0};
    double clutchEngagement{1.0};
    bool revLimiterActive{false};
};

struct VehicleTelemetrySnapshot {
    double simTime{0.0};
    Vec3 chassisPosition{0.0, 0.0, 0.0};
    Quat chassisRotation{0.0, 0.0, 0.0, 1.0};
    Vec3 centerOfMassWorld{0.0, 0.0, 0.0};
    Vec3 velocityWorld{0.0, 0.0, 0.0};
    Vec3 angularVelocityWorld{0.0, 0.0, 0.0};
    double speedMps{0.0};
    double speedKmh{0.0};
    double forwardSpeedMps{0.0};
    double lateralSpeedMps{0.0};
    Vec3 accelerationWorld{0.0, 0.0, 0.0};
    double longitudinalAccelMps2{0.0};
    double lateralAccelMps2{0.0};
    double longitudinalG{0.0};
    double lateralG{0.0};
    double pitchRad{0.0};
    double rollRad{0.0};
    double yawRad{0.0};
    double steeringAngleRad{0.0};
    DrivetrainTelemetry drivetrain{};
    AeroForceOutput aero{};
    std::array<WheelState, 4> wheels{};
    Vec3 forwardDirWorld{0.0, 0.0, 1.0};
    Vec3 upDirWorld{0.0, 1.0, 0.0};
    Vec3 lateralDirWorld{1.0, 0.0, 0.0};
    int collisionContactCount{0};
    bool numericalSanityOk{true};
};

struct InterpolatedVehicleTransforms {
    Vec3 chassisPosition{0.0, 0.0, 0.0};
    Quat chassisRotation{0.0, 0.0, 0.0, 1.0};
    std::array<Vec3, 4> wheelPositions{};
    std::array<Quat, 4> wheelRotations{};
};

class VehicleDynamics {
public:
    VehicleDynamics(
        btDiscreteDynamicsWorld* world,
        SurfaceSystem* surfaceSystem,
        const VehicleConfig& initialConfig = createDefaultVehicleConfig(),
        const Vec3& spawnPosition = Vec3{0.0, 0.55, 0.0},
        const Quat& spawnRotation = quatIdentity()
    );

    /** Captures current chassis and wheel transforms before stepping for smooth render interpolation. */
    void capturePreviousTransforms();

    /** Executes one fixed-timestep vehicle dynamics update (dt = 1/120 s) BEFORE world->stepSimulation(). */
    void step(double dt);

    /** Called immediately AFTER world->stepSimulation() to update wheel visual positions and finite-difference acceleration. */
    void postWorldStep(double dt);

    /** Resets vehicle pose, wheel states, drivetrain, and velocities. */
    void resetVehicle(const Vec3* position = nullptr, const Quat* rotation = nullptr);

    /** Returns interpolated chassis and wheel visual transforms for smooth rendering. */
    InterpolatedVehicleTransforms getInterpolatedTransforms(double alpha) const;

    /** Returns a complete snapshot of vehicle telemetry for HUD, graphs, and automated tests. */
    VehicleTelemetrySnapshot getTelemetrySnapshot() const;

    VehicleConfig config;
    VehicleBody body;
    std::array<WheelState, 4> wheels;
    DrivetrainState drivetrain;
    double centerSteeringAngle{0.0};
    AeroForceOutput aeroState{};
    double simTime{0.0};
    VehicleControlInputs currentInputs{};

private:
    Quat computeWheelWorldOrientation(const Quat& chassisRot, const WheelState& w) const;

    btDiscreteDynamicsWorld* m_world{nullptr};
    SurfaceSystem* m_surfaceSystem{nullptr};
    Vec3 m_initialSpawnPos{0.0, 0.55, 0.0};
    Quat m_initialSpawnRot{0.0, 0.0, 0.0, 1.0};

    Vec3 m_prevChassisPos{0.0, 0.0, 0.0};
    Quat m_prevChassisRot{0.0, 0.0, 0.0, 1.0};
    std::array<Vec3, 4> m_prevWheelPos{};
    std::array<Quat, 4> m_prevWheelRot{};
};

} // namespace torquelab
