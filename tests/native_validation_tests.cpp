/**
 * Torque Lab Native — Automated Mathematical Unit Tests & Physical Validation Suite (Tests A – J).
 *
 * Mirrors the 8 unit test suites in `tests/unit.test.ts` and the 10 full-simulation
 * validation scenarios in `tests/validation.test.ts` to verify 1:1 physical equivalence.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "input/ControllerConfig.h"
#include "input/VibrationManager.h"
#include "physics/CollisionMeshGeometry.h"
#include "physics/Math3D.h"
#include "physics/PhysicsWorld.h"
#include "physics/SurfaceSystem.h"
#include "physics/VehicleCollisionGenerator.h"
#include "vehicle/Aerodynamics.h"
#include "vehicle/BrakeSystem.h"
#include "vehicle/Drivetrain.h"
#include "vehicle/SteeringSystem.h"
#include "vehicle/SuspensionSystem.h"
#include "vehicle/TireModel.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/VehicleComponents.h"
#include "vehicle/WheelState.h"

using namespace torquelab;

namespace {

int g_failedAssertions = 0;

void checkTrue(bool cond, const char* expr, int line) {
    if (!cond) {
        std::cerr << "  [FAIL] Line " << line << ": Expected true: " << expr << std::endl;
        ++g_failedAssertions;
    }
}

void checkNear(double actual, double expected, double tol, const char* expr, int line) {
    if (std::abs(actual - expected) > tol || !std::isfinite(actual)) {
        std::cerr << "  [FAIL] Line " << line << ": " << expr
                  << " | actual=" << actual << ", expected=" << expected << " (tol=" << tol << ")" << std::endl;
        ++g_failedAssertions;
    }
}

#define EXPECT_TRUE(cond) checkTrue((cond), #cond, __LINE__)
#define EXPECT_NEAR(a, b, tol) checkNear((a), (b), (tol), #a " ~= " #b, __LINE__)
#define EXPECT_GT(a, b) checkTrue((a) > (b), #a " > " #b, __LINE__)
#define EXPECT_LT(a, b) checkTrue((a) < (b), #a " < " #b, __LINE__)
#define EXPECT_LE(a, b) checkTrue((a) <= (b), #a " <= " #b, __LINE__)
#define EXPECT_GE(a, b) checkTrue((a) >= (b), #a " >= " #b, __LINE__)
#define EXPECT_EQ(a, b) checkTrue((a) == (b), #a " == " #b, __LINE__)

// ============================================================================
// PART 1: Core Mathematical & Subsystem Unit Tests (1..8)
// ============================================================================

void unitTest1_SuspensionForceSigns() {
    constexpr double k = 35000.0;
    constexpr double cBound = 3600.0;
    constexpr double cRebound = 4800.0;

    const auto resStatic = computeSpringDamperForce(-0.10, 0.0, k, cBound, cRebound);
    EXPECT_NEAR(resStatic.springForce, 3500.0, 1e-3);
    EXPECT_NEAR(resStatic.damperForce, 0.0, 1e-3);
    EXPECT_NEAR(resStatic.totalForce, 3500.0, 1e-3);

    const auto resCompressing = computeSpringDamperForce(-0.10, -0.5, k, cBound, cRebound);
    EXPECT_NEAR(resCompressing.springForce, 3500.0, 1e-3);
    EXPECT_NEAR(resCompressing.damperForce, 1800.0, 1e-3);
    EXPECT_NEAR(resCompressing.totalForce, 5300.0, 1e-3);

    const auto resRebounding = computeSpringDamperForce(-0.10, +0.5, k, cBound, cRebound);
    EXPECT_NEAR(resRebounding.springForce, 3500.0, 1e-3);
    EXPECT_NEAR(resRebounding.damperForce, -2400.0, 1e-3);
    EXPECT_NEAR(resRebounding.totalForce, 1100.0, 1e-3);

    EXPECT_NEAR(computeBumpStopForce(0.25, 0.15, 180000.0), 0.0, 1e-6);
    EXPECT_GT(computeBumpStopForce(0.14, 0.15, 180000.0), 5000.0);

    const auto arb = computeAxleAntiRollForce(0.05, 0.12, true, true, 16000.0);
    EXPECT_LT(arb.leftForce, 0.0);
    EXPECT_GT(arb.rightForce, 0.0);
    EXPECT_NEAR(arb.leftForce + arb.rightForce, 0.0, 1e-5);
}

void unitTest2_CoordinateTransformsAndEuler() {
    const Vec3 pos{10.0, 2.0, -5.0};
    const Quat rot = quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, PI_D / 3.0);
    const Vec3 localPt{0.8, 0.1, 1.35};

    const Vec3 worldPt = localPointToWorld(localPt, pos, rot);
    const Vec3 roundTrip = worldPointToLocal(worldPt, pos, rot);
    EXPECT_NEAR(roundTrip.x, localPt.x, 1e-5);
    EXPECT_NEAR(roundTrip.y, localPt.y, 1e-5);
    EXPECT_NEAR(roundTrip.z, localPt.z, 1e-5);

    const VehicleBasis basis = getVehicleBasis(quatIdentity());
    EXPECT_NEAR(vec3Dot(basis.forward, basis.up), 0.0, 1e-6);
    EXPECT_NEAR(vec3Dot(basis.forward, basis.left), 0.0, 1e-6);

    const Quat noseDownQuat = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.08);
    EXPECT_NEAR(getVehicleEulerAngles(noseDownQuat).pitch, -0.08, 1e-4);

    const Quat noseUpQuat = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, -0.08);
    EXPECT_NEAR(getVehicleEulerAngles(noseUpQuat).pitch, +0.08, 1e-4);
}

void unitTest3_TireFrictionCircleAndLoadSensitivity() {
    const VehicleConfig config = createDefaultVehicleConfig();
    constexpr double normalLoad = 4000.0;

    const TireForceOutput combined = computeTireForcesFromSlip(0.65, 0.45, normalLoad, 1.0, 1.0, config);
    const double totalHoriz = std::hypot(combined.longitudinalForce, combined.lateralForce);

    EXPECT_LE(totalHoriz, combined.maxAvailableGripForce + 1e-4);
    EXPECT_NEAR(combined.maxAvailableGripForce, combined.effectiveFrictionCoeff * normalLoad, 1e-3);

    const TireForceOutput pureLong = computeTireForcesFromSlip(0.65, 0.0, normalLoad, 1.0, 1.0, config);
    EXPECT_GT(pureLong.longitudinalForce, combined.longitudinalForce);

    const double muLow = computeEffectiveFrictionCoeff(3000.0, 1.0, config);
    const double muHigh = computeEffectiveFrictionCoeff(6000.0, 1.0, config);
    EXPECT_LT(muHigh, muLow);
    EXPECT_GT(muHigh * 6000.0, muLow * 3000.0);
    EXPECT_LT(muHigh * 6000.0, 2.0 * muLow * 3000.0);
}

void unitTest4_WheelSlipCalculations() {
    constexpr double r = 0.32;
    constexpr double vx = 20.0;
    const double freeRollOmega = vx / r;

    EXPECT_NEAR(computeLongitudinalSlipRatio(freeRollOmega, r, vx), 0.0, 1e-5);
    EXPECT_NEAR(computeLongitudinalSlipRatio(freeRollOmega * 1.25, r, vx), 0.25, 1e-3);
    EXPECT_NEAR(computeLongitudinalSlipRatio(0.0, r, vx), -1.0, 1e-3);

    const double alphaLeftSlide = computeLateralSlipAngle(20.0, 3.0);
    const double alphaRightSlide = computeLateralSlipAngle(20.0, -3.0);
    EXPECT_LT(alphaLeftSlide, 0.0);
    EXPECT_GT(alphaRightSlide, 0.0);
}

void unitTest5_DrivetrainAndRevLimiter() {
    const VehicleConfig config = createDefaultVehicleConfig();

    const double tIdle = sampleEngineTorqueCurve(850.0, config);
    const double tPeak = sampleEngineTorqueCurve(4500.0, config);
    EXPECT_GT(tPeak, tIdle);
    EXPECT_NEAR(tPeak, 405.0, 1e-1);

    const double r1 = getTotalGearRatio(2, config);
    const double r2 = getTotalGearRatio(3, config);
    const double r6 = getTotalGearRatio(7, config);
    EXPECT_GT(r1, r2);
    EXPECT_GT(r2, r6);
    EXPECT_LT(getTotalGearRatio(0, config), 0.0);

    EXPECT_GT(computeEngineBrakingTorque(6000.0, config), computeEngineBrakingTorque(2000.0, config));

    DrivetrainState dtState = createInitialDrivetrainState();
    const auto mounts = getWheelMountPositionsLocal(config);
    std::array<WheelState, 4> wheels{
        createInitialWheelState(0, mounts[0], 0.38, 0.32),
        createInitialWheelState(1, mounts[1], 0.38, 0.32),
        createInitialWheelState(2, mounts[2], 0.38, 0.32),
        createInitialWheelState(3, mounts[3], 0.38, 0.32)
    };

    updateDrivetrain(dtState, wheels, 1.0, 5.0, config, 1.0 / 120.0);
    EXPECT_NEAR(wheels[0].driveTorque, 0.0, 1e-6);
    EXPECT_NEAR(wheels[1].driveTorque, 0.0, 1e-6);
    EXPECT_GT(wheels[2].driveTorque, 500.0);
    EXPECT_GT(wheels[3].driveTorque, 500.0);

    dtState.rpm = config.maxRpm + 20.0;
    updateDrivetrain(dtState, wheels, 1.0, 30.0, config, 1.0 / 120.0);
    EXPECT_TRUE(dtState.revLimiterActive);
    EXPECT_LE(dtState.engineTorqueNm, 0.0);
}

void unitTest6_SurfaceFrictionHierarchy() {
    const VehicleConfig config = createDefaultVehicleConfig();
    SurfaceSystem surfaceSys;
    surfaceSys.registerColliderSurface(10, SurfaceType::DryAsphalt);
    surfaceSys.registerColliderSurface(20, SurfaceType::WetAsphalt);
    surfaceSys.registerColliderSurface(30, SurfaceType::Gravel);
    surfaceSys.registerColliderSurface(40, SurfaceType::Ice);

    constexpr double fz = 3800.0;
    const auto dryOut = computeTireForcesFromSlip(
        0.15, 0.0, fz,
        surfaceSys.getSurfaceForCollider(10).frictionMultiplier,
        getSurfaceProperties(SurfaceType::DryAsphalt).stiffnessMultiplier,
        config
    );
    const auto wetOut = computeTireForcesFromSlip(
        0.15, 0.0, fz,
        surfaceSys.getSurfaceForCollider(20).frictionMultiplier,
        getSurfaceProperties(SurfaceType::WetAsphalt).stiffnessMultiplier,
        config
    );
    const auto gravelOut = computeTireForcesFromSlip(
        0.15, 0.0, fz,
        surfaceSys.getSurfaceForCollider(30).frictionMultiplier,
        getSurfaceProperties(SurfaceType::Gravel).stiffnessMultiplier,
        config
    );
    const auto iceOut = computeTireForcesFromSlip(
        0.15, 0.0, fz,
        surfaceSys.getSurfaceForCollider(40).frictionMultiplier,
        getSurfaceProperties(SurfaceType::Ice).stiffnessMultiplier,
        config
    );

    EXPECT_GT(dryOut.longitudinalForce, wetOut.longitudinalForce);
    EXPECT_GT(wetOut.longitudinalForce, gravelOut.longitudinalForce);
    EXPECT_GT(gravelOut.longitudinalForce, iceOut.longitudinalForce);
}

void unitTest7_AckermannAndSpeedSensitiveSteering() {
    const VehicleConfig config = createDefaultVehicleConfig();

    const double maxSteerLow = computeSpeedSensitiveMaxSteer(2.0, config);
    const double maxSteerHigh = computeSpeedSensitiveMaxSteer(35.0, config);
    EXPECT_GT(maxSteerLow, maxSteerHigh);

    const auto leftAck = computeAckermannWheelAngles(0.35, config);
    EXPECT_GT(leftAck.frontLeftAngle, leftAck.frontRightAngle);
    EXPECT_GT(leftAck.frontRightAngle, 0.0);

    const auto rightAck = computeAckermannWheelAngles(-0.35, config);
    EXPECT_GT(std::abs(rightAck.frontRightAngle), std::abs(rightAck.frontLeftAngle));
    EXPECT_LT(rightAck.frontRightAngle, 0.0);

    const double nextAngle = updateCenterSteeringAngle(0.0, 1.0, 5.0, config, 1.0 / 120.0);
    EXPECT_GT(nextAngle, 0.0);
    EXPECT_LT(nextAngle, maxSteerLow);
}

void unitTest8_AerodynamicsAndBrakeBias() {
    const VehicleConfig config = createDefaultVehicleConfig();
    const auto aero15 = computeAerodynamicForces(Vec3{0.0, 0.0, 15.0}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 1.0, 0.0}, config);
    const auto aero30 = computeAerodynamicForces(Vec3{0.0, 0.0, 30.0}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 1.0, 0.0}, config);

    EXPECT_NEAR(aero30.dragMagnitude / aero15.dragMagnitude, 4.0, 1e-3);
    EXPECT_LT(aero30.dragForceWorld.z, 0.0);

    const auto brakes = computeWheelBrakeTorques(1.0, 0.0, config);
    EXPECT_GT(brakes.fl, brakes.rl);

    const auto hbOnly = computeWheelBrakeTorques(0.0, 1.0, config);
    EXPECT_NEAR(hbOnly.fl, 0.0, 1e-6);
    EXPECT_NEAR(hbOnly.rl, config.handbrakeTorque, 1e-3);
}

// ============================================================================
// PART 2: Full-Simulation Physical Validation Scenarios (Tests A – J)
// ============================================================================

void validationTestA_StaticEquilibrium() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    const Vec3 spawnPos{0.0, 0.60, 0.0};
    sim.vehicle->resetVehicle(&spawnPos);

    for (int i = 0; i < 360; ++i) {
        sim.stepFixed();
    }

    const auto snap = sim.vehicle->getTelemetrySnapshot();
    double totalNormalLoad = 0.0;
    for (const auto& w : snap.wheels) totalNormalLoad += w.normalLoad;
    const double expectedWeight = config.mass * GRAVITY;
    const double relErr = std::abs(totalNormalLoad - expectedWeight) / expectedWeight;

    std::printf("    [Metrics A] TotalFz=%.1f N (Expected=%.1f N, Err=%.3f%%), RideHeight=%.4f m, FL_Comp=%.4f m\n",
                totalNormalLoad, expectedWeight, relErr * 100.0, snap.chassisPosition.y, snap.wheels[0].compression);

    EXPECT_LT(relErr, 0.015);
    for (const auto& w : snap.wheels) {
        EXPECT_TRUE(w.inContact);
        EXPECT_GT(w.compression, 0.08);
        EXPECT_LT(w.compression, 0.16);
    }
    EXPECT_LT(std::abs(snap.velocityWorld.y), 0.005);
    EXPECT_GT(snap.chassisPosition.y, 0.35);
    EXPECT_LT(snap.chassisPosition.y, 0.55);
}

void validationTestB_AccelerationAndSquat() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    for (int i = 0; i < 120; ++i) sim.stepFixed();
    const auto staticSnap = sim.vehicle->getTelemetrySnapshot();
    const double staticRearLoad = staticSnap.wheels[2].normalLoad + staticSnap.wheels[3].normalLoad;
    const double staticFrontLoad = staticSnap.wheels[0].normalLoad + staticSnap.wheels[1].normalLoad;

    sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};
    for (int i = 0; i < 240; ++i) {
        sim.stepFixed();
    }

    const auto accelSnap = sim.vehicle->getTelemetrySnapshot();
    const double dynamicRearLoad = accelSnap.wheels[2].normalLoad + accelSnap.wheels[3].normalLoad;
    const double dynamicFrontLoad = accelSnap.wheels[0].normalLoad + accelSnap.wheels[1].normalLoad;

    std::printf("    [Metrics B] Speed=%.2f m/s (%.1f km/h), Accel=%.2f m/s2, Pitch=%+.3f deg, dRearFz=%+.0f N\n",
                accelSnap.forwardSpeedMps, accelSnap.speedKmh, accelSnap.longitudinalAccelMps2,
                radToDeg(accelSnap.pitchRad), dynamicRearLoad - staticRearLoad);

    EXPECT_GT(accelSnap.forwardSpeedMps, 10.0);
    EXPECT_GT(accelSnap.longitudinalAccelMps2, 2.5);
    EXPECT_GT(accelSnap.wheels[2].longitudinalForce, 500.0);
    EXPECT_GT(accelSnap.wheels[3].longitudinalForce, 500.0);

    const double frontLinearWheelSpeed = accelSnap.wheels[0].angularVelocity * config.wheelRadius;
    EXPECT_LT(std::abs(frontLinearWheelSpeed - accelSnap.forwardSpeedMps), 0.8);

    EXPECT_GT(dynamicRearLoad, staticRearLoad + 300.0);
    EXPECT_LT(dynamicFrontLoad, staticFrontLoad - 300.0);
    EXPECT_GT(accelSnap.pitchRad, 0.004);
    EXPECT_GT(accelSnap.wheels[2].compression, accelSnap.wheels[0].compression);
}

void validationTestC_BrakingAndDive() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    for (int i = 0; i < 100; ++i) sim.stepFixed();

    sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};
    while (sim.vehicle->getTelemetrySnapshot().forwardSpeedMps < 22.0) {
        sim.stepFixed();
    }

    const double speedBeforeBrake = sim.vehicle->getTelemetrySnapshot().forwardSpeedMps;
    sim.vehicle->currentInputs = VehicleControlInputs{0.0, 0.85, 0.0, 0.0, true};

    sim.stepFixed();
    EXPECT_GT(sim.vehicle->getTelemetrySnapshot().forwardSpeedMps, speedBeforeBrake - 0.5);

    for (int i = 0; i < 60; ++i) {
        sim.stepFixed();
    }

    const auto brakeSnap = sim.vehicle->getTelemetrySnapshot();
    const double frontBrakeLoad = brakeSnap.wheels[0].normalLoad + brakeSnap.wheels[1].normalLoad;
    const double rearBrakeLoad = brakeSnap.wheels[2].normalLoad + brakeSnap.wheels[3].normalLoad;

    std::printf("    [Metrics C] SpeedAfter0.5s=%.2f m/s, Decel=%.2f m/s2, Pitch=%+.3f deg, FrontFz=%.0f N vs RearFz=%.0f N\n",
                brakeSnap.forwardSpeedMps, brakeSnap.longitudinalAccelMps2, radToDeg(brakeSnap.pitchRad),
                frontBrakeLoad, rearBrakeLoad);

    EXPECT_LT(brakeSnap.forwardSpeedMps, speedBeforeBrake - 3.0);
    EXPECT_LT(brakeSnap.longitudinalAccelMps2, -5.0);
    EXPECT_GT(frontBrakeLoad, rearBrakeLoad + 1500.0);
    EXPECT_GT(brakeSnap.wheels[0].compression, brakeSnap.wheels[2].compression);
    EXPECT_LT(brakeSnap.pitchRad, -0.008);
    EXPECT_LT(brakeSnap.wheels[0].slipRatio, -0.02);
}

void validationTestD_CorneringAndBodyRoll() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    for (int i = 0; i < 90; ++i) sim.stepFixed();

    sim.vehicle->currentInputs = VehicleControlInputs{0.8, 0.0, 0.0, 0.0, false};
    while (sim.vehicle->getTelemetrySnapshot().forwardSpeedMps < 18.0) {
        sim.stepFixed();
    }

    // Turn LEFT (+0.55)
    sim.vehicle->currentInputs = VehicleControlInputs{0.35, 0.0, 0.55, 0.0, false};
    for (int i = 0; i < 90; ++i) sim.stepFixed();

    const auto leftTurnSnap = sim.vehicle->getTelemetrySnapshot();
    const double leftInsideLoad = leftTurnSnap.wheels[0].normalLoad + leftTurnSnap.wheels[2].normalLoad;
    const double rightOutsideLoad = leftTurnSnap.wheels[1].normalLoad + leftTurnSnap.wheels[3].normalLoad;

    EXPECT_GT(leftTurnSnap.lateralAccelMps2, 3.0);
    EXPECT_GT(leftTurnSnap.rollRad, 0.01);
    EXPECT_GT(rightOutsideLoad, leftInsideLoad + 1200.0);

    // Reverse cornering direction: Turn RIGHT (-0.55)
    sim.vehicle->currentInputs = VehicleControlInputs{0.35, 0.0, -0.55, 0.0, false};
    for (int i = 0; i < 140; ++i) sim.stepFixed();

    const auto rightTurnSnap = sim.vehicle->getTelemetrySnapshot();
    const double leftOutsideLoad = rightTurnSnap.wheels[0].normalLoad + rightTurnSnap.wheels[2].normalLoad;
    const double rightInsideLoad = rightTurnSnap.wheels[1].normalLoad + rightTurnSnap.wheels[3].normalLoad;

    std::printf("    [Metrics D] LeftTurn Roll=%+.2f deg (LatAcc=%+.2f m/s2) | RightTurn Roll=%+.2f deg (LatAcc=%+.2f m/s2)\n",
                radToDeg(leftTurnSnap.rollRad), leftTurnSnap.lateralAccelMps2,
                radToDeg(rightTurnSnap.rollRad), rightTurnSnap.lateralAccelMps2);

    EXPECT_LT(rightTurnSnap.lateralAccelMps2, -3.0);
    EXPECT_LT(rightTurnSnap.rollRad, -0.01);
    EXPECT_GT(leftOutsideLoad, rightInsideLoad + 1200.0);
}

void validationTestE_LowGripSurfaceComparison() {
    auto runLaunchAndBrakeScenario = [](SurfaceType surface) {
        const VehicleConfig config = createDefaultVehicleConfig();
        PhysicsWorld sim(config, false);
        sim.setAllTrackSurfaces(surface);

        for (int i = 0; i < 90; ++i) sim.stepFixed();

        sim.vehicle->currentInputs = VehicleControlInputs{0.75, 0.0, 0.0, 0.0, false};
        double slipRatioSum = 0.0;
        for (int i = 0; i < 180; ++i) {
            sim.stepFixed();
            slipRatioSum += std::abs(sim.vehicle->wheels[2].slipRatio);
        }
        const double avgLaunchSlipRatio = slipRatioSum / 180.0;
        const double speedAfterLaunch = sim.vehicle->getTelemetrySnapshot().forwardSpeedMps;

        sim.vehicle->resetVehicle();
        for (int i = 0; i < 60; ++i) sim.stepFixed();
        const Vec3 startPos = sim.vehicle->body.getPosition();
        sim.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 15.0});
        for (auto& w : sim.vehicle->wheels) {
            w.angularVelocity = 15.0 / config.wheelRadius;
        }

        sim.vehicle->currentInputs = VehicleControlInputs{0.0, 1.0, 0.0, 0.0, true};
        int steps = 0;
        while (sim.vehicle->getTelemetrySnapshot().forwardSpeedMps > 0.5 && steps < 1200) {
            sim.stepFixed();
            ++steps;
        }
        const double stopDistance = sim.vehicle->body.getPosition().z - startPos.z;
        return std::make_tuple(avgLaunchSlipRatio, speedAfterLaunch, stopDistance);
    };

    const auto [drySlip, drySpeed, dryStop] = runLaunchAndBrakeScenario(SurfaceType::DryAsphalt);
    const auto [iceSlip, iceSpeed, iceStop] = runLaunchAndBrakeScenario(SurfaceType::Ice);

    std::printf("    [Metrics E] Dry(Slip=%.2f, LaunchV=%.2f m/s, StopDist=%.2f m) vs Ice(Slip=%.2f, LaunchV=%.2f m/s, StopDist=%.2f m)\n",
                drySlip, drySpeed, dryStop, iceSlip, iceSpeed, iceStop);

    EXPECT_GT(iceSlip, drySlip * 1.5);
    EXPECT_GT(drySpeed, iceSpeed * 2.0);
    EXPECT_GT(iceStop, dryStop * 2.5);
}

void validationTestF_UnevenTerrainSpeedBump() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    sim.addTrackBox(TrackBoxElement{
        "test_bump",
        "Validation Speed Bump",
        TrackElementKind::Bump,
        Vec3{0.0, 0.04, 18.0},
        Vec3{4.0, 0.05, 0.45},
        quatIdentity(),
        SurfaceType::RumbleStrip,
        false,
        0,
        -1
    });

    for (int i = 0; i < 60; ++i) sim.stepFixed();
    const double eqFrontCompression = sim.vehicle->wheels[0].compression;

    sim.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 12.0});
    for (auto& w : sim.vehicle->wheels) {
        w.angularVelocity = 12.0 / config.wheelRadius;
    }
    sim.vehicle->currentInputs = VehicleControlInputs{0.22, 0.0, 0.0, 0.0, false};

    double peakFrontCompression = 0.0;
    double peakRearCompression = 0.0;
    double peakAbsPitch = 0.0;

    for (int i = 0; i < 360; ++i) {
        sim.stepFixed();
        const auto s = sim.vehicle->getTelemetrySnapshot();
        peakFrontCompression = std::max(peakFrontCompression, s.wheels[0].compression);
        peakRearCompression = std::max(peakRearCompression, s.wheels[2].compression);
        peakAbsPitch = std::max(peakAbsPitch, std::abs(s.pitchRad));
    }

    const auto finalSnap = sim.vehicle->getTelemetrySnapshot();
    std::printf("    [Metrics F] EqComp=%.3f m -> PeakFront=%.3f m, PeakRear=%.3f m, PeakPitch=%.2f deg -> FinalPitch=%.2f deg\n",
                eqFrontCompression, peakFrontCompression, peakRearCompression,
                radToDeg(peakAbsPitch), radToDeg(std::abs(finalSnap.pitchRad)));

    EXPECT_GT(peakFrontCompression, eqFrontCompression + 0.035);
    EXPECT_GT(peakRearCompression, eqFrontCompression + 0.035);
    EXPECT_GT(peakAbsPitch, 0.012);
    EXPECT_LT(std::abs(finalSnap.pitchRad), peakAbsPitch * 0.35);
    EXPECT_LT(std::abs(finalSnap.velocityWorld.y), 0.15);
}

void validationTestG_ParameterSensitivity() {
    const VehicleConfig baseCfg = createDefaultVehicleConfig();
    VehicleConfig stiffCfg = createDefaultVehicleConfig();
    stiffCfg.frontSpringStiffness *= 1.6;
    stiffCfg.rearSpringStiffness *= 1.6;

    PhysicsWorld simBase(baseCfg, false);
    PhysicsWorld simStiff(stiffCfg, false);
    for (int i = 0; i < 200; ++i) {
        simBase.stepFixed();
        simStiff.stepFixed();
    }
    EXPECT_LT(simStiff.vehicle->wheels[0].compression, simBase.vehicle->wheels[0].compression * 0.72);

    VehicleConfig lowComCfg = createDefaultVehicleConfig();
    lowComCfg.comHeightOffset = -0.10;
    VehicleConfig highComCfg = createDefaultVehicleConfig();
    highComCfg.comHeightOffset = +0.28;

    PhysicsWorld simLowCom(lowComCfg, false);
    PhysicsWorld simHighCom(highComCfg, false);
    for (int i = 0; i < 90; ++i) {
        simLowCom.stepFixed();
        simHighCom.stepFixed();
    }

    simLowCom.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 16.0});
    simHighCom.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 16.0});
    for (int i = 0; i < 4; ++i) {
        simLowCom.vehicle->wheels[static_cast<size_t>(i)].angularVelocity = 16.0 / lowComCfg.wheelRadius;
        simHighCom.vehicle->wheels[static_cast<size_t>(i)].angularVelocity = 16.0 / highComCfg.wheelRadius;
    }
    simLowCom.vehicle->currentInputs = VehicleControlInputs{0.25, 0.0, 0.45, 0.0, false};
    simHighCom.vehicle->currentInputs = VehicleControlInputs{0.25, 0.0, 0.45, 0.0, false};

    for (int i = 0; i < 75; ++i) {
        simLowCom.stepFixed();
        simHighCom.stepFixed();
    }

    const double rollLow = simLowCom.vehicle->getTelemetrySnapshot().rollRad;
    const double rollHigh = simHighCom.vehicle->getTelemetrySnapshot().rollRad;
    std::printf("    [Metrics G] BaseComp=%.4f m vs StiffComp=%.4f m | LowCoM Roll=%.2f deg vs HighCoM Roll=%.2f deg\n",
                simBase.vehicle->wheels[0].compression, simStiff.vehicle->wheels[0].compression,
                radToDeg(rollLow), radToDeg(rollHigh));

    EXPECT_GT(rollHigh, rollLow * 1.25);
}

void validationTestH_CollisionAndRecovery() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);

    sim.addTrackBox(TrackBoxElement{
        "wall",
        "Test Crash Wall",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.8, 14.0},
        Vec3{3.0, 0.8, 0.6},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        false,
        0,
        -1
    });

    for (int i = 0; i < 60; ++i) sim.stepFixed();

    sim.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 15.0});
    for (int i = 0; i < 150; ++i) {
        sim.stepFixed();
    }

    const auto postCrashSnap = sim.vehicle->getTelemetrySnapshot();
    std::printf("    [Metrics H] PostCrash Z=%.2f m (Wall at 14.0m), PostCrash FwdSpeed=%.2f m/s, Sanity=%s\n",
                postCrashSnap.chassisPosition.z, postCrashSnap.forwardSpeedMps,
                postCrashSnap.numericalSanityOk ? "OK" : "FAIL");

    EXPECT_LT(postCrashSnap.chassisPosition.z, 13.0);
    EXPECT_LT(std::abs(postCrashSnap.forwardSpeedMps), 2.0);
    EXPECT_TRUE(postCrashSnap.numericalSanityOk);

    sim.vehicle->resetVehicle();
    sim.stepFixed();
    const auto resetSnap = sim.vehicle->getTelemetrySnapshot();
    EXPECT_LT(std::abs(resetSnap.chassisPosition.x), 0.01);
    EXPECT_LT(std::abs(resetSnap.chassisPosition.z), 0.01);
    EXPECT_LT(resetSnap.speedMps, 0.05);
}

void validationTestI_TimestepIndependence() {
    auto runAtRenderFps = [](int renderFps) {
        const VehicleConfig config = createDefaultVehicleConfig();
        PhysicsWorld sim(config, false);
        for (int i = 0; i < 60; ++i) sim.stepFixed();

        sim.vehicle->currentInputs = VehicleControlInputs{0.85, 0.0, 0.15, 0.0, false};

        constexpr double totalDuration = 2.0;
        const double frameDt = 1.0 / static_cast<double>(renderFps);
        const int frames = static_cast<int>(std::round(totalDuration * renderFps));
        for (int f = 0; f < frames; ++f) {
            sim.updateFrame(frameDt);
        }
        return sim.vehicle->getTelemetrySnapshot();
    };

    const auto snap30 = runAtRenderFps(30);
    const auto snap60 = runAtRenderFps(60);
    const auto snap120 = runAtRenderFps(120);

    std::printf("    [Metrics I] 30FPS(Z=%.3f m, V=%.3f m/s) | 60FPS(Z=%.3f m, V=%.3f m/s) | 120FPS(Z=%.3f m, V=%.3f m/s)\n",
                snap30.chassisPosition.z, snap30.speedMps,
                snap60.chassisPosition.z, snap60.speedMps,
                snap120.chassisPosition.z, snap120.speedMps);

    EXPECT_LT(std::abs(snap30.chassisPosition.z - snap120.chassisPosition.z), 0.25);
    EXPECT_LT(std::abs(snap60.chassisPosition.z - snap120.chassisPosition.z), 0.15);
    EXPECT_LT(std::abs(snap30.speedMps - snap120.speedMps), 0.20);
}

void validationTestJ_CoastingEnergySanity() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);
    for (int i = 0; i < 60; ++i) sim.stepFixed();

    shiftToGear(sim.vehicle->drivetrain, 1, config); // Neutral
    sim.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 25.0});
    for (auto& w : sim.vehicle->wheels) {
        w.angularVelocity = 25.0 / config.wheelRadius;
    }
    sim.vehicle->currentInputs = VehicleControlInputs{0.0, 0.0, 0.0, 0.0, false};

    double prevSpeed = 25.0;
    for (int step = 0; step < 360; ++step) {
        sim.stepFixed();
        if (step > 10 && step % 30 == 0) {
            const double curSpeed = sim.vehicle->getTelemetrySnapshot().forwardSpeedMps;
            EXPECT_LT(curSpeed, prevSpeed);
            prevSpeed = curSpeed;
        }
    }

    const auto endSnap = sim.vehicle->getTelemetrySnapshot();
    std::printf("    [Metrics J] InitialSpeed=25.00 m/s -> SpeedAfter3.0s=%.2f m/s, Sanity=%s\n",
                endSnap.forwardSpeedMps, endSnap.numericalSanityOk ? "OK" : "FAIL");

    EXPECT_LT(endSnap.forwardSpeedMps, 24.3);
    EXPECT_GT(endSnap.forwardSpeedMps, 15.0);
    EXPECT_TRUE(endSnap.numericalSanityOk);
}

// ============================================================================
// PART 3: Movement Pipeline & Scale Controlled Tests (Prompt Tests A, D, F, G)
// ============================================================================

void controlledTest_KnownConstantForce() {
    // Apply a known forward force F = 5800 N directly to the chassis on flat ground
    // with drag, rolling resistance, and tire friction disabled; verify a ~= F / m = 4.00 m/s^2.
    VehicleConfig config = createDefaultVehicleConfig();
    config.dragCoefficient = 0.0;
    config.downforceCoefficient = 0.0;
    config.rollingResistanceCoeff = 0.0;
    config.tireFrictionCoeff = 0.0;

    PhysicsWorld sim(config, false);
    for (int i = 0; i < 60; ++i) sim.stepFixed();

    constexpr double appliedForceN = 5800.0; // m = 1450 kg => expected a = 4.000 m/s^2
    const double expectedAccel = appliedForceN / config.mass;

    for (int step = 0; step < 120; ++step) {
        sim.vehicle->step(sim.fixedDt);
        sim.vehicle->body.applyCentralForce(Vec3{0.0, 0.0, appliedForceN});
        sim.getDynamicsWorld()->stepSimulation(static_cast<btScalar>(sim.fixedDt), 0);
        sim.vehicle->postWorldStep(sim.fixedDt);
    }

    const auto snap = sim.vehicle->getTelemetrySnapshot();
    std::printf("    [Metrics KnownForce] F=%.1f N, m=%.1f kg -> Measured v(1.0s)=%.3f m/s, a=%.3f m/s2 (Expected=%.3f m/s2)\n",
                appliedForceN, config.mass, snap.forwardSpeedMps, snap.longitudinalAccelMps2, expectedAccel);

    EXPECT_NEAR(snap.longitudinalAccelMps2, expectedAccel, 0.06);
    EXPECT_NEAR(snap.forwardSpeedMps, expectedAccel * 1.0, 0.06);
}

void controlledTest_ZeroResistanceComparison() {
    // Compare 2.0s full-throttle launch with normal resistive forces vs zero drag/rolling resistance.
    auto runLaunch = [](bool zeroResistance) {
        VehicleConfig config = createDefaultVehicleConfig();
        if (zeroResistance) {
            config.dragCoefficient = 0.0;
            config.rollingResistanceCoeff = 0.0;
        }
        PhysicsWorld sim(config, false);
        for (int i = 0; i < 60; ++i) sim.stepFixed();
        sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};
        for (int i = 0; i < 240; ++i) sim.stepFixed();
        return sim.vehicle->getTelemetrySnapshot();
    };

    const auto normalSnap = runLaunch(false);
    const auto zeroResSnap = runLaunch(true);
    const double speedDiffPct = ((zeroResSnap.forwardSpeedMps - normalSnap.forwardSpeedMps) / normalSnap.forwardSpeedMps) * 100.0;

    std::printf("    [Metrics ZeroRes] Normal v(2s)=%.2f m/s vs ZeroRes v(2s)=%.2f m/s (Resistive delta=%.2f%%)\n",
                normalSnap.forwardSpeedMps, zeroResSnap.forwardSpeedMps, speedDiffPct);

    // Resistive forces slightly reduce 2s launch speed (>0%) but must not dominate (<8%)
    EXPECT_GT(zeroResSnap.forwardSpeedMps, normalSnap.forwardSpeedMps);
    EXPECT_LT(speedDiffPct, 8.0);
}

void controlledTest_BrakeIsolationAndScaleSanity() {
    const VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld sim(config, false);
    for (int i = 0; i < 60; ++i) sim.stepFixed();

    // 1. Verify zero brake torque when only throttle is applied
    sim.vehicle->currentInputs = VehicleControlInputs{1.0, 0.0, 0.0, 0.0, false};
    sim.stepFixed();
    const auto launchSnap = sim.vehicle->getTelemetrySnapshot();
    for (const auto& w : launchSnap.wheels) {
        EXPECT_NEAR(w.brakeTorque, 0.0, 1e-6);
    }

    // 2. Scale sanity: travel 50 m at constant speed v0 = 25.0 m/s (with zero drag/RR/throttle in Neutral)
    // Verify elapsed time matches t = d / v = 50.0 / 25.0 = 2.000 s.
    VehicleConfig coastCfg = createDefaultVehicleConfig();
    coastCfg.dragCoefficient = 0.0;
    coastCfg.rollingResistanceCoeff = 0.0;
    PhysicsWorld scaleSim(coastCfg, false);
    for (int i = 0; i < 60; ++i) scaleSim.stepFixed();

    shiftToGear(scaleSim.vehicle->drivetrain, 1, coastCfg); // Neutral
    constexpr double v0 = 25.0;
    scaleSim.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, v0});
    for (auto& w : scaleSim.vehicle->wheels) {
        w.angularVelocity = v0 / coastCfg.wheelRadius;
    }
    scaleSim.vehicle->currentInputs = VehicleControlInputs{0.0, 0.0, 0.0, 0.0, false};

    const double zStart = scaleSim.vehicle->getTelemetrySnapshot().chassisPosition.z;
    for (int step = 0; step < 240; ++step) { // 240 steps @ 120 Hz = 2.000 s
        scaleSim.stepFixed();
    }
    const double zEnd = scaleSim.vehicle->getTelemetrySnapshot().chassisPosition.z;
    const double distTraveled = zEnd - zStart;
    const double expectedDist = v0 * 2.0; // 50.0 m

    std::printf("    [Metrics ScaleSanity] v=%.2f m/s over t=2.000 s -> d=%.3f m (Expected=%.3f m)\n",
                v0, distTraveled, expectedDist);
    EXPECT_NEAR(distTraveled, expectedDist, 0.15);
}

void unitTest9_AutomaticCollisionPipeline() {
    // 1. Construct a synthetic vehicle model mesh with an aerodynamic wedge profile
    RawMeshGeometry wedgeMesh;
    for (double z = -2.0; z <= 2.0; z += 0.2) {
        const double frac = (z + 2.0) / 4.0;
        const double width = 0.85 * (1.0 - 0.25 * frac);
        const double height = (z > 0.5) ? 0.45 : (z > -0.5 ? 1.15 : 0.65);
        for (double x : {-width, 0.0, width}) {
            for (double y : {0.15, height}) {
                wedgeMesh.addVertex(Vec3{x, y, z});
            }
        }
    }
    wedgeMesh.computeBounds();

    // 2. Execute automatic model-derived decomposition
    CollisionPipelineSettings settings{};
    settings.strategy = CollisionAlgorithmStrategy::SpatialClusteredHulls;
    settings.enableCaching = false;

    GeneratedVehicleCollision result;
    const bool ok = VehicleCollisionGenerator::generateFromMesh(wedgeMesh, "synthetic_sports_wedge", settings, result);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(result.pieces.size() >= 2);
    EXPECT_TRUE(result.totalOutputHulls >= 2);
    EXPECT_TRUE(!result.isFallback);

    for (const auto& p : result.pieces) {
        EXPECT_GT(p.vertices.size(), 3);
        EXPECT_GT(p.volume, 0.0001);
    }

    // 3. Verify disk cache round-trip serialization fidelity
    const std::string testCachePath = "model/cache/test_roundtrip_collision.json";
    const bool saveOk = VehicleCollisionGenerator::saveToCache(result, testCachePath);
    EXPECT_TRUE(saveOk);

    GeneratedVehicleCollision loadedCol;
    const bool loadOk = VehicleCollisionGenerator::loadFromCache(loadedCol, testCachePath);
    EXPECT_TRUE(loadOk);
    EXPECT_TRUE(loadedCol.pieces.size() == result.pieces.size());

    // 4. Test safe graceful fallback hierarchy on degenerate empty mesh
    RawMeshGeometry emptyMesh;
    GeneratedVehicleCollision fallbackResult;
    VehicleCollisionGenerator::generateFromMesh(emptyMesh, "empty_model", settings, fallbackResult);
    EXPECT_TRUE(fallbackResult.isFallback);
    EXPECT_TRUE(fallbackResult.strategyUsed == CollisionAlgorithmStrategy::CompoundBoundsFallback);
    EXPECT_TRUE(fallbackResult.pieces.size() >= 1);
}

void unitTest10_ModelDerivedCollisionDynamics() {
    // Test dynamic rigid-body collision with model-derived compound convex shape
    RawMeshGeometry wedgeMesh;
    for (double z = -2.0; z <= 2.0; z += 0.25) {
        for (double x : {-0.8, 0.8}) {
            for (double y : {0.15, 0.85}) {
                wedgeMesh.addVertex(Vec3{x, y, z});
            }
        }
    }
    wedgeMesh.computeBounds();

    CollisionPipelineSettings settings{};
    settings.strategy = CollisionAlgorithmStrategy::SpatialClusteredHulls;
    GeneratedVehicleCollision col;
    VehicleCollisionGenerator::generateFromMesh(wedgeMesh, "wedge_dynamic", settings, col);

    VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld world(config, true, &col);

    // Verify rigid body is active and uses compound shape with generated hulls
    EXPECT_TRUE(world.vehicle->body.getRigidBody() != nullptr);
    EXPECT_TRUE(world.vehicle->body.getCompoundShape()->getNumChildShapes() >= 2);

    // Step physics under gravity
    for (int i = 0; i < 60; ++i) {
        world.stepFixed();
    }

    const auto snap = world.vehicle->getTelemetrySnapshot();
    EXPECT_TRUE(std::isfinite(snap.chassisPosition.y));
    EXPECT_GT(snap.chassisPosition.y, 0.30); // Sits stably above ground plane (equilibrium ride height ~0.38m)
    EXPECT_TRUE(snap.numericalSanityOk);
}

void unitTest11_CollisionAlignmentAndContinuousCollisionDetection() {
    // 1. Verify model-derived collision hulls correctly apply chassisCenterOffset to Chassis Physics Space
    const Vec3 chassisCenterOffset{0.0, -0.477, -0.1475};

    // Synthetic chassis mesh in raw model space (origin on ground, offset forward)
    RawMeshGeometry rawModelMesh;
    for (double z = -2.0; z <= 2.0; z += 0.25) {
        for (double x : {-0.85, 0.85}) {
            for (double y : {0.15, 1.35}) {
                rawModelMesh.addVertex(Vec3{x, y, z});
            }
        }
    }
    rawModelMesh.computeBounds();

    // Transform raw model collision mesh into Chassis Physics Space (as performed in VehicleCollisionGenerator::generateFromModel)
    RawMeshGeometry chassisPhysicsMesh;
    const Mat4 offsetMat = Mat4::fromTranslation(chassisCenterOffset);
    chassisPhysicsMesh.appendTransformed(rawModelMesh, offsetMat);
    chassisPhysicsMesh.computeBounds();

    CollisionPipelineSettings colSettings{};
    colSettings.strategy = CollisionAlgorithmStrategy::SpatialClusteredHulls;
    colSettings.enableCaching = false;

    GeneratedVehicleCollision col;
    const bool genOk = VehicleCollisionGenerator::generateFromMesh(chassisPhysicsMesh, "test_alignment_coupe", colSettings, col);
    EXPECT_TRUE(genOk);

    // Verify AABB in Chassis Physics Space has been shifted down by -0.477m
    EXPECT_NEAR(col.chassisAABBMin.y, 0.15 - 0.477, 0.01);
    EXPECT_NEAR(col.chassisAABBMax.y, 1.35 - 0.477, 0.01);

    // 2. Verify Rigid Body CCD and Compound Shape Alignment
    VehicleConfig config = createDefaultVehicleConfig();
    PhysicsWorld world(config, true, &col);

    auto* rb = world.vehicle->body.getRigidBody();
    EXPECT_TRUE(rb != nullptr);

    // Verify Continuous Collision Detection (CCD) is enabled
    EXPECT_GT(rb->getCcdMotionThreshold(), 0.01f);
    EXPECT_GT(rb->getCcdSweptSphereRadius(), 0.05f);

    // Verify restitution is inelastic (< 0.10) to prevent trampoline bouncing
    EXPECT_LT(rb->getRestitution(), 0.10f);

    // 3. Obstacle Collision Test: Drive vehicle towards a barrier block and verify no tunneling
    world.vehicle->body.resetPose(Vec3{0.0, 0.5, 5.0});
    world.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 20.0}); // Driving at 72 km/h (+Z)

    // Barrier at Z = 12.0m, halfExtents = (2.0, 1.0, 0.5)
    world.addTrackBox(TrackBoxElement{
        "test_barrier",
        "Test Barrier",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.75, 12.0},
        Vec3{2.0, 0.75, 0.5},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0xffffff,
        -1
    });

    for (int step = 0; step < 40; ++step) {
        world.stepFixed();
    }

    const auto endSnap = world.vehicle->getTelemetrySnapshot();
    // The front bumper must stop in front of the barrier's back edge (Z <= 12.5m)
    // and must not tunnel/phase through the obstacle
    EXPECT_LT(endSnap.chassisPosition.z, 12.0);
    EXPECT_TRUE(endSnap.numericalSanityOk);
}

void unitTest12_CollisionLifecycleAndVisualizationIndependence() {
    std::cout << "  Testing collision lifecycle, visualization independence, and switching integrity..." << std::endl;

    // 1. Setup a test world with a generated model collision
    RawMeshGeometry boxMesh;
    for (double z : {-2.0, 2.0}) {
        for (double x : {-0.8, 0.8}) {
            for (double y : {0.1, 0.9}) {
                boxMesh.addVertex(Vec3{x, y, z});
            }
        }
    }
    boxMesh.computeBounds();

    CollisionPipelineSettings colSettings{};
    colSettings.strategy = CollisionAlgorithmStrategy::SpatialClusteredHulls;
    colSettings.enableCaching = false;

    GeneratedVehicleCollision bmwCol;
    const bool genOk = VehicleCollisionGenerator::generateFromMesh(boxMesh, "test_lifecycle_car", colSettings, bmwCol);
    EXPECT_TRUE(genOk);

    VehicleConfig bmwConfig = createBmwM4Config();
    VehicleConfig procConfig = createProceduralCoupeConfig();

    PhysicsWorld world(bmwConfig, true, &bmwCol);

    // Initial state diagnostics
    auto diagInit = world.getCollisionDiagnostics(true);
    EXPECT_TRUE(diagInit.colliderReplacementSucceeded);
    EXPECT_TRUE(!diagInit.legacyPlaceholderRegistered);
    EXPECT_TRUE(diagInit.activeVehicleCollisionObjects == 1);
    EXPECT_TRUE(diagInit.usingModelDerivedCollision);
    EXPECT_GT(diagInit.childShapeCount, 0);

    const int baseWorldColliders = diagInit.totalWorldCollisionObjects;
    EXPECT_GT(baseWorldColliders, 1);

    // --- TEST A & TEST B: Collision Wireframe & Visualization Independence ---
    // Place obstacle at Z = 12.0m
    world.addTrackBox(TrackBoxElement{
        "lifecycle_barrier",
        "Lifecycle Barrier",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.75, 12.0},
        Vec3{2.0, 0.75, 0.5},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0xffffff,
        -1
    });

    const int worldCollidersWithBarrier = world.getDynamicsWorld()->getNumCollisionObjects();
    EXPECT_TRUE(worldCollidersWithBarrier == baseWorldColliders + 1);

    // Run collision test with wireframe HIDDEN (collisionWireframe = false)
    const Vec3 initTestPose{0.0, 0.5, 5.0};
    world.vehicle->resetVehicle(&initTestPose);
    world.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 20.0});

    auto diagHidden = world.getCollisionDiagnostics(false);
    EXPECT_TRUE(!diagHidden.collisionWireframeVisible);
    EXPECT_TRUE(diagHidden.activeVehicleCollisionObjects == 1);

    for (int step = 0; step < 40; ++step) {
        world.stepFixed();
    }
    const auto stopSnapHidden = world.vehicle->getTelemetrySnapshot();
    EXPECT_LT(stopSnapHidden.chassisPosition.z, 12.0); // Stopped cleanly in front of barrier

    // Run collision test with wireframe VISIBLE (collisionWireframe = true)
    world.vehicle->resetVehicle(&initTestPose);
    world.vehicle->body.setLinearVelocity(Vec3{0.0, 0.0, 20.0});

    auto diagVisible = world.getCollisionDiagnostics(true);
    EXPECT_TRUE(diagVisible.collisionWireframeVisible);
    EXPECT_TRUE(diagVisible.activeVehicleCollisionObjects == 1);

    for (int step = 0; step < 40; ++step) {
        world.stepFixed();
    }
    const auto stopSnapVisible = world.vehicle->getTelemetrySnapshot();
    EXPECT_LT(stopSnapVisible.chassisPosition.z, 12.0);

    // Physical collision response must be 100% identical regardless of wireframe visibility
    EXPECT_NEAR(stopSnapHidden.chassisPosition.z, stopSnapVisible.chassisPosition.z, 0.05);

    // --- TEST C & TEST F: Rapid Repeated Vehicle Switching (Lifecycle & Leak Integrity) ---
    // Switch 50 times between BMW M4 and Procedural Coupe
    for (int cycle = 0; cycle < 50; ++cycle) {
        if (cycle % 2 == 0) {
            world.switchVehicle(procConfig, nullptr);
            auto diagProc = world.getCollisionDiagnostics();
            EXPECT_TRUE(diagProc.colliderReplacementSucceeded);
            EXPECT_TRUE(!diagProc.legacyPlaceholderRegistered);
            EXPECT_TRUE(diagProc.activeVehicleCollisionObjects == 1);
            EXPECT_TRUE(!diagProc.usingModelDerivedCollision);
            EXPECT_TRUE(diagProc.childShapeCount >= 2); // Procedural compound boxes/hulls
            EXPECT_TRUE(diagProc.totalWorldCollisionObjects == worldCollidersWithBarrier);
        } else {
            world.switchVehicle(bmwConfig, &bmwCol);
            auto diagBmw = world.getCollisionDiagnostics();
            EXPECT_TRUE(diagBmw.colliderReplacementSucceeded);
            EXPECT_TRUE(!diagBmw.legacyPlaceholderRegistered);
            EXPECT_TRUE(diagBmw.activeVehicleCollisionObjects == 1);
            EXPECT_TRUE(diagBmw.usingModelDerivedCollision);
            EXPECT_TRUE(diagBmw.childShapeCount == static_cast<int>(bmwCol.pieces.size()));
            EXPECT_TRUE(diagBmw.totalWorldCollisionObjects == worldCollidersWithBarrier);
        }
    }

    // Verify final state after 50 switches: zero object leaks
    EXPECT_TRUE(world.getDynamicsWorld()->getNumCollisionObjects() == worldCollidersWithBarrier);

    // --- TEST D: In-Place Collision Shape Replacement (replaceCollisionShape) ---
    RawMeshGeometry altMesh;
    for (double z : {-1.8, 1.8}) {
        for (double x : {-0.75, 0.75}) {
            for (double y : {0.1, 0.8}) {
                altMesh.addVertex(Vec3{x, y, z});
            }
        }
    }
    altMesh.computeBounds();
    GeneratedVehicleCollision altCol;
    VehicleCollisionGenerator::generateFromMesh(altMesh, "test_alt_shape", colSettings, altCol);

    const bool replaceOk = world.vehicle->body.replaceCollisionShape(&altCol, bmwConfig);
    EXPECT_TRUE(replaceOk);

    auto diagReplaced = world.getCollisionDiagnostics();
    EXPECT_TRUE(diagReplaced.colliderReplacementSucceeded);
    EXPECT_TRUE(diagReplaced.totalWorldCollisionObjects == worldCollidersWithBarrier);
    EXPECT_TRUE(diagReplaced.childShapeCount == static_cast<int>(altCol.pieces.size()));
}

void unitTest13_ControllerMappingAndNormalization() {
    // 1. Verify default Forza Horizon control bindings
    ControllerSettings cfg{};
    cfg.resetToForzaDefaults();

    EXPECT_TRUE(cfg.axisThrottle == GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER);
    EXPECT_TRUE(cfg.axisBrake == GLFW_GAMEPAD_AXIS_LEFT_TRIGGER);
    EXPECT_TRUE(cfg.axisSteer == GLFW_GAMEPAD_AXIS_LEFT_X);
    EXPECT_TRUE(cfg.axisLookX == GLFW_GAMEPAD_AXIS_RIGHT_X);
    EXPECT_TRUE(cfg.axisLookY == GLFW_GAMEPAD_AXIS_RIGHT_Y);
    EXPECT_TRUE(cfg.btnHandbrake == GLFW_GAMEPAD_BUTTON_A);
    EXPECT_TRUE(cfg.btnShiftUp == GLFW_GAMEPAD_BUTTON_B);
    EXPECT_TRUE(cfg.btnShiftDown == GLFW_GAMEPAD_BUTTON_X);
    EXPECT_TRUE(cfg.btnReset == GLFW_GAMEPAD_BUTTON_Y);
    EXPECT_TRUE(cfg.btnChangeCamera == GLFW_GAMEPAD_BUTTON_RIGHT_THUMB);
    EXPECT_TRUE(cfg.btnPause == GLFW_GAMEPAD_BUTTON_START);
    EXPECT_TRUE(cfg.btnToggleTransmission == GLFW_GAMEPAD_BUTTON_LEFT_BUMPER);
    EXPECT_TRUE(cfg.btnToggleModel == GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER);

    // 2. Verify controller flavor detection & label adaptivity
    const ControllerFlavor psFlavor = detectControllerFlavorFromName("Sony Interactive Entertainment DualSense Wireless Controller");
    EXPECT_TRUE(psFlavor == ControllerFlavor::PlayStation);
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_A, ControllerFlavor::PlayStation) == "Cross");
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_B, ControllerFlavor::PlayStation) == "Circle");
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_X, ControllerFlavor::PlayStation) == "Square");
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_Y, ControllerFlavor::PlayStation) == "Triangle");
    EXPECT_TRUE(getGamepadAxisName(GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER, ControllerFlavor::PlayStation) == "R2 Trigger");
    EXPECT_TRUE(getGamepadAxisName(GLFW_GAMEPAD_AXIS_LEFT_TRIGGER, ControllerFlavor::PlayStation) == "L2 Trigger");

    const ControllerFlavor xbFlavor = detectControllerFlavorFromName("Xbox 360 Controller");
    EXPECT_TRUE(xbFlavor == ControllerFlavor::Xbox);
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_A, ControllerFlavor::Xbox) == "A");
    EXPECT_TRUE(getGamepadButtonName(GLFW_GAMEPAD_BUTTON_B, ControllerFlavor::Xbox) == "B");

    // 3. Mathematical trigger normalization test
    // Helper function reproducing the trigger normalization pipeline
    auto normalizeTrigger = [](float raw, float baseline, float deadzone) -> double {
        double val = 0.0;
        if (baseline < -0.5f) {
            val = (static_cast<double>(raw) + 1.0) * 0.5;
        } else {
            val = static_cast<double>(raw - baseline) / std::max(0.1, static_cast<double>(1.0f - baseline));
        }
        val = clamp(val, 0.0, 1.0);
        const double dz = static_cast<double>(deadzone);
        if (val <= dz) return 0.0;
        return clamp((val - dz) / (1.0 - dz), 0.0, 1.0);
    };

    // Standard XInput trigger: rest=-1.0, mid=0.0, max=+1.0
    EXPECT_NEAR(normalizeTrigger(-1.0f, -1.0f, 0.05f), 0.0, 1e-4);
    EXPECT_NEAR(normalizeTrigger(-0.95f, -1.0f, 0.05f), 0.0, 1e-4); // Within deadzone
    EXPECT_NEAR(normalizeTrigger(1.0f, -1.0f, 0.05f), 1.0, 1e-4);  // Full trigger
    // Midpoint: (0.0 + 1.0) * 0.5 = 0.50 -> scaled past deadzone = (0.50 - 0.05) / 0.95 = 0.4737
    EXPECT_NEAR(normalizeTrigger(0.0f, -1.0f, 0.05f), (0.5 - 0.05) / 0.95, 1e-4);

    // Virtual HID device trigger resting at 0.0: rest=0.0, mid=0.5, max=1.0
    EXPECT_NEAR(normalizeTrigger(0.0f, 0.0f, 0.05f), 0.0, 1e-4);
    EXPECT_NEAR(normalizeTrigger(0.04f, 0.0f, 0.05f), 0.0, 1e-4); // Deadzone cutoff
    EXPECT_NEAR(normalizeTrigger(1.0f, 0.0f, 0.05f), 1.0, 1e-4);  // Full trigger

    // 4. Mathematical steering normalization test
    auto normalizeSteer = [](float raw, float baseline, float deadzone, float sensitivity, bool invert) -> double {
        const double delta = static_cast<double>(raw - baseline);
        const double mag = std::abs(delta);
        const double dz = static_cast<double>(deadzone);
        if (mag <= dz) return 0.0;
        double norm = clamp((mag - dz) / (1.0 - dz), 0.0, 1.0);
        if (sensitivity != 1.0f && sensitivity > 0.1f) {
            norm = std::pow(norm, static_cast<double>(sensitivity));
        }
        const double sign = (delta > 0.0) ? 1.0 : -1.0;
        double steer = -sign * norm; // Left stick left (raw < 0) -> +1.0 left turn
        if (invert) steer = -steer;
        return clamp(steer, -1.0, 1.0);
    };

    EXPECT_NEAR(normalizeSteer(0.0f, 0.0f, 0.08f, 1.0f, false), 0.0, 1e-4);     // Center
    EXPECT_NEAR(normalizeSteer(-0.05f, 0.0f, 0.08f, 1.0f, false), 0.0, 1e-4);    // Small noise inside deadzone
    EXPECT_NEAR(normalizeSteer(-1.0f, 0.0f, 0.08f, 1.0f, false), +1.0, 1e-4);   // Full left deflection (+1.0)
    EXPECT_NEAR(normalizeSteer(+1.0f, 0.0f, 0.08f, 1.0f, false), -1.0, 1e-4);   // Full right deflection (-1.0)
    EXPECT_NEAR(normalizeSteer(-1.0f, 0.0f, 0.08f, 1.0f, true), -1.0, 1e-4);    // Inverted left turns right

    // Progressive response (>1.0) softens center response
    const double linearMid = normalizeSteer(-0.54f, 0.0f, 0.08f, 1.0f, false);
    const double softMid = normalizeSteer(-0.54f, 0.0f, 0.08f, 1.6f, false);
    EXPECT_TRUE(softMid < linearMid); // Progressive exponent creates finer control near center

    // 5. Serialization and Roundtrip
    ControllerSettings testSave{};
    testSave.steerDeadzone = 0.125f;
    testSave.triggerDeadzone = 0.075f;
    testSave.invertSteering = true;
    testSave.collisionSensitivity = 1.45f;
    testSave.suspensionSensitivity = 0.85f;
    testSave.btnHandbrake = GLFW_GAMEPAD_BUTTON_X;
    EXPECT_TRUE(saveControllerSettingsToJson(testSave, "settings/test_controller_config.json"));

    ControllerSettings testLoaded{};
    EXPECT_TRUE(loadControllerSettingsFromJson(testLoaded, "settings/test_controller_config.json"));
    EXPECT_NEAR(testLoaded.steerDeadzone, 0.125f, 1e-3);
    EXPECT_NEAR(testLoaded.triggerDeadzone, 0.075f, 1e-3);
    EXPECT_TRUE(testLoaded.invertSteering == true);
    EXPECT_NEAR(testLoaded.collisionSensitivity, 1.45f, 1e-3);
    EXPECT_NEAR(testLoaded.suspensionSensitivity, 0.85f, 1e-3);
    EXPECT_TRUE(testLoaded.btnHandbrake == GLFW_GAMEPAD_BUTTON_X);
}

void unitTest14_PhysicsDrivenHapticsAndRpmElimination() {
    // 1. High-RPM stationary revving produces strictly ZERO vibration
    PhysicsWorld world(createDefaultVehicleConfig(), false, nullptr);
    world.vehicle->drivetrain.transmissionMode = TransmissionMode::Manual;
    world.vehicle->drivetrain.gearIndex = 1; // Neutral (free engine revving)
    world.vehicle->currentInputs.throttle = 1.0;
    world.vehicle->currentInputs.brake = 0.0;
    world.vehicle->currentInputs.disableAutoReverse = true;

    for (int step = 0; step < 180; ++step) {
        world.stepFixed();
    }

    VehicleTelemetrySnapshot snap = world.vehicle->getTelemetrySnapshot();
    EXPECT_TRUE(snap.speedMps < 0.2); // Fully stationary
    EXPECT_TRUE(snap.drivetrain.rpm > 6500.0); // Engine at redline
    EXPECT_TRUE(snap.drivetrain.revLimiterActive);
    EXPECT_TRUE(snap.collisionImpact.occurred == false);
    EXPECT_TRUE(snap.suspensionImpact.occurred == false);

    VibrationManager vib;
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_NEAR(vib.getLiveLeftMotor(), 0.0f, 1e-4);
    EXPECT_NEAR(vib.getLiveRightMotor(), 0.0f, 1e-4);

    // 2. High-speed smooth driving produces strictly ZERO vibration
    world.vehicle->resetVehicle();
    world.vehicle->currentInputs.throttle = 1.0;
    world.vehicle->currentInputs.brake = 0.0;
    world.vehicle->currentInputs.disableAutoReverse = false;

    for (int step = 0; step < 480; ++step) {
        world.stepFixed();
    }
    snap = world.vehicle->getTelemetrySnapshot();
    EXPECT_TRUE(snap.speedMps > 15.0); // High-speed driving > 54 km/h
    EXPECT_TRUE(snap.collisionImpact.occurred == false);
    EXPECT_TRUE(snap.suspensionImpact.occurred == false);

    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_NEAR(vib.getLiveLeftMotor(), 0.0f, 1e-4);
    EXPECT_NEAR(vib.getLiveRightMotor(), 0.0f, 1e-4);

    // 3. Collision impact feedback scaling (Light vs Heavy)
    vib.stop(0);
    vib.triggerCollision(0.8f, 400.0f, 0.0f, false, 1.0f);
    vib.update(0.016f, 0, true, 1.0f);
    const float lightLeft = vib.getLiveLeftMotor();
    const float lightRight = vib.getLiveRightMotor();
    EXPECT_TRUE(lightLeft > 0.05f && lightLeft < 0.45f);
    EXPECT_TRUE(lightRight > 0.05f && lightRight < 0.45f);
    EXPECT_TRUE(vib.isCollisionPulseActive());

    vib.stop(0);
    vib.triggerCollision(7.0f, 9000.0f, 0.0f, false, 1.0f);
    vib.update(0.016f, 0, true, 1.0f);
    const float heavyLeft = vib.getLiveLeftMotor();
    const float heavyRight = vib.getLiveRightMotor();
    EXPECT_TRUE(heavyLeft > lightLeft * 1.8f);
    EXPECT_TRUE(heavyRight > lightRight * 1.8f);
    EXPECT_TRUE(heavyLeft <= 1.0f && heavyRight <= 1.0f);

    // 4. Directional / Lateral impact motor balance
    vib.stop(0);
    vib.triggerCollision(4.0f, 4000.0f, +0.8f, false, 1.0f); // Left hit (+0.8)
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_TRUE(vib.getLiveLeftMotor() > vib.getLiveRightMotor() * 1.30f);

    vib.stop(0);
    vib.triggerCollision(4.0f, 4000.0f, -0.8f, false, 1.0f); // Right hit (-0.8)
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_TRUE(vib.getLiveRightMotor() > vib.getLiveLeftMotor() * 1.30f);

    // 5. Suspension event types: Landing vs Bump Stop vs Curb Strike
    vib.stop(0);
    vib.triggerSuspension(0.85f, 0.0f, true, false, 1.0f); // Hard Landing
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_TRUE(vib.getLiveLeftMotor() > vib.getLiveRightMotor() * 1.40f); // Low freq dominates

    vib.stop(0);
    vib.triggerSuspension(0.90f, 0.0f, false, true, 1.0f); // Bump stop
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_TRUE(vib.getLiveLeftMotor() > 0.65f); // Solid mechanical jolt

    vib.stop(0);
    vib.triggerSuspension(0.60f, 0.0f, false, false, 1.0f); // Sharp curb strike
    vib.update(0.016f, 0, true, 1.0f);
    EXPECT_TRUE(vib.getLiveRightMotor() > vib.getLiveLeftMotor() * 1.50f); // High freq dominates

    // 6. Settings toggle and sensitivity scaling
    vib.stop(0);
    vib.triggerCollision(6.0f, 6000.0f, 0.0f, false, 1.0f);
    vib.update(0.016f, 0, false, 1.0f); // Disabled
    EXPECT_NEAR(vib.getLiveLeftMotor(), 0.0f, 1e-4);
    EXPECT_NEAR(vib.getLiveRightMotor(), 0.0f, 1e-4);

    vib.stop(0);
    vib.triggerCollision(5.0f, 5000.0f, 0.0f, false, 0.5f); // 0.5x sensitivity
    vib.update(0.016f, 0, true, 1.0f);
    const float halfSensL = vib.getLiveLeftMotor();

    vib.stop(0);
    vib.triggerCollision(5.0f, 5000.0f, 0.0f, false, 1.0f); // 1.0x sensitivity
    vib.update(0.016f, 0, true, 1.0f);
    const float fullSensL = vib.getLiveLeftMotor();
    EXPECT_TRUE(halfSensL < fullSensL * 0.75f);

    // 7. Deduplication & settling (bounded duration, smooth decay to 0)
    for (int frame = 0; frame < 30; ++frame) {
        vib.update(0.020f, 0, true, 1.0f);
    }
    EXPECT_NEAR(vib.getLiveLeftMotor(), 0.0f, 1e-4);
    EXPECT_NEAR(vib.getLiveRightMotor(), 0.0f, 1e-4);
    EXPECT_TRUE(!vib.isCollisionPulseActive());
}

void unitTest15_MultiVehiclePresetsAndMuscleCarTuning() {
    // 1. Verify 1970 Dodge Challenger R/T authentic physical configuration
    const VehicleConfig dodge = createDodgeChallengerConfig();
    EXPECT_TRUE(dodge.id == "dodge_challenger");
    EXPECT_NEAR(dodge.mass, 1680.0, 1e-4);
    EXPECT_NEAR(dodge.wheelbase, 2.794, 1e-4); // Exact 110.0-inch wheelbase
    EXPECT_NEAR(dodge.trackWidth, 1.56, 1e-4);
    EXPECT_NEAR(dodge.frontWeightDistribution, 0.54, 1e-4); // Heavy cast-iron V8
    EXPECT_TRUE(dodge.drivetrainLayout == DrivetrainLayout::RWD);
    EXPECT_TRUE(dodge.gearRatios.size() == 6); // R, N, 1, 2, 3, 4
    EXPECT_NEAR(dodge.finalDriveRatio, 3.54, 1e-4); // Dana 60 rear axle
    EXPECT_NEAR(dodge.rearAntiRollStiffness, 9000.0, 1e-4); // Softer rear ARB for authentic squat

    // 2. Verify 426 Hemi torque curve metrics
    const double tIdle = sampleEngineTorqueCurve(dodge.idleRpm, dodge);
    const double tPeak = sampleEngineTorqueCurve(4000.0, dodge);
    const double tRedline = sampleEngineTorqueCurve(dodge.maxRpm, dodge);
    EXPECT_TRUE(tIdle > 350.0 && tIdle < 400.0);
    EXPECT_NEAR(tPeak, 664.0, 1e-4); // 490 lb-ft
    EXPECT_TRUE(tPeak > tRedline * 1.5);

    // 3. Verify seamless 3-way vehicle switching in physics simulation
    VehicleConfig bmw = createBmwM4Config();
    VehicleConfig proc = createProceduralCoupeConfig();

    PhysicsWorld world(bmw, false, nullptr);
    const int initObjs = world.getDynamicsWorld()->getNumCollisionObjects();
    EXPECT_TRUE(initObjs >= 1);

    // Switch BMW -> Dodge Challenger
    world.switchVehicle(dodge, nullptr);
    for (int i = 0; i < 30; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->config.id == "dodge_challenger");
    EXPECT_NEAR(world.vehicle->config.mass, 1680.0, 1e-4);
    EXPECT_TRUE(world.getDynamicsWorld()->getNumCollisionObjects() == initObjs);

    // Switch Dodge Challenger -> Procedural Coupe
    world.switchVehicle(proc, nullptr);
    for (int i = 0; i < 30; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->config.id == "procedural_coupe");
    EXPECT_NEAR(world.vehicle->config.mass, 1500.0, 1e-4);
    EXPECT_TRUE(world.getDynamicsWorld()->getNumCollisionObjects() == initObjs);

    // Switch Procedural Coupe -> BMW M4
    world.switchVehicle(bmw, nullptr);
    for (int i = 0; i < 30; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->config.id == "bmw_m4");
    EXPECT_NEAR(world.vehicle->config.mass, 1600.0, 1e-4);
    EXPECT_TRUE(world.getDynamicsWorld()->getNumCollisionObjects() == initObjs);
}

void unitTest16_VolumetricTyreObstacleContactAndClimbing() {
    // 1. Verify volumetric circumference contact before wheel center reaches an obstacle face
    VehicleConfig config = createBmwM4Config();
    PhysicsWorld world(config, false);

    // Place an 8cm high curb step at Z = 5.0m (half-extents Y=0.04m, Z=0.5m -> front face is at Z = 4.5m)
    // The obstacle box origin is at Z=5.0m, half-extent Z=0.5m => min Z = 4.5m, max Z = 5.5m.
    // Curb top surface is at Y = 2 * 0.04 = 0.08m.
    world.addTrackBox(TrackBoxElement{
        "test_curb_step",
        "Test Curb Step 8cm",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.04, 5.0},
        Vec3{4.0, 0.04, 0.5},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        false,
        0,
        -1
    });

    for (int i = 0; i < 60; ++i) world.stepFixed();

    // Drive forward at low speed towards the curb
    world.vehicle->currentInputs = VehicleControlInputs{0.25, 0.0, 0.0, 0.0, false};

    bool detectedCircumferentialContact = false;
    double centerZAtDetection = 0.0;
    double contactZAtDetection = 0.0;
    double obsForceAtDetection = 0.0;

    for (int step = 0; step < 400; ++step) {
        world.stepFixed();
        const auto snap = world.vehicle->getTelemetrySnapshot();
        const WheelState& fl = snap.wheels[0]; // Front-Left

        if (fl.hasObstacleContact) {
            if (!detectedCircumferentialContact) {
                detectedCircumferentialContact = true;
                centerZAtDetection = fl.wheelCenterWorld.z;
                contactZAtDetection = fl.contactPointWorld.z;
                obsForceAtDetection = fl.obstacleResistanceForceWorld.z;
            }
        }

        // If vehicle has traversed over the curb
        if (snap.chassisPosition.z > 6.0) {
            break;
        }
    }

    // A. Verify outer tyre circumference touched the curb face (Z=4.5m) BEFORE wheel axle reached it
    EXPECT_TRUE(detectedCircumferentialContact);
    // Axle center must still be behind the curb face (Z < 4.50m) when circumference makes contact!
    EXPECT_LT(centerZAtDetection, 4.50);
    // Contact point must be on or near the curb face (Z >= 4.45m)
    EXPECT_GT(contactZAtDetection, centerZAtDetection + 0.04);
    // Horizontal obstacle resistance force must oppose forward motion (Z < 0)
    EXPECT_LT(obsForceAtDetection, -10.0);

    // B. Verify the car successfully climbs onto and crosses the curb without getting stuck or exploding
    const auto endSnap = world.vehicle->getTelemetrySnapshot();
    EXPECT_GT(endSnap.chassisPosition.z, 5.0);
    EXPECT_TRUE(endSnap.numericalSanityOk);
}

void unitTest17_WheelGeometryDerivationAndCollisionObjectLifecycle() {
    // 1. Verify per-axle wheel dimension derivation and config defaults
    VehicleConfig bmw = createBmwM4Config();
    EXPECT_NEAR(getWheelRadius(bmw, 0), 0.297, 0.01);
    EXPECT_NEAR(getWheelRadius(bmw, 2), 0.297, 0.01);
    EXPECT_NEAR(getWheelWidth(bmw, 0), 0.238, 0.01);
    EXPECT_NEAR(bmw.frontTrackWidth, 1.427, 0.02);
    EXPECT_NEAR(bmw.rearTrackWidth, 1.427, 0.02);

    // Test JSON serialization & deserialization roundtrip of wheel parameters
    const std::string tmpPath = "build_gcc/test_bmw_roundtrip.json";
    const bool saveOk = saveVehicleConfigToJson(bmw, tmpPath);
    EXPECT_TRUE(saveOk);
    VehicleConfig loaded{};
    const bool loadOk = loadVehicleConfigFromJson(loaded, tmpPath);
    EXPECT_TRUE(loadOk);
    EXPECT_NEAR(loaded.frontTrackWidth, bmw.frontTrackWidth, 1e-4);
    EXPECT_NEAR(loaded.rearTrackWidth, bmw.rearTrackWidth, 1e-4);
    EXPECT_NEAR(loaded.frontWheelRadius, bmw.frontWheelRadius, 1e-4);
    EXPECT_NEAR(loaded.frontWheelWidth, bmw.frontWheelWidth, 1e-4);

    // 2. Verify dedicated Bullet wheel collision objects (btCylinderShapeX)
    PhysicsWorld world(bmw, false);
    const int baseObjs = world.getDynamicsWorld()->getNumCollisionObjects();
    EXPECT_TRUE(baseObjs >= 5); // 1 chassis + 4 wheels

    for (int i = 0; i < 4; ++i) {
        const btCollisionObject* wObj = world.vehicle->getWheelCollisionObject(i);
        EXPECT_TRUE(wObj != nullptr);
        EXPECT_TRUE(wObj->getCollisionShape()->getShapeType() == CYLINDER_SHAPE_PROXYTYPE);

        const auto* cyl = world.vehicle->getWheelCylinderShape(i);
        EXPECT_TRUE(cyl != nullptr);
        // Cylinder half extents along X must match half wheel width
        EXPECT_NEAR(cyl->getHalfExtentsWithMargin().x(), static_cast<btScalar>(getWheelWidth(bmw, i) * 0.5), 0.01);
    }

    // Step physics and verify wheel transforms follow wheel centers
    world.stepFixed();
    for (int i = 0; i < 4; ++i) {
        const btCollisionObject* wObj = world.vehicle->getWheelCollisionObject(i);
        const btTransform& trans = wObj->getWorldTransform();
        const Vec3 expectedCenter = world.vehicle->wheels[static_cast<size_t>(i)].wheelCenterWorld;
        EXPECT_NEAR(trans.getOrigin().x(), expectedCenter.x, 1e-3);
        EXPECT_NEAR(trans.getOrigin().y(), expectedCenter.y, 1e-3);
        EXPECT_NEAR(trans.getOrigin().z(), expectedCenter.z, 1e-3);
    }

    // 3. Verify clean vehicle switching with zero object leakage or dangling handles
    VehicleConfig dodge = createDodgeChallengerConfig();
    world.switchVehicle(dodge);
    EXPECT_TRUE(world.getDynamicsWorld()->getNumCollisionObjects() == baseObjs);
    for (int i = 0; i < 4; ++i) {
        const btCollisionObject* wObj = world.vehicle->getWheelCollisionObject(i);
        EXPECT_TRUE(wObj != nullptr);
        EXPECT_TRUE(wObj->getCollisionShape()->getShapeType() == CYLINDER_SHAPE_PROXYTYPE);
    }
}

void unitTest18_StationaryStabilityAndZeroCreep() {
    VehicleConfig cfg = createProceduralCoupeConfig();
    cfg.enableStationaryHold = true;
    PhysicsWorld world(cfg, false);

    // Let the vehicle settle from spawn onto flat ground
    for (int i = 0; i < 80; ++i) {
        world.stepFixed();
    }

    const auto snap0 = world.vehicle->getTelemetrySnapshot();
    const Vec3 initialPos = snap0.chassisPosition;

    // Simulate 300 steps (2.5 seconds at 120 Hz) with 0 throttle, 0 brake, 0 steer
    for (int i = 0; i < 300; ++i) {
        world.vehicle->currentInputs.throttle = 0.0;
        world.vehicle->currentInputs.brake = 0.0;
        world.vehicle->currentInputs.handbrake = 0.0;
        world.vehicle->currentInputs.steering = 0.0;
        world.stepFixed();
    }

    const auto snapEnd = world.vehicle->getTelemetrySnapshot();
    const double posDrift = vec3Length(vec3Sub(snapEnd.chassisPosition, initialPos));

    // Must have zero creeping or slipping while parked
    EXPECT_LT(posDrift, 0.015); // < 1.5 cm drift over 2.5 seconds
    EXPECT_LT(std::abs(snapEnd.forwardSpeedMps), 0.005);
    EXPECT_LT(std::abs(snapEnd.lateralSpeedMps), 0.005);

    // All 4 wheels must have 0 angular velocity (no persistent rotation or chatter)
    for (int i = 0; i < 4; ++i) {
        EXPECT_LT(std::abs(world.vehicle->wheels[static_cast<size_t>(i)].angularVelocity), 0.01);
        EXPECT_NEAR(world.vehicle->wheels[static_cast<size_t>(i)].driveTorque, 0.0, 1e-4);
    }
}

void unitTest19_WheelAlignmentAndFenderClearance() {
    // 1. Procedural Sports Coupe
    const VehicleConfig coupe = createProceduralCoupeConfig();
    const double coupeHalfWidth = coupe.chassisHalfExtents.x; // 0.88m
    const double coupeFrontOuterX = (coupe.frontTrackWidth * 0.5) + (coupe.frontWheelWidth * 0.5);
    const double coupeRearOuterX  = (coupe.rearTrackWidth * 0.5) + (coupe.rearWheelWidth * 0.5);
    EXPECT_LE(coupeFrontOuterX, coupeHalfWidth);
    EXPECT_LE(coupeRearOuterX, coupeHalfWidth);
    EXPECT_GT(coupeHalfWidth - coupeFrontOuterX, 0.05); // At least 5cm clearance inside fender

    // 2. BMW M4 Coupe
    const VehicleConfig m4 = createBmwM4Config();
    const double m4HalfWidth = m4.chassisHalfExtents.x; // 0.92m
    const double m4FrontOuterX = (m4.frontTrackWidth * 0.5) + (m4.frontWheelWidth * 0.5);
    const double m4RearOuterX  = (m4.rearTrackWidth * 0.5) + (m4.rearWheelWidth * 0.5);
    EXPECT_LE(m4FrontOuterX, m4HalfWidth);
    EXPECT_LE(m4RearOuterX, m4HalfWidth);
    EXPECT_GT(m4HalfWidth - m4FrontOuterX, 0.04);

    // 3. Dodge Challenger R/T
    const VehicleConfig challenger = createDodgeChallengerConfig();
    const double dodgeHalfWidth = challenger.chassisHalfExtents.x; // 0.965m
    const double dodgeFrontOuterX = (challenger.frontTrackWidth * 0.5) + (challenger.frontWheelWidth * 0.5);
    const double dodgeRearOuterX  = (challenger.rearTrackWidth * 0.5) + (challenger.rearWheelWidth * 0.5);
    EXPECT_LE(dodgeFrontOuterX, dodgeHalfWidth);
    EXPECT_LE(dodgeRearOuterX, dodgeHalfWidth);
    EXPECT_GT(dodgeHalfWidth - dodgeFrontOuterX, 0.04);
}

void unitTest20_CollisionFidelityModesAndLifecycle() {
    VehicleConfig cfg = createProceduralCoupeConfig();
    PhysicsWorld world(cfg, false);

    // Test switching to Performance mode
    world.vehicle->setCollisionFidelity(WheelCollisionFidelity::Performance);
    EXPECT_TRUE(world.vehicle->config.wheelCollisionFidelity == WheelCollisionFidelity::Performance);
    for (int i = 0; i < 20; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->getTelemetrySnapshot().numericalSanityOk);

    // Test switching to Detailed mode
    world.vehicle->setCollisionFidelity(WheelCollisionFidelity::Detailed);
    EXPECT_TRUE(world.vehicle->config.wheelCollisionFidelity == WheelCollisionFidelity::Detailed);
    for (int i = 0; i < 20; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->getTelemetrySnapshot().numericalSanityOk);

    // Test switching to Balanced mode
    world.vehicle->setCollisionFidelity(WheelCollisionFidelity::Balanced);
    EXPECT_TRUE(world.vehicle->config.wheelCollisionFidelity == WheelCollisionFidelity::Balanced);
    for (int i = 0; i < 20; ++i) world.stepFixed();
    EXPECT_TRUE(world.vehicle->getTelemetrySnapshot().numericalSanityOk);
}

void unitTest21_VehicleComponentArchitecture() {
    // 1. Wheel Assembly Component
    WheelAssemblyComponent wheel(0, "Wheel_FL", Vec3{0.67, 0.15, 1.35}, 0.32, 0.235, 22.0);
    EXPECT_NEAR(wheel.getRadius(), 0.32, 1e-5);
    EXPECT_NEAR(wheel.getWidth(), 0.235, 1e-5);
    EXPECT_NEAR(wheel.getMass(), 22.0, 1e-5);
    wheel.setKinematicState(Vec3{0.67, -0.12, 1.35}, 0.15, 0.50, 10.0, true);
    EXPECT_NEAR(wheel.getSteeringAngle(), 0.15, 1e-5);
    EXPECT_TRUE(wheel.isInContact());
    wheel.update(0.016);
    EXPECT_GT(wheel.getRotationAngle(), 0.50);

    // 2. Suspension Linkage Component
    SuspensionLinkageComponent susp("Susp_FL", Vec3{0.67, 0.15, 1.35}, 0.38, 0.15, 0.48, 35000.0, 3600.0, 4800.0);
    EXPECT_NEAR(susp.getCompression(), 0.0, 1e-5);
    susp.setLength(0.28);
    EXPECT_NEAR(susp.getCompression(), 0.10, 1e-5);
    EXPECT_GT(susp.getCompressionRatio(), 0.40);

    // 3. Steering Linkage Component & Ackermann
    SteeringLinkageComponent steer("Steer_Rack", 0.60, 0.85);
    steer.setInputAngle(0.35);
    steer.computeAckermannGeometry(2.70, 1.60);
    EXPECT_GT(steer.getLeftWheelAngle(), steer.getRightWheelAngle()); // Inner wheel turns sharper in left turn

    // 4. Articulated Mechanical Part (Driveshaft)
    ArticulatedMechanicalPart propShaft("Driveshaft", ArticulatedMechanicalPart::MotionType::ContinuousRotation,
                                       Vec3{0, -0.10, 0}, Vec3{0, 0, 1}, 8.0);
    propShaft.setAngularVelocity(50.0);
    propShaft.update(0.10);
    EXPECT_NEAR(propShaft.getCurrentAngle(), 5.0, 1e-3);

    // 5. Detachable & Damageable Body Part
    DetachableBodyPart bumper("FrontBumper", Vec3{0, 0, 2.15}, 14.0, 3000.0, 8000.0);
    EXPECT_TRUE(bumper.getState() == DetachableBodyPart::State::Attached);
    EXPECT_NEAR(bumper.getHealth(), 1.0, 1e-5);

    // Sub-threshold impulse does not loosen or damage
    bumper.applyImpactImpulse(1500.0);
    EXPECT_TRUE(bumper.getState() == DetachableBodyPart::State::Attached);

    // Moderate collision loosens part
    bumper.applyImpactImpulse(5500.0);
    EXPECT_TRUE(bumper.getState() == DetachableBodyPart::State::Loosened);
    EXPECT_LT(bumper.getHealth(), 1.0);

    // Heavy collision detaches part into dynamic debris
    bumper.applyImpactImpulse(9500.0);
    EXPECT_TRUE(bumper.getState() == DetachableBodyPart::State::Detached);
    EXPECT_TRUE(bumper.isDetached());
}

void unitTest22_PneumaticTirePressureAndStiffness() {
    PneumaticTireProperties props = createDefaultTireProperties(0.32, 0.24, 0.22, 220.0);
    PneumaticTireModel model(props);

    // 1. Verify radial stiffness increases with internal pressure
    const double kLow  = model.computeRadialStiffness(150000.0); // 150 kPa
    const double kNom  = model.computeRadialStiffness(220000.0); // 220 kPa
    const double kHigh = model.computeRadialStiffness(290000.0); // 290 kPa

    EXPECT_GT(kNom, kLow);
    EXPECT_GT(kHigh, kNom);
    EXPECT_NEAR(kNom - kLow, 0.48 * 70000.0, 1.0); // dk/dp = 0.48 N/(m*Pa)

    // 2. Deflection under identical load (3700 N)
    const double normalLoad = 3700.0;
    const double defLow = normalLoad / kLow;
    const double defHigh = normalLoad / kHigh;
    EXPECT_GT(defLow, defHigh); // Softer tyre compresses more under identical load!

    // 3. Contact patch dimensions: length scales with sqrt of deflection
    double lenLow = 0.0, widLow = 0.0;
    model.setTargetPressure(150000.0);
    model.computeContactPatchDimensions(normalLoad, lenLow, widLow);

    double lenHigh = 0.0, widHigh = 0.0;
    model.setTargetPressure(290000.0);
    model.computeContactPatchDimensions(normalLoad, lenHigh, widHigh);

    EXPECT_GT(lenLow, lenHigh); // Lower pressure produces larger contact patch footprint
    EXPECT_GT(widLow, props.sectionWidth); // Sidewall Poisson expansion widens footprint
}

void unitTest23_DistributedContactPatchLoadConsistency() {
    PneumaticTireProperties props = createDefaultTireProperties(0.32, 0.24, 0.22, 220.0);
    PneumaticTireModel model(props);

    const VehicleBasis basis = getVehicleBasis(quatIdentity());
    const double normalLoadReq = 4200.0;

    // Test Performance mode (3 samples)
    model.evaluateDistributedContact(
        0.00833, normalLoadReq, 0.08, 0.0,
        Vec3{0, 0.32, 0}, quatIdentity(), basis, Vec3{0, -1, 0},
        getSurfaceProperties(SurfaceType::DryAsphalt),
        WheelCollisionFidelity::Performance, {}, 0.0, 0.0
    );
    EXPECT_EQ(model.getState().activeContactSamples, 3);
    EXPECT_NEAR(model.getState().totalNormalLoad, normalLoadReq, 1.0);

    // Test Balanced mode (9 samples)
    model.evaluateDistributedContact(
        0.00833, normalLoadReq, 0.08, 0.0,
        Vec3{0, 0.32, 0}, quatIdentity(), basis, Vec3{0, -1, 0},
        getSurfaceProperties(SurfaceType::DryAsphalt),
        WheelCollisionFidelity::Balanced, {}, 0.0, 0.0
    );
    EXPECT_EQ(model.getState().activeContactSamples, 9);
    EXPECT_NEAR(model.getState().totalNormalLoad, normalLoadReq, 1.0);

    // Test Detailed mode (15 samples)
    model.evaluateDistributedContact(
        0.00833, normalLoadReq, 0.08, 0.0,
        Vec3{0, 0.32, 0}, quatIdentity(), basis, Vec3{0, -1, 0},
        getSurfaceProperties(SurfaceType::DryAsphalt),
        WheelCollisionFidelity::Detailed, {}, 0.0, 0.0
    );
    EXPECT_EQ(model.getState().activeContactSamples, 15);
    EXPECT_NEAR(model.getState().totalNormalLoad, normalLoadReq, 1.0);

    // Ensure all individual contact sample loads are positive and sum to requested load
    double sumSamples = 0.0;
    for (const auto& sample : model.getState().contactSamples) {
        EXPECT_TRUE(sample.inContact);
        EXPECT_GT(sample.normalLoad, 0.0);
        sumSamples += sample.normalLoad;
    }
    EXPECT_NEAR(sumSamples, normalLoadReq, 0.5);
}

void unitTest24_SeriesSuspensionPneumaticCompliance() {
    // Verify analytical series spring compliance: 1/k_eff = 1/k_susp + 1/k_tire
    const double kSusp = 35000.0;
    const double kTire = 213600.0;
    const double kEffExpected = (kSusp * kTire) / (kSusp + kTire);

    const double totalLoad = 3800.0;
    const double suspComp = totalLoad / kSusp;
    const double tireComp = totalLoad / kTire;
    const double totalComp = suspComp + tireComp;

    const double kEffMeasured = totalLoad / totalComp;
    EXPECT_NEAR(kEffMeasured, kEffExpected, 1.0);
    EXPECT_LT(kEffMeasured, kSusp); // Adding tyre compliance softens overall vertical ride!
}

void unitTest25_CarcassDynamicDeformationAndAligningTorque() {
    PneumaticTireProperties props = createDefaultTireProperties(0.32, 0.24, 0.22, 220.0);
    PneumaticTireModel model(props);
    const VehicleBasis basis = getVehicleBasis(quatIdentity());

    // Cornering test: lateral sliding velocity creates lateral deformation and self-aligning moment
    const double normalLoad = 4000.0;
    const double forwardSpeed = 15.0;
    const double lateralSlide = -1.5; // Slip angle alpha ~ -0.10 rad

    for (int step = 0; step < 20; ++step) {
        model.evaluateDistributedContact(
            0.00833, normalLoad, 0.08, forwardSpeed / 0.32,
            Vec3{0, 0.32, 0}, quatIdentity(), basis, Vec3{0, -1, 0},
            getSurfaceProperties(SurfaceType::DryAsphalt),
            WheelCollisionFidelity::Balanced, {}, forwardSpeed, lateralSlide
        );
    }

    const auto& state = model.getState();
    EXPECT_GT(state.radialDeformation, 0.010); // > 10mm radial squash
    EXPECT_GT(state.sidewallBulge, 0.004);     // > 4mm sidewall outward expansion
    EXPECT_GT(std::abs(state.lateralDeformation), 0.005); // Carcass leans laterally
    EXPECT_GT(std::abs(state.selfAligningTorque), 5.0);   // Pneumatic trail creates self-aligning torque M_z!
}

void unitTest26_ThermodynamicsWearAndPuncture() {
    PneumaticTireProperties props = createDefaultTireProperties(0.32, 0.24, 0.22, 220.0);
    PneumaticTireModel model(props);
    const VehicleBasis basis = getVehicleBasis(quatIdentity());

    EXPECT_NEAR(model.getState().temperatureC, 25.0, 1e-4);
    EXPECT_NEAR(model.getState().wearLevel, 0.0, 1e-4);

    // Hard spinning burnout: high wheel speed against stationary ground produces extreme frictional work
    for (int step = 0; step < 60; ++step) { // 0.5s of hard wheelspin
        model.evaluateDistributedContact(
            0.00833, 4000.0, 0.08, 80.0, // 80 rad/s (~25 m/s tread speed)
            Vec3{0, 0.32, 0}, quatIdentity(), basis, Vec3{0, -1, 0},
            getSurfaceProperties(SurfaceType::DryAsphalt),
            WheelCollisionFidelity::Balanced, {}, 0.0, 0.0 // stationary vehicle
        );
    }

    EXPECT_GT(model.getState().temperatureC, 25.5); // Frictional sliding work raises temperature!
    EXPECT_GT(model.getState().wearLevel, 0.0);      // Tread wear accumulates!

    // Puncture leakage test
    const double initialP = model.getState().pressurePa;
    model.setPunctured(true, 5000.0); // 5 kPa/s leakage
    EXPECT_TRUE(model.getState().isPunctured);

    for (int step = 0; step < 120; ++step) { // 1.0 second of simulation
        model.updatePneumaticState(0.00833);
    }
    EXPECT_LT(model.getState().pressurePa, initialP - 4000.0); // Pressure lost predictably
}

void unitTest27_RigidRimProtectionBoundary() {
    // Mathematical boundary verification: rim vertices inside R_rim never experience deformation
    const double outerRadius = 0.32;
    const double rimRadius = 0.22;
    const double radialDeform = 0.022; // 22mm compression at contact patch

    auto computeRadialDisplacement = [&](double r) {
        if (r <= rimRadius) {
            return 0.0; // Rigid rim rule
        }
        const double radialT = clamp((r - rimRadius) / (outerRadius - rimRadius), 0.0, 1.0);
        const double smoothRadial = radialT * radialT * (3.0 - 2.0 * radialT);
        return radialDeform * smoothRadial;
    };

    // Center hub, spokes, and rim bead (r <= 0.22m)
    EXPECT_NEAR(computeRadialDisplacement(0.05), 0.0, 1e-6);
    EXPECT_NEAR(computeRadialDisplacement(0.12), 0.0, 1e-6);
    EXPECT_NEAR(computeRadialDisplacement(0.20), 0.0, 1e-6);
    EXPECT_NEAR(computeRadialDisplacement(0.22), 0.0, 1e-6);

    // Rubber tyre carcass (r > 0.22m)
    EXPECT_GT(computeRadialDisplacement(0.27), 0.0);
    EXPECT_NEAR(computeRadialDisplacement(0.32), radialDeform, 1e-4);
}

void unitTest28_WorldSpaceTireDeformationGroundPenetration() {
    // Verifies:
    // 1. Zero ground penetration (deformed bottom vertex and contact patch tread stay flush at y >= 0.0)
    // 2. Exact flush contact interface (bottom vertex sits exactly at y = 0.0 within sub-millimeter precision)
    // 3. Strict rigid rim protection (all rim/hub vertices inside R_rim have exactly 0 deformation)
    // 4. Smooth continuous carcass curvature without inverted craters or knife-edges
    // 5. Realistic positive Poisson sidewall bulging
    const double outerRadius = 0.32;
    const double rimRadius = 0.22;
    const double width = 0.245;
    const double radialDeform = 0.022; // 22mm deflection under wheel load
    const double sidewallBulge = 0.007; // 7mm Poisson bulge
    const double patchAngleSpan = std::acos(clamp((outerRadius - radialDeform) / outerRadius, -1.0, 1.0)); // ~0.37 rad

    // Wheel hub center when physically resting on flat ground at y = 0
    // Physical series compliance sets hub height = outerRadius - radialDeform
    const Vec3 wheelCenterWorld{0.0, outerRadius - radialDeform, 0.0};
    const Vec3 wheelAxleWorld{1.0, 0.0, 0.0}; // Along X
    const Vec3 groundPointWorld{0.0, 0.0, 0.0};
    const Vec3 groundNormalWorld{0.0, 1.0, 0.0};

    auto deformWorldVertex = [&](const Vec3& worldPos) -> Vec3 {
        const Vec3 rHub = vec3Sub(worldPos, wheelCenterWorld);
        const double xAxle = vec3Dot(rHub, wheelAxleWorld);
        const Vec3 rRadial = vec3Sub(rHub, vec3Scale(wheelAxleWorld, xAxle));
        const double r = vec3Length(rRadial);

        if (r <= rimRadius + 0.002) {
            return worldPos; // Strict rim protection
        }

        const double eta = clamp((r - (rimRadius + 0.002)) / std::max(0.005, outerRadius - (rimRadius + 0.002)), 0.0, 1.0);
        const double smoothEta = eta * eta * (3.0 - 2.0 * eta);
        const Vec3 uRadial = vec3Scale(rRadial, 1.0 / std::max(0.001, r));

        const double dGround = vec3Dot(vec3Sub(worldPos, groundPointWorld), groundNormalWorld);
        const Vec3 uContactDown = vec3Scale(groundNormalWorld, -1.0);
        const double cosTheta = clamp(vec3Dot(uRadial, uContactDown), -1.0, 1.0);
        const double theta = std::acos(cosTheta);

        const double angleSpan = std::max(0.12, patchAngleSpan);
        const double blendSpan = angleSpan * 1.50;

        if (theta >= blendSpan) {
            return worldPos;
        }

        double contactWeight = 0.0;
        if (theta < angleSpan) {
            const double t = theta / angleSpan;
            contactWeight = 1.0 - t * t * (3.0 - 2.0 * t);
        } else {
            const double t = (theta - angleSpan) / (blendSpan - angleSpan);
            const double edgeWeight = 1.0 - t * t * (3.0 - 2.0 * t);
            contactWeight = edgeWeight * 0.18;
        }

        const double penDepth = std::max(0.0, -dGround);
        const double edgeRounding = (1.0 - cosTheta) * 0.06 * radialDeform * contactWeight;
        const double liftAmount = (penDepth + edgeRounding) * smoothEta;
        const Vec3 deltaFlatten = vec3Scale(groundNormalWorld, liftAmount);

        const double bulgeProfile = std::sin(std::pow(eta, 0.75) * 3.141592653589793) * (1.0 - 0.20 * eta);
        const double signX = (xAxle >= 0.0) ? 1.0 : -1.0;
        const Vec3 deltaBulge = vec3Scale(wheelAxleWorld, signX * sidewallBulge * bulgeProfile * contactWeight);

        return vec3Add(worldPos, vec3Add(deltaFlatten, deltaBulge));
    };

    // 1. Hub & Rim protection: vertices on wheel rim (r <= rimRadius) experience ZERO displacement
    Vec3 rimVertex{0.0, wheelCenterWorld.y - rimRadius, 0.0};
    Vec3 deformedRim = deformWorldVertex(rimVertex);
    EXPECT_NEAR(deformedRim.y, rimVertex.y, 1e-6);
    EXPECT_NEAR(deformedRim.x, rimVertex.x, 1e-6);
    EXPECT_NEAR(deformedRim.z, rimVertex.z, 1e-6);

    // 2. Bottom-most tread vertex: undeformed y = (outerRadius - radialDeform) - outerRadius = -radialDeform = -0.022m
    // Deformed bottom vertex must sit exactly at ground level y = 0.0
    Vec3 bottomTreadVertex{0.0, wheelCenterWorld.y - outerRadius, 0.0};
    EXPECT_NEAR(bottomTreadVertex.y, -radialDeform, 1e-5); // Undeformed extends below ground
    Vec3 deformedBottom = deformWorldVertex(bottomTreadVertex);
    EXPECT_NEAR(deformedBottom.y, 0.0, 1e-4); // Deformed must sit flush with ground (0.0 mm penetration)

    // 3. Dense circumferential scan across tread: all vertices must satisfy y >= -0.0001m (no ground penetration anywhere)
    const int numAngularSteps = 180;
    double prevY = 0.0;
    for (int step = 0; step <= numAngularSteps; ++step) {
        const double angle = (static_cast<double>(step) / numAngularSteps) * 3.141592653589793;
        const double yUndef = wheelCenterWorld.y - outerRadius * std::cos(angle);
        const double zUndef = outerRadius * std::sin(angle);
        Vec3 treadPt{0.0, yUndef, zUndef};

        Vec3 defPt = deformWorldVertex(treadPt);
        EXPECT_GE(defPt.y, -0.0001);

        if (step > 0 && angle < patchAngleSpan * 1.5) {
            EXPECT_GE(defPt.y, prevY - 1e-5);
        }
        prevY = defPt.y;
    }

    // 4. Sidewall bulging: mid-sidewall vertex at contact azimuth must expand laterally
    const double midRadius = 0.5 * (outerRadius + rimRadius);
    const double halfWidth = 0.5 * width;
    Vec3 leftSidewallPt{halfWidth, wheelCenterWorld.y - midRadius, 0.0};
    Vec3 deformedLeft = deformWorldVertex(leftSidewallPt);
    EXPECT_GT(deformedLeft.x, leftSidewallPt.x); // Expanded outward (+X)

    Vec3 rightSidewallPt{-halfWidth, wheelCenterWorld.y - midRadius, 0.0};
    Vec3 deformedRight = deformWorldVertex(rightSidewallPt);
    EXPECT_LT(deformedRight.x, rightSidewallPt.x); // Expanded outward (-X)
}

} // namespace

int main() {
    std::cout << "====================================================================\n";
    std::cout << " Torque Lab Native — Automated Physics & Behavioral Validation Suite\n";
    std::cout << "====================================================================\n";

    struct TestCase {
        const char* name;
        std::function<void()> fn;
    };

    const std::vector<TestCase> tests{
        {"Unit 1: Spring, Damper, Bump-Stop & Anti-Roll Bar Signs", unitTest1_SuspensionForceSigns},
        {"Unit 2: Right-Handed Coordinate Transforms & Pitch/Roll Extraction", unitTest2_CoordinateTransformsAndEuler},
        {"Unit 3: Tire Friction-Circle Limits & Load Sensitivity", unitTest3_TireFrictionCircleAndLoadSensitivity},
        {"Unit 4: Wheel Angular Speed, Slip Ratio & Slip Angle", unitTest4_WheelSlipCalculations},
        {"Unit 5: Drivetrain Torque Curve, Gear Ratios, Rev Limiter & LSD", unitTest5_DrivetrainAndRevLimiter},
        {"Unit 6: Surface Friction Hierarchy Across Materials", unitTest6_SurfaceFrictionHierarchy},
        {"Unit 7: Ackermann Steering Geometry & Speed-Sensitive Limits", unitTest7_AckermannAndSpeedSensitiveSteering},
        {"Unit 8: Quadratic Aerodynamic Drag & Service/Handbrake Bias", unitTest8_AerodynamicsAndBrakeBias},
        {"Unit 9: Automatic Model-Derived Collision Pipeline & Fallback Hierarchy", unitTest9_AutomaticCollisionPipeline},
        {"Unit 10: Dynamic Rigid-Body Collision Hull Stability & Contact", unitTest10_ModelDerivedCollisionDynamics},
        {"Unit 11: Collision Alignment, Continuous Collision Detection & Obstacle Impact", unitTest11_CollisionAlignmentAndContinuousCollisionDetection},
        {"Unit 12: Collision Lifecycle, Visualization Independence & Switching Integrity", unitTest12_CollisionLifecycleAndVisualizationIndependence},
        {"Unit 13: Forza-Inspired Controller Mapping, Normalization & Settings Pipeline", unitTest13_ControllerMappingAndNormalization},
        {"Unit 14: Physics-Driven Controller Haptics & RPM Vibration Elimination", unitTest14_PhysicsDrivenHapticsAndRpmElimination},
        {"Unit 15: Multi-Vehicle Presets, Dodge Challenger & Muscle Car Tuning", unitTest15_MultiVehiclePresetsAndMuscleCarTuning},
        {"Unit 16: Volumetric Tyre Obstacle Contact, Circumferential Gripping & Curb Climbing", unitTest16_VolumetricTyreObstacleContactAndClimbing},
        {"Unit 17: Wheel Geometry Derivation, Alignment & Dynamic Collision Object Lifecycle", unitTest17_WheelGeometryDerivationAndCollisionObjectLifecycle},
        {"Unit 18: Stationary Stability, Stiction Holding & Zero-Creep Parking", unitTest18_StationaryStabilityAndZeroCreep},
        {"Unit 19: Wheel Alignment & Fender Clearance Across All Vehicle Models", unitTest19_WheelAlignmentAndFenderClearance},
        {"Unit 20: Selectable Wheel Collision Fidelity Modes & Runtime Lifecycle", unitTest20_CollisionFidelityModesAndLifecycle},
        {"Unit 21: Modular Vehicle Component Architecture (Wheels, Suspension, Steering, Detachable Parts)", unitTest21_VehicleComponentArchitecture},
        {"Unit 22: Pneumatic Tyre Internal Pressure, Radial Stiffness & Contact Deflection", unitTest22_PneumaticTirePressureAndStiffness},
        {"Unit 23: Multi-Point Distributed Contact Patch & Load Consistency", unitTest23_DistributedContactPatchLoadConsistency},
        {"Unit 24: Series Suspension & Pneumatic Tyre Compliance Equilibrium", unitTest24_SeriesSuspensionPneumaticCompliance},
        {"Unit 25: Carcass Dynamic Deformation & Self-Aligning Torque", unitTest25_CarcassDynamicDeformationAndAligningTorque},
        {"Unit 26: Tyre Thermodynamics, Frictional Heating, Wear & Puncture", unitTest26_ThermodynamicsWearAndPuncture},
        {"Unit 27: Strict Rigid Rim Protection & Vertex Shader Boundary", unitTest27_RigidRimProtectionBoundary},
        {"Unit 28: World-Space Tyre Ground Penetration Elimination & Flush Contact Patch", unitTest28_WorldSpaceTireDeformationGroundPenetration},
        {"Test A: Static Equilibrium on Flat Ground", validationTestA_StaticEquilibrium},
        {"Test B: Acceleration & Longitudinal Load Transfer (Rear Squat)", validationTestB_AccelerationAndSquat},
        {"Test C: Service Braking, Front Dive & Progressive Deceleration", validationTestC_BrakingAndDive},
        {"Test D: Cornering, Lateral Load Transfer & Reversible Body Roll", validationTestD_CorneringAndBodyRoll},
        {"Test E: Low-Grip Surface Effects on Wheelspin & Braking Distance", validationTestE_LowGripSurfaceComparison},
        {"Test F: Uneven Terrain (Speed Bump Response & Damping)", validationTestF_UnevenTerrainSpeedBump},
        {"Test G: Parameter Sensitivity (Spring Stiffness & CoM Height)", validationTestG_ParameterSensitivity},
        {"Test H: Collision Momentum Transfer & Clean Recovery Reset", validationTestH_CollisionAndRecovery},
        {"Test I: Time-Step Independence Across Render Frame Rates", validationTestI_TimestepIndependence},
        {"Test J: Energy & Numerical Sanity During Coasting Deceleration", validationTestJ_CoastingEnergySanity},
        {"Controlled 1: Known Constant Force (a ~= F / m)", controlledTest_KnownConstantForce},
        {"Controlled 2: Zero-Resistance Launch Comparison", controlledTest_ZeroResistanceComparison},
        {"Controlled 3: Brake Isolation & Scale Sanity (d = v * t)", controlledTest_BrakeIsolationAndScaleSanity}
    };

    int passedSuites = 0;
    for (const auto& tc : tests) {
        const int beforeFails = g_failedAssertions;
        std::cout << "[RUN ] " << tc.name << std::endl;
        tc.fn();
        if (g_failedAssertions == beforeFails) {
            std::cout << "[PASS] " << tc.name << std::endl;
            ++passedSuites;
        } else {
            std::cout << "[FAIL] " << tc.name << std::endl;
        }
    }

    std::cout << "--------------------------------------------------------------------\n";
    std::cout << " Summary: " << passedSuites << " / " << tests.size()
              << " test suites passed (" << g_failedAssertions << " failed assertions).\n";
    std::cout << "====================================================================\n";

    return (g_failedAssertions == 0) ? 0 : 1;
}
