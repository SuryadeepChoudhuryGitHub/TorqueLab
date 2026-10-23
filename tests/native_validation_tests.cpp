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

#include "physics/Math3D.h"
#include "physics/PhysicsWorld.h"
#include "physics/SurfaceSystem.h"
#include "vehicle/Aerodynamics.h"
#include "vehicle/BrakeSystem.h"
#include "vehicle/Drivetrain.h"
#include "vehicle/SteeringSystem.h"
#include "vehicle/SuspensionSystem.h"
#include "vehicle/TireModel.h"
#include "vehicle/VehicleConfig.h"
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
    for (int i = 0; i < 120; ++i) {
        sim.stepFixed();
    }

    const auto postCrashSnap = sim.vehicle->getTelemetrySnapshot();
    std::printf("    [Metrics H] PostCrash Z=%.2f m (Wall at 14.0m), PostCrash FwdSpeed=%.2f m/s, Sanity=%s\n",
                postCrashSnap.chassisPosition.z, postCrashSnap.forwardSpeedMps,
                postCrashSnap.numericalSanityOk ? "OK" : "FAIL");

    EXPECT_LT(postCrashSnap.chassisPosition.z, 13.0);
    EXPECT_LT(postCrashSnap.forwardSpeedMps, 2.0);
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
