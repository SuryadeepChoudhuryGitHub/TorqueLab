#include "vehicle/Drivetrain.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

DrivetrainState createInitialDrivetrainState(double idleRpm) {
    DrivetrainState s{};
    s.rpm = idleRpm;
    s.gearIndex = 2; // Start in 1st gear
    s.transmissionMode = TransmissionMode::Automatic;
    s.shiftTimer = 0.0;
    s.shiftCooldown = 0.0;
    s.engineTorqueNm = 0.0;
    s.clutchEngagement = 1.0;
    s.revLimiterActive = false;
    return s;
}

std::string getGearDisplayName(int gearIndex) {
    if (gearIndex == 0) return "R";
    if (gearIndex == 1) return "N";
    return std::to_string(gearIndex - 1);
}

double sampleEngineTorqueCurve(double rpm, const VehicleConfig& config) {
    const auto& pts = config.torqueCurve;
    if (pts.empty()) return 250.0 * config.engineTorqueMultiplier;
    if (rpm <= pts.front().rpm) {
        return pts.front().torqueNm * config.engineTorqueMultiplier;
    }
    if (rpm >= pts.back().rpm) {
        return pts.back().torqueNm * config.engineTorqueMultiplier;
    }
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto& p0 = pts[i];
        const auto& p1 = pts[i + 1];
        if (rpm >= p0.rpm && rpm <= p1.rpm) {
            const double t = (rpm - p0.rpm) / std::max(1.0, p1.rpm - p0.rpm);
            return lerp(p0.torqueNm, p1.torqueNm, t) * config.engineTorqueMultiplier;
        }
    }
    return pts.back().torqueNm * config.engineTorqueMultiplier;
}

double computeEngineBrakingTorque(double rpm, const VehicleConfig& config) {
    if (rpm <= config.idleRpm + 50.0) {
        return 0.0;
    }
    const double normalizedRpm = clamp(
        (rpm - config.idleRpm) / std::max(500.0, config.maxRpm - config.idleRpm),
        0.0,
        1.2
    );
    return config.engineBrakingTorque * normalizedRpm;
}

double getTotalGearRatio(int gearIndex, const VehicleConfig& config) {
    if (gearIndex < 0 || gearIndex >= static_cast<int>(config.gearRatios.size())) {
        return 0.0;
    }
    return config.gearRatios[static_cast<size_t>(gearIndex)] * config.finalDriveRatio;
}

void shiftToGear(
    DrivetrainState& state,
    int targetGearIndex,
    const VehicleConfig& config
) {
    const int clamped = static_cast<int>(
        clamp(static_cast<double>(targetGearIndex), 0.0, static_cast<double>(config.gearRatios.size()) - 1.0)
    );
    if (clamped != state.gearIndex) {
        state.gearIndex = clamped;
        state.shiftTimer = config.shiftDuration;
        state.shiftCooldown = 0.45;
    }
}

namespace {

void distributeAxleTorqueWithLSD(
    WheelState& leftWheel,
    WheelState& rightWheel,
    double axleTorque,
    double lockingCoeff
) {
    const double halfTorque = axleTorque * 0.5;
    const double omegaDiff = leftWheel.angularVelocity - rightWheel.angularVelocity;
    const double maxLockTorque = std::abs(halfTorque) * 0.65 + 120.0;
    const double lsdTorque = clamp(omegaDiff * lockingCoeff, -maxLockTorque, maxLockTorque);

    leftWheel.driveTorque = halfTorque - lsdTorque;
    rightWheel.driveTorque = halfTorque + lsdTorque;
}

} // namespace

void updateDrivetrain(
    DrivetrainState& state,
    std::array<WheelState, 4>& wheels,
    double throttleInput,
    double forwardSpeedMps,
    const VehicleConfig& config,
    double dt
) {
    const double throttle = clamp(throttleInput, 0.0, 1.0);

    if (state.shiftTimer > 0.0) {
        state.shiftTimer = std::max(0.0, state.shiftTimer - dt);
    }
    if (state.shiftCooldown > 0.0) {
        state.shiftCooldown = std::max(0.0, state.shiftCooldown - dt);
    }

    const bool isFrontDriven =
        config.drivetrainLayout == DrivetrainLayout::FWD ||
        config.drivetrainLayout == DrivetrainLayout::AWD;
    const bool isRearDriven =
        config.drivetrainLayout == DrivetrainLayout::RWD ||
        config.drivetrainLayout == DrivetrainLayout::AWD;

    double drivenOmegaSum = 0.0;
    int drivenCount = 0;
    if (isFrontDriven) {
        drivenOmegaSum += wheels[0].angularVelocity + wheels[1].angularVelocity;
        drivenCount += 2;
    }
    if (isRearDriven) {
        drivenOmegaSum += wheels[2].angularVelocity + wheels[3].angularVelocity;
        drivenCount += 2;
    }
    const double avgDrivenWheelOmega = (drivenCount > 0) ? (drivenOmegaSum / static_cast<double>(drivenCount)) : 0.0;

    const double totalRatio = getTotalGearRatio(state.gearIndex, config);
    const double absRatio = std::abs(totalRatio);

    const double coupledRpm = std::max(0.0, avgDrivenWheelOmega * totalRatio * RAD_PER_SEC_TO_RPM);

    // Automatic transmission shift logic
    if (state.transmissionMode == TransmissionMode::Automatic &&
        state.shiftCooldown <= 0.0 &&
        state.shiftTimer <= 0.0) {
        if (state.gearIndex >= 2) {
            const bool anyDrivenContact =
                (isFrontDriven && (wheels[0].inContact || wheels[1].inContact)) ||
                (isRearDriven && (wheels[2].inContact || wheels[3].inContact));

            const double upshiftWheelOmega =
                (config.autoUpshiftRpm * RPM_TO_RAD_PER_SEC) / std::max(0.1, absRatio);
            const double minRoadSpeedForUpshift = upshiftWheelOmega * config.wheelRadius * 0.80;

            if (anyDrivenContact &&
                state.rpm >= config.autoUpshiftRpm &&
                forwardSpeedMps >= minRoadSpeedForUpshift &&
                state.gearIndex < static_cast<int>(config.gearRatios.size()) - 1) {
                shiftToGear(state, state.gearIndex + 1, config);
            } else if (state.gearIndex > 2 &&
                       coupledRpm < config.autoDownshiftRpm &&
                       forwardSpeedMps > 1.0) {
                const double lowerGearRatio = std::abs(getTotalGearRatio(state.gearIndex - 1, config));
                const double projectedRpm =
                    std::abs(avgDrivenWheelOmega) * lowerGearRatio * RAD_PER_SEC_TO_RPM;
                if (projectedRpm < config.autoUpshiftRpm * 0.92) {
                    shiftToGear(state, state.gearIndex - 1, config);
                }
            }
        }
    }

    // Clutch engagement model
    if (state.gearIndex == 1 || state.shiftTimer > 0.0) {
        state.clutchEngagement = 0.0;
    } else {
        const double launchSlipZoneRpm = config.idleRpm + 950.0;
        if (coupledRpm < launchSlipZoneRpm) {
            const double baseEngagement = clamp((coupledRpm - 200.0) / (launchSlipZoneRpm - 200.0), 0.35, 1.0);
            state.clutchEngagement = lerp(baseEngagement, 1.0, throttle * 0.45);
        } else {
            state.clutchEngagement = 1.0;
        }
    }

    // Rev limiter check (evaluated at current state.rpm matching Drivetrain.ts)
    state.revLimiterActive = (state.rpm >= config.maxRpm);
    const double effectiveThrottle = state.revLimiterActive ? 0.0 : throttle;

    const double maxTorqueAtRpm = sampleEngineTorqueCurve(state.rpm, config);
    const double brakingTorque = computeEngineBrakingTorque(state.rpm, config);

    double idleAssistTorque = 0.0;
    if (state.rpm < config.idleRpm && effectiveThrottle < 0.05) {
        idleAssistTorque = (config.idleRpm - state.rpm) * 0.4;
    }

    if (state.revLimiterActive && throttle > 0.05) {
        // Soft fuel-cut rev limiter: cut positive combustion torque to zero at redline
        // rather than slamming full closed-throttle vacuum braking while throttle is held.
        state.engineTorqueNm = 0.0;
    } else {
        state.engineTorqueNm =
            effectiveThrottle * maxTorqueAtRpm -
            (1.0 - effectiveThrottle) * brakingTorque +
            idleAssistTorque;
    }

    if (state.clutchEngagement <= 0.01 || absRatio < 1e-4) {
        const double omegaDot = state.engineTorqueNm / std::max(0.05, config.engineInertia);
        const double rpmDot = omegaDot * RAD_PER_SEC_TO_RPM;
        state.rpm = clamp(state.rpm + rpmDot * dt, config.idleRpm, config.maxRpm + 100.0);
        for (int i = 0; i < 4; ++i) {
            wheels[static_cast<size_t>(i)].driveTorque = 0.0;
        }
        return;
    }

    const double launchTargetRpm = config.idleRpm + effectiveThrottle * 2100.0;
    const double targetRpm = std::max(
        config.idleRpm,
        lerp(std::max(coupledRpm, launchTargetRpm), coupledRpm, state.clutchEngagement)
    );
    const double rpmBlendRate = clamp(18.0 * dt, 0.0, 1.0);
    state.rpm = clamp(lerp(state.rpm, targetRpm, rpmBlendRate), config.idleRpm, config.maxRpm + 80.0);

    const double activeTotalRatio = getTotalGearRatio(state.gearIndex, config);
    const double totalDriveTorque =
        state.engineTorqueNm * activeTotalRatio * config.drivetrainEfficiency;

    for (int i = 0; i < 4; ++i) {
        wheels[static_cast<size_t>(i)].driveTorque = 0.0;
    }

    if (config.drivetrainLayout == DrivetrainLayout::RWD) {
        distributeAxleTorqueWithLSD(wheels[2], wheels[3], totalDriveTorque, config.differentialLockingCoeff);
    } else if (config.drivetrainLayout == DrivetrainLayout::FWD) {
        distributeAxleTorqueWithLSD(wheels[0], wheels[1], totalDriveTorque, config.differentialLockingCoeff);
    } else {
        distributeAxleTorqueWithLSD(wheels[0], wheels[1], totalDriveTorque * 0.40, config.differentialLockingCoeff);
        distributeAxleTorqueWithLSD(wheels[2], wheels[3], totalDriveTorque * 0.60, config.differentialLockingCoeff);
    }
}

} // namespace torquelab
