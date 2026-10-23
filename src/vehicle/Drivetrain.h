#pragma once
/**
 * Engine, Transmission, Gearbox, Clutch, and Limited-Slip Differential (LSD) System.
 */

#include <array>
#include <string>
#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/WheelState.h"

namespace torquelab {

enum class TransmissionMode {
    Automatic,
    Manual
};

struct DrivetrainState {
    double rpm{850.0};
    int gearIndex{2}; // 0 = R, 1 = N, 2 = 1st, 3 = 2nd, ..., 7 = 6th
    TransmissionMode transmissionMode{TransmissionMode::Automatic};
    double shiftTimer{0.0};
    double shiftCooldown{0.0};
    double engineTorqueNm{0.0};
    double clutchEngagement{1.0};
    bool revLimiterActive{false};
};

DrivetrainState createInitialDrivetrainState(double idleRpm = 850.0);

std::string getGearDisplayName(int gearIndex);

double sampleEngineTorqueCurve(double rpm, const VehicleConfig& config);

double computeEngineBrakingTorque(double rpm, const VehicleConfig& config);

double getTotalGearRatio(int gearIndex, const VehicleConfig& config);

void shiftToGear(
    DrivetrainState& state,
    int targetGearIndex,
    const VehicleConfig& config
);

void updateDrivetrain(
    DrivetrainState& state,
    std::array<WheelState, 4>& wheels,
    double throttleInput,
    double forwardSpeedMps,
    const VehicleConfig& config,
    double dt
);

} // namespace torquelab
