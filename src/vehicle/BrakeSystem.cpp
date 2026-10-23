#include "vehicle/BrakeSystem.h"

namespace torquelab {

WheelBrakeTorques computeWheelBrakeTorques(
    double brakeInput,
    double handbrakeInput,
    const VehicleConfig& config
) {
    const double b = clamp(brakeInput, 0.0, 1.0);
    const double hb = clamp(handbrakeInput, 0.0, 1.0);
    const double frontBias = clamp(config.brakeFrontBias, 0.3, 0.85);
    const double rearBias = 1.0 - frontBias;

    const double frontWheelServiceBrake = b * config.maxBrakeTorque * frontBias;
    const double rearWheelServiceBrake = b * config.maxBrakeTorque * rearBias;
    const double rearHandbrake = hb * config.handbrakeTorque;

    return WheelBrakeTorques{
        frontWheelServiceBrake,
        frontWheelServiceBrake,
        rearWheelServiceBrake + rearHandbrake,
        rearWheelServiceBrake + rearHandbrake
    };
}

} // namespace torquelab
