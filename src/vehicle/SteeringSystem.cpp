#include "vehicle/SteeringSystem.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

double computeSpeedSensitiveMaxSteer(
    double forwardSpeedMps,
    const VehicleConfig& config
) {
    const double speedFactor = 1.0 + config.steeringSpeedSensitivity * std::abs(forwardSpeedMps);
    const double minHighSpeedSteer = config.maxSteeringAngle * 0.22;
    return std::max(minHighSpeedSteer, config.maxSteeringAngle / speedFactor);
}

double updateCenterSteeringAngle(
    double currentAngle,
    double steerInput,
    double forwardSpeedMps,
    const VehicleConfig& config,
    double dt
) {
    const double clampedInput = clamp(steerInput, -1.0, 1.0);
    const double maxSteer = computeSpeedSensitiveMaxSteer(forwardSpeedMps, config);
    const double targetAngle = clampedInput * maxSteer;

    const double diff = targetAngle - currentAngle;
    const bool isReturningToCenter =
        std::abs(targetAngle) < std::abs(currentAngle) || (targetAngle * currentAngle < 0.0);
    const double rate = isReturningToCenter ? config.steeringReturnSpeed : config.steeringSpeed;
    const double maxDelta = rate * dt;

    if (std::abs(diff) <= maxDelta) {
        return targetAngle;
    }
    return currentAngle + std::copysign(maxDelta, diff);
}

AckermannSteeringAngles computeAckermannWheelAngles(
    double centerAngle,
    const VehicleConfig& config
) {
    if (std::abs(centerAngle) < 1e-5) {
        return AckermannSteeringAngles{0.0, 0.0, 0.0};
    }

    const double L = config.wheelbase;
    const double halfT = config.trackWidth * 0.5;
    const double tanDelta = std::tan(std::abs(centerAngle));
    const double R = L / std::max(1e-5, tanDelta);

    const double innerAckermann = std::atan(L / std::max(0.2, R - halfT));
    const double outerAckermann = std::atan(L / (R + halfT));

    const double blend = clamp(config.ackermannFactor, 0.0, 1.0);
    const double innerBlended = lerp(std::abs(centerAngle), innerAckermann, blend);
    const double outerBlended = lerp(std::abs(centerAngle), outerAckermann, blend);

    if (centerAngle > 0.0) {
        // Left turn (+): Front-Left is inner wheel, Front-Right is outer wheel
        return AckermannSteeringAngles{
            centerAngle,
            +innerBlended,
            +outerBlended
        };
    }
    // Right turn (-): Front-Right is inner wheel, Front-Left is outer wheel
    return AckermannSteeringAngles{
        centerAngle,
        -outerBlended,
        -innerBlended
    };
}

} // namespace torquelab
