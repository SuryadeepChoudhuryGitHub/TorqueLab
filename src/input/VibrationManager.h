#pragma once
/**
 * Physics-Driven Controller Force-Feedback & Rumble Vibration Manager.
 *
 * Implements an event-driven haptic pipeline inspired by Forza Horizon:
 * - Dynamic collision impact feedback with severity scaling & lateral directionality
 * - Discrete suspension events (jump landings, bump-stop bottoming out, curb strikes)
 * - Chassis underside scrape transient buzz
 * - Purely event-driven: zero vibration from engine RPM, throttle, or smooth cruising
 * - Independent low-frequency (heavy) and high-frequency (light) motor envelopes
 * - Smooth nonlinear decay curves and collision deduplication
 * - Dynamic Windows XInput loading with safe multi-platform fallbacks
 */

#include <algorithm>
#include <cmath>
#include <string>

namespace torquelab {

struct HapticPulseEnvelope {
    float leftMotor{0.0f};
    float rightMotor{0.0f};
    float duration{0.0f};
    float timer{0.0f};

    bool isActive() const { return timer < duration && (leftMotor > 0.001f || rightMotor > 0.001f); }
    void clear() { leftMotor = 0.0f; rightMotor = 0.0f; duration = 0.0f; timer = 0.0f; }
};

class VibrationManager {
public:
    VibrationManager();
    ~VibrationManager();

    bool isAvailable() const { return m_available; }

    /**
     * Triggers event-driven collision impact haptics.
     * @param impactSpeedMps Instantaneous delta-v magnitude across impact
     * @param normalImpulse Bullet solver normal contact impulse (N*s)
     * @param lateralBias [-1.0 .. +1.0], +1 = full left, -1 = full right
     * @param isChassisScrape True if contact was an underside floor scrape
     * @param sensitivity Collision sensitivity multiplier [0.0 .. 2.0]
     */
    void triggerCollision(
        float impactSpeedMps,
        float normalImpulse,
        float lateralBias,
        bool isChassisScrape,
        float sensitivity = 1.0f
    );

    /**
     * Triggers event-driven suspension feedback.
     * @param severity Normalized event severity [0.0 .. 1.0]
     * @param lateralBias [-1.0 .. +1.0], +1 = full left, -1 = full right
     * @param isHardLanding True if airborne touchdown with high downward velocity
     * @param isBottomOut True if progressive bump stop travel limit was hit
     * @param sensitivity Suspension sensitivity multiplier [0.0 .. 2.0]
     */
    void triggerSuspension(
        float severity,
        float lateralBias,
        bool isHardLanding,
        bool isBottomOut,
        float sensitivity = 1.0f
    );

    /** Triggers a direct synthetic pulse (for testing and dashboard calibration). */
    void triggerTestPulse(float left = 0.6f, float right = 0.8f, float duration = 0.25f);

    /** Backward-compatible single-float impact trigger */
    void triggerImpact(float intensity) {
        triggerCollision(intensity * 6.0f, intensity * 5000.0f, 0.0f, false, 1.0f);
    }

    /** Updates active envelopes and delivers vibration words to XInput. */
    void update(float dt, int userIndex, bool enabled, float masterIntensity);

    /** Immediately zeroes both vibration motors. */
    void stop(int userIndex);

    // Diagnostics & live state queries for dashboard inspection & automated tests
    float getLiveLeftMotor() const { return m_currentLeft; }
    float getLiveRightMotor() const { return m_currentRight; }
    bool isCollisionPulseActive() const { return m_collisionEnvelope.isActive(); }
    bool isSuspensionPulseActive() const { return m_suspensionEnvelope.isActive(); }

private:
    bool m_available{false};
    void* m_xinputModule{nullptr};
    void* m_pfnSetState{nullptr};

    HapticPulseEnvelope m_collisionEnvelope{};
    HapticPulseEnvelope m_suspensionEnvelope{};

    float m_collisionCooldown{0.0f};
    float m_suspensionCooldown{0.0f};

    float m_currentLeft{0.0f};
    float m_currentRight{0.0f};

    unsigned short m_lastLeftMotor{0};
    unsigned short m_lastRightMotor{0};
};

} // namespace torquelab
