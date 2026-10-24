#include "input/VibrationManager.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

typedef struct _TL_XINPUT_VIBRATION {
    WORD wLeftMotorSpeed;
    WORD wRightMotorSpeed;
} TL_XINPUT_VIBRATION;

typedef DWORD (WINAPI *PFN_TL_XInputSetState)(DWORD dwUserIndex, TL_XINPUT_VIBRATION* pVibration);
#endif

namespace torquelab {

VibrationManager::VibrationManager() {
#ifdef _WIN32
    const char* dlls[] = {"xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"};
    for (const char* dllName : dlls) {
        HMODULE hMod = LoadLibraryA(dllName);
        if (hMod) {
            auto fn = reinterpret_cast<PFN_TL_XInputSetState>(GetProcAddress(hMod, "XInputSetState"));
            if (fn) {
                m_xinputModule = static_cast<void*>(hMod);
                m_pfnSetState = reinterpret_cast<void*>(fn);
                m_available = true;
                break;
            } else {
                FreeLibrary(hMod);
            }
        }
    }
#endif
}

VibrationManager::~VibrationManager() {
    stop(0);
#ifdef _WIN32
    if (m_xinputModule) {
        FreeLibrary(static_cast<HMODULE>(m_xinputModule));
        m_xinputModule = nullptr;
    }
#endif
}

void VibrationManager::triggerCollision(
    float impactSpeedMps,
    float normalImpulse,
    float lateralBias,
    bool isChassisScrape,
    float sensitivity
) {
    if (!m_available || sensitivity <= 0.001f) return;

    const float leftMultiplier = std::clamp(1.0f + lateralBias * 0.60f, 0.30f, 1.35f);
    const float rightMultiplier = std::clamp(1.0f - lateralBias * 0.60f, 0.30f, 1.35f);

    if (isChassisScrape) {
        // High-frequency texture buzz for underside ground/curb scrape
        const float scrapeIntensity = std::clamp(sensitivity * 0.45f, 0.12f, 0.65f);
        m_collisionEnvelope.leftMotor = 0.0f;
        m_collisionEnvelope.rightMotor = std::clamp(scrapeIntensity * rightMultiplier, 0.0f, 1.0f);
        m_collisionEnvelope.duration = 0.08f;
        m_collisionEnvelope.timer = 0.0f;
        return;
    }

    // Impact speed normalized [0.4 m/s threshold .. 8.0 m/s ceiling]
    const float normSpeed = std::clamp((impactSpeedMps - 0.4f) / 7.6f, 0.0f, 1.0f);
    const float normImpulse = std::clamp(normalImpulse / 8000.0f, 0.0f, 1.0f);
    const float baseSeverity = std::max(normSpeed, normImpulse * 0.85f);
    if (baseSeverity < 0.02f) return;

    // Nonlinear response curve provides subtle light taps and punchy crashes
    const float curved = std::pow(baseSeverity, 0.70f);
    const float intensity = std::clamp(curved * sensitivity, 0.12f, 1.0f);

    // Deduplication: suppress weaker secondary contact chatter during an existing impact pulse
    if (m_collisionCooldown > 0.0f && intensity <= m_collisionEnvelope.leftMotor && m_collisionEnvelope.isActive()) {
        return;
    }

    const float duration = std::clamp(0.12f + intensity * 0.24f, 0.12f, 0.38f);
    const float left = std::clamp(intensity * 0.90f * leftMultiplier, 0.0f, 1.0f);
    const float right = std::clamp(intensity * 1.00f * rightMultiplier, 0.0f, 1.0f);

    m_collisionEnvelope.leftMotor = left;
    m_collisionEnvelope.rightMotor = right;
    m_collisionEnvelope.duration = duration;
    m_collisionEnvelope.timer = 0.0f;
    m_collisionCooldown = 0.08f;
}

void VibrationManager::triggerSuspension(
    float severity,
    float lateralBias,
    bool isHardLanding,
    bool isBottomOut,
    float sensitivity
) {
    if (!m_available || sensitivity <= 0.001f || severity < 0.05f) return;

    const float s = std::clamp(severity * sensitivity, 0.0f, 1.0f);
    if (m_suspensionCooldown > 0.0f && s <= m_suspensionEnvelope.leftMotor && m_suspensionEnvelope.isActive()) {
        return;
    }

    const float leftMultiplier = std::clamp(1.0f + lateralBias * 0.60f, 0.30f, 1.35f);
    const float rightMultiplier = std::clamp(1.0f - lateralBias * 0.60f, 0.30f, 1.35f);

    float left = 0.0f;
    float right = 0.0f;
    float duration = 0.10f;

    if (isHardLanding) {
        // Airborne touchdown: heavy thud on low-frequency motor
        left = std::clamp(s * 0.85f * leftMultiplier, 0.0f, 1.0f);
        right = std::clamp(s * 0.40f * rightMultiplier, 0.0f, 1.0f);
        duration = std::clamp(0.14f + s * 0.12f, 0.14f, 0.26f);
    } else if (isBottomOut) {
        // Bump stop travel limit: solid mechanical jolt
        left = std::clamp(s * 0.95f * leftMultiplier, 0.0f, 1.0f);
        right = std::clamp(s * 0.50f * rightMultiplier, 0.0f, 1.0f);
        duration = std::clamp(0.12f + s * 0.10f, 0.12f, 0.22f);
    } else {
        // Sharp curb / bump strike: crisp high-frequency tick
        left = std::clamp(s * 0.25f * leftMultiplier, 0.0f, 1.0f);
        right = std::clamp(s * 0.75f * rightMultiplier, 0.0f, 1.0f);
        duration = 0.10f;
    }

    m_suspensionEnvelope.leftMotor = left;
    m_suspensionEnvelope.rightMotor = right;
    m_suspensionEnvelope.duration = duration;
    m_suspensionEnvelope.timer = 0.0f;
    m_suspensionCooldown = 0.06f;
}

void VibrationManager::triggerTestPulse(float left, float right, float duration) {
    m_collisionEnvelope.leftMotor = std::clamp(left, 0.0f, 1.0f);
    m_collisionEnvelope.rightMotor = std::clamp(right, 0.0f, 1.0f);
    m_collisionEnvelope.duration = std::clamp(duration, 0.05f, 1.0f);
    m_collisionEnvelope.timer = 0.0f;
    m_collisionCooldown = 0.08f;
}

void VibrationManager::update(float dt, int userIndex, bool enabled, float masterIntensity) {
    if (!m_available) return;

    if (m_collisionCooldown > 0.0f) m_collisionCooldown = std::max(0.0f, m_collisionCooldown - dt);
    if (m_suspensionCooldown > 0.0f) m_suspensionCooldown = std::max(0.0f, m_suspensionCooldown - dt);

    float colLeft = 0.0f;
    float colRight = 0.0f;
    if (m_collisionEnvelope.isActive()) {
        m_collisionEnvelope.timer += dt;
        const float progress = std::clamp(m_collisionEnvelope.timer / m_collisionEnvelope.duration, 0.0f, 1.0f);
        const float decay = std::cos(progress * 1.5707963f);
        colLeft = m_collisionEnvelope.leftMotor * decay;
        colRight = m_collisionEnvelope.rightMotor * decay;
    } else {
        m_collisionEnvelope.clear();
    }

    float suspLeft = 0.0f;
    float suspRight = 0.0f;
    if (m_suspensionEnvelope.isActive()) {
        m_suspensionEnvelope.timer += dt;
        const float progress = std::clamp(m_suspensionEnvelope.timer / m_suspensionEnvelope.duration, 0.0f, 1.0f);
        const float decay = std::cos(progress * 1.5707963f);
        suspLeft = m_suspensionEnvelope.leftMotor * decay;
        suspRight = m_suspensionEnvelope.rightMotor * decay;
    } else {
        m_suspensionEnvelope.clear();
    }

    m_currentLeft = std::clamp(colLeft + suspLeft, 0.0f, 1.0f);
    m_currentRight = std::clamp(colRight + suspRight, 0.0f, 1.0f);

    if (!enabled || masterIntensity <= 0.001f || userIndex < 0 || userIndex > 3) {
        stop(userIndex);
        return;
    }

    const float outLeft = std::clamp(m_currentLeft * masterIntensity, 0.0f, 1.0f);
    const float outRight = std::clamp(m_currentRight * masterIntensity, 0.0f, 1.0f);

    const unsigned short leftWord = static_cast<unsigned short>(std::round(outLeft * 65535.0f));
    const unsigned short rightWord = static_cast<unsigned short>(std::round(outRight * 65535.0f));

    if (leftWord != m_lastLeftMotor || rightWord != m_lastRightMotor) {
        m_lastLeftMotor = leftWord;
        m_lastRightMotor = rightWord;

#ifdef _WIN32
        if (m_pfnSetState) {
            TL_XINPUT_VIBRATION vib;
            vib.wLeftMotorSpeed = leftWord;
            vib.wRightMotorSpeed = rightWord;
            auto fn = reinterpret_cast<PFN_TL_XInputSetState>(m_pfnSetState);
            fn(static_cast<DWORD>(userIndex), &vib);
        }
#endif
    }
}

void VibrationManager::stop(int userIndex) {
    m_collisionEnvelope.clear();
    m_suspensionEnvelope.clear();
    m_collisionCooldown = 0.0f;
    m_suspensionCooldown = 0.0f;
    m_currentLeft = 0.0f;
    m_currentRight = 0.0f;

    if (m_lastLeftMotor != 0 || m_lastRightMotor != 0) {
        m_lastLeftMotor = 0;
        m_lastRightMotor = 0;

#ifdef _WIN32
        if (m_pfnSetState && userIndex >= 0 && userIndex <= 3) {
            TL_XINPUT_VIBRATION vib;
            vib.wLeftMotorSpeed = 0;
            vib.wRightMotorSpeed = 0;
            auto fn = reinterpret_cast<PFN_TL_XInputSetState>(m_pfnSetState);
            fn(static_cast<DWORD>(userIndex), &vib);
        }
#endif
    }
}

} // namespace torquelab
