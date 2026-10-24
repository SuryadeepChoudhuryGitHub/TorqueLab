#pragma once
/**
 * Comprehensive Keyboard and Gamepad Input Controller (GLFW).
 *
 * Implements a unified action-based input pipeline inspired by Forza Horizon:
 * - Full analog trigger acceleration & braking with auto-reverse integration
 * - Smooth analog steering with configurable deadzones and sensitivity curves
 * - Multi-controller detection, hot-plugging, and auto-fallback
 * - Baseline activation guard against virtual HID controllers (Monect / vJoy)
 * - Right-stick free-look and camera controls
 * - Configurable button rebinding, Xbox & PlayStation display adaptivity
 * - Windows XInput rumble vibration with collision & traction feedback
 * - Safe keyboard + gamepad coexistence with defined priority policies
 */

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <GLFW/glfw3.h>
#include "input/ControllerConfig.h"
#include "input/VibrationManager.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

struct InputCallbacks {
    std::function<void()> onReset;
    std::function<void()> onCycleCamera;
    std::function<void()> onTogglePause;
    std::function<void()> onToggleTelemetry;
    std::function<void()> onToggleGizmos3D;
    std::function<void()> onToggleTransmissionMode;
    std::function<void()> onShiftUp;
    std::function<void()> onShiftDown;
    std::function<void(int)> onSelectPreset;
    std::function<void(int)> onStepPreset; // Relative step (+1 or -1)
    std::function<void()> onToggleModel;
    std::function<void()> onToggleVisualDebug;
};

struct GamepadDeviceInfo {
    int joystickId{-1};
    std::string name{"None"};
    bool isGamepad{false};
    ControllerFlavor flavor{ControllerFlavor::Generic};
};

class InputControllerGLFW {
public:
    explicit InputControllerGLFW(InputCallbacks callbacks);
    ~InputControllerGLFW();

    // Event hooks
    void onKeyEvent(int key, int action);
    void onJoystickEvent(int jid, int event);
    void releaseAll();

    // Main frame sampling (dt in seconds)
    VehicleControlInputs sampleInputs(GLFWwindow* window, double dt);

    // Camera Look-around (Right stick X, Y in [-1.0 .. 1.0])
    struct LookAroundInput {
        double x{0.0};
        double y{0.0};
    };
    LookAroundInput getCameraLookAround() const { return m_cameraLook; }

    // Controller Hardware & Diagnostics
    bool isGamepadConnected() const { return m_activeJoystickId >= 0; }
    int getActiveJoystickId() const { return m_activeJoystickId; }
    std::string getActiveGamepadName() const { return m_activeGamepadName; }
    bool isActiveGamepadStandard() const { return m_activeGamepadIsStandard; }
    ControllerFlavor getActiveFlavor() const;
    std::vector<GamepadDeviceInfo> enumerateDevices() const;

    // Controller Configuration & Rebinding
    ControllerSettings& getSettings() { return m_settings; }
    const ControllerSettings& getSettings() const { return m_settings; }
    void saveSettings();
    void resetSettingsToDefaults();

    void startRebinding(LogicalVehicleAction action);
    bool isRebinding() const { return m_rebindingActive; }
    LogicalVehicleAction getRebindingAction() const { return m_rebindingAction; }
    void cancelRebinding() { m_rebindingActive = false; }

    // Live Telemetry / UI Meters
    float getLiveThrottle() const { return static_cast<float>(m_liveThrottle); }
    float getLiveBrake() const { return static_cast<float>(m_liveBrake); }
    float getLiveSteering() const { return static_cast<float>(m_liveSteer); }
    float getLiveHandbrake() const { return static_cast<float>(m_liveHandbrake); }
    float getLiveLookX() const { return static_cast<float>(m_cameraLook.x); }
    float getLiveLookY() const { return static_cast<float>(m_cameraLook.y); }
    float getResetHoldProgress() const;

    // Vibration Feedback
    VibrationManager& getVibrationManager() { return m_vibration; }
    void triggerCollisionImpact(
        float impactSpeedMps,
        float normalImpulse,
        float lateralBias,
        bool isChassisScrape
    ) {
        m_vibration.triggerCollision(
            impactSpeedMps,
            normalImpulse,
            lateralBias,
            isChassisScrape,
            m_settings.collisionSensitivity
        );
    }
    void triggerSuspensionImpact(
        float severity,
        float lateralBias,
        bool isHardLanding,
        bool isBottomOut
    ) {
        m_vibration.triggerSuspension(
            severity,
            lateralBias,
            isHardLanding,
            isBottomOut,
            m_settings.suspensionSensitivity
        );
    }
    void triggerTestPulse(float left = 0.6f, float right = 0.8f, float duration = 0.25f) {
        m_vibration.triggerTestPulse(left, right, duration);
    }

private:
    void pollGamepad(double dt, double& padThrottle, double& padBrake, double& padSteer, double& padHandbrake);
    void updateActiveDevice();
    void calibrateBaseline(int jid, const GLFWgamepadstate& padState);

    InputCallbacks m_callbacks;
    ControllerSettings m_settings;
    VibrationManager m_vibration;

    // Keyboard state
    std::unordered_set<int> m_keysHeld;
    double m_throttleState{0.0};
    double m_brakeState{0.0};
    double m_handbrakeState{0.0};

    // Gamepad state
    int m_activeJoystickId{-1};
    std::string m_activeGamepadName{"None"};
    bool m_activeGamepadIsStandard{false};
    ControllerFlavor m_detectedFlavor{ControllerFlavor::Xbox};

    bool m_gamepadBaselineCaptured{false};
    std::array<float, 6> m_gamepadBaselineAxes{0.0f, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f};
    std::array<bool, 6> m_gamepadAxisActivated{false, false, false, false, false, false};

    std::array<unsigned char, 15> m_prevPadButtons{0};
    double m_resetHoldTimer{0.0};
    bool m_resetTriggeredThisHold{false};

    LookAroundInput m_cameraLook{0.0, 0.0};

    // Rebinding state
    bool m_rebindingActive{false};
    LogicalVehicleAction m_rebindingAction{LogicalVehicleAction::Handbrake};

    // Cached live values for dashboard inspection
    double m_liveThrottle{0.0};
    double m_liveBrake{0.0};
    double m_liveSteer{0.0};
    double m_liveHandbrake{0.0};
};

} // namespace torquelab
