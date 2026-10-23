#pragma once
/**
 * Continuous Keyboard and Gamepad Input Controller (GLFW).
 *
 * Maps keyboard and optional gamepad inputs into continuous control values:
 * - W / Up Arrow: Throttle [0..1]
 * - S / Down Arrow: Service Brake / Reverse [0..1]
 * - A / Left Arrow: Steer Left (+1)
 * - D / Right Arrow: Steer Right (-1)
 * - Space: Handbrake [0..1]
 * - R: Reset vehicle
 * - C: Switch camera
 * - P: Pause/Resume simulation
 * - T: Toggle telemetry overlay
 * - G: Toggle 3D force vector gizmos
 * - M: Toggle Automatic / Manual transmission
 * - E / Q: Manual Gear Upshift / Downshift
 * - 1..7: Proving Ground Zone Teleport Presets
 * - Escape: Release inputs and toggle pause
 */

#include <array>
#include <functional>
#include <unordered_set>
#include <GLFW/glfw3.h>
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
};

class InputControllerGLFW {
public:
    explicit InputControllerGLFW(InputCallbacks callbacks);

    void onKeyEvent(int key, int action);
    void releaseAll();

    VehicleControlInputs sampleInputs(GLFWwindow* window, double dt);

private:
    InputCallbacks m_callbacks;
    std::unordered_set<int> m_keysHeld;

    double m_throttleState{0.0};
    double m_brakeState{0.0};
    double m_handbrakeState{0.0};

    bool m_gamepadBaselineCaptured{false};
    std::array<float, 6> m_gamepadBaselineAxes{0.0f, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f};
    std::array<bool, 6> m_gamepadAxisActivated{false, false, false, false, false, false};
};

} // namespace torquelab
