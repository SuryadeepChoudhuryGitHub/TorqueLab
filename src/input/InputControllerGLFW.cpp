#include "input/InputControllerGLFW.h"
#include <algorithm>
#include <cmath>
#include "physics/Math3D.h"

namespace torquelab {

InputControllerGLFW::InputControllerGLFW(InputCallbacks callbacks)
    : m_callbacks(std::move(callbacks))
{}

void InputControllerGLFW::onKeyEvent(int key, int action) {
    if (action == GLFW_PRESS) {
        const bool wasHeld = (m_keysHeld.find(key) != m_keysHeld.end());
        m_keysHeld.insert(key);

        if (!wasHeld) {
            switch (key) {
                case GLFW_KEY_R:
                    if (m_callbacks.onReset) m_callbacks.onReset();
                    break;
                case GLFW_KEY_C:
                    if (m_callbacks.onCycleCamera) m_callbacks.onCycleCamera();
                    break;
                case GLFW_KEY_P:
                    if (m_callbacks.onTogglePause) m_callbacks.onTogglePause();
                    break;
                case GLFW_KEY_T:
                    if (m_callbacks.onToggleTelemetry) m_callbacks.onToggleTelemetry();
                    break;
                case GLFW_KEY_G:
                    if (m_callbacks.onToggleGizmos3D) m_callbacks.onToggleGizmos3D();
                    break;
                case GLFW_KEY_B:
                    if (m_callbacks.onToggleModel) m_callbacks.onToggleModel();
                    break;
                case GLFW_KEY_V:
                    if (m_callbacks.onToggleVisualDebug) m_callbacks.onToggleVisualDebug();
                    break;
                case GLFW_KEY_M:
                    if (m_callbacks.onToggleTransmissionMode) m_callbacks.onToggleTransmissionMode();
                    break;
                case GLFW_KEY_E:
                    if (m_callbacks.onShiftUp) m_callbacks.onShiftUp();
                    break;
                case GLFW_KEY_Q:
                    if (m_callbacks.onShiftDown) m_callbacks.onShiftDown();
                    break;
                case GLFW_KEY_ESCAPE:
                    releaseAll();
                    if (m_callbacks.onTogglePause) m_callbacks.onTogglePause();
                    break;
                case GLFW_KEY_1:
                case GLFW_KEY_2:
                case GLFW_KEY_3:
                case GLFW_KEY_4:
                case GLFW_KEY_5:
                case GLFW_KEY_6:
                case GLFW_KEY_7:
                    if (m_callbacks.onSelectPreset) {
                        m_callbacks.onSelectPreset(key - GLFW_KEY_1);
                    }
                    break;
                default:
                    break;
            }
        }
    } else if (action == GLFW_RELEASE) {
        m_keysHeld.erase(key);
    }
}

void InputControllerGLFW::releaseAll() {
    m_keysHeld.clear();
    m_throttleState = 0.0;
    m_brakeState = 0.0;
    m_handbrakeState = 0.0;
}

VehicleControlInputs InputControllerGLFW::sampleInputs(GLFWwindow* window, double dt) {
    const auto isHeld = [&](int k) {
        return (window && glfwGetKey(window, k) == GLFW_PRESS) ||
               (m_keysHeld.find(k) != m_keysHeld.end());
    };

    const bool wHeld = isHeld(GLFW_KEY_W) || isHeld(GLFW_KEY_UP);
    const bool sHeld = isHeld(GLFW_KEY_S) || isHeld(GLFW_KEY_DOWN);
    const bool aHeld = isHeld(GLFW_KEY_A) || isHeld(GLFW_KEY_LEFT);
    const bool dHeld = isHeld(GLFW_KEY_D) || isHeld(GLFW_KEY_RIGHT);
    const bool spaceHeld = isHeld(GLFW_KEY_SPACE);

    const double engageRate = 9.0 * dt;
    const double releaseRate = 12.0 * dt;

    m_throttleState = wHeld
        ? std::min(1.0, m_throttleState + engageRate)
        : std::max(0.0, m_throttleState - releaseRate);

    m_brakeState = sHeld
        ? std::min(1.0, m_brakeState + engageRate)
        : std::max(0.0, m_brakeState - releaseRate);

    m_handbrakeState = spaceHeld
        ? std::min(1.0, m_handbrakeState + engageRate * 1.5)
        : std::max(0.0, m_handbrakeState - releaseRate * 1.5);

    double steerTarget = 0.0;
    if (aHeld) steerTarget += 1.0;
    if (dHeld) steerTarget -= 1.0;

    double throttle = m_throttleState;
    double brake = m_brakeState;
    double handbrake = m_handbrakeState;

    // Poll GLFW Gamepad 1 if connected.
    // Virtual HID devices (e.g., Monect Hid Device, vJoy) often report 0.0f on all axes
    // including triggers when idle instead of the XInput -1.0f resting baseline.
    // We capture the initial resting baseline per axis and only activate an axis once
    // it deviates from its resting state, preventing phantom 50% brake/throttle lockup.
    GLFWgamepadstate padState{};
    if (glfwJoystickIsGamepad(GLFW_JOYSTICK_1) && glfwGetGamepadState(GLFW_JOYSTICK_1, &padState)) {
        if (!m_gamepadBaselineCaptured) {
            for (size_t i = 0; i < 6; ++i) {
                m_gamepadBaselineAxes[i] = padState.axes[i];
                m_gamepadAxisActivated[i] = false;
            }
            m_gamepadBaselineCaptured = true;
        }

        for (size_t i = 0; i < 6; ++i) {
            if (std::abs(padState.axes[i] - m_gamepadBaselineAxes[i]) > 0.12f) {
                m_gamepadAxisActivated[i] = true;
            }
        }

        if (m_gamepadAxisActivated[GLFW_GAMEPAD_AXIS_LEFT_X]) {
            const double leftStickX = static_cast<double>(padState.axes[GLFW_GAMEPAD_AXIS_LEFT_X]);
            if (std::abs(leftStickX) > 0.08) {
                steerTarget = clamp(-leftStickX, -1.0, 1.0);
            }
        }

        auto evalTrigger = [&](int axisIdx) -> double {
            if (!m_gamepadAxisActivated[static_cast<size_t>(axisIdx)]) {
                return 0.0;
            }
            const float raw = padState.axes[axisIdx];
            const float base = m_gamepadBaselineAxes[static_cast<size_t>(axisIdx)];
            if (base < -0.5f) {
                return clamp((static_cast<double>(raw) + 1.0) * 0.5, 0.0, 1.0);
            }
            return clamp(static_cast<double>(raw - base), 0.0, 1.0);
        };

        const double r2 = evalTrigger(GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER);
        const double l2 = evalTrigger(GLFW_GAMEPAD_AXIS_LEFT_TRIGGER);
        if (r2 > 0.05) throttle = std::max(throttle, r2);
        if (l2 > 0.05) brake = std::max(brake, l2);
        if (padState.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS) {
            handbrake = 1.0;
        }
    } else {
        m_gamepadBaselineCaptured = false;
        m_gamepadAxisActivated.fill(false);
    }

    return VehicleControlInputs{
        clamp(throttle, 0.0, 1.0),
        clamp(brake, 0.0, 1.0),
        clamp(steerTarget, -1.0, 1.0),
        clamp(handbrake, 0.0, 1.0),
        false
    };
}

} // namespace torquelab
