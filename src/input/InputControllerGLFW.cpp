#include "input/InputControllerGLFW.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "physics/Math3D.h"

namespace torquelab {

InputControllerGLFW::InputControllerGLFW(InputCallbacks callbacks)
    : m_callbacks(std::move(callbacks))
{
    // Load persisted settings or initialize Forza defaults
    if (!loadControllerSettingsFromJson(m_settings)) {
        m_settings.resetToForzaDefaults();
        saveControllerSettingsToJson(m_settings);
    }

    updateActiveDevice();
}

InputControllerGLFW::~InputControllerGLFW() {
    if (m_activeJoystickId >= 0) {
        m_vibration.stop(m_activeJoystickId);
    }
}

void InputControllerGLFW::onJoystickEvent(int jid, int event) {
    if (event == GLFW_CONNECTED) {
        std::printf("[Gamepad Hotplug] Connected: Joystick ID=%d, Name='%s', IsGamepad=%d\n",
                    jid,
                    glfwGetJoystickName(jid) ? glfwGetJoystickName(jid) : "Unknown",
                    glfwJoystickIsGamepad(jid));
        updateActiveDevice();
    } else if (event == GLFW_DISCONNECTED) {
        std::printf("[Gamepad Hotplug] Disconnected: Joystick ID=%d\n", jid);
        if (m_activeJoystickId == jid) {
            m_vibration.stop(jid);
            m_activeJoystickId = -1;
            m_activeGamepadName = "None";
            m_activeGamepadIsStandard = false;
            m_gamepadBaselineCaptured = false;
            m_gamepadAxisActivated.fill(false);
            m_prevPadButtons.fill(0);
            m_cameraLook = {0.0, 0.0};
            m_liveThrottle = 0.0;
            m_liveBrake = 0.0;
            m_liveSteer = 0.0;
            m_liveHandbrake = 0.0;
        }
        updateActiveDevice();
    }
}

void InputControllerGLFW::updateActiveDevice() {
    int candidate = -1;

    // 1. Check user-preferred device first
    if (m_settings.preferredJoystickId >= 0 &&
        m_settings.preferredJoystickId <= GLFW_JOYSTICK_LAST &&
        glfwJoystickPresent(m_settings.preferredJoystickId)) {
        candidate = m_settings.preferredJoystickId;
    }

    // 2. Auto-detect: first standard gamepad across all 16 GLFW joystick ports
    if (candidate < 0) {
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
            if (glfwJoystickPresent(jid) && glfwJoystickIsGamepad(jid)) {
                candidate = jid;
                break;
            }
        }
    }

    // 3. Fallback: first present raw joystick
    if (candidate < 0) {
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
            if (glfwJoystickPresent(jid)) {
                candidate = jid;
                break;
            }
        }
    }

    if (candidate != m_activeJoystickId) {
        if (m_activeJoystickId >= 0) {
            m_vibration.stop(m_activeJoystickId);
        }
        m_activeJoystickId = candidate;
        m_gamepadBaselineCaptured = false;
        m_gamepadAxisActivated.fill(false);
        m_prevPadButtons.fill(0);
        m_resetHoldTimer = 0.0;
        m_resetTriggeredThisHold = false;
        m_cameraLook = {0.0, 0.0};

        if (candidate >= 0) {
            m_activeGamepadIsStandard = (glfwJoystickIsGamepad(candidate) != 0);
            const char* name = m_activeGamepadIsStandard
                ? glfwGetGamepadName(candidate)
                : glfwGetJoystickName(candidate);
            m_activeGamepadName = name ? name : ("Joystick " + std::to_string(candidate));
            m_detectedFlavor = detectControllerFlavorFromName(m_activeGamepadName);
            std::printf("[Gamepad Select] Active controller switched to: ID=%d ('%s', Standard=%d, Flavor=%s)\n",
                        candidate, m_activeGamepadName.c_str(), m_activeGamepadIsStandard,
                        getControllerFlavorName(m_detectedFlavor).c_str());
        } else {
            m_activeGamepadName = "No Controller Connected";
            m_activeGamepadIsStandard = false;
            m_detectedFlavor = ControllerFlavor::Generic;
        }
    }
}

std::vector<GamepadDeviceInfo> InputControllerGLFW::enumerateDevices() const {
    std::vector<GamepadDeviceInfo> devices;
    for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        if (glfwJoystickPresent(jid)) {
            GamepadDeviceInfo info;
            info.joystickId = jid;
            info.isGamepad = (glfwJoystickIsGamepad(jid) != 0);
            const char* name = info.isGamepad ? glfwGetGamepadName(jid) : glfwGetJoystickName(jid);
            info.name = name ? name : ("Joystick " + std::to_string(jid));
            info.flavor = detectControllerFlavorFromName(info.name);
            devices.push_back(info);
        }
    }
    return devices;
}

ControllerFlavor InputControllerGLFW::getActiveFlavor() const {
    if (m_settings.flavor != ControllerFlavor::Auto) {
        return m_settings.flavor;
    }
    return m_detectedFlavor;
}

void InputControllerGLFW::saveSettings() {
    saveControllerSettingsToJson(m_settings);
}

void InputControllerGLFW::resetSettingsToDefaults() {
    m_settings.resetToForzaDefaults();
    saveSettings();
}

void InputControllerGLFW::startRebinding(LogicalVehicleAction action) {
    m_rebindingAction = action;
    m_rebindingActive = true;
}

float InputControllerGLFW::getResetHoldProgress() const {
    if (m_settings.resetHoldDuration <= 0.01f) {
        return m_resetTriggeredThisHold ? 1.0f : 0.0f;
    }
    return std::clamp(static_cast<float>(m_resetHoldTimer / m_settings.resetHoldDuration), 0.0f, 1.0f);
}

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
    m_liveThrottle = 0.0;
    m_liveBrake = 0.0;
    m_liveSteer = 0.0;
    m_liveHandbrake = 0.0;
    m_cameraLook = {0.0, 0.0};
    m_resetHoldTimer = 0.0;
    m_resetTriggeredThisHold = false;
    if (m_activeJoystickId >= 0) {
        m_vibration.stop(m_activeJoystickId);
    }
}

void InputControllerGLFW::calibrateBaseline(int /*jid*/, const GLFWgamepadstate& padState) {
    for (size_t i = 0; i < 6; ++i) {
        m_gamepadBaselineAxes[i] = padState.axes[i];
        m_gamepadAxisActivated[i] = false;
    }
    m_gamepadBaselineCaptured = true;
}

void InputControllerGLFW::pollGamepad(double dt, double& padThrottle, double& padBrake, double& padSteer, double& padHandbrake) {
    padThrottle = 0.0;
    padBrake = 0.0;
    padSteer = 0.0;
    padHandbrake = 0.0;
    m_cameraLook = {0.0, 0.0};

    if (m_activeJoystickId < 0 || !glfwJoystickPresent(m_activeJoystickId)) {
        updateActiveDevice();
        if (m_activeJoystickId < 0) return;
    }

    GLFWgamepadstate padState{};
    bool hasState = false;

    if (m_activeGamepadIsStandard) {
        hasState = (glfwGetGamepadState(m_activeJoystickId, &padState) != 0);
    } else {
        // Fallback for non-standard joysticks: synthesize GLFWgamepadstate from raw axes & buttons
        int axesCount = 0;
        int btnCount = 0;
        const float* rawAxes = glfwGetJoystickAxes(m_activeJoystickId, &axesCount);
        const unsigned char* rawBtns = glfwGetJoystickButtons(m_activeJoystickId, &btnCount);
        if (rawAxes && axesCount > 0) {
            for (int i = 0; i < std::min(6, axesCount); ++i) {
                padState.axes[i] = rawAxes[i];
            }
            if (rawBtns && btnCount > 0) {
                for (int i = 0; i < std::min(15, btnCount); ++i) {
                    padState.buttons[i] = rawBtns[i];
                }
            }
            hasState = true;
        }
    }

    if (!hasState) return;

    // 1. Initial resting baseline calibration per device connection
    if (!m_gamepadBaselineCaptured) {
        calibrateBaseline(m_activeJoystickId, padState);
    }

    // 2. Check for axis deviation from initial resting baseline
    // Guard against virtual HID devices (Monect, vJoy) reporting 0.0f idle trigger values
    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(padState.axes[i] - m_gamepadBaselineAxes[i]) > 0.10f) {
            m_gamepadAxisActivated[i] = true;
        }
    }

    // 3. Normalized trigger evaluation
    auto evalTrigger = [&](int axisIdx) -> double {
        if (axisIdx < 0 || axisIdx >= 6 || !m_gamepadAxisActivated[static_cast<size_t>(axisIdx)]) {
            return 0.0;
        }
        const float raw = padState.axes[axisIdx];
        const float base = m_gamepadBaselineAxes[static_cast<size_t>(axisIdx)];
        double val = 0.0;
        if (base < -0.5f) {
            // Standard XInput trigger: rest is -1.0, fully pressed is +1.0
            val = (static_cast<double>(raw) + 1.0) * 0.5;
        } else {
            // Devices resting at 0.0 or intermediate baselines
            val = static_cast<double>(raw - base) / std::max(0.1, static_cast<double>(1.0f - base));
        }
        val = clamp(val, 0.0, 1.0);

        // Configurable trigger deadzone
        const double dz = static_cast<double>(m_settings.triggerDeadzone);
        if (val <= dz) return 0.0;
        return clamp((val - dz) / (1.0 - dz), 0.0, 1.0);
    };

    padThrottle = evalTrigger(m_settings.axisThrottle);
    padBrake = evalTrigger(m_settings.axisBrake);

    // 4. Normalized steering stick evaluation
    if (m_settings.axisSteer >= 0 && m_settings.axisSteer < 6 &&
        m_gamepadAxisActivated[static_cast<size_t>(m_settings.axisSteer)]) {
        const float raw = padState.axes[m_settings.axisSteer];
        const float base = m_gamepadBaselineAxes[static_cast<size_t>(m_settings.axisSteer)];
        const double delta = static_cast<double>(raw - base);
        const double mag = std::abs(delta);
        const double dz = static_cast<double>(m_settings.steerDeadzone);

        if (mag > dz) {
            double norm = clamp((mag - dz) / (1.0 - dz), 0.0, 1.0);
            if (m_settings.steerSensitivity != 1.0f && m_settings.steerSensitivity > 0.1f) {
                norm = std::pow(norm, static_cast<double>(m_settings.steerSensitivity));
            }
            const double sign = (delta > 0.0) ? 1.0 : -1.0;
            // Coordinate convention: Left is +1.0, Right is -1.0. Standard stick left is raw < 0.
            double steer = -sign * norm;
            if (m_settings.invertSteering) steer = -steer;
            padSteer = clamp(steer, -1.0, 1.0);
        }
    }

    // 5. Right stick look-around evaluation
    if (m_settings.axisLookX >= 0 && m_settings.axisLookX < 6 &&
        m_settings.axisLookY >= 0 && m_settings.axisLookY < 6 &&
        (m_gamepadAxisActivated[static_cast<size_t>(m_settings.axisLookX)] ||
         m_gamepadAxisActivated[static_cast<size_t>(m_settings.axisLookY)])) {
        const float rawX = padState.axes[m_settings.axisLookX] - m_gamepadBaselineAxes[static_cast<size_t>(m_settings.axisLookX)];
        const float rawY = padState.axes[m_settings.axisLookY] - m_gamepadBaselineAxes[static_cast<size_t>(m_settings.axisLookY)];
        const double mag = std::hypot(rawX, rawY);
        if (mag > 0.08) {
            const double norm = clamp((mag - 0.08) / 0.92, 0.0, 1.0);
            const double scale = norm / mag;
            m_cameraLook.x = clamp(static_cast<double>(rawX) * scale, -1.0, 1.0);
            m_cameraLook.y = clamp(static_cast<double>(rawY) * scale, -1.0, 1.0);
        }
    }

    // 6. Interactive Button Rebinding Mode
    if (m_rebindingActive) {
        for (int b = 0; b < 15; ++b) {
            if (padState.buttons[b] == GLFW_PRESS && !m_prevPadButtons[static_cast<size_t>(b)]) {
                switch (m_rebindingAction) {
                    case LogicalVehicleAction::Handbrake:          m_settings.btnHandbrake = b; break;
                    case LogicalVehicleAction::ShiftUp:            m_settings.btnShiftUp = b; break;
                    case LogicalVehicleAction::ShiftDown:          m_settings.btnShiftDown = b; break;
                    case LogicalVehicleAction::ResetVehicle:       m_settings.btnReset = b; break;
                    case LogicalVehicleAction::ChangeCamera:       m_settings.btnChangeCamera = b; break;
                    case LogicalVehicleAction::PauseResume:        m_settings.btnPause = b; break;
                    case LogicalVehicleAction::ToggleTransmission: m_settings.btnToggleTransmission = b; break;
                    case LogicalVehicleAction::ToggleModel:        m_settings.btnToggleModel = b; break;
                    case LogicalVehicleAction::ToggleVisualDebug:  m_settings.btnToggleVisualDebug = b; break;
                    case LogicalVehicleAction::ToggleTelemetry:    m_settings.btnToggleTelemetry = b; break;
                    case LogicalVehicleAction::ToggleGizmos:       m_settings.btnToggleGizmos = b; break;
                    case LogicalVehicleAction::NextPreset:         m_settings.btnNextPreset = b; break;
                    case LogicalVehicleAction::PrevPreset:         m_settings.btnPrevPreset = b; break;
                    default: break;
                }
                m_rebindingActive = false;
                saveSettings();
                break;
            }
        }
    } else {
        // 7. Edge-triggered actions
        auto justPressed = [&](int btnIdx) -> bool {
            if (btnIdx < 0 || btnIdx >= 15) return false;
            return (padState.buttons[btnIdx] == GLFW_PRESS &&
                    !m_prevPadButtons[static_cast<size_t>(btnIdx)]);
        };

        if (justPressed(m_settings.btnShiftUp)) {
            if (m_callbacks.onShiftUp) m_callbacks.onShiftUp();
        }
        if (justPressed(m_settings.btnShiftDown)) {
            if (m_callbacks.onShiftDown) m_callbacks.onShiftDown();
        }
        if (justPressed(m_settings.btnChangeCamera)) {
            if (m_callbacks.onCycleCamera) m_callbacks.onCycleCamera();
        }
        if (justPressed(m_settings.btnPause)) {
            if (m_callbacks.onTogglePause) m_callbacks.onTogglePause();
        }
        if (justPressed(m_settings.btnToggleTransmission)) {
            if (m_callbacks.onToggleTransmissionMode) m_callbacks.onToggleTransmissionMode();
        }
        if (justPressed(m_settings.btnToggleModel)) {
            if (m_callbacks.onToggleModel) m_callbacks.onToggleModel();
        }
        if (justPressed(m_settings.btnToggleVisualDebug)) {
            if (m_callbacks.onToggleVisualDebug) m_callbacks.onToggleVisualDebug();
        }
        if (justPressed(m_settings.btnToggleTelemetry)) {
            if (m_callbacks.onToggleTelemetry) m_callbacks.onToggleTelemetry();
        }
        if (justPressed(m_settings.btnToggleGizmos)) {
            if (m_callbacks.onToggleGizmos3D) m_callbacks.onToggleGizmos3D();
        }
        if (justPressed(m_settings.btnNextPreset)) {
            if (m_callbacks.onStepPreset) m_callbacks.onStepPreset(+1);
        }
        if (justPressed(m_settings.btnPrevPreset)) {
            if (m_callbacks.onStepPreset) m_callbacks.onStepPreset(-1);
        }

        // 8. Vehicle Reset with configurable hold duration (Forza Horizon style)
        if (m_settings.btnReset >= 0 && m_settings.btnReset < 15 &&
            padState.buttons[m_settings.btnReset] == GLFW_PRESS) {
            m_resetHoldTimer += dt;
            if (m_resetHoldTimer >= static_cast<double>(m_settings.resetHoldDuration) && !m_resetTriggeredThisHold) {
                if (m_callbacks.onReset) m_callbacks.onReset();
                m_resetTriggeredThisHold = true;
            }
        } else {
            m_resetHoldTimer = 0.0;
            m_resetTriggeredThisHold = false;
        }

        // 9. Continuous Handbrake
        if (m_settings.btnHandbrake >= 0 && m_settings.btnHandbrake < 15 &&
            padState.buttons[m_settings.btnHandbrake] == GLFW_PRESS) {
            padHandbrake = 1.0;
        }
    }

    for (size_t b = 0; b < 15; ++b) {
        m_prevPadButtons[b] = padState.buttons[b];
    }

    // 10. Update vibration feedback
    m_vibration.update(
        static_cast<float>(dt),
        m_activeJoystickId,
        m_settings.vibrationEnabled,
        m_settings.vibrationIntensity
    );
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

    double kbSteer = 0.0;
    if (aHeld) kbSteer += 1.0;
    if (dHeld) kbSteer -= 1.0;

    // Sample Gamepad
    double padThrottle = 0.0;
    double padBrake = 0.0;
    double padSteer = 0.0;
    double padHandbrake = 0.0;
    pollGamepad(dt, padThrottle, padBrake, padSteer, padHandbrake);

    // Combine Keyboard and Gamepad inputs safely:
    // Throttle & Brake: highest analog command takes authority
    double finalThrottle = std::max(m_throttleState, padThrottle);
    double finalBrake = std::max(m_brakeState, padBrake);
    double finalHandbrake = std::max(m_handbrakeState, padHandbrake);

    // Steering: If keyboard is being pressed, blend or prioritize; otherwise full analog gamepad
    double finalSteer = 0.0;
    if (std::abs(kbSteer) > 0.01) {
        if (std::abs(padSteer) > 0.01) {
            // Both active: clamp algebraic combination safely within [-1.0, +1.0]
            finalSteer = clamp(kbSteer + padSteer * 0.5, -1.0, 1.0);
        } else {
            finalSteer = kbSteer;
        }
    } else {
        finalSteer = padSteer;
    }

    m_liveThrottle = clamp(finalThrottle, 0.0, 1.0);
    m_liveBrake = clamp(finalBrake, 0.0, 1.0);
    m_liveSteer = clamp(finalSteer, -1.0, 1.0);
    m_liveHandbrake = clamp(finalHandbrake, 0.0, 1.0);

    return VehicleControlInputs{
        m_liveThrottle,
        m_liveBrake,
        m_liveSteer,
        m_liveHandbrake,
        false
    };
}

} // namespace torquelab
