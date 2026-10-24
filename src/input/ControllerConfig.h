#pragma once
/**
 * Controller Configuration, Action Bindings & Persistence Specification.
 *
 * Implements a customizable logical action-mapping system inspired by Forza Horizon:
 * - Default Xbox-style mapping:
 *     Accelerate: RT
 *     Brake / Reverse: LT
 *     Steer: Left Analog Stick (horizontal)
 *     Handbrake: A
 *     Shift Up: B
 *     Shift Down: X
 *     Reset Vehicle: Y (configurable hold)
 *     Change Camera: Right Stick Click (R3)
 *     Look Around: Right Analog Stick
 *     Pause / Menu: Start
 *     Toggle Transmission Mode: LB
 *     Toggle Vehicle 3D Model: RB
 *     Toggle Visual Debug: Back / View
 *     Telemetry Overlay: D-Pad Up
 *     3D Vectors Gizmos: D-Pad Down
 *     Teleport Next/Prev Preset: D-Pad Right / Left
 * - PlayStation-style label adaptivity (Cross, Circle, Square, Triangle, L2/R2, Options, etc.)
 * - Configurable deadzones, sensitivity curves, steering inversion, vibration toggle.
 * - JSON persistence to settings/controller_config.json.
 */

#include <string>
#include <vector>
#include <GLFW/glfw3.h>

namespace torquelab {

enum class ControllerFlavor : int {
    Auto = 0,
    Xbox = 1,
    PlayStation = 2,
    Nintendo = 3,
    Generic = 4
};

enum class LogicalVehicleAction : int {
    Accelerate = 0,
    BrakeReverse = 1,
    Steer = 2,
    Handbrake = 3,
    ShiftUp = 4,
    ShiftDown = 5,
    ResetVehicle = 6,
    ChangeCamera = 7,
    LookAround = 8,
    PauseResume = 9,
    ToggleTransmission = 10,
    ToggleModel = 11,
    ToggleVisualDebug = 12,
    ToggleTelemetry = 13,
    ToggleGizmos = 14,
    NextPreset = 15,
    PrevPreset = 16,
    Count = 17
};

struct ControllerSettings {
    // Hardware Selection & Flavor
    int preferredJoystickId{-1}; // -1 = Auto-detect first active gamepad
    ControllerFlavor flavor{ControllerFlavor::Auto};

    // Steering Tuning
    float steerDeadzone{0.08f};      // Center deadzone [0.0 .. 0.35]
    float steerSensitivity{1.0f};   // Response exponent: 1.0 = linear, 1.4 = softer center
    bool invertSteering{false};      // Invert horizontal steering axis

    // Trigger Tuning
    float triggerDeadzone{0.05f};    // Trigger deadzone [0.0 .. 0.30]

    // Reset Action
    float resetHoldDuration{0.45f};  // Seconds to hold Reset button before triggering (0.0 = instant tap)

    // Vibration / Force Feedback
    bool vibrationEnabled{true};
    float vibrationIntensity{0.80f};       // [0.0 .. 1.0] master rumble scale
    float collisionSensitivity{1.0f};      // [0.0 .. 2.0] collision impact sensitivity
    float suspensionSensitivity{1.0f};     // [0.0 .. 2.0] suspension & road impact sensitivity

    // Configurable Button Bindings (GLFW gamepad button indices 0..14)
    int btnHandbrake{GLFW_GAMEPAD_BUTTON_A};
    int btnShiftUp{GLFW_GAMEPAD_BUTTON_B};
    int btnShiftDown{GLFW_GAMEPAD_BUTTON_X};
    int btnReset{GLFW_GAMEPAD_BUTTON_Y};
    int btnChangeCamera{GLFW_GAMEPAD_BUTTON_RIGHT_THUMB};
    int btnPause{GLFW_GAMEPAD_BUTTON_START};
    int btnToggleTransmission{GLFW_GAMEPAD_BUTTON_LEFT_BUMPER};
    int btnToggleModel{GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER};
    int btnToggleVisualDebug{GLFW_GAMEPAD_BUTTON_BACK};
    int btnToggleTelemetry{GLFW_GAMEPAD_BUTTON_DPAD_UP};
    int btnToggleGizmos{GLFW_GAMEPAD_BUTTON_DPAD_DOWN};
    int btnNextPreset{GLFW_GAMEPAD_BUTTON_DPAD_RIGHT};
    int btnPrevPreset{GLFW_GAMEPAD_BUTTON_DPAD_LEFT};

    // Configurable Axis Bindings (GLFW gamepad axis indices 0..5)
    int axisSteer{GLFW_GAMEPAD_AXIS_LEFT_X};
    int axisThrottle{GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER};
    int axisBrake{GLFW_GAMEPAD_AXIS_LEFT_TRIGGER};
    int axisLookX{GLFW_GAMEPAD_AXIS_RIGHT_X};
    int axisLookY{GLFW_GAMEPAD_AXIS_RIGHT_Y};

    void resetToForzaDefaults();
};

// UI Label Helpers
std::string getControllerFlavorName(ControllerFlavor flavor);
std::string getGamepadButtonName(int buttonIdx, ControllerFlavor flavor);
std::string getGamepadAxisName(int axisIdx, ControllerFlavor flavor);
std::string getLogicalActionName(LogicalVehicleAction action);
std::string getLogicalActionDescription(LogicalVehicleAction action);
ControllerFlavor detectControllerFlavorFromName(const std::string& name);

// JSON Persistence
bool loadControllerSettingsFromJson(ControllerSettings& outSettings, const std::string& path = "settings/controller_config.json");
bool saveControllerSettingsToJson(const ControllerSettings& settings, const std::string& path = "settings/controller_config.json");

} // namespace torquelab
