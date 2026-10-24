#include "input/ControllerConfig.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace torquelab {

namespace {

std::string toLower(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower;
}

std::string trim(const std::string& str) {
    const auto first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

std::string extractString(const std::string& line, const std::string& key) {
    const std::string search = "\"" + key + "\"";
    const auto pos = line.find(search);
    if (pos == std::string::npos) return "";
    const auto col = line.find(':', pos + search.size());
    if (col == std::string::npos) return "";
    const auto q1 = line.find('\"', col);
    if (q1 == std::string::npos) return "";
    const auto q2 = line.find('\"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return line.substr(q1 + 1, q2 - q1 - 1);
}

double extractDouble(const std::string& line, const std::string& key, double defaultVal = 0.0) {
    const std::string search = "\"" + key + "\"";
    const auto pos = line.find(search);
    if (pos == std::string::npos) return defaultVal;
    const auto col = line.find(':', pos + search.size());
    if (col == std::string::npos) return defaultVal;
    std::string valStr = line.substr(col + 1);
    const auto comma = valStr.find(',');
    if (comma != std::string::npos) valStr = valStr.substr(0, comma);
    const auto brace = valStr.find_first_of("}\r\n");
    if (brace != std::string::npos) valStr = valStr.substr(0, brace);
    valStr = trim(valStr);
    try {
        return std::stod(valStr);
    } catch (...) {
        return defaultVal;
    }
}

int extractInt(const std::string& line, const std::string& key, int defaultVal = 0) {
    return static_cast<int>(std::round(extractDouble(line, key, defaultVal)));
}

bool extractBool(const std::string& line, const std::string& key, bool defaultVal = false) {
    const std::string search = "\"" + key + "\"";
    const auto pos = line.find(search);
    if (pos == std::string::npos) return defaultVal;
    const auto col = line.find(':', pos + search.size());
    if (col == std::string::npos) return defaultVal;
    std::string valStr = line.substr(col + 1);
    const auto comma = valStr.find(',');
    if (comma != std::string::npos) valStr = valStr.substr(0, comma);
    const auto brace = valStr.find_first_of("}\r\n");
    if (brace != std::string::npos) valStr = valStr.substr(0, brace);
    valStr = toLower(trim(valStr));
    if (valStr == "true" || valStr == "1") return true;
    if (valStr == "false" || valStr == "0") return false;
    return defaultVal;
}

} // anonymous namespace

void ControllerSettings::resetToForzaDefaults() {
    preferredJoystickId = -1;
    flavor = ControllerFlavor::Auto;
    steerDeadzone = 0.08f;
    steerSensitivity = 1.0f;
    invertSteering = false;
    triggerDeadzone = 0.05f;
    resetHoldDuration = 0.45f;
    vibrationEnabled = true;
    vibrationIntensity = 0.80f;
    collisionSensitivity = 1.0f;
    suspensionSensitivity = 1.0f;

    btnHandbrake = GLFW_GAMEPAD_BUTTON_A;
    btnShiftUp = GLFW_GAMEPAD_BUTTON_B;
    btnShiftDown = GLFW_GAMEPAD_BUTTON_X;
    btnReset = GLFW_GAMEPAD_BUTTON_Y;
    btnChangeCamera = GLFW_GAMEPAD_BUTTON_RIGHT_THUMB;
    btnPause = GLFW_GAMEPAD_BUTTON_START;
    btnToggleTransmission = GLFW_GAMEPAD_BUTTON_LEFT_BUMPER;
    btnToggleModel = GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER;
    btnToggleVisualDebug = GLFW_GAMEPAD_BUTTON_BACK;
    btnToggleTelemetry = GLFW_GAMEPAD_BUTTON_DPAD_UP;
    btnToggleGizmos = GLFW_GAMEPAD_BUTTON_DPAD_DOWN;
    btnNextPreset = GLFW_GAMEPAD_BUTTON_DPAD_RIGHT;
    btnPrevPreset = GLFW_GAMEPAD_BUTTON_DPAD_LEFT;

    axisSteer = GLFW_GAMEPAD_AXIS_LEFT_X;
    axisThrottle = GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER;
    axisBrake = GLFW_GAMEPAD_AXIS_LEFT_TRIGGER;
    axisLookX = GLFW_GAMEPAD_AXIS_RIGHT_X;
    axisLookY = GLFW_GAMEPAD_AXIS_RIGHT_Y;
}

std::string getControllerFlavorName(ControllerFlavor flavor) {
    switch (flavor) {
        case ControllerFlavor::Auto:        return "Auto (Detect from Device)";
        case ControllerFlavor::Xbox:        return "Xbox (A, B, X, Y, RT/LT)";
        case ControllerFlavor::PlayStation: return "PlayStation (Cross, Circle, R2/L2)";
        case ControllerFlavor::Nintendo:    return "Nintendo (B, A, Y, X, ZR/ZL)";
        case ControllerFlavor::Generic:     return "Generic Gamepad";
    }
    return "Auto";
}

ControllerFlavor detectControllerFlavorFromName(const std::string& name) {
    const std::string lower = toLower(name);
    if (lower.find("playstation") != std::string::npos ||
        lower.find("dualsense") != std::string::npos ||
        lower.find("dualshock") != std::string::npos ||
        lower.find("ps4") != std::string::npos ||
        lower.find("ps5") != std::string::npos ||
        lower.find("sony") != std::string::npos) {
        return ControllerFlavor::PlayStation;
    }
    if (lower.find("nintendo") != std::string::npos ||
        lower.find("switch") != std::string::npos ||
        lower.find("pro controller") != std::string::npos ||
        lower.find("joy-con") != std::string::npos) {
        return ControllerFlavor::Nintendo;
    }
    if (lower.find("xbox") != std::string::npos ||
        lower.find("xinput") != std::string::npos ||
        lower.find("microsoft") != std::string::npos) {
        return ControllerFlavor::Xbox;
    }
    return ControllerFlavor::Xbox; // Default to Xbox standard
}

std::string getGamepadButtonName(int buttonIdx, ControllerFlavor flavor) {
    if (flavor == ControllerFlavor::PlayStation) {
        switch (buttonIdx) {
            case GLFW_GAMEPAD_BUTTON_A:            return "Cross";
            case GLFW_GAMEPAD_BUTTON_B:            return "Circle";
            case GLFW_GAMEPAD_BUTTON_X:            return "Square";
            case GLFW_GAMEPAD_BUTTON_Y:            return "Triangle";
            case GLFW_GAMEPAD_BUTTON_LEFT_BUMPER:  return "L1";
            case GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER: return "R1";
            case GLFW_GAMEPAD_BUTTON_BACK:         return "Share / Create";
            case GLFW_GAMEPAD_BUTTON_START:        return "Options";
            case GLFW_GAMEPAD_BUTTON_GUIDE:        return "PS";
            case GLFW_GAMEPAD_BUTTON_LEFT_THUMB:   return "L3 (Left Stick Click)";
            case GLFW_GAMEPAD_BUTTON_RIGHT_THUMB:  return "R3 (Right Stick Click)";
            case GLFW_GAMEPAD_BUTTON_DPAD_UP:      return "D-Pad Up";
            case GLFW_GAMEPAD_BUTTON_DPAD_RIGHT:   return "D-Pad Right";
            case GLFW_GAMEPAD_BUTTON_DPAD_DOWN:    return "D-Pad Down";
            case GLFW_GAMEPAD_BUTTON_DPAD_LEFT:    return "D-Pad Left";
            default:                               return "Button " + std::to_string(buttonIdx);
        }
    } else if (flavor == ControllerFlavor::Nintendo) {
        switch (buttonIdx) {
            case GLFW_GAMEPAD_BUTTON_A:            return "B";
            case GLFW_GAMEPAD_BUTTON_B:            return "A";
            case GLFW_GAMEPAD_BUTTON_X:            return "Y";
            case GLFW_GAMEPAD_BUTTON_Y:            return "X";
            case GLFW_GAMEPAD_BUTTON_LEFT_BUMPER:  return "L";
            case GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER: return "R";
            case GLFW_GAMEPAD_BUTTON_BACK:         return "- (Minus)";
            case GLFW_GAMEPAD_BUTTON_START:        return "+ (Plus)";
            case GLFW_GAMEPAD_BUTTON_GUIDE:        return "Home";
            case GLFW_GAMEPAD_BUTTON_LEFT_THUMB:   return "Left Stick Click";
            case GLFW_GAMEPAD_BUTTON_RIGHT_THUMB:  return "Right Stick Click";
            case GLFW_GAMEPAD_BUTTON_DPAD_UP:      return "D-Pad Up";
            case GLFW_GAMEPAD_BUTTON_DPAD_RIGHT:   return "D-Pad Right";
            case GLFW_GAMEPAD_BUTTON_DPAD_DOWN:    return "D-Pad Down";
            case GLFW_GAMEPAD_BUTTON_DPAD_LEFT:    return "D-Pad Left";
            default:                               return "Button " + std::to_string(buttonIdx);
        }
    } else { // Xbox / Generic
        switch (buttonIdx) {
            case GLFW_GAMEPAD_BUTTON_A:            return "A";
            case GLFW_GAMEPAD_BUTTON_B:            return "B";
            case GLFW_GAMEPAD_BUTTON_X:            return "X";
            case GLFW_GAMEPAD_BUTTON_Y:            return "Y";
            case GLFW_GAMEPAD_BUTTON_LEFT_BUMPER:  return "LB";
            case GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER: return "RB";
            case GLFW_GAMEPAD_BUTTON_BACK:         return "View / Back";
            case GLFW_GAMEPAD_BUTTON_START:        return "Menu / Start";
            case GLFW_GAMEPAD_BUTTON_GUIDE:        return "Xbox Guide";
            case GLFW_GAMEPAD_BUTTON_LEFT_THUMB:   return "LS Click (L3)";
            case GLFW_GAMEPAD_BUTTON_RIGHT_THUMB:  return "RS Click (R3)";
            case GLFW_GAMEPAD_BUTTON_DPAD_UP:      return "D-Pad Up";
            case GLFW_GAMEPAD_BUTTON_DPAD_RIGHT:   return "D-Pad Right";
            case GLFW_GAMEPAD_BUTTON_DPAD_DOWN:    return "D-Pad Down";
            case GLFW_GAMEPAD_BUTTON_DPAD_LEFT:    return "D-Pad Left";
            default:                               return "Button " + std::to_string(buttonIdx);
        }
    }
}

std::string getGamepadAxisName(int axisIdx, ControllerFlavor flavor) {
    if (flavor == ControllerFlavor::PlayStation) {
        switch (axisIdx) {
            case GLFW_GAMEPAD_AXIS_LEFT_X:        return "Left Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_LEFT_Y:        return "Left Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_RIGHT_X:       return "Right Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_RIGHT_Y:       return "Right Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_LEFT_TRIGGER:  return "L2 Trigger";
            case GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER: return "R2 Trigger";
            default:                              return "Axis " + std::to_string(axisIdx);
        }
    } else if (flavor == ControllerFlavor::Nintendo) {
        switch (axisIdx) {
            case GLFW_GAMEPAD_AXIS_LEFT_X:        return "Left Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_LEFT_Y:        return "Left Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_RIGHT_X:       return "Right Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_RIGHT_Y:       return "Right Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_LEFT_TRIGGER:  return "ZL Trigger";
            case GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER: return "ZR Trigger";
            default:                              return "Axis " + std::to_string(axisIdx);
        }
    } else {
        switch (axisIdx) {
            case GLFW_GAMEPAD_AXIS_LEFT_X:        return "Left Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_LEFT_Y:        return "Left Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_RIGHT_X:       return "Right Stick (Horizontal)";
            case GLFW_GAMEPAD_AXIS_RIGHT_Y:       return "Right Stick (Vertical)";
            case GLFW_GAMEPAD_AXIS_LEFT_TRIGGER:  return "LT (Left Trigger)";
            case GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER: return "RT (Right Trigger)";
            default:                              return "Axis " + std::to_string(axisIdx);
        }
    }
}

std::string getLogicalActionName(LogicalVehicleAction action) {
    switch (action) {
        case LogicalVehicleAction::Accelerate:         return "Accelerate";
        case LogicalVehicleAction::BrakeReverse:       return "Brake / Reverse";
        case LogicalVehicleAction::Steer:              return "Steering";
        case LogicalVehicleAction::Handbrake:          return "Handbrake";
        case LogicalVehicleAction::ShiftUp:            return "Shift Up";
        case LogicalVehicleAction::ShiftDown:          return "Shift Down";
        case LogicalVehicleAction::ResetVehicle:       return "Reset / Recover Vehicle";
        case LogicalVehicleAction::ChangeCamera:       return "Change Camera Mode";
        case LogicalVehicleAction::LookAround:         return "Look Around";
        case LogicalVehicleAction::PauseResume:        return "Pause / Resume";
        case LogicalVehicleAction::ToggleTransmission: return "Auto / Manual Transmission";
        case LogicalVehicleAction::ToggleModel:        return "Toggle Vehicle 3D Model";
        case LogicalVehicleAction::ToggleVisualDebug:  return "Model Alignment Debug";
        case LogicalVehicleAction::ToggleTelemetry:    return "Toggle Telemetry HUD";
        case LogicalVehicleAction::ToggleGizmos:       return "Toggle 3D Force Vectors";
        case LogicalVehicleAction::NextPreset:         return "Next Proving Ground Zone";
        case LogicalVehicleAction::PrevPreset:         return "Previous Proving Ground Zone";
        default:                                       return "Action";
    }
}

std::string getLogicalActionDescription(LogicalVehicleAction action) {
    switch (action) {
        case LogicalVehicleAction::Accelerate:         return "Analog throttle (RT / R2)";
        case LogicalVehicleAction::BrakeReverse:       return "Analog brake & auto-reverse (LT / L2)";
        case LogicalVehicleAction::Steer:              return "Continuous left analog stick steering";
        case LogicalVehicleAction::Handbrake:          return "Rear mechanical handbrake (A / Cross)";
        case LogicalVehicleAction::ShiftUp:            return "Manual upshift (B / Circle)";
        case LogicalVehicleAction::ShiftDown:          return "Manual downshift (X / Square)";
        case LogicalVehicleAction::ResetVehicle:       return "Hold Y / Triangle to reset position";
        case LogicalVehicleAction::ChangeCamera:       return "Click Right Stick (R3) to cycle camera";
        case LogicalVehicleAction::LookAround:         return "Right analog stick camera free-look";
        case LogicalVehicleAction::PauseResume:        return "Pause simulation & navigate menus";
        case LogicalVehicleAction::ToggleTransmission: return "Switch Automatic / Manual mode (LB / L1)";
        case LogicalVehicleAction::ToggleModel:        return "Hot-swap BMW M4 / Procedural coupe (RB / R1)";
        case LogicalVehicleAction::ToggleVisualDebug:  return "Toggle collision / visual alignment (Back)";
        case LogicalVehicleAction::ToggleTelemetry:    return "Toggle telemetry inspector (D-Pad Up)";
        case LogicalVehicleAction::ToggleGizmos:       return "Toggle 3D vector gizmos (D-Pad Down)";
        case LogicalVehicleAction::NextPreset:         return "Teleport forward to next test track zone";
        case LogicalVehicleAction::PrevPreset:         return "Teleport backward to previous test track zone";
        default:                                       return "";
    }
}

bool loadControllerSettingsFromJson(ControllerSettings& outSettings, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("\"preferredJoystickId\"") != std::string::npos) {
            outSettings.preferredJoystickId = extractInt(line, "preferredJoystickId", -1);
        } else if (line.find("\"flavor\"") != std::string::npos) {
            outSettings.flavor = static_cast<ControllerFlavor>(extractInt(line, "flavor", 0));
        } else if (line.find("\"steerDeadzone\"") != std::string::npos) {
            outSettings.steerDeadzone = static_cast<float>(extractDouble(line, "steerDeadzone", 0.08));
        } else if (line.find("\"steerSensitivity\"") != std::string::npos) {
            outSettings.steerSensitivity = static_cast<float>(extractDouble(line, "steerSensitivity", 1.0));
        } else if (line.find("\"invertSteering\"") != std::string::npos) {
            outSettings.invertSteering = extractBool(line, "invertSteering", false);
        } else if (line.find("\"triggerDeadzone\"") != std::string::npos) {
            outSettings.triggerDeadzone = static_cast<float>(extractDouble(line, "triggerDeadzone", 0.05));
        } else if (line.find("\"resetHoldDuration\"") != std::string::npos) {
            outSettings.resetHoldDuration = static_cast<float>(extractDouble(line, "resetHoldDuration", 0.45));
        } else if (line.find("\"vibrationEnabled\"") != std::string::npos) {
            outSettings.vibrationEnabled = extractBool(line, "vibrationEnabled", true);
        } else if (line.find("\"vibrationIntensity\"") != std::string::npos) {
            outSettings.vibrationIntensity = static_cast<float>(extractDouble(line, "vibrationIntensity", 0.80));
        } else if (line.find("\"collisionSensitivity\"") != std::string::npos) {
            outSettings.collisionSensitivity = static_cast<float>(extractDouble(line, "collisionSensitivity", 1.00));
        } else if (line.find("\"suspensionSensitivity\"") != std::string::npos) {
            outSettings.suspensionSensitivity = static_cast<float>(extractDouble(line, "suspensionSensitivity", 1.00));
        } else if (line.find("\"btnHandbrake\"") != std::string::npos) {
            outSettings.btnHandbrake = extractInt(line, "btnHandbrake", GLFW_GAMEPAD_BUTTON_A);
        } else if (line.find("\"btnShiftUp\"") != std::string::npos) {
            outSettings.btnShiftUp = extractInt(line, "btnShiftUp", GLFW_GAMEPAD_BUTTON_B);
        } else if (line.find("\"btnShiftDown\"") != std::string::npos) {
            outSettings.btnShiftDown = extractInt(line, "btnShiftDown", GLFW_GAMEPAD_BUTTON_X);
        } else if (line.find("\"btnReset\"") != std::string::npos) {
            outSettings.btnReset = extractInt(line, "btnReset", GLFW_GAMEPAD_BUTTON_Y);
        } else if (line.find("\"btnChangeCamera\"") != std::string::npos) {
            outSettings.btnChangeCamera = extractInt(line, "btnChangeCamera", GLFW_GAMEPAD_BUTTON_RIGHT_THUMB);
        } else if (line.find("\"btnPause\"") != std::string::npos) {
            outSettings.btnPause = extractInt(line, "btnPause", GLFW_GAMEPAD_BUTTON_START);
        } else if (line.find("\"btnToggleTransmission\"") != std::string::npos) {
            outSettings.btnToggleTransmission = extractInt(line, "btnToggleTransmission", GLFW_GAMEPAD_BUTTON_LEFT_BUMPER);
        } else if (line.find("\"btnToggleModel\"") != std::string::npos) {
            outSettings.btnToggleModel = extractInt(line, "btnToggleModel", GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER);
        } else if (line.find("\"btnToggleVisualDebug\"") != std::string::npos) {
            outSettings.btnToggleVisualDebug = extractInt(line, "btnToggleVisualDebug", GLFW_GAMEPAD_BUTTON_BACK);
        } else if (line.find("\"btnToggleTelemetry\"") != std::string::npos) {
            outSettings.btnToggleTelemetry = extractInt(line, "btnToggleTelemetry", GLFW_GAMEPAD_BUTTON_DPAD_UP);
        } else if (line.find("\"btnToggleGizmos\"") != std::string::npos) {
            outSettings.btnToggleGizmos = extractInt(line, "btnToggleGizmos", GLFW_GAMEPAD_BUTTON_DPAD_DOWN);
        } else if (line.find("\"btnNextPreset\"") != std::string::npos) {
            outSettings.btnNextPreset = extractInt(line, "btnNextPreset", GLFW_GAMEPAD_BUTTON_DPAD_RIGHT);
        } else if (line.find("\"btnPrevPreset\"") != std::string::npos) {
            outSettings.btnPrevPreset = extractInt(line, "btnPrevPreset", GLFW_GAMEPAD_BUTTON_DPAD_LEFT);
        } else if (line.find("\"axisSteer\"") != std::string::npos) {
            outSettings.axisSteer = extractInt(line, "axisSteer", GLFW_GAMEPAD_AXIS_LEFT_X);
        } else if (line.find("\"axisThrottle\"") != std::string::npos) {
            outSettings.axisThrottle = extractInt(line, "axisThrottle", GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER);
        } else if (line.find("\"axisBrake\"") != std::string::npos) {
            outSettings.axisBrake = extractInt(line, "axisBrake", GLFW_GAMEPAD_AXIS_LEFT_TRIGGER);
        } else if (line.find("\"axisLookX\"") != std::string::npos) {
            outSettings.axisLookX = extractInt(line, "axisLookX", GLFW_GAMEPAD_AXIS_RIGHT_X);
        } else if (line.find("\"axisLookY\"") != std::string::npos) {
            outSettings.axisLookY = extractInt(line, "axisLookY", GLFW_GAMEPAD_AXIS_RIGHT_Y);
        }
    }
    return true;
}

bool saveControllerSettingsToJson(const ControllerSettings& s, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) return false;

    out << "{\n";
    out << "  \"preferredJoystickId\": " << s.preferredJoystickId << ",\n";
    out << "  \"flavor\": " << static_cast<int>(s.flavor) << ",\n";
    out << "  \"steerDeadzone\": " << std::fixed << std::setprecision(3) << s.steerDeadzone << ",\n";
    out << "  \"steerSensitivity\": " << std::fixed << std::setprecision(3) << s.steerSensitivity << ",\n";
    out << "  \"invertSteering\": " << (s.invertSteering ? "true" : "false") << ",\n";
    out << "  \"triggerDeadzone\": " << std::fixed << std::setprecision(3) << s.triggerDeadzone << ",\n";
    out << "  \"resetHoldDuration\": " << std::fixed << std::setprecision(3) << s.resetHoldDuration << ",\n";
    out << "  \"vibrationEnabled\": " << (s.vibrationEnabled ? "true" : "false") << ",\n";
    out << "  \"vibrationIntensity\": " << std::fixed << std::setprecision(3) << s.vibrationIntensity << ",\n";
    out << "  \"collisionSensitivity\": " << std::fixed << std::setprecision(3) << s.collisionSensitivity << ",\n";
    out << "  \"suspensionSensitivity\": " << std::fixed << std::setprecision(3) << s.suspensionSensitivity << ",\n";

    out << "  \"btnHandbrake\": " << s.btnHandbrake << ",\n";
    out << "  \"btnShiftUp\": " << s.btnShiftUp << ",\n";
    out << "  \"btnShiftDown\": " << s.btnShiftDown << ",\n";
    out << "  \"btnReset\": " << s.btnReset << ",\n";
    out << "  \"btnChangeCamera\": " << s.btnChangeCamera << ",\n";
    out << "  \"btnPause\": " << s.btnPause << ",\n";
    out << "  \"btnToggleTransmission\": " << s.btnToggleTransmission << ",\n";
    out << "  \"btnToggleModel\": " << s.btnToggleModel << ",\n";
    out << "  \"btnToggleVisualDebug\": " << s.btnToggleVisualDebug << ",\n";
    out << "  \"btnToggleTelemetry\": " << s.btnToggleTelemetry << ",\n";
    out << "  \"btnToggleGizmos\": " << s.btnToggleGizmos << ",\n";
    out << "  \"btnNextPreset\": " << s.btnNextPreset << ",\n";
    out << "  \"btnPrevPreset\": " << s.btnPrevPreset << ",\n";

    out << "  \"axisSteer\": " << s.axisSteer << ",\n";
    out << "  \"axisThrottle\": " << s.axisThrottle << ",\n";
    out << "  \"axisBrake\": " << s.axisBrake << ",\n";
    out << "  \"axisLookX\": " << s.axisLookX << ",\n";
    out << "  \"axisLookY\": " << s.axisLookY << "\n";
    out << "}\n";

    return true;
}

} // namespace torquelab
