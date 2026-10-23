#include "ui/ImGuiDashboard.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

namespace {

ImU32 hexToImU32(unsigned int hex, uint8_t alpha = 255) {
    const uint8_t r = static_cast<uint8_t>((hex >> 16) & 0xFF);
    const uint8_t g = static_cast<uint8_t>((hex >> 8) & 0xFF);
    const uint8_t b = static_cast<uint8_t>(hex & 0xFF);
    return IM_COL32(r, g, b, alpha);
}

ImVec4 hexToImVec4(unsigned int hex, float alpha = 1.0f) {
    const float r = static_cast<float>((hex >> 16) & 0xFF) / 255.0f;
    const float g = static_cast<float>((hex >> 8) & 0xFF) / 255.0f;
    const float b = static_cast<float>(hex & 0xFF) / 255.0f;
    return ImVec4(r, g, b, alpha);
}

} // namespace

ImGuiDashboard::ImGuiDashboard(DashboardCallbacks callbacks)
    : m_callbacks(std::move(callbacks))
{}

void ImGuiDashboard::applyTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 5.0f;
    style.GrabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(10.0f, 8.0f);
    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(7.0f, 5.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]       = hexToImVec4(0x0f172a, 0.88f);
    colors[ImGuiCol_Border]         = hexToImVec4(0x334155, 0.70f);
    colors[ImGuiCol_FrameBg]        = hexToImVec4(0x1e293b, 0.90f);
    colors[ImGuiCol_FrameBgHovered] = hexToImVec4(0x334155, 0.95f);
    colors[ImGuiCol_FrameBgActive]  = hexToImVec4(0x475569, 1.00f);
    colors[ImGuiCol_TitleBg]        = hexToImVec4(0x0b1120, 0.95f);
    colors[ImGuiCol_TitleBgActive]  = hexToImVec4(0x1e293b, 0.98f);
    colors[ImGuiCol_Button]         = hexToImVec4(0x1e293b, 0.95f);
    colors[ImGuiCol_ButtonHovered]  = hexToImVec4(0x2563eb, 0.95f);
    colors[ImGuiCol_ButtonActive]   = hexToImVec4(0x1d4ed8, 1.00f);
    colors[ImGuiCol_SliderGrab]     = hexToImVec4(0x38bdf8, 0.95f);
    colors[ImGuiCol_SliderGrabActive]= hexToImVec4(0x0ea5e9, 1.00f);
    colors[ImGuiCol_Header]         = hexToImVec4(0x1e293b, 0.90f);
    colors[ImGuiCol_HeaderHovered]  = hexToImVec4(0x334155, 0.95f);
    colors[ImGuiCol_HeaderActive]   = hexToImVec4(0x2563eb, 0.95f);
    colors[ImGuiCol_Text]           = hexToImVec4(0xf8fafc, 1.00f);
    colors[ImGuiCol_TextDisabled]   = hexToImVec4(0x94a3b8, 1.00f);
    colors[ImGuiCol_PlotHistogram]  = hexToImVec4(0x38bdf8, 1.00f);
}

void ImGuiDashboard::showValidationReport(const std::string& message, bool isSuccess) {
    m_validationMessage = message;
    m_validationSuccess = isSuccess;
    m_validationToastTimer = 6.5;
}

void ImGuiDashboard::render(
    VehicleDynamics& vehicle,
    const VehicleTelemetrySnapshot& snap,
    const VehicleControlInputs& inputs,
    bool paused,
    const std::string& cameraLabel,
    bool gizmosEnabled,
    GraphicsQuality currentQuality,
    double fps,
    double physicsStepTimeMs,
    int windowWidth,
    int windowHeight
) {
    if (!paused) {
        m_history.push_back(TelemetrySample{
            static_cast<float>(snap.speedKmh),
            static_cast<float>(snap.longitudinalAccelMps2),
            static_cast<float>(snap.lateralAccelMps2),
            static_cast<float>(radToDeg(snap.pitchRad)),
            static_cast<float>(radToDeg(snap.rollRad)),
            {
                static_cast<float>(snap.wheels[0].compression * 1000.0),
                static_cast<float>(snap.wheels[1].compression * 1000.0),
                static_cast<float>(snap.wheels[2].compression * 1000.0),
                static_cast<float>(snap.wheels[3].compression * 1000.0)
            }
        });
        while (m_history.size() > MAX_SAMPLES) {
            m_history.pop_front();
        }
    }

    if (m_validationToastTimer > 0.0) {
        m_validationToastTimer = std::max(0.0, m_validationToastTimer - (1.0 / std::max(10.0, fps)));
    }

    renderTopBar(paused, cameraLabel, gizmosEnabled, currentQuality, windowWidth);
    renderLeftPanels(vehicle, windowHeight);
    if (telemetryEnabled) {
        renderRightTelemetryPanel(snap, fps, physicsStepTimeMs, windowWidth, windowHeight);
    }
    renderBottomInstrumentCluster(snap, inputs, vehicle.config.maxRpm, windowWidth, windowHeight);

    // Paused Banner or Validation Toast
    if (paused) {
        ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, 64.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, hexToImVec4(0xd97706, 0.94f));
        if (ImGui::Begin("##PauseBanner", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing)) {
            ImGui::TextUnformatted("SIMULATION PAUSED — Press P to Resume");
        }
        ImGui::End();
        ImGui::PopStyleColor();
    } else if (m_validationToastTimer > 0.0 && !m_validationMessage.empty()) {
        ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, 64.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, hexToImVec4(m_validationSuccess ? 0x065f46 : 0x991b1b, 0.95f));
        if (ImGui::Begin("##ValidationToast", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing)) {
            ImGui::TextUnformatted(m_validationMessage.c_str());
        }
        ImGui::End();
        ImGui::PopStyleColor();
    }
}

void ImGuiDashboard::renderTopBar(
    bool paused,
    const std::string& cameraLabel,
    bool gizmosEnabled,
    GraphicsQuality currentQuality,
    int windowWidth
) {
    ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(windowWidth - 16), 46.0f), ImGuiCond_Always);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##TopBar", nullptr, flags)) {
        ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x2563eb, 0.95f));
        ImGui::Button("ARGON 4");
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::Text("TORQUE LAB NATIVE");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        // Proving Ground Zone Presets (1..7)
        const auto& presets = getProvingGroundPresets();
        for (size_t i = 0; i < presets.size(); ++i) {
            if (ImGui::SmallButton(presets[i].label.c_str())) {
                if (m_callbacks.onSelectPreset) m_callbacks.onSelectPreset(presets[i]);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", presets[i].description.c_str());
            }
            ImGui::SameLine();
        }

        ImGui::TextDisabled("|");
        ImGui::SameLine();

        if (ImGui::SmallButton(paused ? "Resume (P)" : "Pause (P)")) {
            if (m_callbacks.onTogglePause) m_callbacks.onTogglePause();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset (R)")) {
            if (m_callbacks.onReset) m_callbacks.onReset();
        }
        ImGui::SameLine();
        const std::string camBtn = cameraLabel.substr(0, 14) + " (C)";
        if (ImGui::SmallButton(camBtn.c_str())) {
            if (m_callbacks.onCycleCamera) m_callbacks.onCycleCamera();
        }
        ImGui::SameLine();

        if (gizmosEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x0284c7, 0.95f));
        if (ImGui::SmallButton("3D Vectors (G)")) {
            if (m_callbacks.onToggleGizmos3D) m_callbacks.onToggleGizmos3D();
        }
        if (gizmosEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        if (telemetryEnabled) ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x0284c7, 0.95f));
        if (ImGui::SmallButton("Telemetry (T)")) {
            if (m_callbacks.onToggleTelemetry) m_callbacks.onToggleTelemetry();
        }
        if (telemetryEnabled) ImGui::PopStyleColor();
        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button, hexToImVec4(0x059669, 0.95f));
        if (ImGui::SmallButton("Verify Physics")) {
            if (m_callbacks.onRunValidationCheck) m_callbacks.onRunValidationCheck();
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();

        ImGui::SetNextItemWidth(105.0f);
        int qIdx = static_cast<int>(currentQuality);
        const char* qItems[] = {"Quality: High", "Quality: Med", "Quality: Low"};
        if (ImGui::Combo("##Quality", &qIdx, qItems, 3)) {
            if (m_callbacks.onChangeQuality) {
                m_callbacks.onChangeQuality(static_cast<GraphicsQuality>(qIdx));
            }
        }
    }
    ImGui::End();
}

void ImGuiDashboard::renderLeftPanels(VehicleDynamics& vehicle, int windowHeight) {
    const float maxPanelHeight = std::max(320.0f, static_cast<float>(windowHeight) - 210.0f);
    ImGui::SetNextWindowPos(ImVec2(8.0f, 60.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(315.0f, maxPanelHeight), ImGuiCond_Always);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("LIVE VEHICLE TUNING & CONTROLS", nullptr, flags)) {
        if (ImGui::CollapsingHeader("DRIVING CONTROLS", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("W/Up:");    ImGui::SameLine(90); ImGui::TextUnformatted("Throttle");
            ImGui::TextDisabled("S/Down:");  ImGui::SameLine(90); ImGui::TextUnformatted("Brake / Reverse");
            ImGui::TextDisabled("A / D:");   ImGui::SameLine(90); ImGui::TextUnformatted("Steer Left / Right");
            ImGui::TextDisabled("Space:");   ImGui::SameLine(90); ImGui::TextUnformatted("Handbrake (Rear)");
            ImGui::TextDisabled("M + E/Q:"); ImGui::SameLine(90); ImGui::TextUnformatted("Auto/Manual Shift");
            ImGui::TextDisabled("R / C:");   ImGui::SameLine(90); ImGui::TextUnformatted("Reset / Cycle Cam");
            ImGui::TextDisabled("T / G:");   ImGui::SameLine(90); ImGui::TextUnformatted("Telemetry / 3D Vectors");
            ImGui::TextDisabled("1 - 7:");   ImGui::SameLine(90); ImGui::TextUnformatted("Teleport Test Zones");
        }

        ImGui::Separator();
        ImGui::TextColored(hexToImVec4(0x38bdf8), "LIVE VEHICLE TUNING LAB");
        ImGui::SameLine(215.0f);
        if (ImGui::SmallButton("Defaults")) {
            vehicle.config = createDefaultVehicleConfig();
            vehicle.body.syncMassProperties(vehicle.config);
        }

        VehicleConfig& c = vehicle.config;

        // Drivetrain Layout Selector
        int layoutIdx = (c.drivetrainLayout == DrivetrainLayout::RWD) ? 0
                      : (c.drivetrainLayout == DrivetrainLayout::FWD) ? 1 : 2;
        const char* layouts[] = {"RWD (Rear-Wheel Drive)", "FWD (Front-Wheel Drive)", "AWD (40/60 All-Wheel)"};
        if (ImGui::Combo("Drivetrain", &layoutIdx, layouts, 3)) {
            c.drivetrainLayout = (layoutIdx == 0) ? DrivetrainLayout::RWD
                               : (layoutIdx == 1) ? DrivetrainLayout::FWD
                                                  : DrivetrainLayout::AWD;
        }

        // 1. Vehicle Mass
        float mass = static_cast<float>(c.mass);
        if (ImGui::SliderFloat("Mass (kg)", &mass, 800.0f, 2600.0f, "%.0f kg")) {
            c.mass = static_cast<double>(mass);
            vehicle.body.syncMassProperties(c);
        }

        // 2. CoM Height Offset
        float comH = static_cast<float>(c.comHeightOffset);
        if (ImGui::SliderFloat("CoM Height", &comH, -0.20f, 0.45f, "%+.2f m")) {
            c.comHeightOffset = static_cast<double>(comH);
            vehicle.body.syncMassProperties(c);
        }

        // 3. Front Mass Bias
        float frontBiasPct = static_cast<float>(c.frontWeightDistribution * 100.0);
        if (ImGui::SliderFloat("Front Bias", &frontBiasPct, 36.0f, 66.0f, "%.0f%% F")) {
            c.frontWeightDistribution = static_cast<double>(frontBiasPct) / 100.0;
            vehicle.body.syncMassProperties(c);
        }

        // 4. Spring Stiffness
        float springK = static_cast<float>(c.frontSpringStiffness / 1000.0);
        if (ImGui::SliderFloat("Spring Rate", &springK, 14.0f, 85.0f, "%.1f kN/m")) {
            c.frontSpringStiffness = static_cast<double>(springK) * 1000.0;
            c.rearSpringStiffness = std::round(c.frontSpringStiffness * 0.93);
        }

        // 5. Suspension Damping
        float dampC = static_cast<float>(c.frontDampingCompression);
        if (ImGui::SliderFloat("Damping", &dampC, 800.0f, 11000.0f, "%.0f N*s/m")) {
            c.frontDampingCompression = static_cast<double>(dampC);
            c.frontDampingRebound = std::round(c.frontDampingCompression * 1.33);
            c.rearDampingCompression = std::round(c.frontDampingCompression * 0.94);
            c.rearDampingRebound = std::round(c.frontDampingCompression * 1.25);
        }

        // 6. Anti-Roll Stiffness
        float arbK = static_cast<float>(c.frontAntiRollStiffness / 1000.0);
        if (ImGui::SliderFloat("Anti-Roll", &arbK, 0.0f, 45.0f, "%.1f kN/m")) {
            c.frontAntiRollStiffness = static_cast<double>(arbK) * 1000.0;
            c.rearAntiRollStiffness = std::round(c.frontAntiRollStiffness * 0.75);
        }

        // 7. Tire Friction
        float tireMu = static_cast<float>(c.tireFrictionCoeff);
        if (ImGui::SliderFloat("Tire Grip u0", &tireMu, 0.25f, 1.85f, "%.2f")) {
            c.tireFrictionCoeff = static_cast<double>(tireMu);
        }

        // 8. Tire Cornering Stiffness
        float tireLat = static_cast<float>(c.tireLateralStiffness);
        if (ImGui::SliderFloat("Corner Stiff", &tireLat, 4.0f, 22.0f, "%.1f")) {
            c.tireLateralStiffness = static_cast<double>(tireLat);
        }

        // 9. Max Steering Angle
        float steerDeg = static_cast<float>(radToDeg(c.maxSteeringAngle));
        if (ImGui::SliderFloat("Max Steer", &steerDeg, 15.0f, 45.0f, "%.0f deg")) {
            c.maxSteeringAngle = degToRad(static_cast<double>(steerDeg));
        }

        // 10. Service Brake Torque
        float brakeTorque = static_cast<float>(c.maxBrakeTorque);
        if (ImGui::SliderFloat("Brake Torque", &brakeTorque, 800.0f, 8000.0f, "%.0f N*m")) {
            c.maxBrakeTorque = static_cast<double>(brakeTorque);
        }

        // 11. Engine Torque Multiplier
        float engMult = static_cast<float>(c.engineTorqueMultiplier);
        if (ImGui::SliderFloat("Engine Mult", &engMult, 0.40f, 2.50f, "%.2fx")) {
            c.engineTorqueMultiplier = static_cast<double>(engMult);
        }

        // 12. Aero Drag (Cd)
        float dragCd = static_cast<float>(c.dragCoefficient);
        if (ImGui::SliderFloat("Aero Drag Cd", &dragCd, 0.05f, 0.95f, "%.2f")) {
            c.dragCoefficient = static_cast<double>(dragCd);
        }
    }
    ImGui::End();
}

void ImGuiDashboard::renderRightTelemetryPanel(
    const VehicleTelemetrySnapshot& snap,
    double fps,
    double physicsStepTimeMs,
    int windowWidth,
    int windowHeight
) {
    const float panelW = 435.0f;
    const float maxPanelH = std::max(340.0f, static_cast<float>(windowHeight) - 210.0f);
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowWidth) - panelW - 8.0f, 60.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelW, maxPanelH), ImGuiCond_Always);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("PHYSICS TELEMETRY & OSCILLOSCOPE (120 Hz)", nullptr, flags)) {
        // 1. Solver, Timing, Position & Global Force Balance Summary
        const double frameDtMs = 1000.0 / std::max(1.0, fps);
        double totalTireFx = 0.0;
        double totalRollRes = 0.0;
        double totalDriveTrq = 0.0;
        double totalBrakeTrq = 0.0;
        for (const auto& w : snap.wheels) {
            totalTireFx += w.longitudinalForce;
            totalDriveTrq += w.driveTorque;
            totalBrakeTrq += w.brakeTorque;
            if (w.inContact && w.normalLoad > 0.0) {
                const double crr = 0.014 * w.surface.rollingResistanceMultiplier;
                totalRollRes += -crr * w.normalLoad * clamp(w.patchSpeedLongitudinal / 0.4, -1.0, 1.0);
            }
        }
        const double netLongFx = totalTireFx - snap.aero.dragMagnitude + totalRollRes;

        ImGui::Text("FPS: %.0f (%.1f ms) | Phys dt: 8.33 ms (%.2f ms) | Contacts: %d",
                    fps, frameDtMs, physicsStepTimeMs, snap.collisionContactCount);
        ImGui::Text("Pos: (%.1f, %.2f, %.1f) m | v: %.2f m/s (%.1f km/h) | a_x: %+.2f m/s2",
                    snap.chassisPosition.x, snap.chassisPosition.y, snap.chassisPosition.z,
                    snap.forwardSpeedMps, snap.speedKmh, snap.longitudinalAccelMps2);
        ImGui::Text("Eng Trq: %+.1f Nm | Axle Drive: %+.0f Nm | Brake Trq: %.0f Nm",
                    snap.drivetrain.engineTorqueNm, totalDriveTrq, totalBrakeTrq);
        ImGui::Text("Drive Fx: %+.0f N | Drag: -%.0f N | RollRes: %+.0f N | Net Fx: %+.0f N",
                    totalTireFx, snap.aero.dragMagnitude, totalRollRes, netLongFx);
        ImGui::Text("Downforce: %.0f N | Numerical Sanity: ", snap.aero.downforceMagnitude);
        ImGui::SameLine();
        if (snap.numericalSanityOk) {
            ImGui::TextColored(hexToImVec4(0x22c55e), "FINITE OK");
        } else {
            ImGui::TextColored(hexToImVec4(0xef4444), "WARN");
        }

        ImGui::Separator();

        // 2. 4-Wheel Physics Table
        if (ImGui::BeginTable("WheelTable", 6, ImGuiTableFlags_BordersInner | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Wheel", ImGuiTableColumnFlags_WidthFixed, 42.0f);
            ImGui::TableSetupColumn("Comp (mm)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("Fz/Grip(N)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("Slip k / a", ImGuiTableColumnFlags_WidthFixed, 82.0f);
            ImGui::TableSetupColumn("Fx / Fy (N)", ImGuiTableColumnFlags_WidthFixed, 78.0f);
            ImGui::TableSetupColumn("w(r/s)", ImGuiTableColumnFlags_WidthFixed, 45.0f);
            ImGui::TableHeadersRow();

            for (int i = 0; i < 4; ++i) {
                const WheelState& w = snap.wheels[static_cast<size_t>(i)];
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                const ImVec4 statusCol = w.inContact
                    ? (w.isSkidding ? hexToImVec4(0xef4444) : hexToImVec4(0x22c55e))
                    : hexToImVec4(0x64748b);
                ImGui::TextColored(statusCol, "%s", w.id.c_str());

                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.0f (%d%%)", w.compression * 1000.0, static_cast<int>(std::round(w.compressionRatio * 100.0)));

                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.0f/%.0f", w.normalLoad, w.maxAvailableGripForce);

                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+.2f/%.1f", w.slipRatio, radToDeg(w.slipAngle));

                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.0f/%.0f", w.longitudinalForce, w.lateralForce);

                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.1f", w.angularVelocity);
            }
            ImGui::EndTable();
        }

        ImGui::Separator();
        ImGui::TextDisabled("TIME-SERIES GRAPHS (Speed, Accel, Pitch/Roll, Suspension)");

        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float canvasW = std::max(260.0f, avail.x);
        const float canvasH = std::max(180.0f, avail.y - 4.0f);
        drawOscilloscopeCanvas(canvasW, canvasH);
    }
    ImGui::End();
}

void ImGuiDashboard::drawOscilloscopeCanvas(float width, float height) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + width, p0.y + height);

    drawList->AddRectFilled(p0, p1, hexToImU32(0x0b1120));
    drawList->AddRect(p0, p1, hexToImU32(0x334155));

    const float bandH = height / 4.0f;
    const size_t n = m_history.size();

    auto drawBand = [&](int bandIdx, const char* title, float minVal, float maxVal,
                        std::initializer_list<std::pair<ImU32, std::function<float(const TelemetrySample&)>>> series) {
        const float y0 = p0.y + bandIdx * bandH;
        const float y1 = y0 + bandH;

        drawList->AddRect(ImVec2(p0.x, y0), ImVec2(p1.x, y1), hexToImU32(0x1e293b));

        if (minVal < 0.0f && maxVal > 0.0f) {
            const float zeroY = y1 - ((0.0f - minVal) / (maxVal - minVal)) * bandH;
            drawList->AddLine(ImVec2(p0.x, zeroY), ImVec2(p1.x, zeroY), hexToImU32(0x334155));
        }

        if (n > 1) {
            for (const auto& s : series) {
                for (size_t i = 0; i + 1 < n; ++i) {
                    const float xA = p0.x + (static_cast<float>(i) / static_cast<float>(MAX_SAMPLES - 1)) * width;
                    const float xB = p0.x + (static_cast<float>(i + 1) / static_cast<float>(MAX_SAMPLES - 1)) * width;

                    const float vA = std::clamp(s.second(m_history[i]), minVal, maxVal);
                    const float vB = std::clamp(s.second(m_history[i + 1]), minVal, maxVal);

                    const float normA = (vA - minVal) / (maxVal - minVal);
                    const float normB = (vB - minVal) / (maxVal - minVal);

                    const float yA = y1 - 3.0f - normA * (bandH - 6.0f);
                    const float yB = y1 - 3.0f - normB * (bandH - 6.0f);

                    drawList->AddLine(ImVec2(xA, yA), ImVec2(xB, yB), s.first, 1.5f);
                }
            }
        }

        drawList->AddText(ImVec2(p0.x + 6.0f, y0 + 3.0f), hexToImU32(0x94a3b8), title);
    };

    drawBand(0, "Speed (km/h) [0..180]", 0.0f, 180.0f, {
        {hexToImU32(0x38bdf8), [](const TelemetrySample& s) { return s.speedKmh; }}
    });

    drawBand(1, "Accel Long (grn) / Lat (org) [-15..+15 m/s2]", -15.0f, 15.0f, {
        {hexToImU32(0x22c55e), [](const TelemetrySample& s) { return s.longAccel; }},
        {hexToImU32(0xf97316), [](const TelemetrySample& s) { return s.latAccel; }}
    });

    drawBand(2, "Pitch (cyn) / Roll (pnk) [-8..+8 deg]", -8.0f, 8.0f, {
        {hexToImU32(0x06b6d4), [](const TelemetrySample& s) { return s.pitchDeg; }},
        {hexToImU32(0xec4899), [](const TelemetrySample& s) { return s.rollDeg; }}
    });

    drawBand(3, "Suspension Travel FL/FR/RL/RR [0..230 mm]", 0.0f, 230.0f, {
        {hexToImU32(0x38bdf8), [](const TelemetrySample& s) { return s.compMM[0]; }},
        {hexToImU32(0xa855f7), [](const TelemetrySample& s) { return s.compMM[1]; }},
        {hexToImU32(0x22c55e), [](const TelemetrySample& s) { return s.compMM[2]; }},
        {hexToImU32(0xeab308), [](const TelemetrySample& s) { return s.compMM[3]; }}
    });

    ImGui::Dummy(ImVec2(width, height));
}

void ImGuiDashboard::renderBottomInstrumentCluster(
    const VehicleTelemetrySnapshot& snap,
    const VehicleControlInputs& inputs,
    double maxRpm,
    int windowWidth,
    int windowHeight
) {
    const float clusterW = std::min(880.0f, static_cast<float>(windowWidth - 16));
    const float clusterH = 136.0f;
    ImGui::SetNextWindowPos(
        ImVec2(static_cast<float>(windowWidth) * 0.5f, static_cast<float>(windowHeight) - 8.0f),
        ImGuiCond_Always,
        ImVec2(0.5f, 1.0f)
    );
    ImGui::SetNextWindowSize(ImVec2(clusterW, clusterH), ImGuiCond_Always);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##InstrumentCluster", nullptr, flags)) {
        if (ImGui::BeginTable("ClusterGrid", 3, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextRow();

            // Column 0: Speed & Gear Block
            ImGui::TableSetColumnIndex(0);
            ImGui::SetWindowFontScale(2.1f);
            ImGui::TextColored(hexToImVec4(0x38bdf8), "%3.0f", std::round(snap.speedKmh));
            ImGui::SetWindowFontScale(1.0f);
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextUnformatted("km/h");
            ImGui::TextDisabled("%.1f m/s", snap.forwardSpeedMps);
            ImGui::EndGroup();

            ImGui::SameLine(145.0f);
            ImGui::BeginGroup();
            const std::string modeStr = (snap.drivetrain.transmissionMode == TransmissionMode::Automatic) ? "AUTO" : "MAN";
            const std::string gearBtn = "GEAR: " + snap.drivetrain.gearLabel + " [" + modeStr + "]";
            if (ImGui::Button(gearBtn.c_str(), ImVec2(115.0f, 38.0f))) {
                if (m_callbacks.onToggleTransmission) m_callbacks.onToggleTransmission();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Click or press M to toggle Auto/Manual (E/Q to shift)");
            }
            ImGui::EndGroup();

            // Active surface badge & handbrake pill
            bool anyContact = false;
            SurfaceProperties activeSurf = snap.wheels[0].surface;
            for (const auto& w : snap.wheels) {
                if (w.inContact) {
                    anyContact = true;
                    activeSurf = w.surface;
                    break;
                }
            }
            if (anyContact) {
                ImGui::TextColored(hexToImVec4(0x38bdf8), "Surface: %s (u*%.2f)", activeSurf.label.c_str(), activeSurf.frictionMultiplier);
            } else {
                ImGui::TextColored(hexToImVec4(0xef4444), "Surface: AIRBORNE");
            }
            if (inputs.handbrake > 0.05) {
                ImGui::SameLine();
                ImGui::TextColored(hexToImVec4(0xef4444), "[HANDBRAKE]");
            }

            // Column 1: Tachometer, Pedals & Steering
            ImGui::TableSetColumnIndex(1);
            const float rpm = static_cast<float>(snap.drivetrain.rpm);
            const float rpmFrac = std::clamp(rpm / static_cast<float>(maxRpm), 0.0f, 1.0f);
            const bool redline = (rpm >= static_cast<float>(maxRpm * 0.90));
            char rpmBuf[64];
            std::snprintf(rpmBuf, sizeof(rpmBuf), "%.0f RPM%s", rpm, snap.drivetrain.revLimiterActive ? " [LIM]" : "");

            if (redline) ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hexToImVec4(0xef4444));
            ImGui::ProgressBar(rpmFrac, ImVec2(-1.0f, 18.0f), rpmBuf);
            if (redline) ImGui::PopStyleColor();

            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hexToImVec4(0x22c55e));
            ImGui::ProgressBar(static_cast<float>(inputs.throttle), ImVec2(125.0f, 13.0f), "THR");
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, hexToImVec4(0xef4444));
            ImGui::ProgressBar(static_cast<float>(inputs.brake), ImVec2(125.0f, 13.0f), "BRK");
            ImGui::PopStyleColor();

            const double steerDeg = radToDeg(snap.steeringAngleRad);
            ImGui::Text("STEER: %+5.1f deg", steerDeg);

            // Column 2: 2D G-G Diagram & Attitude Readouts
            ImGui::TableSetColumnIndex(2);
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const ImVec2 ggOrigin = ImGui::GetCursorScreenPos();
            const float ggRadius = 32.0f;
            const ImVec2 ggCenter(ggOrigin.x + ggRadius + 4.0f, ggOrigin.y + ggRadius + 4.0f);

            dl->AddCircleFilled(ggCenter, ggRadius, hexToImU32(0x0b1120));
            dl->AddCircle(ggCenter, ggRadius, hexToImU32(0x475569), 24, 1.5f);
            dl->AddLine(ImVec2(ggCenter.x - ggRadius, ggCenter.y), ImVec2(ggCenter.x + ggRadius, ggCenter.y), hexToImU32(0x334155));
            dl->AddLine(ImVec2(ggCenter.x, ggCenter.y - ggRadius), ImVec2(ggCenter.x, ggCenter.y + ggRadius), hexToImU32(0x334155));

            const float gx = std::clamp(static_cast<float>(-snap.lateralG / 1.5), -1.0f, 1.0f) * (ggRadius - 5.0f);
            const float gy = std::clamp(static_cast<float>(-snap.longitudinalG / 1.5), -1.0f, 1.0f) * (ggRadius - 5.0f);
            dl->AddCircleFilled(ImVec2(ggCenter.x + gx, ggCenter.y + gy), 4.5f, hexToImU32(0x38bdf8));

            ImGui::Dummy(ImVec2(ggRadius * 2.0f + 10.0f, ggRadius * 2.0f + 8.0f));
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::Text("Long G: %+5.2f G", snap.longitudinalG);
            ImGui::Text("Lat G:  %+5.2f G", snap.lateralG);
            ImGui::Text("Pitch:  %+5.2f deg", radToDeg(snap.pitchRad));
            ImGui::Text("Roll:   %+5.2f deg", radToDeg(snap.rollRad));
            ImGui::EndGroup();

            ImGui::EndTable();
        }
    }
    ImGui::End();
}

} // namespace torquelab
