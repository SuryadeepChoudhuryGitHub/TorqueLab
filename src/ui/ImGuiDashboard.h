#pragma once
/**
 * Dear ImGui Vehicle Instrument Cluster, Telemetry Inspector, 4-Band Oscilloscope,
 * and Live Parameter Tuning Laboratory.
 *
 * All displayed values are read directly from the live VehicleTelemetrySnapshot.
 */

#include <array>
#include <deque>
#include <functional>
#include <string>
#include "physics/PhysicsWorld.h"
#include "renderer/TrackRendererGL.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

struct DashboardCallbacks {
    std::function<void()> onReset;
    std::function<void()> onTogglePause;
    std::function<void()> onCycleCamera;
    std::function<void()> onToggleTelemetry;
    std::function<void()> onToggleGizmos3D;
    std::function<void()> onToggleTransmission;
    std::function<void(const SpawnPreset&)> onSelectPreset;
    std::function<void(GraphicsQuality)> onChangeQuality;
    std::function<void()> onRunValidationCheck;
    std::function<void(bool)> onToggleImportedModel;
    std::function<void()> onToggleVisualDebug;
    std::function<void(const std::string&)> onSelectVehicle;
    std::function<void()> onSaveVehicleJson;
    std::function<void()> onReloadVehicleJson;
    std::function<void()> onResetVehicleDefaults;
    std::function<void(bool)> onToggleCollisionWireframe;
    std::function<void(CollisionAlgorithmStrategy)> onRegenerateCollision;
};

class ImGuiDashboard {
public:
    explicit ImGuiDashboard(DashboardCallbacks callbacks);

    void applyTheme();

    void render(
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
        int windowHeight,
        bool useImportedModel = true,
        bool hasImportedModel = true,
        const std::string& modelName = "BMW M4 Coupe",
        bool visualDebugEnabled = false,
        bool collisionWireframeEnabled = true
    );

    void showValidationReport(const std::string& message, bool isSuccess = true);

    bool telemetryEnabled{true};

private:
    struct TelemetrySample {
        float speedKmh{0.0f};
        float longAccel{0.0f};
        float latAccel{0.0f};
        float pitchDeg{0.0f};
        float rollDeg{0.0f};
        std::array<float, 4> compMM{0.0f, 0.0f, 0.0f, 0.0f};
    };

    void renderTopBar(
        bool paused,
        const std::string& cameraLabel,
        bool gizmosEnabled,
        GraphicsQuality currentQuality,
        int windowWidth,
        bool useImportedModel,
        bool hasImportedModel,
        bool visualDebugEnabled
    );
    void renderLeftPanels(
        VehicleDynamics& vehicle,
        int windowHeight,
        bool useImportedModel,
        bool hasImportedModel,
        const std::string& modelName,
        bool visualDebugEnabled,
        bool collisionWireframeEnabled
    );
    void renderRightTelemetryPanel(
        const VehicleTelemetrySnapshot& snap,
        double fps,
        double physicsStepTimeMs,
        int windowWidth,
        int windowHeight
    );
    void renderBottomInstrumentCluster(
        const VehicleTelemetrySnapshot& snap,
        const VehicleControlInputs& inputs,
        double maxRpm,
        int windowWidth,
        int windowHeight
    );
    void drawOscilloscopeCanvas(float width, float height);

    DashboardCallbacks m_callbacks;
    std::deque<TelemetrySample> m_history;
    static constexpr size_t MAX_SAMPLES = 180; // 3 seconds at 60 Hz

    std::string m_validationMessage;
    bool m_validationSuccess{true};
    double m_validationToastTimer{0.0};
};

} // namespace torquelab
