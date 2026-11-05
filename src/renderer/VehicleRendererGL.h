#pragma once
/**
 * OpenGL 3.3 Core Vehicle Renderer.
 *
 * Synchronizes the visual representation of the car purely from the simulated physical state:
 * - Direct support for Blender-created 3D vehicle models (.glb / glTF 2.0)
 * - Independent multi-part articulation: chassis body shell, rotating/steered wheels,
 *   brake calipers, steering wheel, and reactive brake/reverse lights.
 * - Procedural sports coupe fallback with instant hot-swapping.
 * - Emergent 6-DOF chassis pitch, roll, yaw, and heave.
 * - Four independent wheel assemblies (position from suspension travel, steering angle, spin angle).
 * - Visual alignment debugging overlay for wheel pivots, model bounds, and CoM.
 */

#include <array>
#include <string>
#include <unordered_map>
#include <vector>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/ModelLoaderGLB.h"
#include "renderer/Shader.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

class VehicleRendererGL {
public:
    VehicleRendererGL() = default;
    ~VehicleRendererGL();

    void init(const VehicleConfig& config);
    void syncWithVehicle(const VehicleDynamics& vehicle, double alpha);
    void renderShadowPass(Shader& depthShader) const;
    void renderMainPass(Shader& mainShader) const;
    void renderVisualDebug(Shader& lineShader, Shader& mainShader, const Mat4& viewProj) const;

    // Model mode & switching
    bool isUsingImportedModel() const { return m_useImportedModel && m_activeModel && m_activeModel->loadedSuccessfully; }
    void setUseImportedModel(bool enable) {
        if (!enable) {
            m_useImportedModel = false;
        } else if (m_activeModel && m_activeModel->loadedSuccessfully) {
            m_useImportedModel = true;
        }
    }
    void setActiveVehicleModel(const std::string& vehicleId) {
        m_activeModelId = vehicleId;
        auto it = m_importedModels.find(vehicleId);
        if (it != m_importedModels.end() && it->second.loadedSuccessfully) {
            m_activeModel = &it->second;
            m_useImportedModel = true;
        } else {
            m_activeModel = nullptr;
            m_useImportedModel = false;
        }
    }
    bool hasImportedModel(const std::string& vehicleId = "") const {
        if (vehicleId.empty()) {
            return m_activeModel && m_activeModel->loadedSuccessfully;
        }
        auto it = m_importedModels.find(vehicleId);
        return (it != m_importedModels.end() && it->second.loadedSuccessfully);
    }
    const LoadedVehicleModel& getImportedModel(const std::string& vehicleId = "") const {
        if (!vehicleId.empty()) {
            auto it = m_importedModels.find(vehicleId);
            if (it != m_importedModels.end()) return it->second;
        }
        if (m_activeModel) return *m_activeModel;
        static const LoadedVehicleModel emptyModel{};
        return emptyModel;
    }
    struct VehicleChoice {
        std::string id;
        std::string displayName;
    };

    const std::vector<VehicleChoice>& getAvailableVehicles() const { return m_availableVehicles; }
    const std::unordered_map<std::string, LoadedVehicleModel>& getAllImportedModels() const { return m_importedModels; }

    const std::string& getActiveModelName() const {
        return (m_useImportedModel && m_activeModel && m_activeModel->loadedSuccessfully) ? m_activeModel->modelName : m_proceduralModelName;
    }
    const std::string& getActiveModelId() const {
        return m_useImportedModel ? m_activeModelId : m_proceduralId;
    }

    // Visual alignment debug overlay toggle
    bool isVisualDebugEnabled() const { return m_showVisualDebug; }
    void setVisualDebugEnabled(bool enable) { m_showVisualDebug = enable; }
    void toggleVisualDebug() { m_showVisualDebug = !m_showVisualDebug; }

    struct WheelDeformState {
        float radialDeform{0.0f};
        float lateralDeform{0.0f};
        float longDeform{0.0f};
        float sidewallBulge{0.0f};
        float patchAngleSpan{0.45f};
        float outerRadius{0.32f};
        float rimRadius{0.22f};
        float sectionWidth{0.24f};
        Vec3 contactDirLocal{0.0, -1.0, 0.0};
        Vec3 axleLocal{1.0, 0.0, 0.0};
        Vec3 wheelCenterWorld{0.0, 0.0, 0.0};
        Vec3 wheelAxleWorld{1.0, 0.0, 0.0};
        Vec3 groundPointWorld{0.0, 0.0, 0.0};
        Vec3 groundNormalWorld{0.0, 1.0, 0.0};
        bool inContact{false};
    };

private:
    struct ChassisPart {
        Mesh mesh;
        Mat4 localMatrix;
        MaterialProps material;
        bool isBrakeLight{false};
        bool isReverseLight{false};
        bool isTransparent{false};
    };

    struct WheelSubPart {
        Mesh mesh;
        Mat4 localMatrix;
        MaterialProps material;
        bool isTire{false};
    };

    // Procedural sports coupe model data
    std::string m_proceduralModelName{"Procedural Sports Coupe"};
    std::string m_proceduralId{"procedural_coupe"};
    std::vector<VehicleChoice> m_availableVehicles;
    std::vector<ChassisPart> m_chassisParts;

    // Steering wheel parts (relative to steering wheel pivot in chassis space)
    Mesh m_steerRimMesh;
    Mesh m_steerSpokeMesh;
    Mesh m_steerMarkerMesh;
    MaterialProps m_darkTrimMat;
    MaterialProps m_markerMat;
    double m_steerWheelAngleZ{0.0};

    // Procedural wheel & Knuckle templates (left vs right)
    std::vector<WheelSubPart> m_wheelPartsLeft;
    std::vector<WheelSubPart> m_wheelPartsRight;
    Mesh m_caliperMesh;
    MaterialProps m_caliperMat;
    float m_wheelRadius{0.32f};
    float m_wheelWidth{0.235f};

    // Coilover suspension strut mesh
    Mesh m_strutMesh;
    MaterialProps m_strutMat;

    // Imported Blender GLB vehicle models
    std::unordered_map<std::string, LoadedVehicleModel> m_importedModels;
    std::string m_activeModelId{"bmw_m4"};
    LoadedVehicleModel* m_activeModel{nullptr};
    bool m_useImportedModel{true};
    bool m_showVisualDebug{false};

    // Dynamic world transforms updated each frame from interpolated physical state
    Mat4 m_chassisWorldMatrix;
    std::array<Mat4, 4> m_wheelWorldMatrices;
    std::array<Mat4, 4> m_knuckleWorldMatrices;
    std::array<Mat4, 4> m_strutWorldMatrices;
    std::array<WheelDeformState, 4> m_wheelDeformStates;

    // Calculated transforms for imported model parts
    Mat4 m_importedChassisWorldMatrix;
    std::array<std::vector<Mat4>, 4> m_importedWheelPartMatrices;

    bool m_isBraking{false};
    bool m_isReverse{false};
    Vec3 m_vehiclePos{0.0, 0.0, 0.0};
    Quat m_vehicleRot{0.0, 0.0, 0.0, 1.0};
    std::array<Vec3, 4> m_wheelCentersWorld{};
    std::array<Vec3, 4> m_wheelMountsWorld{};

    // Visual debug line buffer
    struct DebugLineVertex {
        float x, y, z;
        float r, g, b;
    };
    mutable unsigned int m_debugLineVao{0};
    mutable unsigned int m_debugLineVbo{0};
    Mesh m_debugSphereMesh;
};

} // namespace torquelab
