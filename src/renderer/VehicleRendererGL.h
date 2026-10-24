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

    void init(const VehicleConfig& config);
    void syncWithVehicle(const VehicleDynamics& vehicle, double alpha);
    void renderShadowPass(Shader& depthShader) const;
    void renderMainPass(Shader& mainShader) const;
    void renderVisualDebug(Shader& lineShader, Shader& mainShader, const Mat4& viewProj) const;

    // Model mode & switching
    bool isUsingImportedModel() const { return m_useImportedModel && m_importedModel.loadedSuccessfully; }
    void setUseImportedModel(bool enable) {
        if (m_importedModel.loadedSuccessfully) {
            m_useImportedModel = enable;
        }
    }
    bool hasImportedModel() const { return m_importedModel.loadedSuccessfully; }
    const LoadedVehicleModel& getImportedModel() const { return m_importedModel; }
    const std::string& getActiveModelName() const {
        return (m_useImportedModel && m_importedModel.loadedSuccessfully) ? m_importedModel.modelName : m_proceduralModelName;
    }

    // Visual alignment debug overlay toggle
    bool isVisualDebugEnabled() const { return m_showVisualDebug; }
    void setVisualDebugEnabled(bool enable) { m_showVisualDebug = enable; }
    void toggleVisualDebug() { m_showVisualDebug = !m_showVisualDebug; }

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
    };

    // Procedural sports coupe model data
    std::string m_proceduralModelName{"Procedural Sports Coupe"};
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

    // Imported Blender GLB vehicle model
    LoadedVehicleModel m_importedModel;
    bool m_useImportedModel{true};
    bool m_showVisualDebug{false};

    // Dynamic world transforms updated each frame from interpolated physical state
    Mat4 m_chassisWorldMatrix;
    std::array<Mat4, 4> m_wheelWorldMatrices;
    std::array<Mat4, 4> m_knuckleWorldMatrices;
    std::array<Mat4, 4> m_strutWorldMatrices;

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
