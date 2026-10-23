#pragma once
/**
 * OpenGL 3.3 Core Vehicle Renderer.
 *
 * Synchronizes the visual representation of the car purely from the simulated physical state:
 * - Chassis position & quaternion (including emergent pitch, roll, yaw, heave)
 * - Four independent wheel assemblies (position from suspension compression, steering rotation, and spin angle)
 * - Non-spinning brake calipers mounted on the steered knuckle vs. spinning brake rotors & 5-spoke alloy rims
 * - Visible coilover suspension struts connecting upper mounts to wheel hubs
 * - Interior steering wheel and dynamic brake/reverse lights responding to physical state
 */

#include <array>
#include <vector>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
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

    std::vector<ChassisPart> m_chassisParts;

    // Steering wheel parts (relative to steering wheel pivot in chassis space)
    Mesh m_steerRimMesh;
    Mesh m_steerSpokeMesh;
    Mesh m_steerMarkerMesh;
    MaterialProps m_darkTrimMat;
    MaterialProps m_markerMat;
    double m_steerWheelAngleZ{0.0};

    // Wheel & Knuckle templates (left vs right)
    std::vector<WheelSubPart> m_wheelPartsLeft;
    std::vector<WheelSubPart> m_wheelPartsRight;
    Mesh m_caliperMesh;
    MaterialProps m_caliperMat;
    float m_wheelRadius{0.32f};
    float m_wheelWidth{0.235f};

    // Coilover suspension strut mesh
    Mesh m_strutMesh;
    MaterialProps m_strutMat;

    // Dynamic world transforms updated each frame from interpolated physical state
    Mat4 m_chassisWorldMatrix;
    std::array<Mat4, 4> m_wheelWorldMatrices;
    std::array<Mat4, 4> m_knuckleWorldMatrices;
    std::array<Mat4, 4> m_strutWorldMatrices;

    bool m_isBraking{false};
    bool m_isReverse{false};
};

} // namespace torquelab
