#pragma once
/**
 * OpenGL 3.3 Core 3D Physics Telemetry & Force Vector Overlay.
 *
 * Visualizes internal physical quantities directly in 3D world space:
 * - Suspension raycasts & wheel contact patch markers
 * - Real-time Friction Circle rings (radius proportional to mu_eff * Fz) on the ground under each tire
 * - Tire horizontal force vectors (Fx + Fy) compared against the friction circle boundary
 * - Tire vertical normal load (Fz) vectors
 * - Chassis Center of Mass (CoM) marker
 * - Chassis Forward & Lateral basis arrows
 * - Velocity and Acceleration vectors
 * - Active collision contact markers
 */

#include <vector>
#include <glad/glad.h>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

class DebugOverlayGL {
public:
    DebugOverlayGL() = default;
    ~DebugOverlayGL();

    DebugOverlayGL(const DebugOverlayGL&) = delete;
    DebugOverlayGL& operator=(const DebugOverlayGL&) = delete;

    void init();
    void render(
        const VehicleDynamics& vehicle,
        Shader& lineShader,
        Shader& mainShader,
        const Mat4& viewProj
    );

    bool enabled{true};
    bool showCollisionWireframe{true};

private:
    struct LineVertex {
        float x, y, z;
        float r, g, b;
    };

    void addLine(const Vec3& a, const Vec3& b, const Color3& color, std::vector<LineVertex>& buffer);
    void addArrow(const Vec3& origin, const Vec3& dir, double length, const Color3& color, std::vector<LineVertex>& buffer);
    void addCircleXZ(const Vec3& center, double radius, const Color3& color, std::vector<LineVertex>& buffer, int segments = 32);
    void addBoxWireframe(const btVector3& halfExtents, const btTransform& worldTrans, const Color3& color, std::vector<LineVertex>& buffer);
    void addHullWireframe(const btConvexHullShape* hull, const btTransform& worldTrans, const Color3& color, std::vector<LineVertex>& buffer);
    void addCompoundWireframe(const btRigidBody* body, const btCompoundShape* compound, const Color3& color, std::vector<LineVertex>& buffer);

    GLuint m_lineVao{0};
    GLuint m_lineVbo{0};
    Mesh m_sphereMesh;
    double m_forceScale{0.00022}; // 5000 N -> 1.1 m arrow
};

} // namespace torquelab
