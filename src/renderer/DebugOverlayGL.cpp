#include "renderer/DebugOverlayGL.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

DebugOverlayGL::~DebugOverlayGL() {
    if (m_lineVbo != 0) {
        glDeleteBuffers(1, &m_lineVbo);
        m_lineVbo = 0;
    }
    if (m_lineVao != 0) {
        glDeleteVertexArrays(1, &m_lineVao);
        m_lineVao = 0;
    }
}

void DebugOverlayGL::init() {
    glGenVertexArrays(1, &m_lineVao);
    glGenBuffers(1, &m_lineVbo);

    glBindVertexArray(m_lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVbo);
    glBufferData(GL_ARRAY_BUFFER, 4096 * sizeof(LineVertex), nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), reinterpret_cast<void*>(0));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);

    m_sphereMesh = Mesh::createSphere(1.0f, 12, 16);
}

void DebugOverlayGL::addLine(
    const Vec3& a,
    const Vec3& b,
    const Color3& color,
    std::vector<LineVertex>& buffer
) {
    buffer.push_back(LineVertex{
        static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z),
        color.r, color.g, color.b
    });
    buffer.push_back(LineVertex{
        static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z),
        color.r, color.g, color.b
    });
}

void DebugOverlayGL::addArrow(
    const Vec3& origin,
    const Vec3& dir,
    double length,
    const Color3& color,
    std::vector<LineVertex>& buffer
) {
    if (length < 0.02) return;
    const Vec3 normDir = vec3Normalize(dir);
    const Vec3 tip = vec3Add(origin, vec3Scale(normDir, length));
    addLine(origin, tip, color, buffer);

    // Build two orthogonal arrowhead fins
    Vec3 refUp{0.0, 1.0, 0.0};
    if (std::abs(vec3Dot(normDir, refUp)) > 0.9) {
        refUp = Vec3{1.0, 0.0, 0.0};
    }
    const Vec3 side1 = vec3Normalize(vec3Cross(normDir, refUp));
    const Vec3 side2 = vec3Normalize(vec3Cross(normDir, side1));
    const double headLen = std::min(0.24, length * 0.25);
    const double headSpread = headLen * 0.45;
    const Vec3 base = vec3Add(tip, vec3Scale(normDir, -headLen));

    addLine(tip, vec3Add(base, vec3Scale(side1, +headSpread)), color, buffer);
    addLine(tip, vec3Add(base, vec3Scale(side1, -headSpread)), color, buffer);
    addLine(tip, vec3Add(base, vec3Scale(side2, +headSpread)), color, buffer);
    addLine(tip, vec3Add(base, vec3Scale(side2, -headSpread)), color, buffer);
}

void DebugOverlayGL::addCircleXZ(
    const Vec3& center,
    double radius,
    const Color3& color,
    std::vector<LineVertex>& buffer,
    int segments
) {
    for (int i = 0; i < segments; ++i) {
        const double a0 = (static_cast<double>(i) / static_cast<double>(segments)) * 2.0 * PI_D;
        const double a1 = (static_cast<double>(i + 1) / static_cast<double>(segments)) * 2.0 * PI_D;
        const Vec3 p0{center.x + std::cos(a0) * radius, center.y, center.z + std::sin(a0) * radius};
        const Vec3 p1{center.x + std::cos(a1) * radius, center.y, center.z + std::sin(a1) * radius};
        addLine(p0, p1, color, buffer);
    }
}

void DebugOverlayGL::render(
    const VehicleDynamics& vehicle,
    Shader& lineShader,
    Shader& mainShader,
    const Mat4& viewProj
) {
    if (!enabled) return;

    const VehicleTelemetrySnapshot snap = vehicle.getTelemetrySnapshot();
    const Vec3 com = snap.centerOfMassWorld;
    const Vec3 comOrigin{com.x, com.y + 0.15, com.z};

    std::vector<LineVertex> lines;
    lines.reserve(1024);

    // 1. Chassis Basis, Velocity, and Acceleration Arrows
    addArrow(comOrigin, snap.forwardDirWorld, 2.2, Color3::fromHex(0x38bdf8), lines);
    addArrow(comOrigin, snap.lateralDirWorld, 1.6, Color3::fromHex(0xa855f7), lines);

    if (snap.speedMps > 0.25) {
        addArrow(comOrigin, snap.velocityWorld, std::min(5.0, snap.speedMps * 0.15), Color3::fromHex(0x22c55e), lines);
    }

    const double accelMag = vec3Length(snap.accelerationWorld);
    if (accelMag > 0.4) {
        addArrow(comOrigin, snap.accelerationWorld, std::min(4.0, accelMag * 0.18), Color3::fromHex(0xf97316), lines);
    }

    // 2. Per-Wheel Raycast Lines, Normal Load Arrows, Tire Force Arrows, and Friction Circles
    for (int i = 0; i < 4; ++i) {
        const WheelState& w = snap.wheels[static_cast<size_t>(i)];
        const Vec3 endPt = w.inContact ? w.contactPointWorld : w.wheelCenterWorld;
        addLine(w.mountPointWorld, endPt, Color3::fromHex(0xfef08a), lines);

        if (w.inContact && w.normalLoad > 10.0) {
            // Normal load arrow Fz
            addArrow(
                w.contactPointWorld,
                w.contactNormalWorld,
                std::max(0.15, w.normalLoad * m_forceScale),
                Color3::fromHex(0x38bdf8),
                lines
            );

            // Friction circle ring on road plane
            const double maxGripRadius = std::max(0.05, w.maxAvailableGripForce * m_forceScale);
            const Color3 ringColor = w.isSkidding
                ? Color3::fromHex(0xef4444)
                : (w.gripUsageRatio > 0.7 ? Color3::fromHex(0xfacc15) : Color3::fromHex(0x10b981));
            addCircleXZ(
                Vec3{w.contactPointWorld.x, w.contactPointWorld.y + 0.025, w.contactPointWorld.z},
                maxGripRadius,
                ringColor,
                lines
            );

            // Horizontal tire force vector arrow
            const double fxyMag = vec3Length(w.tireForceWorld);
            if (fxyMag > 25.0) {
                addArrow(
                    Vec3{w.contactPointWorld.x, w.contactPointWorld.y + 0.03, w.contactPointWorld.z},
                    w.tireForceWorld,
                    std::max(0.12, fxyMag * m_forceScale),
                    Color3::fromHex(0xef4444),
                    lines
                );
            }
        }
    }

    // Upload and draw lines
    if (!lines.empty()) {
        lineShader.use();
        lineShader.setMat4("uViewProj", viewProj);

        glBindVertexArray(m_lineVao);
        glBindBuffer(GL_ARRAY_BUFFER, m_lineVbo);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(lines.size() * sizeof(LineVertex)),
            lines.data(),
            GL_DYNAMIC_DRAW
        );
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.size()));
        glBindVertexArray(0);
    }

    // 3. Draw 3D Contact Spheres, CoM Sphere, and Collision Contact Markers
    mainShader.use();
    MaterialProps markerMat{};
    markerMat.unlit = true;
    markerMat.receiveShadows = false;
    markerMat.castShadows = false;

    for (int i = 0; i < 4; ++i) {
        const WheelState& w = snap.wheels[static_cast<size_t>(i)];
        if (w.inContact && w.normalLoad > 10.0) {
            markerMat.baseColor = Color3::fromHex(w.isSkidding ? 0xef4444 : 0x22c55e);
            mainShader.applyMaterial(markerMat);
            mainShader.setMat4(
                "uModel",
                Mat4::fromTRS(
                    Vec3{w.contactPointWorld.x, w.contactPointWorld.y + 0.02, w.contactPointWorld.z},
                    quatIdentity(),
                    Vec3{0.055, 0.055, 0.055}
                )
            );
            m_sphereMesh.draw();
        }
    }

    // CoM sphere & collision markers rendered on top (depth test disabled)
    glDisable(GL_DEPTH_TEST);
    markerMat.baseColor = Color3::fromHex(0xfacc15);
    mainShader.applyMaterial(markerMat);
    mainShader.setMat4("uModel", Mat4::fromTRS(com, quatIdentity(), Vec3{0.11, 0.11, 0.11}));
    m_sphereMesh.draw();

    markerMat.baseColor = Color3::fromHex(0xff0055);
    mainShader.applyMaterial(markerMat);
    for (const auto& c : vehicle.body.activeContacts) {
        mainShader.setMat4("uModel", Mat4::fromTRS(c.pointWorld, quatIdentity(), Vec3{0.12, 0.12, 0.12}));
        m_sphereMesh.draw();
    }
    glEnable(GL_DEPTH_TEST);
}

} // namespace torquelab
