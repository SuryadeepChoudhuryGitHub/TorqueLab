#include "renderer/DebugOverlayGL.h"
#include <LinearMath/btConvexHullComputer.h>
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

static btVector3 toBtVec3(const Vec3& v) {
    return btVector3(static_cast<btScalar>(v.x), static_cast<btScalar>(v.y), static_cast<btScalar>(v.z));
}

static btQuaternion toBtQuat(const Quat& q) {
    return btQuaternion(
        static_cast<btScalar>(q.x),
        static_cast<btScalar>(q.y),
        static_cast<btScalar>(q.z),
        static_cast<btScalar>(q.w)
    );
}

void DebugOverlayGL::addCylinderXWireframe(
    double halfWidth,
    double radius,
    const btTransform& worldTrans,
    const Color3& color,
    std::vector<LineVertex>& buffer,
    int segments
) {
    auto toV = [](const btVector3& p) { return Vec3{p.x(), p.y(), p.z()}; };

    for (int i = 0; i < segments; ++i) {
        const double a0 = (static_cast<double>(i) / static_cast<double>(segments)) * 2.0 * PI_D;
        const double a1 = (static_cast<double>(i + 1) / static_cast<double>(segments)) * 2.0 * PI_D;

        const double y0 = std::cos(a0) * radius;
        const double z0 = std::sin(a0) * radius;
        const double y1 = std::cos(a1) * radius;
        const double z1 = std::sin(a1) * radius;

        // Left rim
        const btVector3 l0(static_cast<btScalar>(-halfWidth), static_cast<btScalar>(y0), static_cast<btScalar>(z0));
        const btVector3 l1(static_cast<btScalar>(-halfWidth), static_cast<btScalar>(y1), static_cast<btScalar>(z1));
        addLine(toV(worldTrans(l0)), toV(worldTrans(l1)), color, buffer);

        // Right rim
        const btVector3 r0(static_cast<btScalar>(+halfWidth), static_cast<btScalar>(y0), static_cast<btScalar>(z0));
        const btVector3 r1(static_cast<btScalar>(+halfWidth), static_cast<btScalar>(y1), static_cast<btScalar>(z1));
        addLine(toV(worldTrans(r0)), toV(worldTrans(r1)), color, buffer);

        // Longitudinal ribs connecting the rims (8 ribs total)
        if (segments >= 8 && (i % (segments / 8) == 0)) {
            addLine(toV(worldTrans(l0)), toV(worldTrans(r0)), color, buffer);
        }
    }

    // Axle center pin
    const btVector3 axleL(static_cast<btScalar>(-halfWidth * 1.15), 0.0f, 0.0f);
    const btVector3 axleR(static_cast<btScalar>(+halfWidth * 1.15), 0.0f, 0.0f);
    addLine(toV(worldTrans(axleL)), toV(worldTrans(axleR)), Color3::fromHex(0xffffff), buffer);
}

void DebugOverlayGL::addBoxWireframe(
    const btVector3& halfExtents,
    const btTransform& worldTrans,
    const Color3& color,
    std::vector<LineVertex>& buffer
) {
    const btVector3 h = halfExtents;
    const btVector3 corners[8] = {
        worldTrans(btVector3(-h.x(), -h.y(), -h.z())),
        worldTrans(btVector3( h.x(), -h.y(), -h.z())),
        worldTrans(btVector3( h.x(),  h.y(), -h.z())),
        worldTrans(btVector3(-h.x(),  h.y(), -h.z())),
        worldTrans(btVector3(-h.x(), -h.y(),  h.z())),
        worldTrans(btVector3( h.x(), -h.y(),  h.z())),
        worldTrans(btVector3( h.x(),  h.y(),  h.z())),
        worldTrans(btVector3(-h.x(),  h.y(),  h.z()))
    };

    auto toV = [](const btVector3& p) { return Vec3{p.x(), p.y(), p.z()}; };

    // Bottom face
    addLine(toV(corners[0]), toV(corners[1]), color, buffer);
    addLine(toV(corners[1]), toV(corners[2]), color, buffer);
    addLine(toV(corners[2]), toV(corners[3]), color, buffer);
    addLine(toV(corners[3]), toV(corners[0]), color, buffer);

    // Top face
    addLine(toV(corners[4]), toV(corners[5]), color, buffer);
    addLine(toV(corners[5]), toV(corners[6]), color, buffer);
    addLine(toV(corners[6]), toV(corners[7]), color, buffer);
    addLine(toV(corners[7]), toV(corners[4]), color, buffer);

    // Vertical pillars
    addLine(toV(corners[0]), toV(corners[4]), color, buffer);
    addLine(toV(corners[1]), toV(corners[5]), color, buffer);
    addLine(toV(corners[2]), toV(corners[6]), color, buffer);
    addLine(toV(corners[3]), toV(corners[7]), color, buffer);
}

void DebugOverlayGL::addHullWireframe(
    const btConvexHullShape* hull,
    const btTransform& worldTrans,
    const Color3& color,
    std::vector<LineVertex>& buffer
) {
    if (!hull) return;
    const int numPts = hull->getNumPoints();
    if (numPts < 2) return;

    auto toV = [](const btVector3& p) { return Vec3{p.x(), p.y(), p.z()}; };

    const btVector3* pts = hull->getUnscaledPoints();

    if (numPts >= 4) {
        btConvexHullComputer computer;
        computer.compute(reinterpret_cast<const btScalar*>(pts), sizeof(btVector3), numPts, 0.0f, 0.0f);
        if (computer.edges.size() > 0) {
            for (int e = 0; e < computer.edges.size(); ++e) {
                const auto& edge = computer.edges[e];
                const int v0 = edge.getSourceVertex();
                const int v1 = edge.getTargetVertex();
                if (v0 < v1) {
                    addLine(toV(worldTrans(computer.vertices[v0])), toV(worldTrans(computer.vertices[v1])), color, buffer);
                }
            }
            return;
        }
    }

    for (int i = 0; i < numPts; ++i) {
        for (int j = i + 1; j < numPts; ++j) {
            addLine(toV(worldTrans(pts[i])), toV(worldTrans(pts[j])), color, buffer);
        }
    }
}

void DebugOverlayGL::addCompoundWireframe(
    const btRigidBody* body,
    const btCompoundShape* compound,
    const Color3& color,
    std::vector<LineVertex>& buffer
) {
    if (!body || !compound) return;
    const btTransform& bodyTrans = body->getWorldTransform();

    static const Color3 pieceColors[] = {
        Color3::fromHex(0x10b981), // Emerald Green
        Color3::fromHex(0x06b6d4), // Cyan
        Color3::fromHex(0xf59e0b), // Amber
        Color3::fromHex(0x8b5cf6), // Purple
        Color3::fromHex(0xec4899), // Pink
        Color3::fromHex(0x3b82f6), // Blue
        Color3::fromHex(0x14b8a6), // Teal
        Color3::fromHex(0xf97316)  // Orange
    };

    const int numChildren = compound->getNumChildShapes();
    for (int i = 0; i < numChildren; ++i) {
        const btTransform childWorld = bodyTrans * compound->getChildTransform(i);
        const btCollisionShape* child = compound->getChildShape(i);
        const Color3 childColor = (numChildren > 1) ? pieceColors[static_cast<size_t>(i) % 8] : color;

        if (child->getShapeType() == BOX_SHAPE_PROXYTYPE) {
            const auto* box = static_cast<const btBoxShape*>(child);
            addBoxWireframe(box->getHalfExtentsWithMargin(), childWorld, childColor, buffer);
        } else if (child->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) {
            const auto* hull = static_cast<const btConvexHullShape*>(child);
            addHullWireframe(hull, childWorld, childColor, buffer);
        }
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

    // 1. Physical Collision Shape Wireframe (exact Bullet physical boundaries)
    if (showCollisionWireframe) {
        if (vehicle.body.getRigidBody() && vehicle.body.getCompoundShape()) {
            const Color3 cageColor = Color3::fromHex(0x22c55e); // Emerald Green for chassis body
            addCompoundWireframe(vehicle.body.getRigidBody(), vehicle.body.getCompoundShape(), cageColor, lines);
        }
    }

    if (showWheelColliders) {
        // Dedicated physical wheel collision cylinders (Cyan for Front, Amber for Rear)
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = snap.wheels[static_cast<size_t>(i)];
            const btCollisionObject* wheelObj = vehicle.getWheelCollisionObject(i);
            btTransform wt;
            if (wheelObj) {
                wt = wheelObj->getWorldTransform();
            } else {
                wt.setIdentity();
                wt.setOrigin(toBtVec3(w.wheelCenterWorld));
                wt.setRotation(toBtQuat(w.worldOrientation));
            }
            const Color3 wheelColor = (i < 2) ? Color3::fromHex(0x06b6d4) : Color3::fromHex(0xf59e0b); // Cyan / Amber
            addCylinderXWireframe(w.wheelWidth * 0.5, w.wheelRadius, wt, wheelColor, lines);
        }
    }

    // Wheel Axles & Steering Pivot Pins
    if (showWheelAxes) {
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = snap.wheels[static_cast<size_t>(i)];
            const Quat steerQ = quatFromAxisAngle(snap.upDirWorld, w.steeringAngle);
            const Vec3 axleDir = quatRotateVec3(steerQ, snap.lateralDirWorld);
            // Draw axle pin through wheel center
            const Vec3 p0 = vec3Add(w.wheelCenterWorld, vec3Scale(axleDir, -w.wheelWidth * 0.65));
            const Vec3 p1 = vec3Add(w.wheelCenterWorld, vec3Scale(axleDir, +w.wheelWidth * 0.65));
            addLine(p0, p1, Color3::fromHex(0xe2e8f0), lines); // White/silver axle pin

            // Draw steering kingpin pivot for front wheels
            if (i < 2) {
                const Vec3 kpTop = vec3Add(w.wheelCenterWorld, vec3Scale(snap.upDirWorld, +0.22));
                const Vec3 kpBot = vec3Add(w.wheelCenterWorld, vec3Scale(snap.upDirWorld, -0.22));
                addLine(kpTop, kpBot, Color3::fromHex(0xa855f7), lines); // Purple steering kingpin
            }
        }
    }

    // Suspension Strut & Travel
    if (showSuspensionTravel) {
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = snap.wheels[static_cast<size_t>(i)];
            const Vec3 endPt = w.inContact ? w.contactPointWorld : w.wheelCenterWorld;
            const Color3 strutColor = w.isBottomedOut
                ? Color3::fromHex(0xef4444) // Red when bottomed out
                : (w.compressionRatio > 0.6 ? Color3::fromHex(0xf59e0b) : Color3::fromHex(0x10b981)); // Yellow or Green
            addLine(w.mountPointWorld, endPt, strutColor, lines);
        }
    }

    // Alignment & Fender Clearance Diagnostics (Draws clearance indicator from chassis outer edge to wheel)
    if (showAlignmentDiagnostics) {
        const double halfChassisWidth = vehicle.config.chassisHalfExtents.x;
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = snap.wheels[static_cast<size_t>(i)];
            const double sideSign = (i % 2 == 0) ? +1.0 : -1.0;
            const Vec3 bodyEdgeLocal{sideSign * halfChassisWidth, w.mountPointLocal.y, w.mountPointLocal.z};
            const Vec3 bodyEdgeWorld = localPointToWorld(bodyEdgeLocal, snap.chassisPosition, snap.chassisRotation);
            const Vec3 wheelOuterEdge = vec3Add(w.wheelCenterWorld, vec3Scale(snap.lateralDirWorld, sideSign * (w.wheelWidth * 0.5)));
            // Green if tucked inside fender, Red if protruding
            const bool isTucked = (sideSign * (bodyEdgeWorld.x - wheelOuterEdge.x) >= 0.0);
            addLine(bodyEdgeWorld, wheelOuterEdge, isTucked ? Color3::fromHex(0x10b981) : Color3::fromHex(0xef4444), lines);
        }
    }

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
    if (showContactForces) {
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

                // Obstacle contact horizontal resistance and climbing traction forces
                if (w.hasObstacleContact) {
                    const double obsResMag = vec3Length(w.obstacleResistanceForceWorld);
                    if (obsResMag > 20.0) {
                        addArrow(
                            w.contactPointWorld,
                            w.obstacleResistanceForceWorld,
                            std::max(0.15, obsResMag * m_forceScale),
                            Color3::fromHex(0xf97316), // Orange
                            lines
                        );
                    }
                    const double climbMag = vec3Length(w.tractionClimbingForceWorld);
                    if (climbMag > 20.0) {
                        addArrow(
                            w.contactPointWorld,
                            w.tractionClimbingForceWorld,
                            std::max(0.15, climbMag * m_forceScale),
                            Color3::fromHex(0x10b981), // Emerald
                            lines
                        );
                    }
                }
            }
        }
    }

    // 3. Deformed Pneumatic Tyre Profile & Multi-Point Contact Patch Samples
    if (showTireDeformation || showContactPatchSamples) {
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = snap.wheels[static_cast<size_t>(i)];
            const auto& tState = w.tireModel.getState();
            const Quat steerQ = quatFromAxisAngle(snap.upDirWorld, w.steeringAngle);
            const Vec3 steeredFwd = quatRotateVec3(steerQ, snap.forwardDirWorld);
            const Vec3 steeredLeft = quatRotateVec3(steerQ, snap.lateralDirWorld);

            // A. Deformed Tyre Profile: flattened contact footprint and sidewall bulge markers
            if (showTireDeformation && w.inContact && tState.radialDeformation > 0.001) {
                const double halfPatchLen = 0.5 * std::max(0.04, tState.contactPatchLength);
                const double halfPatchWid = 0.5 * std::max(0.08, tState.contactPatchWidth);

                // Flattened contact patch rectangular footprint boundary on the road surface
                const Vec3 c0 = vec3Add(w.contactPointWorld, vec3Add(vec3Scale(steeredFwd, +halfPatchLen), vec3Scale(steeredLeft, +halfPatchWid)));
                const Vec3 c1 = vec3Add(w.contactPointWorld, vec3Add(vec3Scale(steeredFwd, +halfPatchLen), vec3Scale(steeredLeft, -halfPatchWid)));
                const Vec3 c2 = vec3Add(w.contactPointWorld, vec3Add(vec3Scale(steeredFwd, -halfPatchLen), vec3Scale(steeredLeft, -halfPatchWid)));
                const Vec3 c3 = vec3Add(w.contactPointWorld, vec3Add(vec3Scale(steeredFwd, -halfPatchLen), vec3Scale(steeredLeft, +halfPatchWid)));
                addLine(c0, c1, Color3::fromHex(0x4ade80), lines); // Bright green footprint box
                addLine(c1, c2, Color3::fromHex(0x4ade80), lines);
                addLine(c2, c3, Color3::fromHex(0x4ade80), lines);
                addLine(c3, c0, Color3::fromHex(0x4ade80), lines);

                // Longitudinal centerline of contact patch
                const Vec3 patchFwd = vec3Add(w.contactPointWorld, vec3Scale(steeredFwd, +halfPatchLen));
                const Vec3 patchRear = vec3Add(w.contactPointWorld, vec3Scale(steeredFwd, -halfPatchLen));
                addLine(patchFwd, patchRear, Color3::fromHex(0x22c55e), lines); // Midline

                // Sidewall bulge indicators (protruding laterally near contact patch)
                const double bulge = tState.sidewallBulge;
                const Vec3 leftBulge = vec3Add(w.contactPointWorld, vec3Scale(steeredLeft, +(halfPatchWid + bulge)));
                const Vec3 rightBulge = vec3Add(w.contactPointWorld, vec3Scale(steeredLeft, -(halfPatchWid + bulge)));
                addLine(leftBulge, rightBulge, Color3::fromHex(0xfacc15), lines); // Yellow width bulge

                // Radial compression line from undeflected bottom to deformed contact point
                const Vec3 undeflectedBottom = vec3Add(w.wheelCenterWorld, vec3Scale(snap.upDirWorld, -w.wheelRadius));
                addLine(undeflectedBottom, w.contactPointWorld, Color3::fromHex(0xef4444), lines); // Red compression delta
            }

            // B. Multi-Point Distributed Contact Patch Samples
            if (showContactPatchSamples && !tState.contactSamples.empty()) {
                const auto& samples = tState.contactSamples;
                for (const auto& sample : samples) {
                    const Vec3 sp = sample.contactPointWorld;
                    const Color3 sColor = sample.inContact
                        ? Color3::fromHex(0x22c55e) // Green active
                        : Color3::fromHex(0x64748b); // Muted slate inactive
                    const double crossSize = 0.02;
                    addLine(vec3Add(sp, vec3Scale(steeredFwd, -crossSize)), vec3Add(sp, vec3Scale(steeredFwd, +crossSize)), sColor, lines);
                    addLine(vec3Add(sp, vec3Scale(steeredLeft, -crossSize)), vec3Add(sp, vec3Scale(steeredLeft, +crossSize)), sColor, lines);

                    // Local normal force vector per sample point
                    if (sample.inContact && sample.normalLoad > 5.0) {
                        addArrow(sp, sample.contactNormalWorld, sample.normalLoad * m_forceScale * 3.5, Color3::fromHex(0x38bdf8), lines);
                    }
                }
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
            if (w.hasObstacleContact) {
                markerMat.baseColor = Color3::fromHex(0xf97316); // Amber-Orange for obstacle/circumference contact
            } else {
                markerMat.baseColor = Color3::fromHex(w.isSkidding ? 0xef4444 : 0x22c55e);
            }
            mainShader.applyMaterial(markerMat);
            mainShader.setMat4(
                "uModel",
                Mat4::fromTRS(
                    Vec3{w.contactPointWorld.x, w.contactPointWorld.y + 0.02, w.contactPointWorld.z},
                    quatIdentity(),
                    Vec3{0.065, 0.065, 0.065}
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
