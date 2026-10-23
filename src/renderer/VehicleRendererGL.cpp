#include "renderer/VehicleRendererGL.h"
#include <glad/glad.h>
#include <cmath>

namespace torquelab {

void VehicleRendererGL::init(const VehicleConfig& config) {
    m_chassisParts.clear();
    m_wheelPartsLeft.clear();
    m_wheelPartsRight.clear();

    m_wheelRadius = static_cast<float>(config.wheelRadius);
    m_wheelWidth = static_cast<float>(config.wheelWidth);

    MaterialProps bodyPaintMat{};
    bodyPaintMat.baseColor = Color3::fromHex(0x1e66f5);
    bodyPaintMat.metalness = 0.55f;
    bodyPaintMat.roughness = 0.24f;

    m_darkTrimMat = MaterialProps{};
    m_darkTrimMat.baseColor = Color3::fromHex(0x181c24);
    m_darkTrimMat.metalness = 0.30f;
    m_darkTrimMat.roughness = 0.65f;

    MaterialProps glassMat{};
    glassMat.baseColor = Color3::fromHex(0x0f172a);
    glassMat.metalness = 0.85f;
    glassMat.roughness = 0.12f;
    glassMat.opacity = 0.78f;
    glassMat.castShadows = false;

    MaterialProps headlightMat{};
    headlightMat.baseColor = Color3::fromHex(0xffffff);
    headlightMat.emissiveColor = Color3::fromHex(0xdbeafe);
    headlightMat.emissiveIntensity = 1.4f;
    headlightMat.roughness = 0.15f;

    MaterialProps brakeLightMat{};
    brakeLightMat.baseColor = Color3::fromHex(0x660808);
    brakeLightMat.emissiveColor = Color3::fromHex(0x330000);
    brakeLightMat.emissiveIntensity = 0.35f;
    brakeLightMat.roughness = 0.25f;

    MaterialProps reverseLightMat{};
    reverseLightMat.baseColor = Color3::fromHex(0xcccccc);
    reverseLightMat.emissiveColor = Color3::fromHex(0xffffff);
    reverseLightMat.emissiveIntensity = 0.0f;
    reverseLightMat.roughness = 0.25f;

    const float w = static_cast<float>(config.chassisHalfExtents.x * 2.0); // 1.76 m
    const float l = static_cast<float>(config.chassisHalfExtents.z * 2.0); // 4.36 m

    auto addPart = [&](Mesh&& mesh, const Vec3& pos, double rotX, const MaterialProps& mat, bool isBrk = false, bool isRev = false, bool isTrans = false) {
        ChassisPart p{};
        p.mesh = std::move(mesh);
        p.localMatrix = Mat4::fromTRS(pos, quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, rotX));
        p.material = mat;
        p.isBrakeLight = isBrk;
        p.isReverseLight = isRev;
        p.isTransparent = isTrans;
        m_chassisParts.push_back(std::move(p));
    };

    // 1. Main lower body shell
    addPart(Mesh::createBox(w, 0.48f, l), Vec3{0.0, 0.24, 0.0}, 0.0, bodyPaintMat);

    // 2. Sculpted hood & front nose slope
    addPart(Mesh::createBox(w * 0.92f, 0.12f, 1.35f), Vec3{0.0, 0.49, 1.32}, 0.06, bodyPaintMat);

    // 3. Cabin greenhouse
    addPart(Mesh::createBox(w * 0.82f, 0.44f, 1.95f), Vec3{0.0, 0.66, -0.18}, 0.0, bodyPaintMat);

    // 4. Front aerodynamic splitter & grille
    addPart(Mesh::createBox(w * 1.01f, 0.06f, 0.35f), Vec3{0.0, 0.03, l * 0.5 - 0.12}, 0.0, m_darkTrimMat);
    addPart(Mesh::createBox(w * 0.62f, 0.22f, 0.08f), Vec3{0.0, 0.20, l * 0.5 - 0.01}, 0.0, m_darkTrimMat);

    // 5. Side skirts & rear diffuser
    addPart(Mesh::createBox(w * 1.02f, 0.08f, 2.10f), Vec3{0.0, 0.04, 0.0}, 0.0, m_darkTrimMat);
    addPart(Mesh::createBox(w * 0.96f, 0.12f, 0.25f), Vec3{0.0, 0.06, -l * 0.5 + 0.10}, 0.0, m_darkTrimMat);

    // 6. Rear trunk lip spoiler
    addPart(Mesh::createBox(w * 0.90f, 0.04f, 0.20f), Vec3{0.0, 0.60, -l * 0.5 + 0.14}, -0.14, m_darkTrimMat);

    // 7. Front Headlights (+Z front)
    addPart(Mesh::createBox(0.32f, 0.10f, 0.08f), Vec3{+w * 0.35, 0.34, l * 0.5 - 0.02}, 0.0, headlightMat);
    addPart(Mesh::createBox(0.32f, 0.10f, 0.08f), Vec3{-w * 0.35, 0.34, l * 0.5 - 0.02}, 0.0, headlightMat);

    // 8. Rear Brake Lights & Reverse Lights (-Z rear)
    addPart(Mesh::createBox(0.38f, 0.11f, 0.06f), Vec3{+w * 0.34, 0.36, -l * 0.5 + 0.01}, 0.0, brakeLightMat, true, false);
    addPart(Mesh::createBox(0.38f, 0.11f, 0.06f), Vec3{-w * 0.34, 0.36, -l * 0.5 + 0.01}, 0.0, brakeLightMat, true, false);
    addPart(Mesh::createBox(0.65f, 0.035f, 0.04f), Vec3{0.0, 0.52, -l * 0.5 + 0.02}, 0.0, brakeLightMat, true, false);

    addPart(Mesh::createBox(0.16f, 0.05f, 0.065f), Vec3{+w * 0.24, 0.30, -l * 0.5 + 0.01}, 0.0, reverseLightMat, false, true);
    addPart(Mesh::createBox(0.16f, 0.05f, 0.065f), Vec3{-w * 0.24, 0.30, -l * 0.5 + 0.01}, 0.0, reverseLightMat, false, true);

    // 9. Windshield, rear window, and side windows (rendered last with alpha blending)
    addPart(Mesh::createBox(w * 0.78f, 0.42f, 0.06f), Vec3{0.0, 0.65, 0.80}, -0.52, glassMat, false, false, true);
    addPart(Mesh::createBox(w * 0.78f, 0.42f, 0.06f), Vec3{0.0, 0.65, -1.16}, 0.56, glassMat, false, false, true);
    addPart(Mesh::createBox(w * 0.84f, 0.32f, 1.55f), Vec3{0.0, 0.67, -0.18}, 0.0, glassMat, false, false, true);

    // 10. Interior Steering Wheel
    m_steerRimMesh = Mesh::createTorusZ(0.16f, 0.022f, 8, 20);
    m_steerSpokeMesh = Mesh::createBox(0.30f, 0.025f, 0.02f);
    m_steerMarkerMesh = Mesh::createBox(0.03f, 0.05f, 0.05f);
    m_markerMat = MaterialProps{};
    m_markerMat.baseColor = Color3::fromHex(0xef4444);
    m_markerMat.roughness = 0.4f;

    // 11. Build Left & Right Wheel Assembly Templates
    MaterialProps tireMat{};
    tireMat.baseColor = Color3::fromHex(0x1a1d24);
    tireMat.roughness = 0.85f;
    tireMat.metalness = 0.08f;

    MaterialProps alloyMat{};
    alloyMat.baseColor = Color3::fromHex(0xd8dee9);
    alloyMat.metalness = 0.78f;
    alloyMat.roughness = 0.22f;

    MaterialProps rotorMat{};
    rotorMat.baseColor = Color3::fromHex(0x94a3b8);
    rotorMat.metalness = 0.85f;
    rotorMat.roughness = 0.30f;

    MaterialProps stripeMat{};
    stripeMat.baseColor = Color3::fromHex(0x38bdf8);
    stripeMat.roughness = 0.40f;

    m_caliperMat = MaterialProps{};
    m_caliperMat.baseColor = Color3::fromHex(0xef4444);
    m_caliperMat.metalness = 0.45f;
    m_caliperMat.roughness = 0.35f;
    m_caliperMesh = Mesh::createBox(0.065f, 0.14f, 0.09f);

    auto buildWheelSide = [&](std::vector<WheelSubPart>& outParts, bool isLeft) {
        // Tire cylinder along X
        outParts.push_back(WheelSubPart{
            Mesh::createCylinderX(m_wheelRadius, m_wheelRadius, m_wheelWidth, 28),
            Mat4::identity(),
            tireMat
        });

        // Rim barrel
        const float rimRadius = m_wheelRadius * 0.66f;
        outParts.push_back(WheelSubPart{
            Mesh::createCylinderX(rimRadius, rimRadius, m_wheelWidth * 1.01f, 20),
            Mat4::identity(),
            alloyMat
        });

        // 5-Spoke pattern + high-visibility rotation indicator spoke
        const double outboardX = isLeft ? (m_wheelWidth * 0.48) : (-m_wheelWidth * 0.48);
        constexpr int numSpokes = 5;
        for (int s = 0; s < numSpokes; ++s) {
            const double angle = (static_cast<double>(s) / static_cast<double>(numSpokes)) * 2.0 * PI_D;
            const Vec3 spokePos{
                outboardX,
                std::cos(angle) * (rimRadius * 0.46),
                std::sin(angle) * (rimRadius * 0.46)
            };
            const Quat spokeRot = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, -angle);
            outParts.push_back(WheelSubPart{
                Mesh::createBox(0.03f, rimRadius * 0.92f, 0.042f),
                Mat4::fromTRS(spokePos, spokeRot),
                (s == 0) ? stripeMat : alloyMat
            });
        }

        // Ventilated Brake Rotor
        const float rotorRadius = m_wheelRadius * 0.50f;
        const double rotorX = isLeft ? (-m_wheelWidth * 0.12) : (m_wheelWidth * 0.12);
        outParts.push_back(WheelSubPart{
            Mesh::createCylinderX(rotorRadius, rotorRadius, 0.026f, 20),
            Mat4::fromTRS(Vec3{rotorX, 0.0, 0.0}, quatIdentity()),
            rotorMat
        });
    };

    buildWheelSide(m_wheelPartsLeft, true);
    buildWheelSide(m_wheelPartsRight, false);

    // 12. Coilover Suspension Strut Mesh
    m_strutMesh = Mesh::createCylinderY(0.032f, 0.028f, 1.0f, 10);
    m_strutMat = MaterialProps{};
    m_strutMat.baseColor = Color3::fromHex(0xeab308);
    m_strutMat.metalness = 0.65f;
    m_strutMat.roughness = 0.35f;
}

void VehicleRendererGL::syncWithVehicle(const VehicleDynamics& vehicle, double alpha) {
    const InterpolatedVehicleTransforms interp = vehicle.getInterpolatedTransforms(alpha);

    m_chassisWorldMatrix = Mat4::fromTRS(interp.chassisPosition, interp.chassisRotation);
    m_steerWheelAngleZ = -vehicle.centerSteeringAngle * 14.0;

    m_isBraking = (vehicle.currentInputs.brake > 0.05 || vehicle.currentInputs.handbrake > 0.05);
    m_isReverse = (vehicle.drivetrain.gearIndex == 0);

    const float rotorRadius = m_wheelRadius * 0.50f;
    for (int i = 0; i < 4; ++i) {
        const Vec3& wPos = interp.wheelPositions[static_cast<size_t>(i)];
        const Quat& wRot = interp.wheelRotations[static_cast<size_t>(i)];
        const WheelState& ws = vehicle.wheels[static_cast<size_t>(i)];
        const bool isLeft = (i == 0 || i == 2);

        m_wheelWorldMatrices[static_cast<size_t>(i)] = Mat4::fromTRS(wPos, wRot);

        const Quat steerQuat = quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, ws.steeringAngle);
        const Quat knuckleQuat = quatMultiply(interp.chassisRotation, steerQuat);
        const Mat4 knuckleBase = Mat4::fromTRS(wPos, knuckleQuat);
        const Mat4 caliperLocal = Mat4::fromTRS(
            Vec3{isLeft ? (-m_wheelWidth * 0.12) : (m_wheelWidth * 0.12), 0.04, -rotorRadius * 0.72},
            quatIdentity()
        );
        m_knuckleWorldMatrices[static_cast<size_t>(i)] = Mat4::multiply(knuckleBase, caliperLocal);

        const Vec3& mount = ws.mountPointWorld;
        const Vec3 mid{
            0.5 * (mount.x + wPos.x),
            0.5 * (mount.y + wPos.y),
            0.5 * (mount.z + wPos.z)
        };
        const double strutLen = std::max(0.05, ws.suspensionLength);
        m_strutWorldMatrices[static_cast<size_t>(i)] = Mat4::fromTRS(
            mid,
            interp.chassisRotation,
            Vec3{1.0, strutLen, 1.0}
        );
    }
}

void VehicleRendererGL::renderShadowPass(Shader& depthShader) const {
    for (const auto& part : m_chassisParts) {
        if (!part.material.castShadows || part.isTransparent) continue;
        depthShader.setMat4("uModel", Mat4::multiply(m_chassisWorldMatrix, part.localMatrix));
        part.mesh.draw();
    }

    for (int i = 0; i < 4; ++i) {
        const auto& wheelParts = (i == 0 || i == 2) ? m_wheelPartsLeft : m_wheelPartsRight;
        for (const auto& wp : wheelParts) {
            depthShader.setMat4("uModel", Mat4::multiply(m_wheelWorldMatrices[static_cast<size_t>(i)], wp.localMatrix));
            wp.mesh.draw();
        }
    }
}

void VehicleRendererGL::renderMainPass(Shader& mainShader) const {
    // 1. Opaque chassis parts
    for (const auto& part : m_chassisParts) {
        if (part.isTransparent) continue;
        MaterialProps mat = part.material;
        if (part.isBrakeLight) {
            mat.baseColor = Color3::fromHex(m_isBraking ? 0xff1e1e : 0x660808);
            mat.emissiveColor = Color3::fromHex(m_isBraking ? 0xff1e1e : 0x330000);
            mat.emissiveIntensity = m_isBraking ? 3.2f : 0.35f;
        } else if (part.isReverseLight) {
            mat.emissiveIntensity = m_isReverse ? 2.5f : 0.0f;
        }
        mainShader.applyMaterial(mat);
        mainShader.setMat4("uModel", Mat4::multiply(m_chassisWorldMatrix, part.localMatrix));
        part.mesh.draw();
    }

    // 2. Interior steering wheel (visible through cabin glass)
    const Quat tiltQuat = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, 0.35);
    const Quat spinQuat = quatFromAxisAngle(Vec3{0.0, 0.0, 1.0}, m_steerWheelAngleZ);
    const Mat4 steerBase = Mat4::multiply(
        m_chassisWorldMatrix,
        Mat4::fromTRS(Vec3{0.36, 0.54, 0.45}, quatMultiply(tiltQuat, spinQuat))
    );
    mainShader.applyMaterial(m_darkTrimMat);
    mainShader.setMat4("uModel", steerBase);
    m_steerRimMesh.draw();
    m_steerSpokeMesh.draw();

    mainShader.applyMaterial(m_markerMat);
    mainShader.setMat4("uModel", Mat4::multiply(steerBase, Mat4::fromTRS(Vec3{0.0, 0.16, 0.0}, quatIdentity())));
    m_steerMarkerMesh.draw();

    // 3. Four independent wheels, brake calipers, and coilover struts
    for (int i = 0; i < 4; ++i) {
        const auto& wheelParts = (i == 0 || i == 2) ? m_wheelPartsLeft : m_wheelPartsRight;
        for (const auto& wp : wheelParts) {
            mainShader.applyMaterial(wp.material);
            mainShader.setMat4("uModel", Mat4::multiply(m_wheelWorldMatrices[static_cast<size_t>(i)], wp.localMatrix));
            wp.mesh.draw();
        }

        // Fixed performance brake caliper on knuckle
        mainShader.applyMaterial(m_caliperMat);
        mainShader.setMat4("uModel", m_knuckleWorldMatrices[static_cast<size_t>(i)]);
        m_caliperMesh.draw();

        // Visible yellow suspension strut
        mainShader.applyMaterial(m_strutMat);
        mainShader.setMat4("uModel", m_strutWorldMatrices[static_cast<size_t>(i)]);
        m_strutMesh.draw();
    }

    // 4. Transparent cabin glass
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const auto& part : m_chassisParts) {
        if (!part.isTransparent) continue;
        mainShader.applyMaterial(part.material);
        mainShader.setMat4("uModel", Mat4::multiply(m_chassisWorldMatrix, part.localMatrix));
        part.mesh.draw();
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

} // namespace torquelab
