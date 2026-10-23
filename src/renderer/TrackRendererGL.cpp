#include "renderer/TrackRendererGL.h"

namespace torquelab {

void TrackRendererGL::init(const std::vector<TrackBoxElement>& trackElements) {
    m_items.clear();
    createProceduralSurfaceMaterials();
    buildTrackMeshes(trackElements);
    buildReferenceMarkers();
    updateSunTarget(Vec3{0.0, 0.0, 0.0});
}

void TrackRendererGL::createProceduralSurfaceMaterials() {
    auto setupMat = [&](SurfaceType type, unsigned int bgHex, unsigned int lineHex, float noiseAlpha, float repX, float repY, float rough, float metal, bool stripe = false) {
        const size_t idx = static_cast<size_t>(type);
        m_surfaceTextures[idx] = Texture2D::createProceduralGrid(bgHex, lineHex, noiseAlpha, stripe);

        MaterialProps mat{};
        mat.baseColor = Color3{1.0f, 1.0f, 1.0f};
        mat.textureId = m_surfaceTextures[idx].getId();
        mat.uvScaleX = repX;
        mat.uvScaleY = repY;
        mat.roughness = rough;
        mat.metalness = metal;
        mat.receiveShadows = true;
        mat.castShadows = false;
        m_surfaceMaterials[idx] = mat;
    };

    setupMat(SurfaceType::DryAsphalt,  0x262b36, 0x323947, 0.05f, 45.0f, 85.0f, 0.78f, 0.08f, false);
    setupMat(SurfaceType::WetAsphalt,  0x18283e, 0x243b5a, 0.04f,  8.0f, 30.0f, 0.16f, 0.45f, false);
    setupMat(SurfaceType::Gravel,      0x7c5836, 0x926841, 0.14f, 10.0f, 35.0f, 0.94f, 0.02f, false);
    setupMat(SurfaceType::Ice,         0x7dd3fc, 0xbae6fd, 0.04f,  8.0f, 30.0f, 0.08f, 0.35f, false);
    setupMat(SurfaceType::Grass,       0x1b3b22, 0x22492a, 0.08f, 90.0f, 90.0f, 0.92f, 0.02f, false);
    setupMat(SurfaceType::RumbleStrip, 0xdc2626, 0xf8fafc, 0.02f,  1.0f, 25.0f, 0.65f, 0.05f, true);
}

void TrackRendererGL::buildTrackMeshes(const std::vector<TrackBoxElement>& elements) {
    for (const auto& el : elements) {
        RenderableTrackItem item{};
        item.mesh = Mesh::createBox(
            static_cast<float>(el.halfExtents.x * 2.0),
            static_cast<float>(el.halfExtents.y * 2.0),
            static_cast<float>(el.halfExtents.z * 2.0)
        );
        item.modelMatrix = Mat4::fromTRS(el.position, el.rotation);

        if (el.hasColorOverride) {
            MaterialProps mat{};
            mat.baseColor = Color3::fromHex(el.colorOverride);
            mat.roughness = (el.kind == TrackElementKind::Obstacle) ? 0.55f : 0.45f;
            mat.metalness = 0.15f;
            mat.receiveShadows = true;
            mat.castShadows = (el.kind == TrackElementKind::Obstacle ||
                               el.kind == TrackElementKind::Ramp ||
                               el.kind == TrackElementKind::Bump);
            item.material = mat;
        } else {
            item.material = m_surfaceMaterials[static_cast<size_t>(el.surfaceType)];
            item.material.castShadows = (el.kind == TrackElementKind::Obstacle ||
                                         el.kind == TrackElementKind::Ramp ||
                                         el.kind == TrackElementKind::Bump);
        }

        m_items.push_back(std::move(item));
    }
}

void TrackRendererGL::buildReferenceMarkers() {
    MaterialProps whiteLineMat{};
    whiteLineMat.baseColor = Color3::fromHex(0xe2e8f0);
    whiteLineMat.unlit = true;
    whiteLineMat.castShadows = false;
    whiteLineMat.receiveShadows = false;

    MaterialProps yellowLineMat{};
    yellowLineMat.baseColor = Color3::fromHex(0xfacc15);
    yellowLineMat.unlit = true;
    yellowLineMat.castShadows = false;
    yellowLineMat.receiveShadows = false;

    // 1. Start line at z = 2.5 m
    {
        RenderableTrackItem startLine{};
        startLine.mesh = Mesh::createPlaneXZ(18.0f, 0.8f);
        startLine.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.014, 2.5}, quatIdentity());
        startLine.material = whiteLineMat;
        m_items.push_back(std::move(startLine));
    }

    // 2. Center dashed lines along straightaway
    for (int z = -20; z <= 190; z += 10) {
        RenderableTrackItem dash{};
        dash.mesh = Mesh::createPlaneXZ(0.22f, 4.0f);
        dash.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.013, static_cast<double>(z)}, quatIdentity());
        dash.material = yellowLineMat;
        m_items.push_back(std::move(dash));
    }

    // 3. Distance marker posts every 25 m along main straight
    for (int dist = 25; dist <= 200; dist += 25) {
        const bool isMajor = (dist % 50 == 0);
        MaterialProps postMat{};
        postMat.baseColor = Color3::fromHex(isMajor ? 0x38bdf8 : 0x94a3b8);
        postMat.roughness = 0.4f;
        postMat.metalness = 0.15f;
        postMat.castShadows = true;
        postMat.receiveShadows = true;

        RenderableTrackItem postL{};
        postL.mesh = Mesh::createBox(0.35f, 1.8f, 0.35f);
        postL.modelMatrix = Mat4::fromTRS(Vec3{+12.2, 0.9, static_cast<double>(dist)}, quatIdentity());
        postL.material = postMat;
        m_items.push_back(std::move(postL));

        RenderableTrackItem postR{};
        postR.mesh = Mesh::createBox(0.35f, 1.8f, 0.35f);
        postR.modelMatrix = Mat4::fromTRS(Vec3{-12.2, 0.9, static_cast<double>(dist)}, quatIdentity());
        postR.material = postMat;
        m_items.push_back(std::move(postR));
    }

    // 4. Handling Circle reference ring on the north apron (x = 0, z = 240, radius = 28m)
    {
        RenderableTrackItem ring{};
        ring.mesh = Mesh::createRingXZ(27.6f, 28.2f, 64);
        ring.modelMatrix = Mat4::fromTRS(Vec3{0.0, 0.014, 240.0}, quatIdentity());
        ring.material = yellowLineMat;
        m_items.push_back(std::move(ring));
    }
}

void TrackRendererGL::updateSunTarget(const Vec3& vehiclePos) {
    const Vec3 sunOffset{45.0, 75.0, 35.0};
    m_sunDirection = vec3Normalize(sunOffset);

    const Vec3 lightEye = vec3Add(vehiclePos, sunOffset);
    const Mat4 lightView = Mat4::lookAt(lightEye, vehiclePos, Vec3{0.0, 1.0, 0.0});
    const Mat4 lightProj = Mat4::ortho(-95.0f, 95.0f, -95.0f, 95.0f, 10.0f, 280.0f);
    m_lightSpaceMatrix = Mat4::multiply(lightProj, lightView);
}

void TrackRendererGL::setQuality(GraphicsQuality quality, ShadowMapFBO& shadowFBO) {
    m_quality = quality;
    if (quality == GraphicsQuality::High) {
        shadowFBO.resize(2048);
    } else if (quality == GraphicsQuality::Medium) {
        shadowFBO.resize(1024);
    }
}

void TrackRendererGL::renderShadowPass(Shader& depthShader) const {
    for (const auto& item : m_items) {
        if (!item.material.castShadows) continue;
        depthShader.setMat4("uModel", item.modelMatrix);
        item.mesh.draw();
    }
}

void TrackRendererGL::renderMainPass(Shader& mainShader) const {
    for (const auto& item : m_items) {
        mainShader.applyMaterial(item.material);
        mainShader.setMat4("uModel", item.modelMatrix);
        item.mesh.draw();
    }
}

} // namespace torquelab
