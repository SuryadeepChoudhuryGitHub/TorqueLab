#include "renderer/SkyRendererGL.h"
#include <iostream>

namespace torquelab {

bool SkyRendererGL::init() {
    if (!m_skyShader.loadFromFiles("shaders/sky.vert", "shaders/sky.frag")) {
        std::cerr << "[SkyRendererGL] Error: Failed to compile atmospheric sky shaders (shaders/sky.vert, shaders/sky.frag)." << std::endl;
        return false;
    }

    // High-tessellation unit sphere dome (24 rings, 32 sectors)
    m_skyMesh = Mesh::createSphere(1.0f, 24, 32);
    m_initialized = true;

    std::cout << "[SkyRendererGL] Initialized atmospheric sky dome renderer (Rayleigh/Mie daytime model)." << std::endl;
    return true;
}

void SkyRendererGL::render(
    const Mat4& viewMatrix,
    const Mat4& projMatrix,
    const Vec3& sunDir,
    float exposure,
    int qualityPreset
) {
    if (!m_initialized) return;

    // Render sky dome at infinite depth (far plane) without writing to depth buffer
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);

    m_skyShader.use();
    m_skyShader.setMat4("uView", viewMatrix);
    m_skyShader.setMat4("uProj", projMatrix);
    m_skyShader.setVec3("uSunDir", sunDir);
    m_skyShader.setVec3("uSunColor", 1.9f, 1.86f, 1.75f);
    m_skyShader.setColor3("uZenithColor", m_zenithColor);
    m_skyShader.setColor3("uMidSkyColor", m_midSkyColor);
    m_skyShader.setColor3("uHorizonColor", m_horizonColor);
    m_skyShader.setColor3("uGroundHazeColor", m_groundHazeColor);
    m_skyShader.setFloat("uExposure", exposure);
    m_skyShader.setInt("uQualityPreset", qualityPreset);

    m_skyMesh.draw();

    // Restore standard 3D depth and culling state for scene rendering
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
}

} // namespace torquelab
