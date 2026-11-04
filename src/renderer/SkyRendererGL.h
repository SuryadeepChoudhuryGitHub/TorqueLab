#pragma once
/**
 * Torque Lab Native — Procedural Atmospheric Sky Dome Renderer.
 *
 * Renders a natural blue daytime sky with Rayleigh atmospheric scattering,
 * Mie solar disc and aureole corona, seamless horizon haze, and anti-banding
 * dithering, fully integrated with the PBR lighting and reflection pipeline.
 */

#include <glad/glad.h>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"

namespace torquelab {

class SkyRendererGL {
public:
    SkyRendererGL() = default;
    ~SkyRendererGL() = default;

    SkyRendererGL(const SkyRendererGL&) = delete;
    SkyRendererGL& operator=(const SkyRendererGL&) = delete;

    /**
     * Initializes the sky dome sphere geometry and compiles atmospheric shaders.
     */
    bool init();

    /**
     * Renders the atmospheric sky dome at infinite depth (far clip plane).
     */
    void render(
        const Mat4& viewMatrix,
        const Mat4& projMatrix,
        const Vec3& sunDir,
        float exposure,
        int qualityPreset
    );

    /**
     * Atmospheric horizon haze color in sRGB space (for seamless distance fog and clear color).
     */
    Color3 getHorizonColor() const { return m_horizonColor; }

    /**
     * Atmospheric zenith deep azure color in linear space.
     */
    Color3 getZenithColor() const { return m_zenithColor; }

    /**
     * Atmospheric mid-sky azure color in linear space.
     */
    Color3 getMidSkyColor() const { return m_midSkyColor; }

    /**
     * Bounced ground haze color in linear space.
     */
    Color3 getGroundHazeColor() const { return m_groundHazeColor; }

private:
    Shader m_skyShader;
    Mesh m_skyMesh;
    bool m_initialized{false};

    // Physically calibrated daytime atmospheric color constants (linear space)
    Color3 m_zenithColor{0.04f, 0.16f, 0.65f};     // Deep rich sapphire blue
    Color3 m_midSkyColor{0.12f, 0.38f, 0.85f};     // Clear azure daylight sky
    Color3 m_horizonColor{0.38f, 0.62f, 0.94f};    // Luminous cerulean horizon blue
    Color3 m_groundHazeColor{0.15f, 0.22f, 0.16f}; // Natural earth ground bounce
};

} // namespace torquelab
