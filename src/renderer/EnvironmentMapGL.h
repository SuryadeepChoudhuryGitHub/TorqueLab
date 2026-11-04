#pragma once
/**
 * Torque Lab Native — Automotive Environment Map & IBL System.
 *
 * Generates and manages high-quality automotive environment panoramic maps
 * for Image-Based Lighting (IBL):
 * - Equirectangular 2D HDR spherical environment mapping (GL_TEXTURE_2D)
 * - Atmospheric sky gradient and golden sun corona
 * - Automotive studio overhead softbox reflection strips
 * - Trilinear mipmap hierarchy for roughness-dependent specular reflections
 * - Diffuse irradiance approximation for ambient illumination
 */

#include <glad/glad.h>
#include <string>
#include <vector>
#include "physics/Math3D.h"
#include "renderer/Shader.h"

namespace torquelab {

class EnvironmentMapGL {
public:
    EnvironmentMapGL() = default;
    ~EnvironmentMapGL();

    EnvironmentMapGL(const EnvironmentMapGL&) = delete;
    EnvironmentMapGL& operator=(const EnvironmentMapGL&) = delete;

    EnvironmentMapGL(EnvironmentMapGL&& other) noexcept;
    EnvironmentMapGL& operator=(EnvironmentMapGL&& other) noexcept;

    /**
     * Initializes the automotive studio & sky panoramic texture with roughness-filtered mip levels.
     */
    void init(int width = 512, int height = 256);

    /**
     * Binds the environment map to the specified texture unit (default: GL_TEXTURE2).
     */
    void bind(int unit = 2) const;

    /**
     * Unbinds the environment map.
     */
    void unbind(int unit = 2) const;

    GLuint getTextureId() const { return m_textureId; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    float getMaxLod() const { return m_maxLod; }
    bool isValid() const { return m_textureId != 0; }

    void release();

private:
    GLuint m_textureId{0};
    int m_width{512};
    int m_height{256};
    float m_maxLod{6.0f};
};

} // namespace torquelab
