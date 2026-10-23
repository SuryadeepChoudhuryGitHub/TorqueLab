#pragma once
/**
 * Directional Shadow Map Depth Framebuffer Object (FBO).
 */

#include <glad/glad.h>

namespace torquelab {

class ShadowMapFBO {
public:
    ShadowMapFBO() = default;
    ~ShadowMapFBO();

    ShadowMapFBO(const ShadowMapFBO&) = delete;
    ShadowMapFBO& operator=(const ShadowMapFBO&) = delete;

    bool init(int resolution = 2048);
    void resize(int resolution);

    void beginPass() const;
    void endPass(int viewportWidth, int viewportHeight) const;

    GLuint getDepthTexture() const { return m_depthTex; }
    int getResolution() const { return m_resolution; }

private:
    void release();

    GLuint m_fbo{0};
    GLuint m_depthTex{0};
    int m_resolution{2048};
};

} // namespace torquelab
