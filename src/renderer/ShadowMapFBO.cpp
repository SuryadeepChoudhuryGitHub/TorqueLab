#include "renderer/ShadowMapFBO.h"

namespace torquelab {

ShadowMapFBO::~ShadowMapFBO() {
    release();
}

void ShadowMapFBO::release() {
    if (m_depthTex != 0) {
        glDeleteTextures(1, &m_depthTex);
        m_depthTex = 0;
    }
    if (m_fbo != 0) {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
}

bool ShadowMapFBO::init(int resolution) {
    release();
    m_resolution = resolution;

    glGenFramebuffers(1, &m_fbo);
    glGenTextures(1, &m_depthTex);

    glBindTexture(GL_TEXTURE_2D, m_depthTex);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_DEPTH_COMPONENT24,
        m_resolution,
        m_resolution,
        0,
        GL_DEPTH_COMPONENT,
        GL_FLOAT,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float borderColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthTex, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    return status == GL_FRAMEBUFFER_COMPLETE;
}

void ShadowMapFBO::resize(int resolution) {
    if (resolution != m_resolution && resolution > 0) {
        init(resolution);
    }
}

void ShadowMapFBO::beginPass() const {
    glViewport(0, 0, m_resolution, m_resolution);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void ShadowMapFBO::endPass(int viewportWidth, int viewportHeight) const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, viewportWidth, viewportHeight);
}

} // namespace torquelab
