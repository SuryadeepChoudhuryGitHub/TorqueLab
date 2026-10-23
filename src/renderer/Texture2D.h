#pragma once
/**
 * Procedural 2D OpenGL Texture Generator.
 *
 * Generates the exact 256x256 procedural surface grid, grain, and rumble-strip
 * textures used in TrackRenderer.ts without external image file dependencies.
 */

#include <glad/glad.h>

namespace torquelab {

class Texture2D {
public:
    Texture2D() = default;
    ~Texture2D();

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;

    Texture2D(Texture2D&& other) noexcept;
    Texture2D& operator=(Texture2D&& other) noexcept;

    GLuint getId() const { return m_textureId; }

    static Texture2D createProceduralGrid(
        unsigned int bgHex,
        unsigned int lineHex,
        float noiseAlpha,
        bool stripeMode = false
    );

private:
    void release();
    GLuint m_textureId{0};
};

} // namespace torquelab
