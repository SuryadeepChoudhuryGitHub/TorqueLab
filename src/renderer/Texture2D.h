#pragma once
/**
 * 2D OpenGL Texture Loader & Procedural Generator.
 *
 * Supports:
 * - Direct image loading (PNG, JPG, BMP) via stb_image
 * - Decal composition (compositing decal image with alpha over base paint color)
 * - Procedural surface grid, grain, and rumble-strip textures
 * - Full mipmapping with trilinear filtering and sRGB / linear color-space support
 */

#include <string>
#include <vector>
#include <glad/glad.h>
#include "renderer/Shader.h"

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
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    bool isValid() const { return m_textureId != 0; }

    static Texture2D createProceduralGrid(
        unsigned int bgHex,
        unsigned int lineHex,
        float noiseAlpha,
        bool stripeMode = false
    );

    static Texture2D loadFromFile(
        const std::string& filePath,
        bool isSRGB = true
    );

    static Texture2D loadDecalComposite(
        const std::string& decalFilePath,
        const Color3& basePaintColor,
        bool isSRGB = true
    );

    static Texture2D loadFromMemory(
        const unsigned char* buffer,
        int length,
        bool isSRGB = true
    );

private:
    void release();
    GLuint m_textureId{0};
    int m_width{0};
    int m_height{0};
};

} // namespace torquelab
