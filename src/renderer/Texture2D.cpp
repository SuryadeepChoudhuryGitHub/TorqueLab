#include "renderer/Texture2D.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifndef GL_SRGB8
#define GL_SRGB8 0x8C41
#endif
#ifndef GL_SRGB8_ALPHA8
#define GL_SRGB8_ALPHA8 0x8C43
#endif

namespace torquelab {

Texture2D::~Texture2D() {
    release();
}

Texture2D::Texture2D(Texture2D&& other) noexcept
    : m_textureId(std::exchange(other.m_textureId, 0))
    , m_width(std::exchange(other.m_width, 0))
    , m_height(std::exchange(other.m_height, 0))
{}

Texture2D& Texture2D::operator=(Texture2D&& other) noexcept {
    if (this != &other) {
        release();
        m_textureId = std::exchange(other.m_textureId, 0);
        m_width = std::exchange(other.m_width, 0);
        m_height = std::exchange(other.m_height, 0);
    }
    return *this;
}

void Texture2D::release() {
    if (m_textureId != 0) {
        glDeleteTextures(1, &m_textureId);
        m_textureId = 0;
        m_width = 0;
        m_height = 0;
    }
}

Texture2D Texture2D::createProceduralGrid(
    unsigned int bgHex,
    unsigned int lineHex,
    float noiseAlpha,
    bool stripeMode
) {
    constexpr int W = 256;
    constexpr int H = 256;
    std::vector<uint8_t> pixels(W * H * 3);

    const uint8_t bgR = static_cast<uint8_t>((bgHex >> 16) & 0xFF);
    const uint8_t bgG = static_cast<uint8_t>((bgHex >> 8) & 0xFF);
    const uint8_t bgB = static_cast<uint8_t>(bgHex & 0xFF);

    const uint8_t lnR = static_cast<uint8_t>((lineHex >> 16) & 0xFF);
    const uint8_t lnG = static_cast<uint8_t>((lineHex >> 8) & 0xFF);
    const uint8_t lnB = static_cast<uint8_t>(lineHex & 0xFF);

    for (int i = 0; i < W * H; ++i) {
        pixels[i * 3 + 0] = bgR;
        pixels[i * 3 + 1] = bgG;
        pixels[i * 3 + 2] = bgB;
    }

    auto setPixel = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        const size_t idx = static_cast<size_t>((y * W + x) * 3);
        pixels[idx + 0] = r;
        pixels[idx + 1] = g;
        pixels[idx + 2] = b;
    };

    if (stripeMode) {
        for (int y = 0; y < 128; ++y) {
            for (int x = 0; x < W; ++x) {
                setPixel(x, y, lnR, lnG, lnB);
            }
        }
    } else {
        for (int i = 0; i < W; ++i) {
            for (int t = 0; t < 2; ++t) {
                setPixel(i, t, lnR, lnG, lnB);
                setPixel(i, H - 1 - t, lnR, lnG, lnB);
                setPixel(t, i, lnR, lnG, lnB);
                setPixel(W - 1 - t, i, lnR, lnG, lnB);
            }
        }

        for (int i = 0; i < 900; ++i) {
            const int gx = (i * 73) % W;
            const int gy = (i * 137) % H;
            const bool bright = (i % 2 == 0);
            const float target = bright ? 255.0f : 0.0f;

            for (int dy = 0; dy < 3; ++dy) {
                for (int dx = 0; dx < 3; ++dx) {
                    const int px = (gx + dx) % W;
                    const int py = (gy + dy) % H;
                    const size_t idx = static_cast<size_t>((py * W + px) * 3);
                    pixels[idx + 0] = static_cast<uint8_t>(std::clamp(pixels[idx + 0] * (1.0f - noiseAlpha) + target * noiseAlpha, 0.0f, 255.0f));
                    pixels[idx + 1] = static_cast<uint8_t>(std::clamp(pixels[idx + 1] * (1.0f - noiseAlpha) + target * noiseAlpha, 0.0f, 255.0f));
                    pixels[idx + 2] = static_cast<uint8_t>(std::clamp(pixels[idx + 2] * (1.0f - noiseAlpha) + target * noiseAlpha, 0.0f, 255.0f));
                }
            }
        }
    }

    Texture2D tex;
    tex.m_width = W;
    tex.m_height = H;
    glGenTextures(1, &tex.m_textureId);
    glBindTexture(GL_TEXTURE_2D, tex.m_textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, W, H, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

Texture2D Texture2D::loadFromFile(const std::string& filePath, bool isSRGB) {
    int w = 0, h = 0, channels = 0;
    stbi_set_flip_vertically_on_load(false); // glTF UVs expect V=0 at top
    unsigned char* data = stbi_load(filePath.c_str(), &w, &h, &channels, 4);
    if (!data) {
        std::cerr << "[Texture2D] Warning: Failed to load image from " << filePath
                  << " (Reason: " << (stbi_failure_reason() ? stbi_failure_reason() : "unknown") << ")" << std::endl;
        return Texture2D{};
    }

    Texture2D tex;
    tex.m_width = w;
    tex.m_height = h;
    glGenTextures(1, &tex.m_textureId);
    glBindTexture(GL_TEXTURE_2D, tex.m_textureId);

    const GLint internalFormat = isSRGB ? GL_SRGB8_ALPHA8 : GL_RGBA8;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    std::cout << "[Texture2D] Loaded texture: " << filePath << " (" << w << "x" << h << ")" << std::endl;
    return tex;
}

Texture2D Texture2D::loadDecalComposite(
    const std::string& decalFilePath,
    const Color3& basePaintColor,
    bool isSRGB
) {
    int w = 0, h = 0, channels = 0;
    stbi_set_flip_vertically_on_load(false);
    unsigned char* data = stbi_load(decalFilePath.c_str(), &w, &h, &channels, 4);
    if (!data) {
        std::cerr << "[Texture2D] Warning: Failed to load decal image from " << decalFilePath << std::endl;
        return Texture2D{};
    }

    const float bgR = std::clamp(basePaintColor.r, 0.0f, 1.0f) * 255.0f;
    const float bgG = std::clamp(basePaintColor.g, 0.0f, 1.0f) * 255.0f;
    const float bgB = std::clamp(basePaintColor.b, 0.0f, 1.0f) * 255.0f;

    // Composite RGBA decal onto solid base paint background
    const size_t totalPixels = static_cast<size_t>(w * h);
    for (size_t i = 0; i < totalPixels; ++i) {
        const float a = static_cast<float>(data[i * 4 + 3]) / 255.0f;
        const float r = static_cast<float>(data[i * 4 + 0]);
        const float g = static_cast<float>(data[i * 4 + 1]);
        const float b = static_cast<float>(data[i * 4 + 2]);

        data[i * 4 + 0] = static_cast<unsigned char>(std::clamp(r * a + bgR * (1.0f - a), 0.0f, 255.0f));
        data[i * 4 + 1] = static_cast<unsigned char>(std::clamp(g * a + bgG * (1.0f - a), 0.0f, 255.0f));
        data[i * 4 + 2] = static_cast<unsigned char>(std::clamp(b * a + bgB * (1.0f - a), 0.0f, 255.0f));
        data[i * 4 + 3] = 255;
    }

    Texture2D tex;
    tex.m_width = w;
    tex.m_height = h;
    glGenTextures(1, &tex.m_textureId);
    glBindTexture(GL_TEXTURE_2D, tex.m_textureId);

    const GLint internalFormat = isSRGB ? GL_SRGB8 : GL_RGB8;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    std::cout << "[Texture2D] Composited decal texture: " << decalFilePath << " (" << w << "x" << h << ") over base paint." << std::endl;
    return tex;
}

Texture2D Texture2D::loadFromMemory(
    const unsigned char* buffer,
    int length,
    bool isSRGB
) {
    if (!buffer || length <= 0) return Texture2D{};

    int w = 0, h = 0, channels = 0;
    stbi_set_flip_vertically_on_load(false);
    unsigned char* data = stbi_load_from_memory(buffer, length, &w, &h, &channels, 4);
    if (!data) {
        std::cerr << "[Texture2D] Warning: Failed to decode embedded image buffer." << std::endl;
        return Texture2D{};
    }

    Texture2D tex;
    tex.m_width = w;
    tex.m_height = h;
    glGenTextures(1, &tex.m_textureId);
    glBindTexture(GL_TEXTURE_2D, tex.m_textureId);

    const GLint internalFormat = isSRGB ? GL_SRGB8_ALPHA8 : GL_RGBA8;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    return tex;
}

} // namespace torquelab
