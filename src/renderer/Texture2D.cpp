#include "renderer/Texture2D.h"
#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace torquelab {

Texture2D::~Texture2D() {
    release();
}

Texture2D::Texture2D(Texture2D&& other) noexcept
    : m_textureId(std::exchange(other.m_textureId, 0))
{}

Texture2D& Texture2D::operator=(Texture2D&& other) noexcept {
    if (this != &other) {
        release();
        m_textureId = std::exchange(other.m_textureId, 0);
    }
    return *this;
}

void Texture2D::release() {
    if (m_textureId != 0) {
        glDeleteTextures(1, &m_textureId);
        m_textureId = 0;
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

    // Fill background
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
        // 2-pixel border reference grid (matching ctx.strokeRect(0, 0, 256, 256))
        for (int i = 0; i < W; ++i) {
            for (int t = 0; t < 2; ++t) {
                setPixel(i, t, lnR, lnG, lnB);
                setPixel(i, H - 1 - t, lnR, lnG, lnB);
                setPixel(t, i, lnR, lnG, lnB);
                setPixel(W - 1 - t, i, lnR, lnG, lnB);
            }
        }

        // Procedural grain (900 3x3 patches matching TrackRenderer.ts)
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
    glGenTextures(1, &tex.m_textureId);
    glBindTexture(GL_TEXTURE_2D, tex.m_textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, W, H, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

} // namespace torquelab
