#include "renderer/EnvironmentMapGL.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace torquelab {

EnvironmentMapGL::~EnvironmentMapGL() {
    release();
}

EnvironmentMapGL::EnvironmentMapGL(EnvironmentMapGL&& other) noexcept
    : m_textureId(std::exchange(other.m_textureId, 0))
    , m_width(std::exchange(other.m_width, 512))
    , m_height(std::exchange(other.m_height, 256))
    , m_maxLod(std::exchange(other.m_maxLod, 6.0f))
{}

EnvironmentMapGL& EnvironmentMapGL::operator=(EnvironmentMapGL&& other) noexcept {
    if (this != &other) {
        release();
        m_textureId = std::exchange(other.m_textureId, 0);
        m_width = std::exchange(other.m_width, 512);
        m_height = std::exchange(other.m_height, 256);
        m_maxLod = std::exchange(other.m_maxLod, 6.0f);
    }
    return *this;
}

void EnvironmentMapGL::release() {
    if (m_textureId != 0) {
        glDeleteTextures(1, &m_textureId);
        m_textureId = 0;
    }
}

namespace {

Color3 evaluateRadiance(const Vec3& d) {
    Color3 col{0.0f, 0.0f, 0.0f};
    const float dy = static_cast<float>(d.y);

    // 1. Atmosphere & Ground Gradient (Calibrated to match atmospheric sky model)
    if (dy >= 0.0f) {
        const float t = std::clamp(dy, 0.0f, 1.0f);
        const Color3 horizon{0.38f, 0.62f, 0.94f};
        const Color3 midSky{0.14f, 0.40f, 0.86f};
        const Color3 zenith{0.05f, 0.18f, 0.68f};
        if (t < 0.25f) {
            const float k = t / 0.25f;
            col.r = horizon.r * (1.0f - k) + midSky.r * k;
            col.g = horizon.g * (1.0f - k) + midSky.g * k;
            col.b = horizon.b * (1.0f - k) + midSky.b * k;
        } else {
            const float k = (t - 0.25f) / 0.75f;
            col.r = midSky.r * (1.0f - k) + zenith.r * k;
            col.g = midSky.g * (1.0f - k) + zenith.g * k;
            col.b = midSky.b * (1.0f - k) + zenith.b * k;
        }
    } else {
        const float t = std::clamp(-dy * 2.5f, 0.0f, 1.0f);
        const Color3 horizonGround{0.22f, 0.30f, 0.24f};
        const Color3 darkGround{0.08f, 0.11f, 0.09f};
        col.r = horizonGround.r * (1.0f - t) + darkGround.r * t;
        col.g = horizonGround.g * (1.0f - t) + darkGround.g * t;
        col.b = horizonGround.b * (1.0f - t) + darkGround.b * t;
    }

    // 2. Golden Sun Disc & Specular Corona
    const Vec3 sunDir = vec3Normalize(Vec3{45.0, 75.0, 35.0});
    const double cosTheta = vec3Dot(d, sunDir);
    if (cosTheta > 0.0) {
        const float glow = static_cast<float>(std::pow(cosTheta, 18.0) * 1.6);
        const float disc = static_cast<float>(std::pow(cosTheta, 256.0) * 14.0);
        col.r += (disc + glow) * 1.0f;
        col.g += (disc + glow) * 0.94f;
        col.b += (disc + glow) * 0.80f;
    }

    // 3. Automotive Studio Reflection Softbox Strips
    // Overhead light banks that produce crisp specular reflection lines across car curvature
    if (dy > 0.20f) {
        const float dx = static_cast<float>(d.x);
        const float dz = static_cast<float>(d.z);

        // Longitudinal Left Light Bank
        const float distL = std::abs(dx - 0.38f);
        if (distL < 0.13f && std::abs(dz) < 0.85f) {
            const float edgeFactor = (1.0f - distL / 0.13f);
            col.r += 2.6f * edgeFactor;
            col.g += 2.7f * edgeFactor;
            col.b += 3.0f * edgeFactor;
        }

        // Longitudinal Right Light Bank
        const float distR = std::abs(dx + 0.38f);
        if (distR < 0.13f && std::abs(dz) < 0.85f) {
            const float edgeFactor = (1.0f - distR / 0.13f);
            col.r += 2.6f * edgeFactor;
            col.g += 2.7f * edgeFactor;
            col.b += 3.0f * edgeFactor;
        }

        // Central Overhead Light Strip
        if (dy > 0.80f && std::abs(dx) < 0.72f && std::abs(dz) < 0.16f) {
            const float edgeZ = (1.0f - std::abs(dz) / 0.16f);
            col.r += 2.0f * edgeZ;
            col.g += 2.1f * edgeZ;
            col.b += 2.4f * edgeZ;
        }
    }

    return col;
}

} // namespace

void EnvironmentMapGL::init(int width, int height) {
    release();
    m_width = width;
    m_height = height;

    constexpr double PI = 3.14159265358979323846;

    std::vector<uint8_t> pixels(static_cast<size_t>(m_width * m_height * 4));

    for (int y = 0; y < m_height; ++y) {
        // v = 0 is bottom (ground, -Y), v = 1 is top (zenith, +Y)
        const double v = (static_cast<double>(y) + 0.5) / static_cast<double>(m_height);
        const double theta = (v - 0.5) * PI; // -PI/2 to +PI/2
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);

        for (int x = 0; x < m_width; ++x) {
            const double u = (static_cast<double>(x) + 0.5) / static_cast<double>(m_width);
            const double phi = (u - 0.5) * 2.0 * PI; // -PI to +PI
            const double sinPhi = std::sin(phi);
            const double cosPhi = std::cos(phi);

            // Normalized Cartesian direction in right-handed chassis/world space
            const Vec3 dir{
                cosTheta * sinPhi,
                sinTheta,
                cosTheta * cosPhi
            };

            const Color3 rad = evaluateRadiance(dir);

            const size_t pIdx = static_cast<size_t>((y * m_width + x) * 4);
            pixels[pIdx + 0] = static_cast<uint8_t>(std::clamp(rad.r * 255.0f, 0.0f, 255.0f));
            pixels[pIdx + 1] = static_cast<uint8_t>(std::clamp(rad.g * 255.0f, 0.0f, 255.0f));
            pixels[pIdx + 2] = static_cast<uint8_t>(std::clamp(rad.b * 255.0f, 0.0f, 255.0f));
            pixels[pIdx + 3] = 255;
        }
    }

    glGenTextures(1, &m_textureId);
    glBindTexture(GL_TEXTURE_2D, m_textureId);

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA8,
        m_width,
        m_height,
        0,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        pixels.data()
    );

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    // Compute max mip level: log2(min(width, height))
    m_maxLod = static_cast<float>(std::floor(std::log2(std::min(m_width, m_height))));

    std::cout << "[EnvironmentMapGL] Initialized automotive panoramic IBL texture (" << m_width << "x"
              << m_height << ", maxLod=" << m_maxLod << ")." << std::endl;
}

void EnvironmentMapGL::bind(int unit) const {
    if (m_textureId != 0) {
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, m_textureId);
    }
}

void EnvironmentMapGL::unbind(int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, 0);
}

} // namespace torquelab
