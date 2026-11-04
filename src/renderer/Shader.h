#pragma once
/**
 * OpenGL 3.3 Core GLSL Shader Program Wrapper.
 */

#include <string>
#include <unordered_map>
#include <glad/glad.h>
#include "physics/Math3D.h"

namespace torquelab {

struct Color3 {
    float r{1.0f};
    float g{1.0f};
    float b{1.0f};

    static constexpr Color3 fromHex(unsigned int hex) {
        return Color3{
            static_cast<float>((hex >> 16) & 0xFF) / 255.0f,
            static_cast<float>((hex >> 8) & 0xFF) / 255.0f,
            static_cast<float>(hex & 0xFF) / 255.0f
        };
    }
};

enum class AlphaMode {
    Opaque,
    Mask,
    Blend
};

struct MaterialProps {
    Color3 baseColor{1.0f, 1.0f, 1.0f};
    float roughness{0.5f};
    float metalness{0.0f};
    float opacity{1.0f};

    Color3 emissiveColor{0.0f, 0.0f, 0.0f};
    float emissiveIntensity{0.0f};

    // glTF 2.0 PBR Texture Maps (0 = none)
    GLuint textureId{0};               // Albedo / Base Color map (sRGB)
    GLuint metallicRoughnessMapId{0};  // Metallic (Blue) + Roughness (Green) map (Linear)
    GLuint normalMapId{0};             // Tangent-space normal map (Linear)
    float normalScale{1.0f};           // Normal map bump intensity
    GLuint occlusionMapId{0};          // Ambient occlusion map (Red channel) (Linear)
    float occlusionStrength{1.0f};     // Ambient occlusion strength
    GLuint emissiveMapId{0};           // Emissive color map (sRGB)

    float uvScaleX{1.0f};
    float uvScaleY{1.0f};

    AlphaMode alphaMode{AlphaMode::Opaque};
    float alphaCutoff{0.5f};
    bool doubleSided{false};
    bool receiveShadows{true};
    bool castShadows{true};
    bool unlit{false};

    std::string name{"DefaultMaterial"};
};

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    bool loadFromFiles(const std::string& vertRelPath, const std::string& fragRelPath);
    bool loadFromSource(const std::string& vertSrc, const std::string& fragSrc);

    void use() const;
    GLuint getProgramId() const { return m_program; }

    void setInt(const std::string& name, int value);
    void setFloat(const std::string& name, float value);
    void setVec2(const std::string& name, float x, float y);
    void setVec3(const std::string& name, float x, float y, float z);
    void setVec3(const std::string& name, const Vec3& v);
    void setColor3(const std::string& name, const Color3& c);
    void setMat4(const std::string& name, const Mat4& mat);
    void applyMaterial(const MaterialProps& mat);

private:
    GLint getUniformLocation(const std::string& name);

    GLuint m_program{0};
    std::unordered_map<std::string, GLint> m_uniformCache;
};

} // namespace torquelab
