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

struct MaterialProps {
    Color3 baseColor{1.0f, 1.0f, 1.0f};
    Color3 emissiveColor{0.0f, 0.0f, 0.0f};
    float emissiveIntensity{0.0f};
    float roughness{0.5f};
    float metalness{0.1f};
    float opacity{1.0f};
    GLuint textureId{0};
    float uvScaleX{1.0f};
    float uvScaleY{1.0f};
    bool receiveShadows{true};
    bool castShadows{true};
    bool unlit{false};
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
