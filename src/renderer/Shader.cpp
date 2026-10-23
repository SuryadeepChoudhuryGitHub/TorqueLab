#include "renderer/Shader.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace torquelab {

namespace {

std::string resolveShaderPath(const std::string& relPath) {
    namespace fs = std::filesystem;
    const std::vector<fs::path> prefixes{
        fs::current_path(),
        fs::current_path() / "..",
        fs::current_path() / ".." / ".."
    };
    for (const auto& p : prefixes) {
        const fs::path candidate = p / relPath;
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }
    return relPath;
}

std::string readTextFile(const std::string& path) {
    std::ifstream file(path, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

GLuint compileSingleShader(GLenum type, const std::string& src, const std::string& debugLabel) {
    const GLuint shader = glCreateShader(type);
    const char* cStr = src.c_str();
    glShaderSource(shader, 1, &cStr, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLint logLen = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLen);
        std::string infoLog(static_cast<size_t>(std::max(1, logLen)), '\0');
        glGetShaderInfoLog(shader, logLen, nullptr, infoLog.data());
        std::cerr << "[Shader Compile Error] (" << debugLabel << "):\n" << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

Shader::~Shader() {
    if (m_program != 0) {
        glDeleteProgram(m_program);
        m_program = 0;
    }
}

bool Shader::loadFromFiles(const std::string& vertRelPath, const std::string& fragRelPath) {
    const std::string vertFull = resolveShaderPath(vertRelPath);
    const std::string fragFull = resolveShaderPath(fragRelPath);
    const std::string vertSrc = readTextFile(vertFull);
    const std::string fragSrc = readTextFile(fragFull);

    if (vertSrc.empty() || fragSrc.empty()) {
        std::cerr << "[Shader Error] Could not read shader files: "
                  << vertFull << " or " << fragFull << std::endl;
        return false;
    }
    return loadFromSource(vertSrc, fragSrc);
}

bool Shader::loadFromSource(const std::string& vertSrc, const std::string& fragSrc) {
    const GLuint vs = compileSingleShader(GL_VERTEX_SHADER, vertSrc, "VERTEX");
    const GLuint fs = compileSingleShader(GL_FRAGMENT_SHADER, fragSrc, "FRAGMENT");
    if (vs == 0 || fs == 0) {
        if (vs != 0) glDeleteShader(vs);
        if (fs != 0) glDeleteShader(fs);
        return false;
    }

    const GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint linked = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint logLen = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLen);
        std::string infoLog(static_cast<size_t>(std::max(1, logLen)), '\0');
        glGetProgramInfoLog(prog, logLen, nullptr, infoLog.data());
        std::cerr << "[Shader Link Error]:\n" << infoLog << std::endl;
        glDeleteProgram(prog);
        return false;
    }

    if (m_program != 0) {
        glDeleteProgram(m_program);
    }
    m_program = prog;
    m_uniformCache.clear();
    return true;
}

void Shader::use() const {
    glUseProgram(m_program);
}

GLint Shader::getUniformLocation(const std::string& name) {
    const auto it = m_uniformCache.find(name);
    if (it != m_uniformCache.end()) {
        return it->second;
    }
    const GLint loc = glGetUniformLocation(m_program, name.c_str());
    m_uniformCache[name] = loc;
    return loc;
}

void Shader::setInt(const std::string& name, int value) {
    const GLint loc = getUniformLocation(name);
    if (loc >= 0) glUniform1i(loc, value);
}

void Shader::setFloat(const std::string& name, float value) {
    const GLint loc = getUniformLocation(name);
    if (loc >= 0) glUniform1f(loc, value);
}

void Shader::setVec2(const std::string& name, float x, float y) {
    const GLint loc = getUniformLocation(name);
    if (loc >= 0) glUniform2f(loc, x, y);
}

void Shader::setVec3(const std::string& name, float x, float y, float z) {
    const GLint loc = getUniformLocation(name);
    if (loc >= 0) glUniform3f(loc, x, y, z);
}

void Shader::setVec3(const std::string& name, const Vec3& v) {
    setVec3(name, static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

void Shader::setColor3(const std::string& name, const Color3& c) {
    setVec3(name, c.r, c.g, c.b);
}

void Shader::setMat4(const std::string& name, const Mat4& mat) {
    const GLint loc = getUniformLocation(name);
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, mat.data());
}

void Shader::applyMaterial(const MaterialProps& mat) {
    setColor3("uBaseColor", mat.baseColor);
    setColor3("uEmissiveColor", mat.emissiveColor);
    setFloat("uEmissiveIntensity", mat.emissiveIntensity);
    setFloat("uRoughness", mat.roughness);
    setFloat("uMetalness", mat.metalness);
    setFloat("uOpacity", mat.opacity);
    setInt("uReceiveShadows", mat.receiveShadows ? 1 : 0);
    setInt("uUnlit", mat.unlit ? 1 : 0);

    if (mat.textureId != 0) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mat.textureId);
        setInt("uAlbedoMap", 0);
        setInt("uUseTexture", 1);
        setVec2("uUvScale", mat.uvScaleX, mat.uvScaleY);
    } else {
        setInt("uUseTexture", 0);
        setVec2("uUvScale", 1.0f, 1.0f);
    }
}

} // namespace torquelab
