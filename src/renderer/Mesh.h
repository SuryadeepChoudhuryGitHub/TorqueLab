#pragma once
/**
 * OpenGL 3.3 Core Mesh VAO/VBO/EBO Wrapper & Procedural Geometry Generators.
 */

#include <vector>
#include <glad/glad.h>
#include "physics/Math3D.h"

namespace torquelab {

struct VertexPNU {
    float px, py, pz;
    float nx, ny, nz;
    float u, v;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload(const std::vector<VertexPNU>& vertices, const std::vector<unsigned int>& indices);
    void draw() const;

    static Mesh createBox(float width, float height, float depth);
    static Mesh createCylinderY(float radiusTop, float radiusBottom, float height, int radialSegments = 24);
    static Mesh createCylinderX(float radiusTop, float radiusBottom, float width, int radialSegments = 24);
    static Mesh createTorusZ(float radius, float tubeRadius, int radialSegments = 10, int tubularSegments = 24);
    static Mesh createSphere(float radius, int rings = 14, int sectors = 18);
    static Mesh createPlaneXZ(float width, float depth);
    static Mesh createRingXZ(float innerRadius, float outerRadius, int segments = 64);

private:
    void release();

    GLuint m_vao{0};
    GLuint m_vbo{0};
    GLuint m_ebo{0};
    GLsizei m_indexCount{0};
};

} // namespace torquelab
