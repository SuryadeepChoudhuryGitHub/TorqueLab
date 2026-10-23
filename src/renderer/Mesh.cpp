#include "renderer/Mesh.h"
#include <cmath>
#include <utility>

namespace torquelab {

Mesh::~Mesh() {
    release();
}

Mesh::Mesh(Mesh&& other) noexcept
    : m_vao(std::exchange(other.m_vao, 0))
    , m_vbo(std::exchange(other.m_vbo, 0))
    , m_ebo(std::exchange(other.m_ebo, 0))
    , m_indexCount(std::exchange(other.m_indexCount, 0))
{}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        release();
        m_vao = std::exchange(other.m_vao, 0);
        m_vbo = std::exchange(other.m_vbo, 0);
        m_ebo = std::exchange(other.m_ebo, 0);
        m_indexCount = std::exchange(other.m_indexCount, 0);
    }
    return *this;
}

void Mesh::release() {
    if (m_ebo != 0) {
        glDeleteBuffers(1, &m_ebo);
        m_ebo = 0;
    }
    if (m_vbo != 0) {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }
    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
    m_indexCount = 0;
}

void Mesh::upload(const std::vector<VertexPNU>& vertices, const std::vector<unsigned int>& indices) {
    release();

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPNU)),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
        indices.data(),
        GL_STATIC_DRAW
    );

    // aPos (location = 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPNU), reinterpret_cast<void*>(0));

    // aNormal (location = 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPNU), reinterpret_cast<void*>(3 * sizeof(float)));

    // aTexCoord (location = 2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPNU), reinterpret_cast<void*>(6 * sizeof(float)));

    glBindVertexArray(0);
    m_indexCount = static_cast<GLsizei>(indices.size());
}

void Mesh::draw() const {
    if (m_vao == 0 || m_indexCount == 0) {
        return;
    }
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

Mesh Mesh::createBox(float width, float height, float depth) {
    const float hx = width * 0.5f;
    const float hy = height * 0.5f;
    const float hz = depth * 0.5f;

    const std::vector<VertexPNU> verts{
        // +Z Front face
        {-hx, -hy,  hz,  0,  0,  1,  0, 0},
        { hx, -hy,  hz,  0,  0,  1,  1, 0},
        { hx,  hy,  hz,  0,  0,  1,  1, 1},
        {-hx,  hy,  hz,  0,  0,  1,  0, 1},
        // -Z Back face
        { hx, -hy, -hz,  0,  0, -1,  0, 0},
        {-hx, -hy, -hz,  0,  0, -1,  1, 0},
        {-hx,  hy, -hz,  0,  0, -1,  1, 1},
        { hx,  hy, -hz,  0,  0, -1,  0, 1},
        // +Y Top face
        {-hx,  hy,  hz,  0,  1,  0,  0, 0},
        { hx,  hy,  hz,  0,  1,  0,  1, 0},
        { hx,  hy, -hz,  0,  1,  0,  1, 1},
        {-hx,  hy, -hz,  0,  1,  0,  0, 1},
        // -Y Bottom face
        {-hx, -hy, -hz,  0, -1,  0,  0, 0},
        { hx, -hy, -hz,  0, -1,  0,  1, 0},
        { hx, -hy,  hz,  0, -1,  0,  1, 1},
        {-hx, -hy,  hz,  0, -1,  0,  0, 1},
        // +X Left face
        { hx, -hy,  hz,  1,  0,  0,  0, 0},
        { hx, -hy, -hz,  1,  0,  0,  1, 0},
        { hx,  hy, -hz,  1,  0,  0,  1, 1},
        { hx,  hy,  hz,  1,  0,  0,  0, 1},
        // -X Right face
        {-hx, -hy, -hz, -1,  0,  0,  0, 0},
        {-hx, -hy,  hz, -1,  0,  0,  1, 0},
        {-hx,  hy,  hz, -1,  0,  0,  1, 1},
        {-hx,  hy, -hz, -1,  0,  0,  0, 1}
    };

    std::vector<unsigned int> indices;
    indices.reserve(36);
    for (unsigned int f = 0; f < 6; ++f) {
        const unsigned int base = f * 4;
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createCylinderY(float radiusTop, float radiusBottom, float height, int radialSegments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    const float halfH = height * 0.5f;
    const int segs = std::max(6, radialSegments);

    // Side barrel
    for (int i = 0; i <= segs; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segs);
        const float theta = u * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);

        verts.push_back(VertexPNU{radiusTop * ct, +halfH, radiusTop * st, ct, 0.0f, st, u, 1.0f});
        verts.push_back(VertexPNU{radiusBottom * ct, -halfH, radiusBottom * st, ct, 0.0f, st, u, 0.0f});
    }

    for (int i = 0; i < segs; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    // Top cap (+Y)
    const unsigned int topCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{0.0f, +halfH, 0.0f, 0.0f, 1.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        verts.push_back(VertexPNU{radiusTop * ct, +halfH, radiusTop * st, 0.0f, 1.0f, 0.0f, ct * 0.5f + 0.5f, st * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(topCenterIdx);
        indices.push_back(topCenterIdx + 1 + static_cast<unsigned int>(i + 1));
        indices.push_back(topCenterIdx + 1 + static_cast<unsigned int>(i));
    }

    // Bottom cap (-Y)
    const unsigned int botCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{0.0f, -halfH, 0.0f, 0.0f, -1.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        verts.push_back(VertexPNU{radiusBottom * ct, -halfH, radiusBottom * st, 0.0f, -1.0f, 0.0f, ct * 0.5f + 0.5f, st * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(botCenterIdx);
        indices.push_back(botCenterIdx + 1 + static_cast<unsigned int>(i));
        indices.push_back(botCenterIdx + 1 + static_cast<unsigned int>(i + 1));
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createCylinderX(float radiusTop, float radiusBottom, float width, int radialSegments) {
    // Cylinder aligned along local X axis (for tires, rims, and brake rotors)
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    const float halfW = width * 0.5f;
    const int segs = std::max(8, radialSegments);

    // Barrel around X axis
    for (int i = 0; i <= segs; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segs);
        const float theta = u * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);

        verts.push_back(VertexPNU{+halfW, radiusTop * cy, radiusTop * sz, 0.0f, cy, sz, u, 1.0f});
        verts.push_back(VertexPNU{-halfW, radiusBottom * cy, radiusBottom * sz, 0.0f, cy, sz, u, 0.0f});
    }

    for (int i = 0; i < segs; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    // +X cap
    const unsigned int posCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{+halfW, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);
        verts.push_back(VertexPNU{+halfW, radiusTop * cy, radiusTop * sz, 1.0f, 0.0f, 0.0f, cy * 0.5f + 0.5f, sz * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(posCenterIdx);
        indices.push_back(posCenterIdx + 1 + static_cast<unsigned int>(i));
        indices.push_back(posCenterIdx + 1 + static_cast<unsigned int>(i + 1));
    }

    // -X cap
    const unsigned int negCenterIdx = static_cast<unsigned int>(verts.size());
    verts.push_back(VertexPNU{-halfW, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.5f, 0.5f});
    for (int i = 0; i <= segs; ++i) {
        const float theta = (static_cast<float>(i) / static_cast<float>(segs)) * 2.0f * PI_F;
        const float cy = std::cos(theta);
        const float sz = std::sin(theta);
        verts.push_back(VertexPNU{-halfW, radiusBottom * cy, radiusBottom * sz, -1.0f, 0.0f, 0.0f, cy * 0.5f + 0.5f, sz * 0.5f + 0.5f});
    }
    for (int i = 0; i < segs; ++i) {
        indices.push_back(negCenterIdx);
        indices.push_back(negCenterIdx + 1 + static_cast<unsigned int>(i + 1));
        indices.push_back(negCenterIdx + 1 + static_cast<unsigned int>(i));
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createTorusZ(float radius, float tubeRadius, int radialSegments, int tubularSegments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int j = 0; j <= radialSegments; ++j) {
        const float v = static_cast<float>(j) / static_cast<float>(radialSegments) * 2.0f * PI_F;
        const float cv = std::cos(v);
        const float sv = std::sin(v);

        for (int i = 0; i <= tubularSegments; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(tubularSegments) * 2.0f * PI_F;
            const float cu = std::cos(u);
            const float su = std::sin(u);

            const float px = (radius + tubeRadius * cv) * cu;
            const float py = (radius + tubeRadius * cv) * su;
            const float pz = tubeRadius * sv;

            const float nx = cv * cu;
            const float ny = cv * su;
            const float nz = sv;

            verts.push_back(VertexPNU{
                px, py, pz,
                nx, ny, nz,
                static_cast<float>(i) / static_cast<float>(tubularSegments),
                static_cast<float>(j) / static_cast<float>(radialSegments)
            });
        }
    }

    for (int j = 1; j <= radialSegments; ++j) {
        for (int i = 1; i <= tubularSegments; ++i) {
            const unsigned int a = static_cast<unsigned int>((tubularSegments + 1) * j + i - 1);
            const unsigned int b = static_cast<unsigned int>((tubularSegments + 1) * (j - 1) + i - 1);
            const unsigned int c = static_cast<unsigned int>((tubularSegments + 1) * (j - 1) + i);
            const unsigned int d = static_cast<unsigned int>((tubularSegments + 1) * j + i);

            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(d);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createSphere(float radius, int rings, int sectors) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int r = 0; r <= rings; ++r) {
        const float v = static_cast<float>(r) / static_cast<float>(rings);
        const float phi = v * PI_F;
        const float sp = std::sin(phi);
        const float cp = std::cos(phi);

        for (int s = 0; s <= sectors; ++s) {
            const float u = static_cast<float>(s) / static_cast<float>(sectors);
            const float theta = u * 2.0f * PI_F;
            const float nx = sp * std::cos(theta);
            const float ny = cp;
            const float nz = sp * std::sin(theta);

            verts.push_back(VertexPNU{
                radius * nx, radius * ny, radius * nz,
                nx, ny, nz,
                u, v
            });
        }
    }

    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const unsigned int cur = static_cast<unsigned int>(r * (sectors + 1) + s);
            const unsigned int next = static_cast<unsigned int>((r + 1) * (sectors + 1) + s);

            indices.push_back(cur);
            indices.push_back(next);
            indices.push_back(cur + 1);

            indices.push_back(cur + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createPlaneXZ(float width, float depth) {
    const float hx = width * 0.5f;
    const float hz = depth * 0.5f;

    const std::vector<VertexPNU> verts{
        {-hx, 0.0f,  hz, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
        { hx, 0.0f,  hz, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f},
        { hx, 0.0f, -hz, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f},
        {-hx, 0.0f, -hz, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f}
    };
    const std::vector<unsigned int> indices{0, 1, 2, 0, 2, 3};

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

Mesh Mesh::createRingXZ(float innerRadius, float outerRadius, int segments) {
    std::vector<VertexPNU> verts;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= segments; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(segments);
        const float theta = u * 2.0f * PI_F;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);

        verts.push_back(VertexPNU{innerRadius * ct, 0.0f, innerRadius * st, 0.0f, 1.0f, 0.0f, u, 0.0f});
        verts.push_back(VertexPNU{outerRadius * ct, 0.0f, outerRadius * st, 0.0f, 1.0f, 0.0f, u, 1.0f});
    }

    for (int i = 0; i < segments; ++i) {
        const unsigned int b = static_cast<unsigned int>(i * 2);
        indices.push_back(b + 0);
        indices.push_back(b + 1);
        indices.push_back(b + 2);
        indices.push_back(b + 1);
        indices.push_back(b + 3);
        indices.push_back(b + 2);
    }

    Mesh mesh;
    mesh.upload(verts, indices);
    return mesh;
}

} // namespace torquelab
