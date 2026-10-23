#pragma once
/**
 * OpenGL 3.3 Core Proving Ground Track & Environment Renderer.
 *
 * Renders all physical track elements from PhysicsWorld::trackElements with
 * procedural 256x256 textures, reference lane markings, distance posts,
 * North Handling Circle ring, and dynamic directional sun shadow mapping.
 */

#include <array>
#include <vector>
#include "physics/Math3D.h"
#include "physics/PhysicsWorld.h"
#include "physics/SurfaceSystem.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"
#include "renderer/ShadowMapFBO.h"
#include "renderer/Texture2D.h"

namespace torquelab {

enum class GraphicsQuality {
    High = 0,
    Medium = 1,
    Low = 2
};

class TrackRendererGL {
public:
    TrackRendererGL() = default;

    void init(const std::vector<TrackBoxElement>& trackElements);
    void updateSunTarget(const Vec3& vehiclePos);
    void setQuality(GraphicsQuality quality, ShadowMapFBO& shadowFBO);
    GraphicsQuality getQuality() const { return m_quality; }

    Mat4 getLightSpaceMatrix() const { return m_lightSpaceMatrix; }
    Vec3 getSunDirection() const { return m_sunDirection; }
    bool areShadowsEnabled() const { return m_quality != GraphicsQuality::Low; }

    void renderShadowPass(Shader& depthShader) const;
    void renderMainPass(Shader& mainShader) const;

private:
    struct RenderableTrackItem {
        Mesh mesh;
        Mat4 modelMatrix;
        MaterialProps material;
    };

    void createProceduralSurfaceMaterials();
    void buildTrackMeshes(const std::vector<TrackBoxElement>& elements);
    void buildReferenceMarkers();

    std::array<Texture2D, 6> m_surfaceTextures;
    std::array<MaterialProps, 6> m_surfaceMaterials;
    std::vector<RenderableTrackItem> m_items;

    GraphicsQuality m_quality{GraphicsQuality::High};
    Vec3 m_sunDirection{vec3Normalize(Vec3{45.0, 75.0, 35.0})};
    Mat4 m_lightSpaceMatrix{Mat4::identity()};
};

} // namespace torquelab
