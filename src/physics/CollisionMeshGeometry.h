#pragma once
/**
 * Collision Mesh Geometry & Convex Decomposition Data Structures.
 *
 * Provides CPU-retained raw mesh storage, spatial decimation,
 * and convex hull decomposition representation for dynamic rigid bodies in Bullet.
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "physics/Math3D.h"

namespace torquelab {

enum class CollisionAlgorithmStrategy {
    AutomaticDecomposition,       // Selects best decomposition (VHACD or topological clustering, with fallback)
    HierarchicalConvexDecomposition, // Volumetric V-HACD
    SpatialClusteredHulls,        // Fast topological/spatial clustered convex hulls
    SingleConvexHull,             // Single watertight convex hull wrapping the chassis
    CompoundBoundsFallback        // Safe bounding boxes fallback with explicit diagnostic
};

inline const char* getStrategyName(CollisionAlgorithmStrategy s) {
    switch (s) {
        case CollisionAlgorithmStrategy::AutomaticDecomposition: return "Automatic Convex Decomposition";
        case CollisionAlgorithmStrategy::HierarchicalConvexDecomposition: return "V-HACD Volumetric Decomposition";
        case CollisionAlgorithmStrategy::SpatialClusteredHulls: return "Spatial Clustered Multi-Hull";
        case CollisionAlgorithmStrategy::SingleConvexHull: return "Single Convex Hull";
        case CollisionAlgorithmStrategy::CompoundBoundsFallback: return "Bounds Box Fallback";
        default: return "Unknown";
    }
}

struct CollisionPipelineSettings {
    CollisionAlgorithmStrategy strategy{CollisionAlgorithmStrategy::AutomaticDecomposition};
    int maxConvexHulls{8};              // Maximum number of convex child pieces in compound shape
    int maxVerticesPerHull{32};         // Maximum vertex count per convex hull
    double concavityTolerance{0.005};   // Concavity sensitivity threshold
    int decimationTargetTriangles{2500}; // Maximum triangles to process (prevents excessive decomposition times)
    double collisionMargin{0.02};       // Bullet collision margin (m)
    bool excludeWheels{true};           // Exclude wheel geometry from chassis collision
    bool enableCaching{true};           // Cache generated convex hulls to disk
    std::string cacheDirectory{"model/cache/"};
};

struct RawMeshGeometry {
    std::vector<Vec3> vertices;        // Transformed vertices in local model space
    std::vector<uint32_t> indices;     // Triangle vertex indices (triplets: i0, i1, i2)
    Vec3 aabbMin{1e9, 1e9, 1e9};
    Vec3 aabbMax{-1e9, -1e9, -1e9};

    void clear() {
        vertices.clear();
        indices.clear();
        aabbMin = Vec3{1e9, 1e9, 1e9};
        aabbMax = Vec3{-1e9, -1e9, -1e9};
    }

    bool empty() const {
        return vertices.empty();
    }

    void addVertex(const Vec3& v) {
        vertices.push_back(v);
        aabbMin.x = std::min(aabbMin.x, v.x);
        aabbMin.y = std::min(aabbMin.y, v.y);
        aabbMin.z = std::min(aabbMin.z, v.z);
        aabbMax.x = std::max(aabbMax.x, v.x);
        aabbMax.y = std::max(aabbMax.y, v.y);
        aabbMax.z = std::max(aabbMax.z, v.z);
    }

    void addTriangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        indices.push_back(i0);
        indices.push_back(i1);
        indices.push_back(i2);
    }

    void computeBounds() {
        aabbMin = Vec3{1e9, 1e9, 1e9};
        aabbMax = Vec3{-1e9, -1e9, -1e9};
        for (const auto& v : vertices) {
            aabbMin.x = std::min(aabbMin.x, v.x);
            aabbMin.y = std::min(aabbMin.y, v.y);
            aabbMin.z = std::min(aabbMin.z, v.z);
            aabbMax.x = std::max(aabbMax.x, v.x);
            aabbMax.y = std::max(aabbMax.y, v.y);
            aabbMax.z = std::max(aabbMax.z, v.z);
        }
    }

    void transform(const Mat4& m) {
        for (auto& v : vertices) {
            v = mat4TransformPoint(m, v);
        }
        computeBounds();
    }

    void appendTransformed(const RawMeshGeometry& other, const Mat4& transformMatrix) {
        const uint32_t baseIdx = static_cast<uint32_t>(vertices.size());
        for (const auto& v : other.vertices) {
            addVertex(mat4TransformPoint(transformMatrix, v));
        }
        for (uint32_t idx : other.indices) {
            indices.push_back(baseIdx + idx);
        }
    }
};

struct ConvexDecompositionPiece {
    std::vector<Vec3> vertices;        // Vertices of this convex hull relative to its local center
    std::vector<uint32_t> indices;     // Hull surface triangle indices
    Vec3 centerModelSpace{0.0, 0.0, 0.0}; // Position of this piece's centroid in model space
    Vec3 halfExtents{0.0, 0.0, 0.0};   // Bounding extents
    double volume{0.0};
    std::string label{"ConvexPiece"};
};

struct GeneratedVehicleCollision {
    std::string vehicleId{"bmw_m4"};
    std::string sourceModelPath{"model/BMW_M4.glb"};
    CollisionAlgorithmStrategy strategyUsed{CollisionAlgorithmStrategy::AutomaticDecomposition};
    std::vector<ConvexDecompositionPiece> pieces;

    // Derived Chassis Bounding Information
    Vec3 chassisAABBMin{0.0, 0.0, 0.0};
    Vec3 chassisAABBMax{0.0, 0.0, 0.0};
    Vec3 chassisDimensions{0.0, 0.0, 0.0};
    Vec3 visualModelOffset{0.0, 0.0, 0.0}; // Offset between model origin and physics body origin

    // Automatically Derived Vehicle Geometry (from actual 3D model vertices)
    bool geometryDerivedSuccessfully{false};
    double derivedWheelbase{0.0};
    double derivedTrackWidth{0.0};
    double derivedWheelRadius{0.0};
    double derivedWheelWidth{0.0};
    double derivedGroundClearance{0.0};
    std::array<Vec3, 4> derivedWheelCenters{
        Vec3{ 0.79, 0.33,  1.38},
        Vec3{-0.79, 0.33,  1.38},
        Vec3{ 0.79, 0.33, -1.38},
        Vec3{-0.79, 0.33, -1.38}
    };

    // Diagnostic & Telemetry Metrics
    int inputVertexCount{0};
    int inputTriangleCount{0};
    int totalOutputHulls{0};
    int totalOutputVertices{0};
    double generationTimeMs{0.0};
    bool loadedFromCache{false};
    bool isFallback{false};
    std::string diagnosticMessage{"Initialized"};
};

} // namespace torquelab
