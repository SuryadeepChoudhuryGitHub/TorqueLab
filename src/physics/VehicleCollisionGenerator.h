#pragma once
/**
 * Generic Model-Derived Shape-Accurate Vehicle Collision Generator.
 *
 * Automatically produces watertight, shape-accurate Bullet collision geometry
 * (compound convex hulls) from arbitrary 3D vehicle models (GLTF/GLB or procedural meshes).
 *
 * Eliminates hardcoded collision boxes and vehicle-specific coordinates:
 * - Decimates high-density visual meshes to efficient collision representations
 * - Executes volumetric approximate convex decomposition (V-HACD) and spatial topological clustering
 * - Leverages Bullet's btConvexHullComputer for minimal watertight hull generation
 * - Extracts and separates wheel geometry to derive exact wheel centers, radius, width, wheelbase, and track width
 * - Provides disk/memory caching for sub-millisecond subsequent startups
 * - Integrates seamlessly with Bullet btCompoundShape and runtime live vehicle switching
 */

#include <memory>
#include <string>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include "physics/CollisionMeshGeometry.h"

namespace torquelab {

// Forward declarations
struct LoadedVehicleModel;
struct VehicleConfig;

class VehicleCollisionGenerator {
public:
    /**
     * Generates a complete collision representation and derived vehicle geometry
     * from a loaded 3D vehicle model. Checks cache first if enabled.
     */
    static bool generateFromModel(
        const LoadedVehicleModel& model,
        const CollisionPipelineSettings& settings,
        GeneratedVehicleCollision& outCollision
    );

    /**
     * Generates collision representation directly from raw chassis mesh geometry.
     */
    static bool generateFromMesh(
        const RawMeshGeometry& chassisMesh,
        const std::string& vehicleId,
        const CollisionPipelineSettings& settings,
        GeneratedVehicleCollision& outCollision
    );

    /**
     * Constructs a live Bullet btCompoundShape ready for dynamic rigid body simulation,
     * centering child convex hulls relative to the vehicle's Center of Mass (CoM).
     */
    static std::unique_ptr<btCompoundShape> createBulletCompoundShape(
        const GeneratedVehicleCollision& collision,
        const Vec3& localCoM,
        std::vector<std::unique_ptr<btCollisionShape>>& outSubShapes
    );

    /**
     * Automatically updates a VehicleConfig with parameters derived directly from 3D model geometry.
     */
    static void applyDerivedGeometryToConfig(
        const GeneratedVehicleCollision& collision,
        VehicleConfig& config
    );

    /** Disk cache serialization & deserialization. */
    static bool saveToCache(const GeneratedVehicleCollision& col, const std::string& cachePath);
    static bool loadFromCache(GeneratedVehicleCollision& col, const std::string& cachePath);
    static std::string getCacheFilePath(const std::string& cacheDir, const std::string& vehicleId);

private:
    /** Subsamples / decimates large vertex arrays into uniform spatial points. */
    static std::vector<Vec3> decimateVertices(
        const std::vector<Vec3>& inputVerts,
        int targetCount,
        const Vec3& aabbMin,
        const Vec3& aabbMax
    );

    /** Executes V-HACD volumetric approximate convex decomposition. */
    static bool executeVHACD(
        const RawMeshGeometry& mesh,
        const CollisionPipelineSettings& settings,
        std::vector<ConvexDecompositionPiece>& outPieces,
        std::string& outDiagnostic
    );

    /** Executes fast spatial topological clustered convex hull generation. */
    static bool executeSpatialClustering(
        const std::vector<Vec3>& vertices,
        const Vec3& aabbMin,
        const Vec3& aabbMax,
        int maxHulls,
        std::vector<ConvexDecompositionPiece>& outPieces,
        std::string& outDiagnostic
    );

    /** Generates a single watertight convex hull wrapping all vertices. */
    static bool executeSingleConvexHull(
        const std::vector<Vec3>& vertices,
        std::vector<ConvexDecompositionPiece>& outPieces,
        std::string& outDiagnostic
    );

    /** Fallback safe bounding boxes with diagnostic warning. */
    static void executeBoundsFallback(
        const Vec3& aabbMin,
        const Vec3& aabbMax,
        std::vector<ConvexDecompositionPiece>& outPieces,
        std::string& outDiagnostic
    );
};

} // namespace torquelab
