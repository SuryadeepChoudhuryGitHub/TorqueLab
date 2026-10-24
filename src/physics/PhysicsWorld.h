#pragma once
/**
 * Bullet 3D Physics World & Proving Ground Environment Manager.
 *
 * Manages:
 * - Bullet btDiscreteDynamicsWorld initialization and fixed-timestep (120 Hz) accumulator loop
 * - Proving Ground track colliders (asphalt straight, wet skidpad, gravel strip, ice patch,
 *   rumble strips, speed bumps, elevation ramps, slalom bollards, and perimeter barriers)
 * - Quick-teleport spawn presets for testing specific driving regimes
 */

#include <memory>
#include <string>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include "physics/Math3D.h"
#include "physics/SurfaceSystem.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

enum class TrackElementKind {
    Ground,
    SurfacePatch,
    Bump,
    Ramp,
    Curb,
    Obstacle
};

struct TrackBoxElement {
    std::string id;
    std::string label;
    TrackElementKind kind{TrackElementKind::Ground};
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 halfExtents{1.0, 1.0, 1.0};
    Quat rotation{0.0, 0.0, 0.0, 1.0};
    SurfaceType surfaceType{SurfaceType::DryAsphalt};
    bool hasColorOverride{false};
    unsigned int colorOverride{0};
    int colliderHandle{-1};
};

struct SpawnPreset {
    std::string id;
    std::string label;
    std::string description;
    Vec3 position{0.0, 0.55, 0.0};
    double yawDeg{0.0};
};

const std::vector<SpawnPreset>& getProvingGroundPresets();

struct VehicleCollisionDiagnostics {
    std::string vehicleId;
    const btRigidBody* rigidBodyPtr{nullptr};
    int rigidBodyUserIndex{0};
    int collisionShapeType{0};
    std::string collisionShapeTypeName;
    int childShapeCount{0};
    int activeVehicleCollisionObjects{0};
    int totalWorldCollisionObjects{0};
    bool collisionWireframeVisible{false};
    bool legacyPlaceholderRegistered{false};
    bool colliderReplacementSucceeded{true};
    bool usingModelDerivedCollision{false};
    std::string strategyName;
};

class PhysicsWorld {
public:
    explicit PhysicsWorld(
        const VehicleConfig& config = createDefaultVehicleConfig(),
        bool buildDefaultTrack = true,
        const GeneratedVehicleCollision* collision = nullptr
    );
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    /**
     * Adds a static oriented box collider to the Bullet world and registers its surface properties.
     */
    int addTrackBox(const TrackBoxElement& element);

    /**
     * Updates the surface type of all registered track colliders (used by surface comparison tests).
     */
    void setAllTrackSurfaces(SurfaceType surface);

    /**
     * Executes a single deterministic fixed-timestep physics step (dt = 1/120 s).
     */
    void stepFixed();

    /**
     * Advances the fixed-timestep accumulator by frameDeltaSeconds (clamped to prevent spiral of death).
     * Returns the interpolation factor alpha in [0..1] for rendering.
     */
    double updateFrame(double frameDeltaSeconds);

    /**
     * Teleports the vehicle to a named Proving Ground zone preset.
     */
    void teleportToPreset(const SpawnPreset& preset);

    /**
     * Replaces the simulated vehicle with a new VehicleConfig (live switching).
     * Re-creates the chassis rigid body and suspension with 0 memory leaks or stale handles.
     */
    void switchVehicle(const VehicleConfig& newConfig, const GeneratedVehicleCollision* collision = nullptr);

    /**
     * Inspects the active Bullet dynamics world and vehicle collision setup to detect
     * duplicate chassis rigid bodies, stale frames, or lifecycle leaks.
     */
    VehicleCollisionDiagnostics getCollisionDiagnostics(bool wireframeVisible = true) const;

    btDiscreteDynamicsWorld* getDynamicsWorld() const { return m_world.get(); }

    SurfaceSystem surfaceSystem;
    std::vector<TrackBoxElement> trackElements;

    double fixedDt{1.0 / 120.0};
    int maxSubSteps{12};
    double accumulator{0.0};
    bool paused{false};
    double lastPhysicsStepTimeMs{0.0};

    std::unique_ptr<VehicleDynamics> vehicle;

private:
    void buildBaseGroundPlane();
    void buildProvingGround();

    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfig;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btDbvtBroadphase> m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;

    struct StaticColliderRecord {
        int handle{0};
        std::unique_ptr<btCollisionShape> shape;
        std::unique_ptr<btDefaultMotionState> motionState;
        std::unique_ptr<btRigidBody> body;
    };
    std::vector<StaticColliderRecord> m_staticColliders;
    int m_nextColliderHandle{1};
};

} // namespace torquelab
