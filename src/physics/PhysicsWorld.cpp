#include "physics/PhysicsWorld.h"
#include <chrono>
#include <cmath>

namespace torquelab {

const std::vector<SpawnPreset>& getProvingGroundPresets() {
    static const std::vector<SpawnPreset> presets{
        {
            "straight",
            "1 Main Straight",
            "High-speed acceleration & braking lane (Dry Asphalt)",
            Vec3{0.0, 0.55, 0.0},
            0.0
        },
        {
            "skidpad_dry",
            "2 Handling Circle",
            "Steady-state cornering, body roll & understeer/oversteer pad",
            Vec3{0.0, 0.55, 215.0},
            0.0
        },
        {
            "wet_pad",
            "3 Wet Skidpad",
            "Reduced-grip wet asphalt braking & cornering zone (East)",
            Vec3{-52.0, 0.55, 40.0},
            0.0
        },
        {
            "gravel_ice",
            "4 Gravel & Ice",
            "Loose gravel rally lane & ultra-low-grip ice strip (West)",
            Vec3{52.0, 0.55, 30.0},
            0.0
        },
        {
            "bumps",
            "5 Bump Course",
            "Rhythmic suspension compression, rebound & damping test lane",
            Vec3{-24.0, 0.55, 10.0},
            0.0
        },
        {
            "ramp",
            "6 Ramp Jump",
            "Inclined launch & landing ramps for airborne & bottom-out testing",
            Vec3{24.0, 0.55, 15.0},
            0.0
        },
        {
            "slalom",
            "7 Slalom",
            "Transient weight-transfer & roll-reversal obstacle course",
            Vec3{0.0, 0.55, 65.0},
            0.0
        }
    };
    return presets;
}

PhysicsWorld::PhysicsWorld(const VehicleConfig& config, bool buildDefaultTrack, const GeneratedVehicleCollision* collision) {
    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(),
        m_broadphase.get(),
        m_solver.get(),
        m_collisionConfig.get()
    );
    m_world->setGravity(btVector3(0.0f, static_cast<btScalar>(-GRAVITY), 0.0f));

    // Configure constraint solver for stable vehicle dynamics:
    btContactSolverInfo& solverInfo = m_world->getSolverInfo();
    solverInfo.m_numIterations = 20; // 20 constraint iterations for robust contact convergence
    solverInfo.m_erp = 0.2f;
    solverInfo.m_globalCfm = 0.0f; // Rigid contact prevents obstacle spongy penetration
    solverInfo.m_splitImpulse = 1; // Pure positional penetration resolution without velocity injection
    solverInfo.m_splitImpulsePenetrationThreshold = -0.01f;
    solverInfo.m_splitImpulseTurnErp = 0.1f;

    if (buildDefaultTrack) {
        buildProvingGround();
    } else {
        buildBaseGroundPlane();
    }

    const double eqHeight = computeEquilibriumChassisHeight(config);
    vehicle = std::make_unique<VehicleDynamics>(
        m_world.get(),
        &surfaceSystem,
        config,
        Vec3{0.0, eqHeight, 0.0},
        quatIdentity(),
        collision
    );
}

PhysicsWorld::~PhysicsWorld() {
    vehicle.reset();
    for (auto& rec : m_staticColliders) {
        if (m_world && rec.body) {
            m_world->removeRigidBody(rec.body.get());
        }
    }
    m_staticColliders.clear();
}

void PhysicsWorld::buildBaseGroundPlane() {
    addTrackBox(TrackBoxElement{
        "base_ground",
        "Flat Test Pad",
        TrackElementKind::Ground,
        Vec3{0.0, -0.5, 0.0},
        Vec3{500.0, 0.5, 500.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        false,
        0,
        -1
    });
}

void PhysicsWorld::buildProvingGround() {
    // 1. Base Turf / Grass Expanse (600m x 600m, top at y = -0.01m)
    addTrackBox(TrackBoxElement{
        "world_grass",
        "Proving Ground Turf",
        TrackElementKind::Ground,
        Vec3{0.0, -0.51, 90.0},
        Vec3{320.0, 0.5, 320.0},
        quatIdentity(),
        SurfaceType::Grass,
        false,
        0,
        -1
    });

    // 2. Main Central Dry Asphalt Straight & Handling Apron (140m wide x 360m long, top at y = 0.0m)
    addTrackBox(TrackBoxElement{
        "main_apron",
        "Dry Asphalt Proving Apron",
        TrackElementKind::Ground,
        Vec3{0.0, -0.25, 110.0},
        Vec3{85.0, 0.25, 185.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        false,
        0,
        -1
    });

    // 3. Rumble Kerb Borders along Main Straight (x = -10.5m and x = +10.5m)
    addTrackBox(TrackBoxElement{
        "kerb_east",
        "East Rumble Strip",
        TrackElementKind::Curb,
        Vec3{-10.5, -0.085, 85.0},
        Vec3{0.9, 0.1, 110.0},
        quatIdentity(),
        SurfaceType::RumbleStrip,
        false,
        0,
        -1
    });
    addTrackBox(TrackBoxElement{
        "kerb_west",
        "West Rumble Strip",
        TrackElementKind::Curb,
        Vec3{10.5, -0.085, 85.0},
        Vec3{0.9, 0.1, 110.0},
        quatIdentity(),
        SurfaceType::RumbleStrip,
        false,
        0,
        -1
    });

    // 4. Wet Asphalt Skidpad Zone (East side: x = -52m, z = 40..150m)
    addTrackBox(TrackBoxElement{
        "wet_skidpad",
        "Wet Asphalt Skidpad",
        TrackElementKind::SurfacePatch,
        Vec3{-52.0, -0.095, 95.0},
        Vec3{22.0, 0.1, 58.0},
        quatIdentity(),
        SurfaceType::WetAsphalt,
        false,
        0,
        -1
    });

    // 5. Loose Gravel Rally Strip (West side: x = +52m, z = 35..110m)
    addTrackBox(TrackBoxElement{
        "gravel_strip",
        "Loose Gravel Rally Lane",
        TrackElementKind::SurfacePatch,
        Vec3{52.0, -0.095, 72.0},
        Vec3{18.0, 0.1, 38.0},
        quatIdentity(),
        SurfaceType::Gravel,
        false,
        0,
        -1
    });

    // 6. Low-Grip Polished Ice Sheet (West-North side: x = +52m, z = 115..195m)
    addTrackBox(TrackBoxElement{
        "ice_sheet",
        "Polished Ice Test Strip",
        TrackElementKind::SurfacePatch,
        Vec3{52.0, -0.095, 155.0},
        Vec3{18.0, 0.1, 40.0},
        quatIdentity(),
        SurfaceType::Ice,
        false,
        0,
        -1
    });

    // 7. Suspension Rhythm Bump Course (x = -24m, z = 25..75m)
    const double bumpZPositions[6] = {28.0, 35.0, 42.0, 48.0, 54.0, 62.0};
    const double bumpHeights[6] = {0.055, 0.07, 0.085, 0.075, 0.095, 0.11};
    for (int i = 0; i < 6; ++i) {
        const double h = bumpHeights[i];
        addTrackBox(TrackBoxElement{
            "susp_bump_" + std::to_string(i),
            "Suspension Bump " + std::to_string(i + 1),
            TrackElementKind::Bump,
            Vec3{-24.0, h * 0.5 - 0.01, bumpZPositions[i]},
            Vec3{4.5, h * 0.5, 0.55},
            quatIdentity(),
            SurfaceType::RumbleStrip,
            true,
            0xe09f3e,
            -1
        });
    }

    // 8. Elevation Launch Ramp & Landing Ramp (x = +24m, z = 45m & z = 72m)
    const double rampPitch = degToRad(-9.5); // Pitches up along +Z
    addTrackBox(TrackBoxElement{
        "launch_ramp",
        "Launch Ramp",
        TrackElementKind::Ramp,
        Vec3{24.0, 0.48, 45.0},
        Vec3{4.0, 0.25, 6.5},
        quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, rampPitch),
        SurfaceType::DryAsphalt,
        true,
        0x3b4254,
        -1
    });

    const double downRampPitch = degToRad(9.5);
    addTrackBox(TrackBoxElement{
        "landing_ramp",
        "Landing Ramp",
        TrackElementKind::Ramp,
        Vec3{24.0, 0.48, 68.0},
        Vec3{4.0, 0.25, 6.5},
        quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, downRampPitch),
        SurfaceType::DryAsphalt,
        true,
        0x3b4254,
        -1
    });

    // 9. Slalom Bollards along Central Straight (z = 80..170m)
    for (int i = 0; i < 5; ++i) {
        const double z = 85.0 + i * 20.0;
        const double xOffset = (i % 2 == 0) ? 1.8 : -1.8;
        addTrackBox(TrackBoxElement{
            "slalom_post_" + std::to_string(i),
            "Slalom Barrier " + std::to_string(i + 1),
            TrackElementKind::Obstacle,
            Vec3{xOffset, 0.55, z},
            Vec3{0.55, 0.55, 0.55},
            quatIdentity(),
            SurfaceType::DryAsphalt,
            true,
            0xff595e,
            -1
        });
    }

    // 10. Perimeter Safety Barriers (for collision momentum transfer testing)
    addTrackBox(TrackBoxElement{
        "wall_north",
        "North Perimeter Barrier",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.75, 294.0},
        Vec3{85.0, 0.75, 1.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0x4a5568,
        -1
    });
    addTrackBox(TrackBoxElement{
        "wall_south",
        "South Perimeter Barrier",
        TrackElementKind::Obstacle,
        Vec3{0.0, 0.75, -74.0},
        Vec3{85.0, 0.75, 1.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0x4a5568,
        -1
    });
    addTrackBox(TrackBoxElement{
        "wall_east",
        "East Perimeter Barrier",
        TrackElementKind::Obstacle,
        Vec3{-84.0, 0.75, 110.0},
        Vec3{1.0, 0.75, 185.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0x4a5568,
        -1
    });
    addTrackBox(TrackBoxElement{
        "wall_west",
        "West Perimeter Barrier",
        TrackElementKind::Obstacle,
        Vec3{84.0, 0.75, 110.0},
        Vec3{1.0, 0.75, 185.0},
        quatIdentity(),
        SurfaceType::DryAsphalt,
        true,
        0x4a5568,
        -1
    });
}

int PhysicsWorld::addTrackBox(const TrackBoxElement& element) {
    const int handle = m_nextColliderHandle++;

    auto shape = std::make_unique<btBoxShape>(btVector3(
        static_cast<btScalar>(element.halfExtents.x),
        static_cast<btScalar>(element.halfExtents.y),
        static_cast<btScalar>(element.halfExtents.z)
    ));

    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(btVector3(
        static_cast<btScalar>(element.position.x),
        static_cast<btScalar>(element.position.y),
        static_cast<btScalar>(element.position.z)
    ));
    trans.setRotation(btQuaternion(
        static_cast<btScalar>(element.rotation.x),
        static_cast<btScalar>(element.rotation.y),
        static_cast<btScalar>(element.rotation.z),
        static_cast<btScalar>(element.rotation.w)
    ));

    auto motionState = std::make_unique<btDefaultMotionState>(trans);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, motionState.get(), shape.get(), btVector3(0, 0, 0));
    rbInfo.m_friction = static_cast<btScalar>(0.8);
    rbInfo.m_restitution = static_cast<btScalar>((element.kind == TrackElementKind::Obstacle) ? 0.02 : 0.0);

    auto body = std::make_unique<btRigidBody>(rbInfo);
    body->setUserIndex(handle);

    if (m_world) {
        m_world->addRigidBody(body.get());
    }

    surfaceSystem.registerColliderSurface(handle, element.surfaceType, element.kind != TrackElementKind::Obstacle);

    TrackBoxElement stored = element;
    stored.colliderHandle = handle;
    trackElements.push_back(stored);

    m_staticColliders.push_back(StaticColliderRecord{
        handle,
        std::move(shape),
        std::move(motionState),
        std::move(body)
    });

    return handle;
}

void PhysicsWorld::setAllTrackSurfaces(SurfaceType surface) {
    surfaceSystem.setDefaultSurface(surface);
    for (auto& el : trackElements) {
        el.surfaceType = surface;
        if (el.colliderHandle >= 0) {
            surfaceSystem.registerColliderSurface(el.colliderHandle, surface);
        }
    }
}

void PhysicsWorld::stepFixed() {
    const auto t0 = std::chrono::high_resolution_clock::now();

    vehicle->step(fixedDt);
    // maxSubSteps = 0 tells Bullet to step by the exact fixedDt in one deterministic pass
    m_world->stepSimulation(static_cast<btScalar>(fixedDt), 0);
    vehicle->postWorldStep(fixedDt);

    const auto t1 = std::chrono::high_resolution_clock::now();
    lastPhysicsStepTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

double PhysicsWorld::updateFrame(double frameDeltaSeconds) {
    if (paused) {
        return accumulator / fixedDt;
    }

    const double clampedFrameDt = std::min(frameDeltaSeconds, fixedDt * static_cast<double>(maxSubSteps));
    accumulator += clampedFrameDt;

    int steps = 0;
    while (accumulator >= fixedDt && steps < maxSubSteps) {
        stepFixed();
        accumulator -= fixedDt;
        ++steps;
    }

    return accumulator / fixedDt;
}

void PhysicsWorld::teleportToPreset(const SpawnPreset& preset) {
    const double eqHeight = computeEquilibriumChassisHeight(vehicle->config);
    const Quat rot = quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, degToRad(preset.yawDeg));
    const Vec3 spawnPos{preset.position.x, eqHeight + 0.04, preset.position.z};
    vehicle->resetVehicle(&spawnPos, &rot);
    accumulator = 0.0;
}

void PhysicsWorld::switchVehicle(const VehicleConfig& newConfig, const GeneratedVehicleCollision* collision) {
    Vec3 curPos{0.0, 0.55, 0.0};
    Quat curRot = quatIdentity();
    Vec3 curLinVel{0.0, 0.0, 0.0};
    Vec3 curAngVel{0.0, 0.0, 0.0};

    if (vehicle) {
        curPos = vehicle->body.getPosition();
        curRot = vehicle->body.getRotation();
        curLinVel = vehicle->body.getLinearVelocity();
        curAngVel = vehicle->body.getAngularVelocity();
    }

    const double eqHeight = computeEquilibriumChassisHeight(newConfig);
    curPos.y = std::max(curPos.y, eqHeight);

    // Cleanly delete previous vehicle (removes btRigidBody from dynamics world)
    vehicle.reset();

    // Construct new vehicle with updated geometry, mass, suspension, and collision shapes
    vehicle = std::make_unique<VehicleDynamics>(
        m_world.get(),
        &surfaceSystem,
        newConfig,
        curPos,
        curRot,
        collision
    );

    vehicle->body.setLinearVelocity(curLinVel);
    vehicle->body.setAngularVelocity(curAngVel);
}

VehicleCollisionDiagnostics PhysicsWorld::getCollisionDiagnostics(bool wireframeVisible) const {
    VehicleCollisionDiagnostics diag{};
    diag.collisionWireframeVisible = wireframeVisible;

    if (!m_world) return diag;

    diag.totalWorldCollisionObjects = m_world->getNumCollisionObjects();

    if (!vehicle || !vehicle->body.getRigidBody()) return diag;

    const btRigidBody* rb = vehicle->body.getRigidBody();
    diag.rigidBodyPtr = rb;
    diag.rigidBodyUserIndex = rb->getUserIndex();
    diag.vehicleId = vehicle->config.id;

    const btCollisionShape* shape = rb->getCollisionShape();
    if (shape) {
        diag.collisionShapeType = shape->getShapeType();
        diag.collisionShapeTypeName = shape->getName() ? shape->getName() : "Unknown";
    }

    const btCompoundShape* compound = vehicle->body.getCompoundShape();
    if (compound) {
        diag.childShapeCount = compound->getNumChildShapes();
    }

    const auto& col = vehicle->body.getGeneratedCollision();
    diag.usingModelDerivedCollision = (!col.pieces.empty());
    diag.strategyName = getStrategyName(col.strategyUsed);

    // Count active vehicle rigid bodies in the Bullet world
    const btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
    int vehicleObjCount = 0;
    int duplicateChassisCount = 0;

    for (int i = 0; i < objs.size(); ++i) {
        const btCollisionObject* obj = objs[i];
        if (!obj) continue;
        if (obj == rb) {
            ++vehicleObjCount;
        } else if (obj->getUserIndex() == -1) {
            // Another collision object marked with vehicle chassis index (-1)
            ++duplicateChassisCount;
        }
    }

    diag.activeVehicleCollisionObjects = vehicleObjCount;
    diag.legacyPlaceholderRegistered = (duplicateChassisCount > 0);
    diag.colliderReplacementSucceeded = (vehicleObjCount == 1 && duplicateChassisCount == 0);

    return diag;
}

} // namespace torquelab
