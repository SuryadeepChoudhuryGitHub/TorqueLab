#include "vehicle/VehicleDynamics.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace torquelab {

namespace {

static inline btVector3 toBtVec3(const Vec3& v) {
    return btVector3(static_cast<btScalar>(v.x), static_cast<btScalar>(v.y), static_cast<btScalar>(v.z));
}

static inline btQuaternion toBtQuat(const Quat& q) {
    return btQuaternion(
        static_cast<btScalar>(q.x),
        static_cast<btScalar>(q.y),
        static_cast<btScalar>(q.z),
        static_cast<btScalar>(q.w)
    );
}

struct ExcludeVehicleRayCallback : public btCollisionWorld::ClosestRayResultCallback {
    const btCollisionObject* m_ignoreObj{nullptr};
    std::array<const btCollisionObject*, 4> m_wheelObjs{};

    ExcludeVehicleRayCallback(
        const btVector3& rayFromWorld,
        const btVector3& rayToWorld,
        const btCollisionObject* ignoreObj,
        const std::array<const btCollisionObject*, 4>& wheelObjs
    )
        : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld)
        , m_ignoreObj(ignoreObj)
        , m_wheelObjs(wheelObjs)
    {}

    bool needsCollision(btBroadphaseProxy* proxy0) const override {
        if (proxy0 && proxy0->m_clientObject) {
            const auto* clientObj = static_cast<const btCollisionObject*>(proxy0->m_clientObject);
            if (clientObj == m_ignoreObj) {
                return false;
            }
            for (const auto* w : m_wheelObjs) {
                if (clientObj == w) {
                    return false;
                }
            }
        }
        return btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy0);
    }
};

struct AnalyticalRayHit {
    bool hit{false};
    double distance{0.0};
    Vec3 pointWorld{0.0, 0.0, 0.0};
    Vec3 normalWorld{0.0, 1.0, 0.0};
    const btCollisionObject* collisionObject{nullptr};
};

/**
 * Exact 64-bit double-precision Kay-Kajiya slab raycast against an oriented box (btBoxShape),
 * avoiding 32-bit float GJK simplex precision drift on large ground boxes while falling back
 * to Bullet's rayTest for any non-box collision shapes.
 */
AnalyticalRayHit castRayExactWorld(
    btDiscreteDynamicsWorld* world,
    const Vec3& rayOriginWorld,
    const Vec3& rayDirWorld,
    double maxRayLength,
    const btCollisionObject* ignoreObj,
    const std::array<const btCollisionObject*, 4>& wheelObjsToIgnore,
    const SurfaceSystem* surfaceSystem = nullptr,
    bool includeObstacles = true
) {
    AnalyticalRayHit best{};
    best.distance = maxRayLength + 1.0;

    if (!world) {
        return best;
    }

    const btCollisionObjectArray& objects = world->getCollisionObjectArray();
    for (int i = 0; i < objects.size(); ++i) {
        const btCollisionObject* obj = objects[i];
        if (!obj || obj == ignoreObj) {
            continue;
        }
        bool isWheel = false;
        for (const auto* w : wheelObjsToIgnore) {
            if (obj == w) {
                isWheel = true;
                break;
            }
        }
        if (isWheel) {
            continue;
        }
        if (!includeObstacles && surfaceSystem && !surfaceSystem->isColliderDriveable(obj->getUserIndex())) {
            continue; // Skip non-driveable obstacles if explicitly requested
        }
        const btCollisionShape* shape = obj->getCollisionShape();
        if (!shape) {
            continue;
        }

        if (shape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
            const auto* boxShape = static_cast<const btBoxShape*>(shape);
            const btVector3 heBt = boxShape->getHalfExtentsWithMargin();
            const Vec3 halfExtents{
                static_cast<double>(heBt.x()),
                static_cast<double>(heBt.y()),
                static_cast<double>(heBt.z())
            };

            const btTransform& tr = obj->getWorldTransform();
            const Vec3 boxPos{
                static_cast<double>(tr.getOrigin().x()),
                static_cast<double>(tr.getOrigin().y()),
                static_cast<double>(tr.getOrigin().z())
            };
            const btQuaternion qBt = tr.getRotation();
            const Quat boxRot = quatNormalize(Quat{
                static_cast<double>(qBt.x()),
                static_cast<double>(qBt.y()),
                static_cast<double>(qBt.z()),
                static_cast<double>(qBt.w())
            });

            const Vec3 localOrigin = worldPointToLocal(rayOriginWorld, boxPos, boxRot);
            const Vec3 localDir = worldVectorToLocal(rayDirWorld, boxRot);

            const double orig[3] = {localOrigin.x, localOrigin.y, localOrigin.z};
            const double dir[3]  = {localDir.x, localDir.y, localDir.z};
            const double ext[3]  = {halfExtents.x, halfExtents.y, halfExtents.z};

            double tMin = 0.0;
            double tMax = maxRayLength;
            Vec3 normalLocal{0.0, 1.0, 0.0};
            bool inside = true;
            bool slabValid = true;

            for (int a = 0; a < 3; ++a) {
                if (std::abs(orig[a]) > ext[a]) {
                    inside = false;
                }
                if (std::abs(dir[a]) < 1e-12) {
                    if (std::abs(orig[a]) > ext[a]) {
                        slabValid = false;
                        break;
                    }
                } else {
                    double t1 = (-ext[a] - orig[a]) / dir[a];
                    double t2 = ( ext[a] - orig[a]) / dir[a];
                    double n1 = -1.0;
                    double n2 =  1.0;
                    if (t1 > t2) {
                        std::swap(t1, t2);
                        std::swap(n1, n2);
                    }
                    if (t1 > tMin) {
                        tMin = t1;
                        normalLocal = Vec3{
                            (a == 0) ? n1 : 0.0,
                            (a == 1) ? n1 : 0.0,
                            (a == 2) ? n1 : 0.0
                        };
                    }
                    tMax = std::min(tMax, t2);
                    if (tMin > tMax) {
                        slabValid = false;
                        break;
                    }
                }
            }

            if (slabValid && tMax >= 0.0 && tMin <= maxRayLength) {
                const double tHit = inside ? 0.0 : tMin;
                if (tHit >= 0.0 && tHit <= maxRayLength && tHit < best.distance) {
                    best.hit = true;
                    best.distance = tHit;
                    best.pointWorld = vec3Add(rayOriginWorld, vec3Scale(rayDirWorld, tHit));
                    best.normalWorld = inside
                        ? vec3Scale(rayDirWorld, -1.0)
                        : vec3Normalize(quatRotateVec3(boxRot, normalLocal));
                    best.collisionObject = obj;
                }
            }
        }
    }

    if (best.hit) {
        return best;
    }

    // Fallback for any non-box colliders
    const Vec3 rayEnd = vec3Add(rayOriginWorld, vec3Scale(rayDirWorld, maxRayLength));
    const btVector3 rayFrom(
        static_cast<btScalar>(rayOriginWorld.x),
        static_cast<btScalar>(rayOriginWorld.y),
        static_cast<btScalar>(rayOriginWorld.z)
    );
    const btVector3 rayTo(
        static_cast<btScalar>(rayEnd.x),
        static_cast<btScalar>(rayEnd.y),
        static_cast<btScalar>(rayEnd.z)
    );
    ExcludeVehicleRayCallback rayCallback(rayFrom, rayTo, ignoreObj, wheelObjsToIgnore);
    world->rayTest(rayFrom, rayTo, rayCallback);
    if (rayCallback.hasHit()) {
        best.hit = true;
        best.distance = maxRayLength * static_cast<double>(rayCallback.m_closestHitFraction);
        best.pointWorld = vec3Add(rayOriginWorld, vec3Scale(rayDirWorld, best.distance));
        best.normalWorld = vec3Normalize(Vec3{
            static_cast<double>(rayCallback.m_hitNormalWorld.x()),
            static_cast<double>(rayCallback.m_hitNormalWorld.y()),
            static_cast<double>(rayCallback.m_hitNormalWorld.z())
        });
        best.collisionObject = rayCallback.m_collisionObject;
    }
    return best;
}

struct VolumetricContactOutput {
    bool inContact{false};
    double effectiveSuspensionLength{0.38};
    Vec3 contactPointWorld{0.0, 0.0, 0.0};
    Vec3 contactNormalWorld{0.0, 1.0, 0.0};
    double circumferentialHitAngle{0.0};
    double lateralOffset{0.0};
    bool hasObstacleContact{false};
    int colliderHandle{-1};
    const btCollisionObject* hitObject{nullptr};
};

VolumetricContactOutput evaluateVolumetricWheelContact(
    btDiscreteDynamicsWorld* world,
    const SurfaceSystem* surfaceSystem,
    const Vec3& mountWorld,
    const Quat& chassisRot,
    const Vec3& chassisDown,
    double steeringAngle,
    double wheelRadius,
    double wheelWidth,
    double suspensionMinLength,
    double suspensionMaxLength,
    double suspensionRestLength,
    const btCollisionObject* ignoreBody,
    const std::array<const btCollisionObject*, 4>& wheelObjsToIgnore,
    WheelCollisionFidelity fidelity = WheelCollisionFidelity::Balanced
) {
    VolumetricContactOutput best{};
    best.effectiveSuspensionLength = suspensionMaxLength;

    const VehicleBasis chassisBasis = getVehicleBasis(chassisRot);
    const Quat steerQuat = quatFromAxisAngle(chassisBasis.up, steeringAngle);
    const Quat steeredRot = quatMultiply(chassisRot, steerQuat);
    const VehicleBasis steeredBasis = getVehicleBasis(steeredRot);

    // Configure probe angles and lateral slices according to selected WheelCollisionFidelity
    std::vector<double> lateralFractions;
    std::vector<double> probeAngles;

    if (fidelity == WheelCollisionFidelity::Performance) {
        lateralFractions = {0.0};
        probeAngles = {0.0, degToRad(20.0), degToRad(45.0), degToRad(-15.0), degToRad(-30.0)};
    } else if (fidelity == WheelCollisionFidelity::Detailed) {
        lateralFractions = {-0.45, -0.22, 0.0, +0.22, +0.45};
        probeAngles = {
            0.0,
            degToRad(10.0), degToRad(20.0), degToRad(30.0), degToRad(40.0),
            degToRad(50.0), degToRad(60.0), degToRad(70.0), degToRad(80.0),
            degToRad(-10.0), degToRad(-20.0), degToRad(-30.0), degToRad(-40.0),
            degToRad(-50.0), degToRad(-60.0), degToRad(-70.0), degToRad(-80.0)
        };
    } else { // Balanced (default)
        lateralFractions = {-0.35, 0.0, +0.35};
        probeAngles = {
            0.0,
            degToRad(15.0),
            degToRad(30.0),
            degToRad(45.0),
            degToRad(60.0),
            degToRad(75.0),
            degToRad(-15.0),
            degToRad(-30.0),
            degToRad(-45.0)
        };
    }

    const double rayStartOffset = 0.20; // 20cm headroom above mount
    const double maxRayLength = rayStartOffset + suspensionMaxLength + 0.15;
    const double maxValidLength = suspensionRestLength + 0.05;

    double minCandidateLength = 1e9;
    bool foundContact = false;

    for (size_t a = 0; a < probeAngles.size(); ++a) {
        const double theta = probeAngles[a];
        const double sinT = std::sin(theta);
        const double cosT = std::cos(theta);

        for (size_t l = 0; l < lateralFractions.size(); ++l) {
            const double wOffset = lateralFractions[l] * wheelWidth;

            // Offset on the 3D tyre outer circumference relative to the wheel hub center:
            // r = w * axle + R * (sin(theta) * forward - cos(theta) * up)
            const Vec3 rWheel = vec3Add(
                vec3Scale(steeredBasis.left, wOffset),
                vec3Add(
                    vec3Scale(steeredBasis.forward, wheelRadius * sinT),
                    vec3Scale(steeredBasis.down, wheelRadius * cosT)
                )
            );

            // Ray start position (at zero suspension stroke, offset by rayStartOffset upwards)
            const Vec3 rayOrigin = vec3Add(
                vec3Add(mountWorld, rWheel),
                vec3Scale(chassisBasis.up, rayStartOffset)
            );

            // Cast ray downwards parallel to suspension stroke (chassisDown)
            AnalyticalRayHit hit = castRayExactWorld(
                world,
                rayOrigin,
                chassisDown,
                maxRayLength,
                ignoreBody,
                wheelObjsToIgnore,
                surfaceSystem,
                true // include obstacles
            );

            if (hit.hit) {
                // Suspension length required for this point on the tyre circumference to contact the hit surface:
                const double candLength = hit.distance - rayStartOffset;

                // Check if this contact is within suspension reach
                if (candLength <= maxValidLength && candLength < minCandidateLength) {
                    Vec3 n = hit.normalWorld;
                    if (vec3Dot(n, chassisBasis.up) < 0.0) {
                        n = vec3Scale(n, -1.0);
                    }

                    // A surface facing roughly upward or inclined (n.y >= 0.15) can support/compress suspension
                    if (vec3Dot(n, chassisBasis.up) >= 0.15 || std::abs(theta) > degToRad(20.0)) {
                        minCandidateLength = candLength;
                        foundContact = true;

                        Vec3 effectiveNormal = vec3Normalize(n);
                        if (std::abs(theta) > degToRad(5.0)) {
                            // Reaction normal between circular tyre circumference and obstacle edge:
                            // Points up (cos theta) and backwards (-sin theta) opposing forward motion
                            const Vec3 circNormal = vec3Normalize(
                                vec3Sub(
                                    vec3Scale(chassisBasis.up, std::cos(theta)),
                                    vec3Scale(steeredBasis.forward, std::sin(theta))
                                )
                            );
                            effectiveNormal = circNormal;
                        }

                        best.inContact = true;
                        best.effectiveSuspensionLength = candLength;
                        best.contactPointWorld = hit.pointWorld;
                        best.contactNormalWorld = effectiveNormal;
                        best.circumferentialHitAngle = theta;
                        best.lateralOffset = wOffset;
                        best.hasObstacleContact = (std::abs(theta) > degToRad(8.0) || vec3Dot(n, chassisBasis.up) < 0.85);
                        best.hitObject = hit.collisionObject;
                        best.colliderHandle = hit.collisionObject ? hit.collisionObject->getUserIndex() : -1;
                    }
                }
            }
        }
    }

    if (!foundContact) {
        best.inContact = false;
        best.effectiveSuspensionLength = suspensionRestLength;
        best.contactNormalWorld = chassisBasis.up;
    }

    return best;
}

} // namespace

VehicleDynamics::VehicleDynamics(
    btDiscreteDynamicsWorld* world,
    SurfaceSystem* surfaceSystem,
    const VehicleConfig& initialConfig,
    const Vec3& spawnPosition,
    const Quat& spawnRotation,
    const GeneratedVehicleCollision* collision
)
    : config(initialConfig)
    , body(world, config, spawnPosition, spawnRotation, collision)
    , drivetrain(createInitialDrivetrainState(config.idleRpm))
    , m_world(world)
    , m_surfaceSystem(surfaceSystem)
    , m_initialSpawnPos(spawnPosition)
    , m_initialSpawnRot(spawnRotation)
{
    const double frontLoad =
        0.5 * config.frontWeightDistribution * config.mass * GRAVITY;
    const double rearLoad =
        0.5 * (1.0 - config.frontWeightDistribution) * config.mass * GRAVITY;
    const double frontEqLen =
        config.suspensionRestLength - frontLoad / config.frontSpringStiffness;
    const double rearEqLen =
        config.suspensionRestLength - rearLoad / config.rearSpringStiffness;

    const auto mounts = getWheelMountPositionsLocal(config);
    const VehicleBasis basis = getVehicleBasis(spawnRotation);
    for (int i = 0; i < 4; ++i) {
        const double eqLen = (i < 2) ? frontEqLen : rearEqLen;
        const double r = getWheelRadius(config, i);
        const double wWidth = getWheelWidth(config, i);
        const double rimR = getTireRimRadius(config, i);
        const double pKpa = torquelab::getTirePressureKpa(config, i);

        wheels[static_cast<size_t>(i)] = createInitialWheelState(
            i,
            mounts[static_cast<size_t>(i)],
            eqLen,
            r,
            wWidth,
            rimR,
            pKpa
        );
        wheels[static_cast<size_t>(i)].mountPointWorld = localPointToWorld(
            mounts[static_cast<size_t>(i)],
            spawnPosition,
            spawnRotation
        );
        wheels[static_cast<size_t>(i)].wheelCenterWorld = vec3Add(
            wheels[static_cast<size_t>(i)].mountPointWorld,
            vec3Scale(basis.down, eqLen)
        );

        // Dedicated 3D physical Bullet cylinder shape and collision object for this wheel
        m_wheelCylinderShapes[static_cast<size_t>(i)] = std::make_unique<btCylinderShapeX>(
            btVector3(
                static_cast<btScalar>(wWidth * 0.5),
                static_cast<btScalar>(r),
                static_cast<btScalar>(r)
            )
        );
        m_wheelCollisionObjects[static_cast<size_t>(i)] = std::make_unique<btCollisionObject>();
        m_wheelCollisionObjects[static_cast<size_t>(i)]->setCollisionShape(
            m_wheelCylinderShapes[static_cast<size_t>(i)].get()
        );
        m_wheelCollisionObjects[static_cast<size_t>(i)]->setCollisionFlags(
            m_wheelCollisionObjects[static_cast<size_t>(i)]->getCollisionFlags() |
            btCollisionObject::CF_NO_CONTACT_RESPONSE
        );
        m_wheelCollisionObjects[static_cast<size_t>(i)]->setUserIndex(100 + i);

        btTransform wt;
        wt.setIdentity();
        wt.setOrigin(toBtVec3(wheels[static_cast<size_t>(i)].wheelCenterWorld));
        wt.setRotation(toBtQuat(computeWheelWorldOrientation(spawnRotation, wheels[static_cast<size_t>(i)])));
        m_wheelCollisionObjects[static_cast<size_t>(i)]->setWorldTransform(wt);

        if (m_world) {
            m_world->addCollisionObject(
                m_wheelCollisionObjects[static_cast<size_t>(i)].get(),
                btBroadphaseProxy::KinematicFilter,
                btBroadphaseProxy::StaticFilter
            );
        }
    }
    m_prevWheelInContact.fill(true);
    m_latchedCollision = CollisionImpactEvent{};
    m_latchedSuspension = SuspensionHapticEvent{};
    capturePreviousTransforms();
}

VehicleDynamics::~VehicleDynamics() {
    if (m_world) {
        for (int i = 0; i < 4; ++i) {
            if (m_wheelCollisionObjects[static_cast<size_t>(i)]) {
                m_world->removeCollisionObject(m_wheelCollisionObjects[static_cast<size_t>(i)].get());
            }
        }
    }
}

void VehicleDynamics::syncWheelCollisionObjects() {
    const Quat chassisRot = body.getRotation();
    for (int i = 0; i < 4; ++i) {
        if (!m_wheelCollisionObjects[static_cast<size_t>(i)]) continue;
        const WheelState& w = wheels[static_cast<size_t>(i)];
        const Quat wRot = computeWheelWorldOrientation(chassisRot, w);
        btTransform wt;
        wt.setIdentity();
        wt.setOrigin(toBtVec3(w.wheelCenterWorld));
        wt.setRotation(toBtQuat(wRot));
        m_wheelCollisionObjects[static_cast<size_t>(i)]->setWorldTransform(wt);
    }
}

void VehicleDynamics::capturePreviousTransforms() {
    m_prevChassisPos = body.getPosition();
    m_prevChassisRot = body.getRotation();
    for (int i = 0; i < 4; ++i) {
        m_prevWheelPos[static_cast<size_t>(i)] = wheels[static_cast<size_t>(i)].wheelCenterWorld;
        m_prevWheelRot[static_cast<size_t>(i)] = computeWheelWorldOrientation(
            m_prevChassisRot,
            wheels[static_cast<size_t>(i)]
        );
    }
}

void VehicleDynamics::step(double dt) {
    capturePreviousTransforms();
    body.prepareForSimulationStep();
    simTime += dt;

    if (body.getRigidBody()) {
        body.getRigidBody()->clearForces();
    }

    const Vec3 chassisPos = body.getPosition();
    const Quat chassisRot = body.getRotation();
    const VehicleBasis basis = getVehicleBasis(chassisRot);
    const Vec3 velWorld = body.getLinearVelocity();
    const Vec3 angVelWorld = body.getAngularVelocity();

    if (!isVec3Finite(chassisPos) || !isVec3Finite(velWorld) || !isVec3Finite(angVelWorld)) {
        resetVehicle();
        return;
    }

    const double forwardSpeed = vec3Dot(velWorld, basis.forward);

    // 0. Update speed-sensitive Ackermann steering ahead of wheel contact queries
    centerSteeringAngle = updateCenterSteeringAngle(
        centerSteeringAngle,
        currentInputs.steering,
        forwardSpeed,
        config,
        dt
    );
    const AckermannSteeringAngles ackermann = computeAckermannWheelAngles(centerSteeringAngle, config);
    wheels[0].steeringAngle = ackermann.frontLeftAngle;
    wheels[1].steeringAngle = ackermann.frontRightAngle;
    wheels[2].steeringAngle = 0.0;
    wheels[3].steeringAngle = 0.0;

    // Array of wheel collision objects to ignore during raycasting
    std::array<const btCollisionObject*, 4> wheelObjsToIgnore{};
    for (int i = 0; i < 4; ++i) {
        wheelObjsToIgnore[static_cast<size_t>(i)] = m_wheelCollisionObjects[static_cast<size_t>(i)].get();
    }

    // 1. Update local suspension mount positions & execute Volumetric Circumferential Tyre Collision Query
    const auto localMounts = getWheelMountPositionsLocal(config);
    const Vec3 chassisDown = basis.down;

    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        const bool wasInContact = w.inContact;
        w.prevSuspensionLength = w.suspensionLength;

        const double wRadius = getWheelRadius(config, i);
        const double wWidth  = getWheelWidth(config, i);
        w.wheelRadius = wRadius;
        w.wheelWidth  = wWidth;

        w.mountPointLocal = localMounts[static_cast<size_t>(i)];
        w.mountPointWorld = localPointToWorld(
            localMounts[static_cast<size_t>(i)],
            chassisPos,
            chassisRot
        );
        const Vec3 mountVelWorld = body.getVelocityAtWorldPoint(w.mountPointWorld);

        // Volumetric circumferential collision query across tyre outer radius and tread width
        VolumetricContactOutput volContact = evaluateVolumetricWheelContact(
            m_world,
            m_surfaceSystem,
            w.mountPointWorld,
            chassisRot,
            chassisDown,
            w.steeringAngle,
            wRadius,
            wWidth,
            config.suspensionMinLength,
            config.suspensionMaxLength,
            config.suspensionRestLength,
            body.getRigidBody(),
            wheelObjsToIgnore,
            config.wheelCollisionFidelity
        );

        if (volContact.inContact) {
            w.inContact = true;
            w.contactPointWorld = volContact.contactPointWorld;
            w.contactNormalWorld = volContact.contactNormalWorld;
            w.circumferentialHitAngle = volContact.circumferentialHitAngle;
            w.lateralContactOffset = volContact.lateralOffset;
            w.hasObstacleContact = volContact.hasObstacleContact;

            if (m_surfaceSystem && volContact.colliderHandle >= 0) {
                w.surface = m_surfaceSystem->getSurfaceForCollider(volContact.colliderHandle);
            }

            double roughnessOffset = 0.0;
            if (w.surface.roughnessAmplitude > 0.0) {
                const double speedHoriz =
                    std::hypot(w.patchSpeedLongitudinal, w.patchSpeedLateral);
                const double speedFactor = clamp(speedHoriz / 2.5, 0.0, 1.0);
                const double spatialPhase =
                    (volContact.contactPointWorld.x * 1.3 + volContact.contactPointWorld.z) *
                    w.surface.roughnessFrequency;
                roughnessOffset =
                    w.surface.roughnessAmplitude * speedFactor * std::sin(spatialPhase);
            }

            const double rawLength = volContact.effectiveSuspensionLength + roughnessOffset;

            // Series Compliance: Suspension spring and pneumatic tyre carcass act in series
            // 1/k_eff = 1/k_susp + 1/k_tire
            const double totalCompression = std::max(0.0, config.suspensionRestLength - rawLength);
            const double kSusp = w.isFront ? config.frontSpringStiffness : config.rearSpringStiffness;
            const double kTire = w.tireModel.getState().radialStiffness;
            const double tireComplianceFraction = kSusp / (kSusp + kTire);
            const double targetTireDeflection = totalCompression * tireComplianceFraction;

            const double effectiveSuspLen = rawLength + targetTireDeflection;
            const double clampedLen = clamp(
                effectiveSuspLen,
                config.suspensionMinLength * 0.85,
                config.suspensionRestLength
            );

            // Relative suspension velocity v = dL/dt
            double lengthVel = 0.0;
            if (wasInContact && dt > 1e-6) {
                lengthVel = (clampedLen - w.prevSuspensionLength) / dt;
            } else {
                lengthVel = vec3Dot(mountVelWorld, basis.up);
            }
            lengthVel = clamp(lengthVel, -15.0, 15.0);
            w.suspensionVelocity = lengthVel;

            const SuspensionForceOutput suspOut = evaluateWheelSuspension(
                effectiveSuspLen,
                lengthVel,
                w.isFront,
                config
            );

            w.suspensionLength = suspOut.clampedLength;
            w.compression = suspOut.compression;
            w.compressionRatio = suspOut.compressionRatio;
            w.springForce = suspOut.springForce;
            w.damperForce = suspOut.damperForce;
            w.bumpStopForce = suspOut.bumpStopForce;
            w.isBottomedOut = suspOut.isBottomedOut;
            w.wheelCenterWorld = vec3Add(w.mountPointWorld, vec3Scale(chassisDown, w.suspensionLength));
        } else {
            // Wheel is airborne
            w.inContact = false;
            w.hasObstacleContact = false;
            w.circumferentialHitAngle = 0.0;
            w.lateralContactOffset = 0.0;
            w.suspensionLength = config.suspensionRestLength;
            w.compression = 0.0;
            w.compressionRatio = 0.0;
            w.suspensionVelocity = 0.0;
            w.springForce = 0.0;
            w.damperForce = 0.0;
            w.bumpStopForce = 0.0;
            w.antiRollForce = 0.0;
            w.normalLoad = 0.0;
            w.isBottomedOut = false;
            w.wheelCenterWorld = vec3Add(w.mountPointWorld, vec3Scale(chassisDown, w.suspensionLength));
            w.contactPointWorld = vec3Add(w.wheelCenterWorld, vec3Scale(chassisDown, wRadius));
            w.contactNormalWorld = basis.up;
            w.obstacleResistanceForceWorld = Vec3{0.0, 0.0, 0.0};
            w.tractionClimbingForceWorld = Vec3{0.0, 0.0, 0.0};

            const Quat steerQuat = quatFromAxisAngle(basis.up, w.steeringAngle);
            const Quat steeredRot = quatMultiply(chassisRot, steerQuat);
            const VehicleBasis steeredBasis = getVehicleBasis(steeredRot);
            w.tireModel.evaluateDistributedContact(
                dt,
                0.0,
                0.0,
                w.angularVelocity,
                w.wheelCenterWorld,
                w.worldOrientation,
                steeredBasis,
                basis.down,
                w.surface,
                config.wheelCollisionFidelity,
                {},
                0.0,
                0.0
            );
        }
    }

    // Detect suspension haptic events (jump landings, bump-stop bottom-out, sharp curb/bump strikes)
    for (int i = 0; i < 4; ++i) {
        const WheelState& w = wheels[static_cast<size_t>(i)];
        const bool wasInContact = m_prevWheelInContact[static_cast<size_t>(i)];
        const float latBias = (i == 0 || i == 2) ? +0.75f : -0.75f;

        if (w.inContact) {
            // Touchdown / Jump Landing
            if (!wasInContact) {
                const Vec3 mountVelWorld = body.getVelocityAtWorldPoint(w.mountPointWorld);
                const double downwardMountSpeed = -vec3Dot(mountVelWorld, basis.up);
                if (downwardMountSpeed > 0.6) {
                    const float sev = static_cast<float>(clamp((downwardMountSpeed - 0.6) / 2.5, 0.15, 1.0));
                    if (sev > m_latchedSuspension.severity) {
                        m_latchedSuspension.occurred = true;
                        m_latchedSuspension.severity = sev;
                        m_latchedSuspension.lateralBias = latBias;
                        m_latchedSuspension.isHardLanding = true;
                        m_latchedSuspension.isBottomOut = false;
                    }
                }
            }

            // Bump-stop bottom-out (progressive travel limit)
            if (w.isBottomedOut || w.bumpStopForce > 800.0) {
                const float sev = static_cast<float>(clamp(w.bumpStopForce / 8000.0, 0.35, 1.0));
                if (sev > m_latchedSuspension.severity) {
                    m_latchedSuspension.occurred = true;
                    m_latchedSuspension.severity = sev;
                    m_latchedSuspension.lateralBias = latBias;
                    m_latchedSuspension.isHardLanding = false;
                    m_latchedSuspension.isBottomOut = true;
                }
            }

            // Sharp bump / curb strike (rapid compression velocity dL/dt < -2.2 m/s)
            if (w.suspensionVelocity < -2.2) {
                const float sev = static_cast<float>(clamp((-w.suspensionVelocity - 2.2) / 4.0, 0.12, 0.70));
                if (sev > m_latchedSuspension.severity) {
                    m_latchedSuspension.occurred = true;
                    m_latchedSuspension.severity = sev;
                    m_latchedSuspension.lateralBias = latBias;
                    m_latchedSuspension.isHardLanding = false;
                    m_latchedSuspension.isBottomOut = false;
                }
            }
        }
        m_prevWheelInContact[static_cast<size_t>(i)] = w.inContact;
    }

    // 2. Compute Front & Rear Anti-Roll Bar forces
    const AxleAntiRollResult frontArb = computeAxleAntiRollForce(
        wheels[0].compression,
        wheels[1].compression,
        wheels[0].inContact,
        wheels[1].inContact,
        config.frontAntiRollStiffness
    );
    wheels[0].antiRollForce = frontArb.leftForce;
    wheels[1].antiRollForce = frontArb.rightForce;

    const AxleAntiRollResult rearArb = computeAxleAntiRollForce(
        wheels[2].compression,
        wheels[3].compression,
        wheels[2].inContact,
        wheels[3].inContact,
        config.rearAntiRollStiffness
    );
    wheels[2].antiRollForce = rearArb.leftForce;
    wheels[3].antiRollForce = rearArb.rightForce;

    // 3. Finalize vertical tire normal load F_z, apply suspension force & obstacle interaction forces
    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        if (w.inContact) {
            const double rawNormalForce =
                w.springForce + w.damperForce + w.bumpStopForce + w.antiRollForce;
            const double normalAlignment = clamp(vec3Dot(w.contactNormalWorld, basis.up), 0.20, 1.0);
            const double maxNormalLoad = config.mass * GRAVITY * 3.5; // Structural physical load ceiling (~51 kN)
            w.normalLoad = clamp(rawNormalForce * normalAlignment, 0.0, maxNormalLoad);

            if (w.normalLoad > 0.0) {
                const Vec3 suspForceWorld = vec3Scale(basis.up, w.normalLoad);
                body.applyForceAtWorldPoint(suspForceWorld, w.mountPointWorld);

                // Multi-directional obstacle contact interaction (curbs, bumps, platforms, steps)
                if (w.hasObstacleContact) {
                    const Quat steerQ = quatFromAxisAngle(basis.up, w.steeringAngle);
                    const Vec3 steeredFwd = quatRotateVec3(steerQ, basis.forward);

                    // Decompose normal into vertical and horizontal components
                    const double nVert = vec3Dot(w.contactNormalWorld, basis.up);
                    const Vec3 nHoriz = vec3Sub(w.contactNormalWorld, vec3Scale(basis.up, nVert));
                    const double nHorizLen = vec3Length(nHoriz);

                    // 1. Horizontal obstacle reaction force opposing vehicle motion into obstacle
                    if (nHorizLen > 0.05) {
                        const Vec3 nHorizUnit = vec3Scale(nHoriz, 1.0 / nHorizLen);
                        const double obsForceMag = clamp(
                            w.normalLoad * (nHorizLen / std::max(0.20, nVert)),
                            0.0,
                            1.2 * w.normalLoad
                        );
                        // The obstacle reaction opposes velocity / forward movement into the obstacle:
                        Vec3 obsDir = nHorizUnit;
                        if (vec3Dot(obsDir, steeredFwd) > 0.0) {
                            obsDir = vec3Scale(obsDir, -1.0);
                        }
                        const Vec3 obsForce = vec3Scale(obsDir, obsForceMag);
                        w.obstacleResistanceForceWorld = obsForce;
                        body.applyForceAtWorldPoint(obsForce, w.contactPointWorld);
                    } else {
                        w.obstacleResistanceForceWorld = Vec3{0.0, 0.0, 0.0};
                    }

                    // 2. Circumferential drive traction pulling wheel UP and OVER the obstacle
                    if (w.driveTorque > 10.0 && w.angularVelocity > 0.1) {
                        const double treadSpeed = w.angularVelocity * w.wheelRadius;
                        const double fwdSpeed = vec3Dot(velWorld, steeredFwd);
                        const double climbSlip = clamp((treadSpeed - fwdSpeed) / 1.5, 0.0, 1.0);
                        const double climbMag = w.normalLoad * w.effectiveFrictionCoeff * climbSlip * 0.70;
                        const Vec3 climbDir = vec3Normalize(vec3Add(basis.up, vec3Scale(steeredFwd, 0.5)));
                        const Vec3 climbForce = vec3Scale(climbDir, climbMag);
                        w.tractionClimbingForceWorld = climbForce;
                        body.applyForceAtWorldPoint(climbForce, w.contactPointWorld);
                    } else {
                        w.tractionClimbingForceWorld = Vec3{0.0, 0.0, 0.0};
                    }
                } else {
                    w.obstacleResistanceForceWorld = Vec3{0.0, 0.0, 0.0};
                    w.tractionClimbingForceWorld = Vec3{0.0, 0.0, 0.0};
                }
            }
        } else {
            w.normalLoad = 0.0;
            w.obstacleResistanceForceWorld = Vec3{0.0, 0.0, 0.0};
            w.tractionClimbingForceWorld = Vec3{0.0, 0.0, 0.0};

            const Quat steerQuat = quatFromAxisAngle(basis.up, w.steeringAngle);
            const Quat steeredRot = quatMultiply(chassisRot, steerQuat);
            const VehicleBasis steeredBasis = getVehicleBasis(steeredRot);
            w.tireModel.evaluateDistributedContact(
                dt,
                0.0,
                0.0,
                w.angularVelocity,
                w.wheelCenterWorld,
                w.worldOrientation,
                steeredBasis,
                basis.down,
                w.surface,
                config.wheelCollisionFidelity,
                {},
                0.0,
                0.0
            );
        }
    }

    // 4. Steering angles were already updated at the start of step() for circumferential probe accuracy

    // 5. Auto-reverse convenience in Automatic mode
    double effectiveThrottle = clamp(currentInputs.throttle, 0.0, 1.0);
    double effectiveBrake = clamp(currentInputs.brake, 0.0, 1.0);

    if (drivetrain.transmissionMode == TransmissionMode::Automatic && !currentInputs.disableAutoReverse) {
        if (drivetrain.gearIndex >= 2) {
            if (forwardSpeed < 0.35 && effectiveBrake > 0.15 && effectiveThrottle < 0.05) {
                shiftToGear(drivetrain, 0, config); // Shift to Reverse
            }
        } else if (drivetrain.gearIndex == 0) {
            if (forwardSpeed > -0.35 && effectiveThrottle > 0.15 && effectiveBrake < 0.05) {
                shiftToGear(drivetrain, 2, config); // Shift to 1st
            } else {
                std::swap(effectiveThrottle, effectiveBrake);
            }
        }
    }

    // 6. Update Drivetrain & Brake Torques
    updateDrivetrain(
        drivetrain,
        wheels,
        effectiveThrottle,
        forwardSpeed,
        config,
        dt
    );

    const WheelBrakeTorques brakeTorques = computeWheelBrakeTorques(
        effectiveBrake,
        currentInputs.handbrake,
        config
    );
    wheels[0].brakeTorque = brakeTorques.fl;
    wheels[1].brakeTorque = brakeTorques.fr;
    wheels[2].brakeTorque = brakeTorques.rl;
    wheels[3].brakeTorque = brakeTorques.rr;

    // 7. Sub-stepped Wheel Rotational Integration + Tire Slip & Friction Circle Forces
    constexpr int wheelSubSteps = 8;
    const double subDt = dt / static_cast<double>(wheelSubSteps);
    const double baseWheelInertia = std::max(0.1, config.wheelInertia);
    const double absTotalGearRatio = std::abs(getTotalGearRatio(drivetrain.gearIndex, config));
    const bool isFrontDriven = (config.drivetrainLayout == DrivetrainLayout::FWD ||
                                config.drivetrainLayout == DrivetrainLayout::AWD);
    const bool isRearDriven = (config.drivetrainLayout == DrivetrainLayout::RWD ||
                               config.drivetrainLayout == DrivetrainLayout::AWD);
    const double numDrivenWheels = (isFrontDriven && isRearDriven) ? 4.0 : 2.0;

    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        const bool isWheelDriven = (i < 2) ? isFrontDriven : isRearDriven;
        double effectiveWheelInertia = baseWheelInertia;
        if (isWheelDriven && drivetrain.clutchEngagement > 0.01 && absTotalGearRatio > 0.1 && effectiveThrottle > 0.05) {
            const double reflectedEngineInertia =
                drivetrain.clutchEngagement *
                (config.engineInertia * absTotalGearRatio * absTotalGearRatio) / numDrivenWheels;
            effectiveWheelInertia += reflectedEngineInertia * 0.25;
        }
        const double invInertia = 1.0 / effectiveWheelInertia;

        const Quat steerQuat = quatFromAxisAngle(basis.up, w.steeringAngle);
        const Vec3 steeredForward = quatRotateVec3(steerQuat, basis.forward);
        const Vec3 normal = w.inContact ? w.contactNormalWorld : basis.up;
        const double normalProj = vec3Dot(steeredForward, normal);
        const Vec3 projectedForward = vec3Sub(steeredForward, vec3Scale(normal, normalProj));
        const Vec3 tireForward = vec3Normalize(projectedForward);
        const Vec3 tireLeft = vec3Normalize(vec3Cross(normal, tireForward));

        const Vec3 patchPoint = w.inContact ? w.contactPointWorld : w.wheelCenterWorld;
        const Vec3 patchVelWorld = body.getVelocityAtWorldPoint(patchPoint);
        const double vx = vec3Dot(patchVelWorld, tireForward);
        const double vy = vec3Dot(patchVelWorld, tireLeft);
        w.patchSpeedLongitudinal = vx;
        w.patchSpeedLateral = vy;

        w.rollingResistanceTorque = computeWheelRollingResistanceTorque(
            w.normalLoad,
            w.wheelRadius,
            w.angularVelocity,
            w.surface.rollingResistanceMultiplier,
            config
        );

        if (!w.inContact || w.normalLoad <= 0.0) {
            // Airborne wheel rotational integration
            for (int s = 0; s < wheelSubSteps; ++s) {
                const double bearingDrag = -0.08 * w.angularVelocity;
                const double omegaStar =
                    w.angularVelocity + (w.driveTorque + bearingDrag) * invInertia * subDt;
                const double maxBrakeDelta = w.brakeTorque * invInertia * subDt;
                if (std::abs(omegaStar) <= maxBrakeDelta) {
                    w.angularVelocity = 0.0;
                } else {
                    w.angularVelocity = omegaStar - std::copysign(maxBrakeDelta, omegaStar);
                }
                w.angularVelocity = clamp(w.angularVelocity, -260.0, 260.0);
            }
            w.rotationAngle += w.angularVelocity * dt;
            w.worldOrientation = computeWheelWorldOrientation(chassisRot, w);
            w.slipRatio = 0.0;
            w.slipAngle = 0.0;
            w.effectiveFrictionCoeff = 0.0;
            w.maxAvailableGripForce = 0.0;
            w.longitudinalForce = 0.0;
            w.lateralForce = 0.0;
            w.gripUsageRatio = 0.0;
            w.isSkidding = false;
            w.tireForceWorld = Vec3{0.0, 0.0, 0.0};
            continue;
        }

        double accumFx = 0.0;
        double accumFy = 0.0;
        TireForceOutput lastTireOut{};

        const double patchSpeed = std::hypot(vx, vy);
        const bool isNearStandstill = (patchSpeed < 0.25 && std::abs(forwardSpeed) < 0.25);
        const bool isThrottleIdle = (effectiveThrottle < 0.03);

        if (config.enableStationaryHold && isNearStandstill && isThrottleIdle && std::abs(w.driveTorque) < 5.0) {
            w.angularVelocity = 0.0;
        }

        for (int s = 0; s < wheelSubSteps; ++s) {
            const double slipRatio = computeLongitudinalSlipRatio(
                w.angularVelocity,
                w.wheelRadius,
                vx
            );
            const double slipAngle = computeLateralSlipAngle(vx, vy);

            const TireForceOutput tireOut = computeTireForcesFromSlip(
                slipRatio,
                slipAngle,
                w.normalLoad,
                w.surface.frictionMultiplier,
                w.surface.stiffnessMultiplier,
                config
            );
            lastTireOut = tireOut;

            const double bearingDrag = -0.12 * w.angularVelocity;
            const double roadReactionTorque = -w.wheelRadius * tireOut.longitudinalForce;
            double subDriveTorque = w.driveTorque;
            if (isWheelDriven && absTotalGearRatio > 0.1 && subDriveTorque * w.angularVelocity > 0.0) {
                const double maxRedlineOmega = (config.maxRpm * (PI_D / 30.0)) / absTotalGearRatio;
                const double absOmega = std::abs(w.angularVelocity);
                const double taperStart = maxRedlineOmega * 0.94;
                if (absOmega >= maxRedlineOmega) {
                    subDriveTorque = 0.0;
                } else if (absOmega > taperStart) {
                    const double taperFactor = (maxRedlineOmega - absOmega) / (maxRedlineOmega - taperStart);
                    subDriveTorque *= clamp(taperFactor, 0.0, 1.0);
                }
            }
            const double netActiveTorque = subDriveTorque + roadReactionTorque + bearingDrag;
            const double omegaStar = w.angularVelocity + netActiveTorque * invInertia * subDt;

            const double maxBrakeDelta = w.brakeTorque * invInertia * subDt;
            if (std::abs(omegaStar) <= maxBrakeDelta) {
                w.angularVelocity = 0.0;
            } else {
                w.angularVelocity = omegaStar - std::copysign(maxBrakeDelta, omegaStar);
            }
            if (config.enableStationaryHold && isNearStandstill && isThrottleIdle && std::abs(w.driveTorque) < 5.0) {
                w.angularVelocity = 0.0;
            }
            w.angularVelocity = clamp(w.angularVelocity, -260.0, 260.0);

            accumFx += tireOut.longitudinalForce;
            accumFy += tireOut.lateralForce;
        }

        const double avgFx = accumFx / static_cast<double>(wheelSubSteps);
        const double avgFy = accumFy / static_cast<double>(wheelSubSteps);

        double finalFx = avgFx;
        double finalFy = avgFy;

        // Static Coulomb friction (stiction) & zero-creep holding
        if (config.enableStationaryHold && isNearStandstill && isThrottleIdle) {
            // Static Coulomb friction opposing any residual micro-drift:
            // critically damps residual contact patch velocity to zero within ~2 frames
            const double maxStiction = lastTireOut.maxAvailableGripForce * 0.98;
            const double stictionFx = clamp(-vx * (config.mass * 8.0), -maxStiction, maxStiction);
            const double stictionFy = clamp(-vy * (config.mass * 8.0), -maxStiction, maxStiction);

            finalFx = stictionFx;
            finalFy = stictionFy;

            // Zero-creep rotational deadband: lock wheel rotation when stopped with no drive torque
            if (patchSpeed < 0.12 && std::abs(w.driveTorque) < 5.0) {
                w.angularVelocity = 0.0;
            }
        } else if (isNearStandstill && w.brakeTorque > 50.0 && isThrottleIdle) {
            const double maxHold = lastTireOut.maxAvailableGripForce * 0.95;
            const double holdFx = clamp(-vx * config.mass * 4.5, -maxHold, maxHold);
            finalFx = clamp(avgFx + holdFx, -maxHold, maxHold);
        } else if (patchSpeed < 1.0 && isThrottleIdle && std::abs(w.angularVelocity * w.wheelRadius - vx) < 0.25) {
            // Synchronize rolling angular speed during low-speed rolling to eliminate numerical sub-step chatter
            w.angularVelocity = vx / w.wheelRadius;
        }

        w.rotationAngle += w.angularVelocity * dt;
        w.worldOrientation = computeWheelWorldOrientation(chassisRot, w);

        w.slipRatio = lastTireOut.slipRatio;
        w.slipAngle = lastTireOut.slipAngle;
        w.effectiveFrictionCoeff = lastTireOut.effectiveFrictionCoeff;
        w.maxAvailableGripForce = lastTireOut.maxAvailableGripForce;
        w.longitudinalForce = finalFx;
        w.lateralForce = finalFy;
        w.gripUsageRatio = lastTireOut.gripUsageRatio;
        w.isSkidding = lastTireOut.isSkidding;

        const Vec3 tireForceVec = vec3Add(
            vec3Scale(tireForward, finalFx),
            vec3Scale(tireLeft, finalFy)
        );
        w.tireForceWorld = tireForceVec;

        // Per-wheel velocity-smooth rolling resistance force along tireForward
        const double crr = config.rollingResistanceCoeff * w.surface.rollingResistanceMultiplier;
        const double maxResistiveForce = crr * w.normalLoad;
        const double rrSmoothSign = clamp(vx / 0.4, -1.0, 1.0);
        const double rollingResistanceForce = -maxResistiveForce * rrSmoothSign;
        const Vec3 rollForceVec = vec3Scale(tireForward, rollingResistanceForce);

        // Evaluate pneumatic tyre deformation & multi-point contact patch
        const Quat steeredRot = quatMultiply(chassisRot, steerQuat);
        const VehicleBasis steeredBasis = getVehicleBasis(steeredRot);
        w.tireModel.evaluateDistributedContact(
            dt,
            w.normalLoad,
            w.compression,
            w.angularVelocity,
            w.wheelCenterWorld,
            w.worldOrientation,
            steeredBasis,
            basis.down,
            w.surface,
            config.wheelCollisionFidelity,
            {},
            vx,
            vy
        );

        if (w.inContact && w.normalLoad > 5.0 && !w.hasObstacleContact) {
            w.contactPointWorld = w.tireModel.getState().contactCentroidWorld;
        }

        const Vec3 totalHorizontalForce = vec3Add(tireForceVec, rollForceVec);
        const Vec3 forceAppPoint = vec3Add(
            w.contactPointWorld,
            vec3Scale(basis.up, config.tireForceApplicationHeightOffset)
        );
        body.applyForceAtWorldPoint(totalHorizontalForce, forceAppPoint);

        // Apply pneumatic trail self-aligning torque M_z to chassis body
        const double mz = w.tireModel.getState().selfAligningTorque;
        if (std::abs(mz) > 0.05) {
            body.applyTorque(vec3Scale(basis.up, mz));
        }
    }

    // 8. Aerodynamic Drag & Downforce
    aeroState = computeAerodynamicForces(velWorld, basis.forward, basis.up, config);
    const Vec3 totalAeroForce = vec3Add(aeroState.dragForceWorld, aeroState.downforceWorld);
    body.applyCentralForce(totalAeroForce);

    syncWheelCollisionObjects();
}

void VehicleDynamics::postWorldStep(double dt) {
    body.postStepUpdate(m_world, dt);

    if (body.lastImpactEvent.occurred) {
        if (!m_latchedCollision.occurred ||
            body.lastImpactEvent.impactSpeedMps > m_latchedCollision.impactSpeedMps ||
            body.lastImpactEvent.normalImpulse > m_latchedCollision.normalImpulse) {
            m_latchedCollision = body.lastImpactEvent;
        }
    }

    const Vec3 chassisPos = body.getPosition();
    const Quat chassisRot = body.getRotation();
    const VehicleBasis basis = getVehicleBasis(chassisRot);
    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        w.mountPointWorld = localPointToWorld(w.mountPointLocal, chassisPos, chassisRot);
        w.wheelCenterWorld = vec3Add(
            w.mountPointWorld,
            vec3Scale(basis.down, w.suspensionLength)
        );
        w.worldOrientation = computeWheelWorldOrientation(chassisRot, w);
    }
    syncWheelCollisionObjects();
}

void VehicleDynamics::resetVehicle(const Vec3* position, const Quat* rotation) {
    const double eqHeight = computeEquilibriumChassisHeight(config);
    const Vec3 defaultPos{m_initialSpawnPos.x, eqHeight, m_initialSpawnPos.z};
    const Vec3 pos = position ? *position : defaultPos;
    const Quat rot = rotation ? *rotation : m_initialSpawnRot;

    body.syncMassProperties(config);
    body.resetPose(pos, rot);

    centerSteeringAngle = 0.0;
    const TransmissionMode prevMode = drivetrain.transmissionMode;
    drivetrain = createInitialDrivetrainState(config.idleRpm);
    drivetrain.transmissionMode = prevMode;
    currentInputs = VehicleControlInputs{};

    const double frontLoad =
        0.5 * config.frontWeightDistribution * config.mass * GRAVITY;
    const double rearLoad =
        0.5 * (1.0 - config.frontWeightDistribution) * config.mass * GRAVITY;
    const double frontEqLen =
        config.suspensionRestLength - frontLoad / config.frontSpringStiffness;
    const double rearEqLen =
        config.suspensionRestLength - rearLoad / config.rearSpringStiffness;

    const auto mounts = getWheelMountPositionsLocal(config);
    const VehicleBasis basis = getVehicleBasis(rot);
    for (int i = 0; i < 4; ++i) {
        const double eqLen = (i < 2) ? frontEqLen : rearEqLen;
        const double r = getWheelRadius(config, i);
        const double wWidth = getWheelWidth(config, i);
        const double rimR = getTireRimRadius(config, i);
        const double pKpa = torquelab::getTirePressureKpa(config, i);

        wheels[static_cast<size_t>(i)] = createInitialWheelState(
            i,
            mounts[static_cast<size_t>(i)],
            eqLen,
            r,
            wWidth,
            rimR,
            pKpa
        );
        wheels[static_cast<size_t>(i)].mountPointWorld = localPointToWorld(
            mounts[static_cast<size_t>(i)],
            pos,
            rot
        );
        wheels[static_cast<size_t>(i)].wheelCenterWorld = vec3Add(
            wheels[static_cast<size_t>(i)].mountPointWorld,
            vec3Scale(basis.down, eqLen)
        );
        wheels[static_cast<size_t>(i)].worldOrientation = computeWheelWorldOrientation(rot, wheels[static_cast<size_t>(i)]);
    }
    syncWheelCollisionObjects();
    m_prevWheelInContact.fill(true);
    m_latchedCollision = CollisionImpactEvent{};
    m_latchedSuspension = SuspensionHapticEvent{};
    capturePreviousTransforms();
}

InterpolatedVehicleTransforms VehicleDynamics::getInterpolatedTransforms(double alpha) const {
    const double t = clamp(alpha, 0.0, 1.0);
    const Vec3 curPos = body.getPosition();
    const Quat curRot = body.getRotation();

    const Vec3 chassisPosition = vec3Lerp(m_prevChassisPos, curPos, t);
    const Quat chassisRotation = quatSlerp(m_prevChassisRot, curRot, t);

    std::array<Vec3, 4> wheelPositions{};
    std::array<Quat, 4> wheelRotations{};
    for (int i = 0; i < 4; ++i) {
        const WheelState& w = wheels[static_cast<size_t>(i)];
        const Quat curWheelRot = computeWheelWorldOrientation(curRot, w);
        wheelPositions[static_cast<size_t>(i)] = vec3Lerp(m_prevWheelPos[static_cast<size_t>(i)], w.wheelCenterWorld, t);
        wheelRotations[static_cast<size_t>(i)] = quatSlerp(m_prevWheelRot[static_cast<size_t>(i)], curWheelRot, t);
    }

    return InterpolatedVehicleTransforms{
        chassisPosition,
        chassisRotation,
        wheelPositions,
        wheelRotations
    };
}

VehicleTelemetrySnapshot VehicleDynamics::getTelemetrySnapshot() const {
    const Vec3 pos = body.getPosition();
    const Quat rot = body.getRotation();
    const Vec3 com = body.getCenterOfMassWorld();
    const Vec3 vel = body.getLinearVelocity();
    const Vec3 angVel = body.getAngularVelocity();
    const VehicleBasis basis = getVehicleBasis(rot);
    const VehicleEulerAngles euler = getVehicleEulerAngles(rot);

    const double speedMps = vec3Length(vel);
    const double forwardSpeedMps = vec3Dot(vel, basis.forward);
    const double lateralSpeedMps = vec3Dot(vel, basis.left);

    const double longitudinalAccelMps2 = vec3Dot(body.accelerationWorld, basis.forward);
    const double lateralAccelMps2 = vec3Dot(body.accelerationWorld, basis.left);

    const bool sanity =
        isVec3Finite(pos) &&
        isVec3Finite(vel) &&
        isVec3Finite(angVel) &&
        std::isfinite(drivetrain.rpm);

    VehicleTelemetrySnapshot snap{};
    snap.simTime = simTime;
    snap.chassisPosition = pos;
    snap.chassisRotation = rot;
    snap.centerOfMassWorld = com;
    snap.velocityWorld = vel;
    snap.angularVelocityWorld = angVel;
    snap.speedMps = speedMps;
    snap.speedKmh = speedMps * MPS_TO_KMH;
    snap.forwardSpeedMps = forwardSpeedMps;
    snap.lateralSpeedMps = lateralSpeedMps;
    snap.accelerationWorld = body.accelerationWorld;
    snap.longitudinalAccelMps2 = longitudinalAccelMps2;
    snap.lateralAccelMps2 = lateralAccelMps2;
    snap.longitudinalG = longitudinalAccelMps2 / GRAVITY;
    snap.lateralG = lateralAccelMps2 / GRAVITY;
    snap.pitchRad = euler.pitch;
    snap.rollRad = euler.roll;
    snap.yawRad = euler.yaw;
    snap.steeringAngleRad = centerSteeringAngle;
    snap.drivetrain = DrivetrainTelemetry{
        drivetrain.rpm,
        drivetrain.gearIndex,
        getGearDisplayName(drivetrain.gearIndex),
        drivetrain.transmissionMode,
        drivetrain.engineTorqueNm,
        drivetrain.clutchEngagement,
        drivetrain.revLimiterActive
    };
    snap.aero = aeroState;
    snap.wheels = wheels;
    snap.forwardDirWorld = basis.forward;
    snap.upDirWorld = basis.up;
    snap.lateralDirWorld = basis.left;
    snap.collisionContactCount = static_cast<int>(body.activeContacts.size());
    snap.collisionImpact = m_latchedCollision;
    snap.suspensionImpact = m_latchedSuspension;
    m_latchedCollision = CollisionImpactEvent{};
    m_latchedSuspension = SuspensionHapticEvent{};
    snap.numericalSanityOk = sanity;
    return snap;
}

Quat VehicleDynamics::computeWheelWorldOrientation(const Quat& chassisRot, const WheelState& w) const {
    const Quat steerQuat = quatFromAxisAngle(Vec3{0.0, 1.0, 0.0}, w.steeringAngle);
    const Quat spinQuat = quatFromAxisAngle(Vec3{1.0, 0.0, 0.0}, w.rotationAngle);
    return quatMultiply(chassisRot, quatMultiply(steerQuat, spinQuat));
}

void VehicleDynamics::setCollisionFidelity(WheelCollisionFidelity fidelity) {
    config.wheelCollisionFidelity = fidelity;
}

void VehicleDynamics::setTirePressureKpa(int wheelIndex, double pressureKpa) {
    if (wheelIndex >= 0 && wheelIndex < 4) {
        wheels[static_cast<size_t>(wheelIndex)].tireModel.setTargetPressure(pressureKpa * 1000.0);
    }
}

double VehicleDynamics::getTirePressureKpa(int wheelIndex) const {
    if (wheelIndex >= 0 && wheelIndex < 4) {
        return wheels[static_cast<size_t>(wheelIndex)].tireModel.getState().pressurePa / 1000.0;
    }
    return 220.0;
}

void VehicleDynamics::setTirePunctured(int wheelIndex, bool punctured, double leakRatePaPerSec) {
    if (wheelIndex >= 0 && wheelIndex < 4) {
        wheels[static_cast<size_t>(wheelIndex)].tireModel.setPunctured(punctured, leakRatePaPerSec);
    }
}

} // namespace torquelab
