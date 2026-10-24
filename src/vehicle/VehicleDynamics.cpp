#include "vehicle/VehicleDynamics.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace torquelab {

namespace {

struct ExcludeVehicleRayCallback : public btCollisionWorld::ClosestRayResultCallback {
    const btCollisionObject* m_ignoreObj{nullptr};

    ExcludeVehicleRayCallback(
        const btVector3& rayFromWorld,
        const btVector3& rayToWorld,
        const btCollisionObject* ignoreObj
    )
        : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld)
        , m_ignoreObj(ignoreObj)
    {}

    bool needsCollision(btBroadphaseProxy* proxy0) const override {
        if (proxy0 && proxy0->m_clientObject == m_ignoreObj) {
            return false;
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
    const SurfaceSystem* surfaceSystem = nullptr
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
        if (surfaceSystem && !surfaceSystem->isColliderDriveable(obj->getUserIndex())) {
            continue; // Skip obstacles/walls: suspension rays pass through to the ground plane beneath
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
    ExcludeVehicleRayCallback rayCallback(rayFrom, rayTo, ignoreObj);
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
        wheels[static_cast<size_t>(i)] = createInitialWheelState(
            i,
            mounts[static_cast<size_t>(i)],
            eqLen,
            config.wheelRadius
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
    }
    m_prevWheelInContact.fill(true);
    m_latchedCollision = CollisionImpactEvent{};
    m_latchedSuspension = SuspensionHapticEvent{};
    capturePreviousTransforms();
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

    // 1. Update local suspension mount positions and raycast along -basis.up
    const auto localMounts = getWheelMountPositionsLocal(config);
    const double rayStartOffset = 0.18;
    const double maxRayLength =
        rayStartOffset + config.suspensionMaxLength + config.wheelRadius + 0.05;
    const double maxContactRayDist = config.suspensionRestLength + config.wheelRadius;
    const Vec3 chassisDown = basis.down;

    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        const bool wasInContact = w.inContact;
        w.prevSuspensionLength = w.suspensionLength;

        w.mountPointLocal = localMounts[static_cast<size_t>(i)];
        w.mountPointWorld = localPointToWorld(
            localMounts[static_cast<size_t>(i)],
            chassisPos,
            chassisRot
        );
        const Vec3 mountVelWorld = body.getVelocityAtWorldPoint(w.mountPointWorld);

        const Vec3 rayOrigin = vec3Add(w.mountPointWorld, vec3Scale(basis.up, rayStartOffset));
        AnalyticalRayHit hit = castRayExactWorld(
            m_world,
            rayOrigin,
            chassisDown,
            maxRayLength,
            body.getRigidBody(),
            m_surfaceSystem
        );

        if (hit.hit && hit.normalWorld.y < 0.35) {
            // A surface with steep vertical slope (normal.y < 0.35) is a wall or obstacle barrier, not a driveable road!
            // Vehicle body collision is handled by Bullet's compound shape.
            hit.hit = false;
        }

        if (hit.hit) {
            const double distFromMount = std::max(0.02, hit.distance - rayStartOffset);
            const int colliderHandle = hit.collisionObject
                ? hit.collisionObject->getUserIndex()
                : -1;
            if (m_surfaceSystem) {
                w.surface = m_surfaceSystem->getSurfaceForCollider(colliderHandle);
            }

            if (distFromMount <= maxContactRayDist) {
                w.inContact = true;
                w.contactPointWorld = hit.pointWorld;

                Vec3 n = hit.normalWorld;
                if (vec3Dot(n, basis.up) < 0.0) {
                    n = vec3Scale(n, -1.0);
                }
                w.contactNormalWorld = vec3Normalize(n);

                double roughnessOffset = 0.0;
                if (w.surface.roughnessAmplitude > 0.0) {
                    const double speedHoriz =
                        std::hypot(w.patchSpeedLongitudinal, w.patchSpeedLateral);
                    const double speedFactor = clamp(speedHoriz / 2.5, 0.0, 1.0);
                    const double spatialPhase =
                        (hit.pointWorld.x * 1.3 + hit.pointWorld.z) *
                        w.surface.roughnessFrequency;
                    roughnessOffset =
                        w.surface.roughnessAmplitude * speedFactor * std::sin(spatialPhase);
                }

                const double rawLength = distFromMount - config.wheelRadius + roughnessOffset;
                const double clampedLen = clamp(
                    rawLength,
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
                    rawLength,
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
                continue;
            }
        }

        // Wheel is airborne
        w.inContact = false;
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
        w.contactPointWorld = vec3Add(w.wheelCenterWorld, vec3Scale(chassisDown, config.wheelRadius));
        w.contactNormalWorld = basis.up;
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

    // 3. Finalize vertical tire normal load F_z and apply suspension force at upper mount points
    for (int i = 0; i < 4; ++i) {
        WheelState& w = wheels[static_cast<size_t>(i)];
        if (w.inContact) {
            const double rawNormalForce =
                w.springForce + w.damperForce + w.bumpStopForce + w.antiRollForce;
            const double normalAlignment = clamp(vec3Dot(w.contactNormalWorld, basis.up), 0.25, 1.0);
            const double maxNormalLoad = config.mass * GRAVITY * 3.5; // Structural physical load ceiling (~51 kN)
            w.normalLoad = clamp(rawNormalForce * normalAlignment, 0.0, maxNormalLoad);

            if (w.normalLoad > 0.0) {
                const Vec3 suspForceWorld = vec3Scale(basis.up, w.normalLoad);
                body.applyForceAtWorldPoint(suspForceWorld, w.mountPointWorld);
            }
        } else {
            w.normalLoad = 0.0;
        }
    }

    // 4. Update speed-sensitive Ackermann steering
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
            config.wheelRadius,
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

        for (int s = 0; s < wheelSubSteps; ++s) {
            const double slipRatio = computeLongitudinalSlipRatio(
                w.angularVelocity,
                config.wheelRadius,
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

            const double roadReactionTorque = -config.wheelRadius * tireOut.longitudinalForce;
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
            const double netActiveTorque = subDriveTorque + roadReactionTorque;
            const double omegaStar = w.angularVelocity + netActiveTorque * invInertia * subDt;

            const double maxBrakeDelta = w.brakeTorque * invInertia * subDt;
            if (std::abs(omegaStar) <= maxBrakeDelta) {
                w.angularVelocity = 0.0;
            } else {
                w.angularVelocity = omegaStar - std::copysign(maxBrakeDelta, omegaStar);
            }
            w.angularVelocity = clamp(w.angularVelocity, -260.0, 260.0);

            accumFx += tireOut.longitudinalForce;
            accumFy += tireOut.lateralForce;
        }

        w.rotationAngle += w.angularVelocity * dt;

        const double avgFx = accumFx / static_cast<double>(wheelSubSteps);
        const double avgFy = accumFy / static_cast<double>(wheelSubSteps);

        // Low-speed parking hold damping when brakes are applied and vehicle is nearly stopped
        // (only when throttle is not actively applied, so parking hold never fights launch)
        double finalFx = avgFx;
        const double finalFy = avgFy;
        if (std::hypot(vx, vy) < 0.8 && w.brakeTorque > 50.0 && effectiveThrottle < 0.05) {
            const double maxHold = lastTireOut.maxAvailableGripForce * 0.95;
            const double holdFx = clamp(-vx * config.mass * 4.5, -maxHold, maxHold);
            finalFx = clamp(avgFx + holdFx, -maxHold, maxHold);
        }

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

        const Vec3 totalHorizontalForce = vec3Add(tireForceVec, rollForceVec);
        const Vec3 forceAppPoint = vec3Add(
            w.contactPointWorld,
            vec3Scale(basis.up, config.tireForceApplicationHeightOffset)
        );
        body.applyForceAtWorldPoint(totalHorizontalForce, forceAppPoint);
    }

    // 8. Aerodynamic Drag & Downforce
    aeroState = computeAerodynamicForces(velWorld, basis.forward, basis.up, config);
    const Vec3 totalAeroForce = vec3Add(aeroState.dragForceWorld, aeroState.downforceWorld);
    body.applyCentralForce(totalAeroForce);
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
    }
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
        wheels[static_cast<size_t>(i)] = createInitialWheelState(
            i,
            mounts[static_cast<size_t>(i)],
            eqLen,
            config.wheelRadius
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
    }
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

} // namespace torquelab
