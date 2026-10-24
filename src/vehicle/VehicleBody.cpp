#include "vehicle/VehicleBody.h"
#include "physics/VehicleCollisionGenerator.h"
#include <algorithm>
#include <cmath>

namespace torquelab {

namespace {

inline btVector3 toBtVec3(const Vec3& v) {
    return btVector3(
        static_cast<btScalar>(v.x),
        static_cast<btScalar>(v.y),
        static_cast<btScalar>(v.z)
    );
}

inline Vec3 fromBtVec3(const btVector3& v) {
    return Vec3{
        static_cast<double>(v.x()),
        static_cast<double>(v.y()),
        static_cast<double>(v.z())
    };
}

inline btQuaternion toBtQuat(const Quat& q) {
    return btQuaternion(
        static_cast<btScalar>(q.x),
        static_cast<btScalar>(q.y),
        static_cast<btScalar>(q.z),
        static_cast<btScalar>(q.w)
    );
}

inline Quat fromBtQuat(const btQuaternion& q) {
    return quatNormalize(Quat{
        static_cast<double>(q.x()),
        static_cast<double>(q.y()),
        static_cast<double>(q.z()),
        static_cast<double>(q.w())
    });
}

std::unique_ptr<btCompoundShape> buildCompoundShapeForVehicle(
    const VehicleConfig& config,
    const Vec3& localCom,
    const GeneratedVehicleCollision* generatedCollision,
    GeneratedVehicleCollision& outCollision,
    std::vector<std::unique_ptr<btCollisionShape>>& outSubShapes
) {
    auto compoundShape = std::make_unique<btCompoundShape>();
    outSubShapes.clear();

    if (generatedCollision && !generatedCollision->pieces.empty()) {
        outCollision = *generatedCollision;
        compoundShape = VehicleCollisionGenerator::createBulletCompoundShape(
            outCollision,
            localCom,
            outSubShapes
        );
    } else {
        outCollision = GeneratedVehicleCollision{};
        const double hx = config.chassisHalfExtents.x;
        const double hy = config.chassisHalfExtents.y;
        const double hz = config.chassisHalfExtents.z;

        if (config.collisionShapeType == CollisionShapeType::TaperedCompound) {
            // 1. Central Passenger Tub Box (Z in [-0.40 * hz, +0.40 * hz])
            auto midBox = std::make_unique<btBoxShape>(btVector3(
                static_cast<btScalar>(hx),
                static_cast<btScalar>(hy * 0.55),
                static_cast<btScalar>(hz * 0.40)
            ));
            btTransform midTrans;
            midTrans.setIdentity();
            midTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, localCom)));
            compoundShape->addChildShape(midTrans, midBox.get());
            outSubShapes.push_back(std::move(midBox));

            // 2. Tapered Front Nose & Curved Bumper Hull (Z in [+0.40 * hz, +hz])
            auto frontNose = std::make_unique<btConvexHullShape>();
            const double frontHalfZ = hz * 0.30;
            const double taperFrac = std::clamp(config.noseTaperWidthFraction, 0.60, 0.95);
            const double wJunction = hx * 0.95;
            const double wTip = hx * taperFrac;
            const double hJunction = hy * 0.55;
            const double hTip = hy * 0.50;

            // Junction vertices (z = -frontHalfZ in nose local space)
            frontNose->addPoint(btVector3( static_cast<btScalar>(wJunction),  static_cast<btScalar>(hJunction), static_cast<btScalar>(-frontHalfZ)));
            frontNose->addPoint(btVector3(-static_cast<btScalar>(wJunction),  static_cast<btScalar>(hJunction), static_cast<btScalar>(-frontHalfZ)));
            frontNose->addPoint(btVector3( static_cast<btScalar>(wJunction), -static_cast<btScalar>(hJunction), static_cast<btScalar>(-frontHalfZ)));
            frontNose->addPoint(btVector3(-static_cast<btScalar>(wJunction), -static_cast<btScalar>(hJunction), static_cast<btScalar>(-frontHalfZ)));

            // Front bumper corners (z = +frontHalfZ)
            frontNose->addPoint(btVector3( static_cast<btScalar>(wTip),  static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ)));
            frontNose->addPoint(btVector3(-static_cast<btScalar>(wTip),  static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ)));
            frontNose->addPoint(btVector3( static_cast<btScalar>(wTip), -static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ)));
            frontNose->addPoint(btVector3(-static_cast<btScalar>(wTip), -static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ)));

            // Front center apex curve (eliminates flat front corners)
            frontNose->addPoint(btVector3(0.0f,  static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ + 0.05)));
            frontNose->addPoint(btVector3(0.0f, -static_cast<btScalar>(hTip), static_cast<btScalar>(frontHalfZ + 0.05)));

            frontNose->recalcLocalAabb();
            frontNose->setMargin(static_cast<btScalar>(0.04));

            btTransform frontTrans;
            frontTrans.setIdentity();
            frontTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.22, hz * 0.70}, localCom)));
            compoundShape->addChildShape(frontTrans, frontNose.get());
            outSubShapes.push_back(std::move(frontNose));

            // 3. Tapered Rear Bumper Hull (Z in [-hz, -0.40 * hz])
            auto rearBumper = std::make_unique<btConvexHullShape>();
            const double rearHalfZ = hz * 0.30;
            const double rearTaperFrac = std::clamp(config.rearTaperWidthFraction, 0.65, 0.98);
            const double wRearTip = hx * rearTaperFrac;
            const double hRearTip = hy * 0.52;

            // Junction vertices (z = +rearHalfZ in rear local space)
            rearBumper->addPoint(btVector3( static_cast<btScalar>(wJunction),  static_cast<btScalar>(hJunction), static_cast<btScalar>(rearHalfZ)));
            rearBumper->addPoint(btVector3(-static_cast<btScalar>(wJunction),  static_cast<btScalar>(hJunction), static_cast<btScalar>(rearHalfZ)));
            rearBumper->addPoint(btVector3( static_cast<btScalar>(wJunction), -static_cast<btScalar>(hJunction), static_cast<btScalar>(rearHalfZ)));
            rearBumper->addPoint(btVector3(-static_cast<btScalar>(wJunction), -static_cast<btScalar>(hJunction), static_cast<btScalar>(rearHalfZ)));

            // Rear bumper corners (z = -rearHalfZ)
            rearBumper->addPoint(btVector3( static_cast<btScalar>(wRearTip),  static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ)));
            rearBumper->addPoint(btVector3(-static_cast<btScalar>(wRearTip),  static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ)));
            rearBumper->addPoint(btVector3( static_cast<btScalar>(wRearTip), -static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ)));
            rearBumper->addPoint(btVector3(-static_cast<btScalar>(wRearTip), -static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ)));

            // Rear center apex curve
            rearBumper->addPoint(btVector3(0.0f,  static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ - 0.04)));
            rearBumper->addPoint(btVector3(0.0f, -static_cast<btScalar>(hRearTip), static_cast<btScalar>(-rearHalfZ - 0.04)));

            rearBumper->recalcLocalAabb();
            rearBumper->setMargin(static_cast<btScalar>(0.04));

            btTransform rearTrans;
            rearTrans.setIdentity();
            rearTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.23, -hz * 0.70}, localCom)));
            compoundShape->addChildShape(rearTrans, rearBumper.get());
            outSubShapes.push_back(std::move(rearBumper));

            // 4. Cabin Greenhouse Box
            auto cabinBox = std::make_unique<btBoxShape>(btVector3(
                static_cast<btScalar>(hx * 0.80),
                static_cast<btScalar>(hy * 0.55),
                static_cast<btScalar>(hz * 0.48)
            ));
            btTransform cabinTrans;
            cabinTrans.setIdentity();
            cabinTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, localCom)));
            compoundShape->addChildShape(cabinTrans, cabinBox.get());
            outSubShapes.push_back(std::move(cabinBox));

        } else {
            // Standard 2-box collision configuration
            auto lowerBox = std::make_unique<btBoxShape>(btVector3(
                static_cast<btScalar>(hx),
                static_cast<btScalar>(hy * 0.55),
                static_cast<btScalar>(hz)
            ));
            btTransform lowerLocalTrans;
            lowerLocalTrans.setIdentity();
            lowerLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, localCom)));
            compoundShape->addChildShape(lowerLocalTrans, lowerBox.get());
            outSubShapes.push_back(std::move(lowerBox));

            auto cabinBox = std::make_unique<btBoxShape>(btVector3(
                static_cast<btScalar>(hx * 0.84),
                static_cast<btScalar>(hy * 0.55),
                static_cast<btScalar>(hz * 0.48)
            ));
            btTransform cabinLocalTrans;
            cabinLocalTrans.setIdentity();
            cabinLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, localCom)));
            compoundShape->addChildShape(cabinLocalTrans, cabinBox.get());
            outSubShapes.push_back(std::move(cabinBox));
        }
    }

    return compoundShape;
}

} // namespace

VehicleBody::VehicleBody(
    btDiscreteDynamicsWorld* world,
    const VehicleConfig& config,
    const Vec3& initialPosition,
    const Quat& initialRotation,
    const GeneratedVehicleCollision* generatedCollision
)
    : m_world(world)
    , m_localCom(getCenterOfMassLocal(config))
{
    m_compoundShape = buildCompoundShapeForVehicle(
        config,
        m_localCom,
        generatedCollision,
        m_collision,
        m_subShapes
    );

    const Vec3 initialComWorld = localPointToWorld(m_localCom, initialPosition, initialRotation);
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(toBtVec3(initialComWorld));
    startTransform.setRotation(toBtQuat(initialRotation));

    m_motionState = std::make_unique<btDefaultMotionState>(startTransform);

    const double massScale = config.mass / 1500.0;
    const btVector3 localInertia(
        static_cast<btScalar>(config.inertiaTensor.x * massScale),
        static_cast<btScalar>(config.inertiaTensor.y * massScale),
        static_cast<btScalar>(config.inertiaTensor.z * massScale)
    );

    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        static_cast<btScalar>(config.mass),
        m_motionState.get(),
        m_compoundShape.get(),
        localInertia
    );
    rbInfo.m_friction = static_cast<btScalar>(0.50);
    rbInfo.m_restitution = static_cast<btScalar>(0.02); // Controlled inelastic impact
    rbInfo.m_linearDamping = static_cast<btScalar>(0.0);
    rbInfo.m_angularDamping = static_cast<btScalar>(0.05);

    m_rigidBody = std::make_unique<btRigidBody>(rbInfo);
    m_rigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_rigidBody->setUserIndex(-1); // Identify vehicle chassis body

    // Continuous Collision Detection (CCD) prevents obstacle tunneling at high velocities
    m_rigidBody->setCcdMotionThreshold(static_cast<btScalar>(0.08)); // Trigger swept test when moving > 8 cm per step
    m_rigidBody->setCcdSweptSphereRadius(static_cast<btScalar>(0.20)); // Swept sphere radius
    m_rigidBody->setContactProcessingThreshold(static_cast<btScalar>(0.01)); // Clean contact threshold prevents edge snagging
    m_rigidBody->setRestitution(0.0f);

    if (m_world) {
        m_world->addRigidBody(m_rigidBody.get());
    }
}

VehicleBody::~VehicleBody() {
    if (m_world && m_rigidBody) {
        m_world->removeRigidBody(m_rigidBody.get());
    }
}

void VehicleBody::syncMassProperties(const VehicleConfig& config) {
    const Vec3 currentOriginPos = getPosition();
    const Quat currentRot = getRotation();

    m_localCom = getCenterOfMassLocal(config);

    // Update child shape offsets relative to new CoM
    if (!m_collision.pieces.empty() && m_compoundShape->getNumChildShapes() == static_cast<int>(m_collision.pieces.size())) {
        for (size_t i = 0; i < m_collision.pieces.size(); ++i) {
            btTransform childTrans;
            childTrans.setIdentity();
            childTrans.setOrigin(toBtVec3(vec3Sub(m_collision.pieces[i].centerModelSpace, m_localCom)));
            m_compoundShape->updateChildTransform(static_cast<int>(i), childTrans, (i + 1 == m_collision.pieces.size()));
        }
    } else if (m_compoundShape->getNumChildShapes() >= 4) {
        const double hz = config.chassisHalfExtents.z;
        btTransform midTrans; midTrans.setIdentity();
        midTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, m_localCom)));
        m_compoundShape->updateChildTransform(0, midTrans, false);

        btTransform frontTrans; frontTrans.setIdentity();
        frontTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.22, hz * 0.70}, m_localCom)));
        m_compoundShape->updateChildTransform(1, frontTrans, false);

        btTransform rearTrans; rearTrans.setIdentity();
        rearTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.23, -hz * 0.70}, m_localCom)));
        m_compoundShape->updateChildTransform(2, rearTrans, false);

        btTransform cabinTrans; cabinTrans.setIdentity();
        cabinTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, m_localCom)));
        m_compoundShape->updateChildTransform(3, cabinTrans, true);
    } else if (m_compoundShape->getNumChildShapes() >= 2) {
        btTransform lowerLocalTrans; lowerLocalTrans.setIdentity();
        lowerLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, m_localCom)));
        m_compoundShape->updateChildTransform(0, lowerLocalTrans, false);

        btTransform cabinLocalTrans; cabinLocalTrans.setIdentity();
        cabinLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, m_localCom)));
        m_compoundShape->updateChildTransform(1, cabinLocalTrans, true);
    }

    // Reposition Bullet CoM transform so geometric chassis origin does not jump
    const Vec3 newComWorld = localPointToWorld(m_localCom, currentOriginPos, currentRot);
    btTransform worldTrans = m_rigidBody->getWorldTransform();
    worldTrans.setOrigin(toBtVec3(newComWorld));
    worldTrans.setRotation(toBtQuat(currentRot));
    m_rigidBody->setWorldTransform(worldTrans);
    if (m_motionState) {
        m_motionState->setWorldTransform(worldTrans);
    }

    const double massScale = config.mass / 1500.0;
    const btVector3 localInertia(
        static_cast<btScalar>(config.inertiaTensor.x * massScale),
        static_cast<btScalar>(config.inertiaTensor.y * massScale),
        static_cast<btScalar>(config.inertiaTensor.z * massScale)
    );
    m_rigidBody->setMassProps(static_cast<btScalar>(config.mass), localInertia);
    m_rigidBody->updateInertiaTensor();
    m_rigidBody->activate(true);

    if (m_world) {
        m_world->updateSingleAabb(m_rigidBody.get());
    }
}

Vec3 VehicleBody::getPosition() const {
    const Vec3 comWorld = getCenterOfMassWorld();
    const Quat rot = getRotation();
    return vec3Sub(comWorld, quatRotateVec3(rot, m_localCom));
}

Quat VehicleBody::getRotation() const {
    return fromBtQuat(m_rigidBody->getWorldTransform().getRotation());
}

Vec3 VehicleBody::getCenterOfMassWorld() const {
    return fromBtVec3(m_rigidBody->getWorldTransform().getOrigin());
}

Vec3 VehicleBody::getLinearVelocity() const {
    return fromBtVec3(m_rigidBody->getLinearVelocity());
}

Vec3 VehicleBody::getAngularVelocity() const {
    return fromBtVec3(m_rigidBody->getAngularVelocity());
}

void VehicleBody::setLinearVelocity(const Vec3& vel) {
    m_rigidBody->setLinearVelocity(toBtVec3(vel));
    m_rigidBody->activate(true);
}

void VehicleBody::setAngularVelocity(const Vec3& angVel) {
    m_rigidBody->setAngularVelocity(toBtVec3(angVel));
    m_rigidBody->activate(true);
}

Vec3 VehicleBody::getVelocityAtWorldPoint(const Vec3& worldPoint) const {
    const Vec3 vCom = getLinearVelocity();
    const Vec3 omega = getAngularVelocity();
    const Vec3 com = getCenterOfMassWorld();
    const Vec3 r = vec3Sub(worldPoint, com);
    return vec3Add(vCom, vec3Cross(omega, r));
}

void VehicleBody::applyForceAtWorldPoint(const Vec3& forceWorld, const Vec3& pointWorld) {
    if (!isVec3Finite(forceWorld) || !isVec3Finite(pointWorld)) {
        return;
    }
    const Vec3 com = getCenterOfMassWorld();
    const Vec3 relPos = vec3Sub(pointWorld, com);
    m_rigidBody->applyForce(toBtVec3(forceWorld), toBtVec3(relPos));
}

void VehicleBody::applyCentralForce(const Vec3& forceWorld) {
    if (!isVec3Finite(forceWorld)) {
        return;
    }
    m_rigidBody->applyCentralForce(toBtVec3(forceWorld));
}

void VehicleBody::applyTorque(const Vec3& torqueWorld) {
    if (!isVec3Finite(torqueWorld)) {
        return;
    }
    m_rigidBody->applyTorque(toBtVec3(torqueWorld));
}

void VehicleBody::prepareForSimulationStep() {
    m_preStepLinearVelocity = getLinearVelocity();
}

void VehicleBody::postStepUpdate(btDiscreteDynamicsWorld* world, double dt) {
    const Vec3 vNow = getLinearVelocity();
    if (dt > 1e-5) {
        const Vec3 rawAcc = vec3Scale(vec3Sub(vNow, m_prevLinearVelocity), 1.0 / dt);
        accelerationWorld = vec3Lerp(accelerationWorld, rawAcc, 0.35);
        accelerationLocal = worldVectorToLocal(accelerationWorld, getRotation());
    }
    m_prevLinearVelocity = vNow;

    lastImpactEvent = CollisionImpactEvent{};
    activeContacts.clear();
    if (!world || !world->getDispatcher()) {
        return;
    }

    const Vec3 deltaV = vec3Sub(vNow, m_preStepLinearVelocity);
    const double deltaVMag = vec3Length(deltaV);

    btDispatcher* dispatcher = world->getDispatcher();
    const int numManifolds = dispatcher->getNumManifolds();

    double maxImpulse = 0.0;
    int minLifeTime = 999999;
    Vec3 worstContactPoint{0.0, 0.0, 0.0};
    Vec3 worstContactNormal{0.0, 1.0, 0.0};
    bool foundChassisContact = false;

    const Vec3 chassisPos = getPosition();
    const Quat chassisRot = getRotation();

    for (int i = 0; i < numManifolds; ++i) {
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
        if (!manifold) continue;

        const btCollisionObject* body0 = manifold->getBody0();
        const btCollisionObject* body1 = manifold->getBody1();
        if (body0 != m_rigidBody.get() && body1 != m_rigidBody.get()) {
            continue;
        }

        const int numContacts = manifold->getNumContacts();
        for (int p = 0; p < numContacts; ++p) {
            const btManifoldPoint& pt = manifold->getContactPoint(p);
            if (pt.getDistance() <= 0.03f) {
                const btVector3& wPt = (body0 == m_rigidBody.get())
                    ? pt.getPositionWorldOnA()
                    : pt.getPositionWorldOnB();
                const Vec3 contactPt = fromBtVec3(wPt);
                const Vec3 contactNorm = fromBtVec3(pt.m_normalWorldOnB);
                const double appliedImpulse = static_cast<double>(pt.getAppliedImpulse());
                const int lifeTime = pt.getLifeTime();

                CollisionContactInfo cInfo;
                cInfo.pointWorld = contactPt;
                cInfo.normalWorld = contactNorm;
                cInfo.appliedImpulse = appliedImpulse;
                cInfo.distance = static_cast<double>(pt.getDistance());
                cInfo.lifeTime = lifeTime;

                if (activeContacts.size() < 16) {
                    activeContacts.push_back(cInfo);
                }

                foundChassisContact = true;
                if (appliedImpulse > maxImpulse || (maxImpulse == 0.0 && lifeTime < minLifeTime)) {
                    maxImpulse = std::max(maxImpulse, appliedImpulse);
                    minLifeTime = std::min(minLifeTime, lifeTime);
                    worstContactPoint = contactPt;
                    worstContactNormal = contactNorm;
                }
            }
        }
    }

    if (!foundChassisContact) {
        return;
    }

    const Vec3 ptLocal = worldPointToLocal(worstContactPoint, chassisPos, chassisRot);
    const float lateralBias = static_cast<float>(clamp(ptLocal.x / 0.79, -1.0, 1.0));
    const double vehicleSpeed = vec3Length(vNow);

    // Chassis underside scrape: contact point near or below bottom ground clearance, upward normal, moving car
    const bool isScrape = (ptLocal.y <= 0.22 && worstContactNormal.y > 0.45 && vehicleSpeed > 1.2);

    // True collision impact requires either a sudden velocity jump or a substantial normal impulse on a fresh contact
    const bool isRestingContact = (minLifeTime > 6 && deltaVMag < 0.22);

    if (!isRestingContact) {
        if (deltaVMag > 0.35 || maxImpulse > 250.0 || (isScrape && minLifeTime <= 3)) {
            lastImpactEvent.occurred = true;
            lastImpactEvent.impactSpeedMps = static_cast<float>(deltaVMag);
            lastImpactEvent.normalImpulse = static_cast<float>(maxImpulse);
            lastImpactEvent.lateralBias = lateralBias;
            lastImpactEvent.isChassisScrape = isScrape;
        }
    } else if (isScrape && vehicleSpeed > 1.5) {
        // Continuous sliding scrape over a low obstacle or curb while in motion
        lastImpactEvent.occurred = true;
        lastImpactEvent.impactSpeedMps = 0.0f;
        lastImpactEvent.normalImpulse = static_cast<float>(maxImpulse);
        lastImpactEvent.lateralBias = lateralBias;
        lastImpactEvent.isChassisScrape = true;
    }
}

void VehicleBody::resetPose(const Vec3& position, const Quat& rotation) {
    const Vec3 comWorld = localPointToWorld(m_localCom, position, rotation);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(toBtVec3(comWorld));
    trans.setRotation(toBtQuat(rotation));

    m_rigidBody->setWorldTransform(trans);
    if (m_motionState) {
        m_motionState->setWorldTransform(trans);
    }
    m_rigidBody->setLinearVelocity(btVector3(0, 0, 0));
    m_rigidBody->setAngularVelocity(btVector3(0, 0, 0));
    m_rigidBody->clearForces();
    m_rigidBody->activate(true);

    if (m_world) {
        m_world->updateSingleAabb(m_rigidBody.get());
    }

    m_prevLinearVelocity = Vec3{0.0, 0.0, 0.0};
    m_preStepLinearVelocity = Vec3{0.0, 0.0, 0.0};
    accelerationWorld = Vec3{0.0, 0.0, 0.0};
    accelerationLocal = Vec3{0.0, 0.0, 0.0};
    activeContacts.clear();
    lastImpactEvent = CollisionImpactEvent{};
}

bool VehicleBody::replaceCollisionShape(
    const GeneratedVehicleCollision* newCollision,
    const VehicleConfig& config
) {
    if (!m_world || !m_rigidBody) {
        return false;
    }

    // 1. Temporarily remove rigid body from world to update broadphase cleanly
    m_world->removeRigidBody(m_rigidBody.get());

    // 2. Recompute local Center of Mass
    m_localCom = getCenterOfMassLocal(config);

    // 3. Rebuild compound shape and child hulls
    m_compoundShape = buildCompoundShapeForVehicle(
        config,
        m_localCom,
        newCollision,
        m_collision,
        m_subShapes
    );

    // 4. Update rigid body collision shape
    m_rigidBody->setCollisionShape(m_compoundShape.get());

    // 5. Update inertia tensor
    const double massScale = config.mass / 1500.0;
    const btVector3 localInertia(
        static_cast<btScalar>(config.inertiaTensor.x * massScale),
        static_cast<btScalar>(config.inertiaTensor.y * massScale),
        static_cast<btScalar>(config.inertiaTensor.z * massScale)
    );
    m_rigidBody->setMassProps(static_cast<btScalar>(config.mass), localInertia);
    m_rigidBody->updateInertiaTensor();

    // 6. Re-add rigid body to Bullet world and update broadphase AABB
    m_world->addRigidBody(m_rigidBody.get());
    m_world->updateSingleAabb(m_rigidBody.get());

    return true;
}

} // namespace torquelab
