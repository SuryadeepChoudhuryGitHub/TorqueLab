#include "vehicle/VehicleBody.h"
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

} // namespace

VehicleBody::VehicleBody(
    btDiscreteDynamicsWorld* world,
    const VehicleConfig& config,
    const Vec3& initialPosition,
    const Quat& initialRotation
)
    : m_world(world)
    , m_localCom(getCenterOfMassLocal(config))
{
    const double hx = config.chassisHalfExtents.x;
    const double hy = config.chassisHalfExtents.y;
    const double hz = config.chassisHalfExtents.z;

    m_lowerBoxShape = std::make_unique<btBoxShape>(btVector3(
        static_cast<btScalar>(hx),
        static_cast<btScalar>(hy * 0.55),
        static_cast<btScalar>(hz)
    ));
    m_cabinBoxShape = std::make_unique<btBoxShape>(btVector3(
        static_cast<btScalar>(hx * 0.84),
        static_cast<btScalar>(hy * 0.55),
        static_cast<btScalar>(hz * 0.48)
    ));

    m_compoundShape = std::make_unique<btCompoundShape>();

    btTransform lowerLocalTrans;
    lowerLocalTrans.setIdentity();
    lowerLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, m_localCom)));
    m_compoundShape->addChildShape(lowerLocalTrans, m_lowerBoxShape.get());

    btTransform cabinLocalTrans;
    cabinLocalTrans.setIdentity();
    cabinLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, m_localCom)));
    m_compoundShape->addChildShape(cabinLocalTrans, m_cabinBoxShape.get());

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
    rbInfo.m_friction = static_cast<btScalar>(0.35);
    rbInfo.m_restitution = static_cast<btScalar>(0.18);
    rbInfo.m_linearDamping = static_cast<btScalar>(0.0);
    rbInfo.m_angularDamping = static_cast<btScalar>(0.015);

    m_rigidBody = std::make_unique<btRigidBody>(rbInfo);
    m_rigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_rigidBody->setUserIndex(-1); // Identify vehicle chassis body

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
    btTransform lowerLocalTrans;
    lowerLocalTrans.setIdentity();
    lowerLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.24, 0.0}, m_localCom)));
    m_compoundShape->updateChildTransform(0, lowerLocalTrans, true);

    btTransform cabinLocalTrans;
    cabinLocalTrans.setIdentity();
    cabinLocalTrans.setOrigin(toBtVec3(vec3Sub(Vec3{0.0, 0.62, -0.15}, m_localCom)));
    m_compoundShape->updateChildTransform(1, cabinLocalTrans, true);

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

void VehicleBody::postStepUpdate(btDiscreteDynamicsWorld* world, double dt) {
    const Vec3 vNow = getLinearVelocity();
    if (dt > 1e-5) {
        const Vec3 rawAcc = vec3Scale(vec3Sub(vNow, m_prevLinearVelocity), 1.0 / dt);
        accelerationWorld = vec3Lerp(accelerationWorld, rawAcc, 0.35);
        accelerationLocal = worldVectorToLocal(accelerationWorld, getRotation());
    }
    m_prevLinearVelocity = vNow;

    activeContacts.clear();
    if (!world || !world->getDispatcher()) {
        return;
    }

    btDispatcher* dispatcher = world->getDispatcher();
    const int numManifolds = dispatcher->getNumManifolds();
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
            if (pt.getDistance() <= 0.02f) {
                const btVector3& wPt = (body0 == m_rigidBody.get())
                    ? pt.getPositionWorldOnA()
                    : pt.getPositionWorldOnB();
                activeContacts.push_back(CollisionContactInfo{
                    fromBtVec3(wPt),
                    fromBtVec3(pt.m_normalWorldOnB)
                });
                if (activeContacts.size() >= 8) {
                    return;
                }
            }
        }
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
    accelerationWorld = Vec3{0.0, 0.0, 0.0};
    accelerationLocal = Vec3{0.0, 0.0, 0.0};
    activeContacts.clear();
}

} // namespace torquelab
