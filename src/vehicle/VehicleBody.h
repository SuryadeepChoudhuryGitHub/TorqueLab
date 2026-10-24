#pragma once
/**
 * 6-DOF Rigid-Body Vehicle Chassis Wrapper (Bullet Physics).
 *
 * Bridges our custom vehicle dynamics subsystems with Bullet's rigid-body solver:
 * - Creates the dynamic chassis btRigidBody with explicit mass, Center-of-Mass (CoM) offset, and principal inertia tensor
 * - Applies compound box collision shapes for vehicle-to-environment and obstacle collisions
 * - Provides force and torque accumulation at arbitrary world-space contact points
 * - Tracks finite-difference linear acceleration (G-forces) and active collision contacts
 */

#include <memory>
#include <vector>
#include <btBulletDynamicsCommon.h>
#include "physics/CollisionMeshGeometry.h"
#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

struct CollisionContactInfo {
    Vec3 pointWorld{0.0, 0.0, 0.0};
    Vec3 normalWorld{0.0, 1.0, 0.0};
};

class VehicleBody {
public:
    VehicleBody(
        btDiscreteDynamicsWorld* world,
        const VehicleConfig& config,
        const Vec3& initialPosition,
        const Quat& initialRotation = quatIdentity(),
        const GeneratedVehicleCollision* generatedCollision = nullptr
    );
    ~VehicleBody();

    VehicleBody(const VehicleBody&) = delete;
    VehicleBody& operator=(const VehicleBody&) = delete;

    /**
     * Synchronizes the Bullet RigidBody mass, local Center of Mass (CoM), and inertia tensor
     * whenever runtime tuning parameters (mass, comHeightOffset, frontWeightDistribution) change.
     */
    void syncMassProperties(const VehicleConfig& config);

    /** Returns the geometric chassis origin in world space. */
    Vec3 getPosition() const;

    /** Returns the chassis orientation quaternion in world space. */
    Quat getRotation() const;

    /** Returns the world-space Center of Mass position. */
    Vec3 getCenterOfMassWorld() const;

    Vec3 getLinearVelocity() const;
    Vec3 getAngularVelocity() const;

    void setLinearVelocity(const Vec3& vel);
    void setAngularVelocity(const Vec3& angVel);

    /**
     * Computes velocity of a world-space point rigidly attached to the chassis:
     *   v(p) = v_com + omega x (p - x_com)
     */
    Vec3 getVelocityAtWorldPoint(const Vec3& worldPoint) const;

    /**
     * Applies a world-space force vector at a specific world-space application point.
     * Automatically generates the physical moment tau = (point - x_com) x F.
     */
    void applyForceAtWorldPoint(const Vec3& forceWorld, const Vec3& pointWorld);

    /** Applies a central world-space force at the Center of Mass. */
    void applyCentralForce(const Vec3& forceWorld);

    /** Applies a pure world-space torque vector (N*m) to the chassis. */
    void applyTorque(const Vec3& torqueWorld);

    /**
     * Updates finite-difference linear acceleration and collects active collision contacts.
     * Called once per fixed physics step after world->stepSimulation().
     */
    void postStepUpdate(btDiscreteDynamicsWorld* world, double dt);

    /** Resets the chassis pose (in geometric origin coordinates) and zeroes all velocities/forces. */
    void resetPose(const Vec3& position, const Quat& rotation = quatIdentity());

    /**
     * Replaces the active collision shape in-place without destroying the vehicle dynamics state.
     * Safely unregisters the rigid body from Bullet, replaces the compound shape & child hulls,
     * recalculates mass properties and inertia tensor, updates broadphase AABBs, and re-registers.
     */
    bool replaceCollisionShape(
        const GeneratedVehicleCollision* newCollision,
        const VehicleConfig& config
    );

    btRigidBody* getRigidBody() const { return m_rigidBody.get(); }
    const btCompoundShape* getCompoundShape() const { return m_compoundShape.get(); }
    const GeneratedVehicleCollision& getGeneratedCollision() const { return m_collision; }

    Vec3 accelerationWorld{0.0, 0.0, 0.0};
    Vec3 accelerationLocal{0.0, 0.0, 0.0};
    std::vector<CollisionContactInfo> activeContacts;

private:
    btDiscreteDynamicsWorld* m_world{nullptr};
    Vec3 m_localCom{0.0, 0.0, 0.0};
    Vec3 m_prevLinearVelocity{0.0, 0.0, 0.0};

    GeneratedVehicleCollision m_collision;
    std::vector<std::unique_ptr<btCollisionShape>> m_subShapes;
    std::unique_ptr<btCompoundShape> m_compoundShape;
    std::unique_ptr<btDefaultMotionState> m_motionState;
    std::unique_ptr<btRigidBody> m_rigidBody;
};

} // namespace torquelab
