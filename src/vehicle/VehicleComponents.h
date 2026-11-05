#pragma once
/**
 * Torque Lab — Modular Vehicle Component & Mechanical Subsystem Architecture.
 *
 * Provides extensible, physically-grounded component interfaces designed for:
 * - Decoupled wheel assemblies (hub, rim, tyre, caliper)
 * - Articulated mechanical suspension linkages & kinematics (control arms, struts, tie rods)
 * - Rotating powertrain components (engine, transmission, driveshafts, differentials)
 * - Dynamic breakable/detachable body panels (bumpers, hoods, doors, mirrors)
 * - Damage accumulation and debris rigid-body generation
 */

#include <memory>
#include <string>
#include <vector>
#include <array>
#include "physics/Math3D.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

/**
 * Base polymorphic vehicle component interface.
 */
class IVehicleComponent {
public:
    virtual ~IVehicleComponent() = default;

    virtual const std::string& getName() const = 0;
    virtual double getMass() const = 0;
    virtual Vec3 getLocalPosition() const = 0;
    virtual Mat4 getTransform() const = 0;
    virtual void update(double dt) = 0;
    virtual void reset() = 0;
};

/**
 * Wheel Assembly Component encapsulating hub, rim, tyre, caliper, and kinematic transform.
 */
class WheelAssemblyComponent : public IVehicleComponent {
public:
    WheelAssemblyComponent(
        int wheelIndex,
        const std::string& name,
        const Vec3& hubMountLocal,
        double radius,
        double width,
        double mass
    )
        : m_wheelIndex(wheelIndex)
        , m_name(name)
        , m_hubMountLocal(hubMountLocal)
        , m_radius(radius)
        , m_width(width)
        , m_mass(mass)
    {}

    const std::string& getName() const override { return m_name; }
    double getMass() const override { return m_mass; }
    Vec3 getLocalPosition() const override { return m_currentPositionLocal; }
    Mat4 getTransform() const override { return m_localTransform; }

    int getWheelIndex() const { return m_wheelIndex; }
    double getRadius() const { return m_radius; }
    double getWidth() const { return m_width; }
    double getAngularVelocity() const { return m_angularVelocity; }
    double getRotationAngle() const { return m_rotationAngle; }
    double getSteeringAngle() const { return m_steeringAngle; }
    double getCamberAngle() const { return m_camberAngle; }
    double getToeAngle() const { return m_toeAngle; }
    bool isInContact() const { return m_inContact; }

    void setKinematicState(
        const Vec3& centerLocal,
        double steerRad,
        double spinRad,
        double omega,
        bool inContact
    ) {
        m_currentPositionLocal = centerLocal;
        m_steeringAngle = steerRad;
        m_rotationAngle = spinRad;
        m_angularVelocity = omega;
        m_inContact = inContact;

        const Quat steerQ = quatFromAxisAngle(Vec3{0, 1, 0}, m_steeringAngle);
        const Quat spinQ  = quatFromAxisAngle(Vec3{1, 0, 0}, m_rotationAngle);
        const Quat combQ  = quatMultiply(steerQ, spinQ);
        m_localTransform = Mat4::fromTRS(m_currentPositionLocal, combQ, Vec3{1, 1, 1});
    }

    void setAlignment(double camberRad, double toeRad) {
        m_camberAngle = camberRad;
        m_toeAngle = toeRad;
    }

    void update(double dt) override {
        // Continuous rotation update
        m_rotationAngle += m_angularVelocity * dt;
        if (m_rotationAngle > PI_D * 2.0) m_rotationAngle -= PI_D * 2.0;
        if (m_rotationAngle < -PI_D * 2.0) m_rotationAngle += PI_D * 2.0;
    }

    void reset() override {
        m_currentPositionLocal = m_hubMountLocal;
        m_angularVelocity = 0.0;
        m_rotationAngle = 0.0;
        m_steeringAngle = 0.0;
        m_inContact = false;
        m_localTransform = Mat4::fromTranslation(m_hubMountLocal);
    }

private:
    int m_wheelIndex{0};
    std::string m_name{"Wheel_Assembly"};
    Vec3 m_hubMountLocal{0, 0, 0};
    Vec3 m_currentPositionLocal{0, 0, 0};
    double m_radius{0.32};
    double m_width{0.235};
    double m_mass{22.0};
    double m_angularVelocity{0.0};
    double m_rotationAngle{0.0};
    double m_steeringAngle{0.0};
    double m_camberAngle{0.0};
    double m_toeAngle{0.0};
    bool m_inContact{false};
    Mat4 m_localTransform{Mat4::identity()};
};

/**
 * Suspension Linkage & Kinematics Component.
 */
class SuspensionLinkageComponent : public IVehicleComponent {
public:
    SuspensionLinkageComponent(
        const std::string& name,
        const Vec3& chassisMountLocal,
        double restLength,
        double minLength,
        double maxLength,
        double springStiffness,
        double damperBound,
        double damperRebound
    )
        : m_name(name)
        , m_chassisMountLocal(chassisMountLocal)
        , m_restLength(restLength)
        , m_minLength(minLength)
        , m_maxLength(maxLength)
        , m_springStiffness(springStiffness)
        , m_damperBound(damperBound)
        , m_damperRebound(damperRebound)
        , m_currentLength(restLength)
    {}

    const std::string& getName() const override { return m_name; }
    double getMass() const override { return 15.0; } // Strut + spring assembly mass
    Vec3 getLocalPosition() const override { return m_chassisMountLocal; }
    Mat4 getTransform() const override {
        return Mat4::fromTranslation(m_chassisMountLocal);
    }

    double getCompression() const { return m_restLength - m_currentLength; }
    double getCompressionRatio() const {
        const double maxComp = m_restLength - m_minLength;
        return (maxComp > 1e-4) ? clamp(getCompression() / maxComp, 0.0, 1.0) : 0.0;
    }
    double getCurrentLength() const { return m_currentLength; }
    double getSpringStiffness() const { return m_springStiffness; }
    double getDamperBound() const { return m_damperBound; }
    double getDamperRebound() const { return m_damperRebound; }

    void setLength(double length) {
        m_currentLength = clamp(length, m_minLength, m_maxLength);
    }

    void update(double /*dt*/) override {}

    void reset() override {
        m_currentLength = m_restLength;
    }

private:
    std::string m_name{"Suspension_Linkage"};
    Vec3 m_chassisMountLocal{0, 0, 0};
    double m_restLength{0.38};
    double m_minLength{0.15};
    double m_maxLength{0.48};
    double m_springStiffness{35000.0};
    double m_damperBound{3600.0};
    double m_damperRebound{4800.0};
    double m_currentLength{0.38};
};

/**
 * Steering Linkage & Knuckle Kinematics Component.
 */
class SteeringLinkageComponent : public IVehicleComponent {
public:
    SteeringLinkageComponent(
        const std::string& name,
        double maxSteerRad,
        double ackermannBlend
    )
        : m_name(name)
        , m_maxSteerRad(maxSteerRad)
        , m_ackermannBlend(ackermannBlend)
    {}

    const std::string& getName() const override { return m_name; }
    double getMass() const override { return 12.0; } // Steering rack + tie rods mass
    Vec3 getLocalPosition() const override { return Vec3{0, -0.15, 1.25}; }
    Mat4 getTransform() const override { return Mat4::identity(); }

    void setInputAngle(double centerAngleRad) {
        m_centerAngle = clamp(centerAngleRad, -m_maxSteerRad, m_maxSteerRad);
    }

    double getCenterAngle() const { return m_centerAngle; }
    double getLeftWheelAngle() const { return m_leftAngle; }
    double getRightWheelAngle() const { return m_rightAngle; }

    void computeAckermannGeometry(double wheelbase, double trackWidth) {
        if (std::abs(m_centerAngle) < 1e-4) {
            m_leftAngle = 0.0;
            m_rightAngle = 0.0;
            return;
        }
        const double turnRadius = wheelbase / std::tan(std::abs(m_centerAngle));
        const double halfTrack = trackWidth * 0.5;

        double innerAngle = std::atan(wheelbase / std::max(0.5, turnRadius - halfTrack));
        double outerAngle = std::atan(wheelbase / (turnRadius + halfTrack));

        innerAngle = lerp(std::abs(m_centerAngle), innerAngle, m_ackermannBlend);
        outerAngle = lerp(std::abs(m_centerAngle), outerAngle, m_ackermannBlend);

        if (m_centerAngle > 0.0) { // Left turn
            m_leftAngle  = +innerAngle;
            m_rightAngle = +outerAngle;
        } else { // Right turn
            m_leftAngle  = -outerAngle;
            m_rightAngle = -innerAngle;
        }
    }

    void update(double /*dt*/) override {}
    void reset() override {
        m_centerAngle = 0.0;
        m_leftAngle = 0.0;
        m_rightAngle = 0.0;
    }

private:
    std::string m_name{"Steering_Linkage"};
    double m_maxSteerRad{0.60};
    double m_ackermannBlend{0.85};
    double m_centerAngle{0.0};
    double m_leftAngle{0.0};
    double m_rightAngle{0.0};
};

/**
 * Articulated Mechanical Part for moving mechanical components
 * (driveshafts, control arms, calipers, hinged doors, hood, trunk).
 */
class ArticulatedMechanicalPart : public IVehicleComponent {
public:
    enum class MotionType {
        ContinuousRotation, // Driveshaft, axle half-shaft, pulley
        AngularOscillation, // Suspension control arm, tie rod
        HingedJoint         // Hood, trunk, door
    };

    ArticulatedMechanicalPart(
        const std::string& name,
        MotionType motionType,
        const Vec3& pivotLocal,
        const Vec3& rotationAxis,
        double mass
    )
        : m_name(name)
        , m_motionType(motionType)
        , m_pivotLocal(pivotLocal)
        , m_rotationAxis(vec3Normalize(rotationAxis))
        , m_mass(mass)
    {}

    const std::string& getName() const override { return m_name; }
    double getMass() const override { return m_mass; }
    Vec3 getLocalPosition() const override { return m_pivotLocal; }
    Mat4 getTransform() const override { return m_transform; }
    MotionType getMotionType() const { return m_motionType; }
    double getCurrentAngle() const { return m_currentAngle; }

    void setAngle(double angleRad) {
        m_currentAngle = angleRad;
        const Quat rot = quatFromAxisAngle(m_rotationAxis, m_currentAngle);
        m_transform = Mat4::fromTRS(m_pivotLocal, rot, Vec3{1, 1, 1});
    }

    void setAngularVelocity(double omega) {
        m_angularVelocity = omega;
    }

    void update(double dt) override {
        if (m_motionType == MotionType::ContinuousRotation) {
            m_currentAngle += m_angularVelocity * dt;
            const Quat rot = quatFromAxisAngle(m_rotationAxis, m_currentAngle);
            m_transform = Mat4::fromTRS(m_pivotLocal, rot, Vec3{1, 1, 1});
        }
    }

    void reset() override {
        m_currentAngle = 0.0;
        m_angularVelocity = 0.0;
        m_transform = Mat4::fromTranslation(m_pivotLocal);
    }

private:
    std::string m_name{"Mechanical_Part"};
    MotionType m_motionType{MotionType::ContinuousRotation};
    Vec3 m_pivotLocal{0, 0, 0};
    Vec3 m_rotationAxis{0, 0, 1};
    double m_mass{5.0};
    double m_currentAngle{0.0};
    double m_angularVelocity{0.0};
    Mat4 m_transform{Mat4::identity()};
};

/**
 * Detachable & Damageable Body Part Component
 * (front/rear bumper, bonnet/hood, boot/trunk, wing mirrors, spoiler).
 */
class DetachableBodyPart : public IVehicleComponent {
public:
    enum class State {
        Attached,
        Loosened,   // Hanging or deformed from impact
        Detached    // Completely broken off into dynamic debris
    };

    DetachableBodyPart(
        const std::string& name,
        const Vec3& restPositionLocal,
        double mass,
        double damageImpulseThreshold,
        double detachImpulseThreshold
    )
        : m_name(name)
        , m_restPositionLocal(restPositionLocal)
        , m_mass(mass)
        , m_damageImpulseThreshold(damageImpulseThreshold)
        , m_detachImpulseThreshold(detachImpulseThreshold)
    {}

    const std::string& getName() const override { return m_name; }
    double getMass() const override { return m_mass; }
    Vec3 getLocalPosition() const override { return m_restPositionLocal; }
    Mat4 getTransform() const override { return m_localTransform; }

    State getState() const { return m_state; }
    double getHealth() const { return m_health; }
    bool isDetached() const { return m_state == State::Detached; }

    void applyImpactImpulse(double normalImpulse) {
        if (m_state == State::Detached) return;

        if (normalImpulse >= m_damageImpulseThreshold) {
            const double damageFactor = (normalImpulse - m_damageImpulseThreshold) /
                                        std::max(100.0, m_detachImpulseThreshold - m_damageImpulseThreshold);
            m_health = clamp(m_health - damageFactor, 0.0, 1.0);

            if (normalImpulse >= m_detachImpulseThreshold || m_health <= 0.0) {
                m_state = State::Detached;
            } else if (m_health < 0.65) {
                m_state = State::Loosened;
                // Add minor geometric sagging/deformation
                m_localTransform = Mat4::fromTRS(
                    vec3Add(m_restPositionLocal, Vec3{0.0, -0.04, 0.02}),
                    quatFromAxisAngle(Vec3{1, 0, 0}, degToRad(3.5)),
                    Vec3{1, 1, 1}
                );
            }
        }
    }

    void update(double /*dt*/) override {}

    void reset() override {
        m_state = State::Attached;
        m_health = 1.0;
        m_localTransform = Mat4::fromTranslation(m_restPositionLocal);
    }

private:
    std::string m_name{"Body_Part"};
    Vec3 m_restPositionLocal{0, 0, 0};
    double m_mass{12.0};
    double m_damageImpulseThreshold{3500.0};  // Impulse required to cause dent/loosening (N*s)
    double m_detachImpulseThreshold{9500.0};  // Impulse required to snap fasteners & detach (N*s)
    State m_state{State::Attached};
    double m_health{1.0};                    // Health fraction [0.0 .. 1.0]
    Mat4 m_localTransform{Mat4::identity()};
};

/**
 * High-level Powertrain Subsystem Interface connecting future engine, transmission,
 * differentials, and half-shaft drives to the vehicle wheels.
 */
class IPowertrainSystem {
public:
    virtual ~IPowertrainSystem() = default;

    virtual void update(
        double throttle,
        double forwardSpeed,
        std::array<double, 4>& outWheelTorques,
        double dt
    ) = 0;

    virtual double getEngineRpm() const = 0;
    virtual double getEngineTorque() const = 0;
    virtual int getCurrentGear() const = 0;
    virtual double getClutchEngagement() const = 0;
    virtual bool isRevLimiterActive() const = 0;
    virtual void reset() = 0;
};

} // namespace torquelab
