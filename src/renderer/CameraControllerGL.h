#pragma once
/**
 * Multi-Mode OpenGL Camera Controller (Follow, Orbit Inspection, Suspension Close-up, Hood).
 *
 * Camera movement is strictly decoupled from the physics simulation.
 */

#include <string>
#include "physics/Math3D.h"
#include "vehicle/VehicleDynamics.h"

namespace torquelab {

enum class CameraMode : int {
    Follow = 0,
    Orbit = 1,
    Suspension = 2,
    Hood = 3
};

class CameraControllerGL {
public:
    explicit CameraControllerGL(float aspect = 16.0f / 9.0f);

    CameraMode cycleMode();
    std::string getModeLabel() const;

    void onResize(float aspect);
    void onMouseDrag(double dx, double dy);
    void onMouseScroll(double scrollDeltaY);
    void setLookAroundStick(double stickX, double stickY);

    void update(const VehicleDynamics& vehicle, double alpha, double dtRender);
    void snapToVehicle(const VehicleDynamics& vehicle);

    Mat4 getViewMatrix() const { return m_viewMatrix; }
    Mat4 getProjectionMatrix() const { return m_projMatrix; }
    Mat4 getViewProjectionMatrix() const { return Mat4::multiply(m_projMatrix, m_viewMatrix); }
    Vec3 getPosition() const { return m_position; }

    CameraMode mode{CameraMode::Follow};
    double followDistance{6.6};
    double followHeight{2.35};
    double followDamping{9.5};

private:
    float m_aspect{16.0f / 9.0f};
    double m_orbitYaw{0.65};
    double m_orbitPitch{0.35};
    double m_orbitDistance{5.4};

    double m_lookStickX{0.0};
    double m_lookStickY{0.0};
    double m_lookAngleYaw{0.0};
    double m_lookAnglePitch{0.0};

    Vec3 m_position{0.0, 3.0, -7.0};
    Vec3 m_currentLookAt{0.0, 0.8, 0.0};
    Vec3 m_smoothedForward{0.0, 0.0, 1.0};
    double m_smoothedHeight{2.8};
    Vec3 m_up{0.0, 1.0, 0.0};
    bool m_initialized{false};

    Mat4 m_viewMatrix{Mat4::identity()};
    Mat4 m_projMatrix{Mat4::identity()};
};

} // namespace torquelab
