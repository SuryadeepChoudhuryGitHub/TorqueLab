#include "renderer/CameraControllerGL.h"
#include <cmath>

namespace torquelab {

CameraControllerGL::CameraControllerGL(float aspect)
    : m_aspect(aspect)
{
    onResize(aspect);
}

CameraMode CameraControllerGL::cycleMode() {
    const int next = (static_cast<int>(mode) + 1) % 4;
    mode = static_cast<CameraMode>(next);
    return mode;
}

std::string CameraControllerGL::getModeLabel() const {
    switch (mode) {
        case CameraMode::Follow:     return "Chase Follow Cam";
        case CameraMode::Orbit:      return "Orbit Inspection Cam (Drag Mouse)";
        case CameraMode::Suspension: return "Wheel & Suspension Rig Cam";
        case CameraMode::Hood:       return "Driver Hood Cam";
    }
    return "Chase Follow Cam";
}

void CameraControllerGL::onResize(float aspect) {
    m_aspect = (aspect > 0.05f) ? aspect : (16.0f / 9.0f);
    m_projMatrix = Mat4::perspective(static_cast<float>(degToRad(58.0)), m_aspect, 0.15f, 650.0f);
}

void CameraControllerGL::onMouseDrag(double dx, double dy) {
    if (mode == CameraMode::Orbit) {
        m_orbitYaw -= dx * 0.007;
        m_orbitPitch = clamp(m_orbitPitch + dy * 0.006, 0.06, 1.45);
    }
}

void CameraControllerGL::onMouseScroll(double scrollDeltaY) {
    const double delta = -scrollDeltaY * 0.45;
    if (mode == CameraMode::Orbit) {
        m_orbitDistance = clamp(m_orbitDistance + delta, 2.5, 18.0);
    } else if (mode == CameraMode::Follow) {
        followDistance = clamp(followDistance + delta, 3.8, 14.0);
    }
}

void CameraControllerGL::setLookAroundStick(double stickX, double stickY) {
    m_lookStickX = stickX;
    m_lookStickY = stickY;
}

void CameraControllerGL::update(const VehicleDynamics& vehicle, double alpha, double dtRender) {
    const InterpolatedVehicleTransforms interp = vehicle.getInterpolatedTransforms(alpha);
    const Vec3& pos = interp.chassisPosition;
    const Quat& rot = interp.chassisRotation;
    const VehicleBasis basis = getVehicleBasis(rot);

    const double flatFwdLen = std::hypot(basis.forward.x, basis.forward.z);
    const Vec3 flatForward = (flatFwdLen > 1e-6)
        ? Vec3{basis.forward.x / flatFwdLen, 0.0, basis.forward.z / flatFwdLen}
        : Vec3{0.0, 0.0, 1.0};

    if (mode == CameraMode::Follow) {
        if (!m_initialized) {
            m_smoothedForward = flatForward;
            m_smoothedHeight = pos.y + followHeight;
            m_initialized = true;
        } else {
            const double tDir = clamp(1.0 - std::exp(-followDamping * dtRender), 0.02, 1.0);
            const double tY = clamp(1.0 - std::exp(-12.0 * dtRender), 0.05, 1.0);
            const Vec3 blendedFwd = vec3Lerp(m_smoothedForward, flatForward, tDir);
            const double len = std::hypot(blendedFwd.x, blendedFwd.z);
            m_smoothedForward = (len > 1e-6)
                ? Vec3{blendedFwd.x / len, 0.0, blendedFwd.z / len}
                : flatForward;
            m_smoothedHeight = lerp(m_smoothedHeight, pos.y + followHeight, tY);
        }

        // Forza Horizon-style right-stick free-look in Chase Cam
        const double stickMag = std::hypot(m_lookStickX, m_lookStickY);
        if (stickMag > 0.08) {
            const double targetYaw = -m_lookStickX * 3.14159265;
            const double targetPitch = -m_lookStickY * 0.45;
            const double tLook = clamp(1.0 - std::exp(-14.0 * dtRender), 0.05, 1.0);
            m_lookAngleYaw = lerp(m_lookAngleYaw, targetYaw, tLook);
            m_lookAnglePitch = lerp(m_lookAnglePitch, targetPitch, tLook);
        } else {
            // Smoothly auto-recenter behind the vehicle when the stick is released
            const double tRecenter = clamp(1.0 - std::exp(-9.0 * dtRender), 0.05, 1.0);
            m_lookAngleYaw = lerp(m_lookAngleYaw, 0.0, tRecenter);
            m_lookAnglePitch = lerp(m_lookAnglePitch, 0.0, tRecenter);
        }

        const double cy = std::cos(m_lookAngleYaw);
        const double sy = std::sin(m_lookAngleYaw);
        const Vec3 lookForward{
            m_smoothedForward.x * cy + m_smoothedForward.z * sy,
            0.0,
            -m_smoothedForward.x * sy + m_smoothedForward.z * cy
        };
        const double pitchHeightOffset = m_lookAnglePitch * 2.2;

        m_position = Vec3{
            pos.x - lookForward.x * followDistance,
            m_smoothedHeight + pitchHeightOffset,
            pos.z - lookForward.z * followDistance
        };
        m_currentLookAt = Vec3{
            pos.x + lookForward.x * 2.4,
            pos.y + 0.65 - pitchHeightOffset * 0.4,
            pos.z + lookForward.z * 2.4
        };
        m_up = Vec3{0.0, 1.0, 0.0};
        m_viewMatrix = Mat4::lookAt(m_position, m_currentLookAt, m_up);
    } else if (mode == CameraMode::Orbit) {
        if (std::abs(m_lookStickX) > 0.08 || std::abs(m_lookStickY) > 0.08) {
            m_orbitYaw -= m_lookStickX * 2.6 * dtRender;
            m_orbitPitch = clamp(m_orbitPitch - m_lookStickY * 2.0 * dtRender, 0.06, 1.45);
        }

        const double vehYaw = std::atan2(flatForward.x, flatForward.z);
        const double totalYaw = vehYaw + m_orbitYaw;
        const double horizDist = m_orbitDistance * std::cos(m_orbitPitch);
        const double vertDist = m_orbitDistance * std::sin(m_orbitPitch);

        m_position = Vec3{
            pos.x + std::sin(totalYaw) * horizDist,
            std::max(0.25, pos.y + vertDist),
            pos.z + std::cos(totalYaw) * horizDist
        };
        m_currentLookAt = Vec3{pos.x, pos.y + 0.35, pos.z};
        m_up = Vec3{0.0, 1.0, 0.0};
        m_viewMatrix = Mat4::lookAt(m_position, m_currentLookAt, m_up);
    } else if (mode == CameraMode::Suspension) {
        m_position = Vec3{
            pos.x + basis.left.x * 2.45 + basis.forward.x * 0.45,
            pos.y + 0.35,
            pos.z + basis.left.z * 2.45 + basis.forward.z * 0.45
        };
        const Vec3& targetWheel = interp.wheelPositions[0];
        m_currentLookAt = Vec3{targetWheel.x, targetWheel.y + 0.08, targetWheel.z};
        m_up = Vec3{0.0, 1.0, 0.0};
        m_viewMatrix = Mat4::lookAt(m_position, m_currentLookAt, m_up);
    } else if (mode == CameraMode::Hood) {
        const Vec3 hoodPos{
            pos.x + basis.up.x * 0.88 + basis.forward.x * 0.42,
            pos.y + basis.up.y * 0.88 + basis.forward.y * 0.42,
            pos.z + basis.up.z * 0.88 + basis.forward.z * 0.42
        };
        const Vec3 lookTarget{
            hoodPos.x + basis.forward.x * 20.0,
            hoodPos.y + basis.forward.y * 20.0 - 0.8,
            hoodPos.z + basis.forward.z * 20.0
        };
        m_position = hoodPos;
        m_currentLookAt = lookTarget;
        m_up = basis.up;
        m_viewMatrix = Mat4::lookAt(m_position, m_currentLookAt, m_up);
    }
}

void CameraControllerGL::snapToVehicle(const VehicleDynamics& vehicle) {
    m_initialized = false;
    update(vehicle, 1.0, 0.016);
}

} // namespace torquelab
