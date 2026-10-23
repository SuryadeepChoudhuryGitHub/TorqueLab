#pragma once
/**
 * Road Surface Material System.
 *
 * Defines physical properties for each driving surface in the Proving Ground
 * and maps Bullet collider handles (IDs) to their surface properties.
 */

#include <array>
#include <string>
#include <unordered_map>

namespace torquelab {

enum class SurfaceType : int {
    DryAsphalt = 0,
    WetAsphalt = 1,
    Gravel = 2,
    Ice = 3,
    Grass = 4,
    RumbleStrip = 5
};

struct SurfaceProperties {
    SurfaceType type{SurfaceType::DryAsphalt};
    std::string label{"Dry Asphalt"};
    double frictionMultiplier{1.0};          // Multiplier on nominal tire peak friction (mu_0)
    double stiffnessMultiplier{1.0};         // Multiplier on cornering/longitudinal slip stiffness
    double rollingResistanceMultiplier{1.0}; // Multiplier on baseline rolling resistance
    double roughnessAmplitude{0.0};          // High-frequency surface roughness amplitude in meters
    double roughnessFrequency{1.0};          // Spatial frequency of roughness (rad/m)
    unsigned int colorHex{0x3b4252};         // Visual hex color for 3D track / UI badge
};

const SurfaceProperties& getSurfaceProperties(SurfaceType type);
SurfaceType surfaceTypeFromString(const std::string& name);
std::string surfaceTypeToString(SurfaceType type);

class SurfaceSystem {
public:
    SurfaceSystem() = default;

    void registerColliderSurface(int colliderHandle, SurfaceType surfaceType);
    const SurfaceProperties& getSurfaceForCollider(int colliderHandle) const;
    void setDefaultSurface(SurfaceType surfaceType);
    SurfaceType getDefaultSurface() const { return m_defaultSurface; }
    void clear();

private:
    std::unordered_map<int, SurfaceType> m_colliderSurfaces;
    SurfaceType m_defaultSurface{SurfaceType::DryAsphalt};
};

} // namespace torquelab
