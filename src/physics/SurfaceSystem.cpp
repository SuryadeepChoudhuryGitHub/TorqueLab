#include "physics/SurfaceSystem.h"

namespace torquelab {

namespace {

const std::array<SurfaceProperties, 6> SURFACE_CATALOG{{
    {SurfaceType::DryAsphalt,  "Dry Asphalt",                   1.00, 1.00, 1.00, 0.000,  1.0, 0x3b4252},
    {SurfaceType::WetAsphalt,  "Wet Asphalt",                   0.65, 0.85, 1.15, 0.001,  8.0, 0x2e5eaa},
    {SurfaceType::Gravel,      "Loose Gravel",                  0.48, 0.65, 2.40, 0.009, 14.0, 0xb8864b},
    {SurfaceType::Ice,         "Polished Ice / Low-Grip Pad",   0.20, 0.45, 0.85, 0.000,  1.0, 0x7fd8f5},
    {SurfaceType::Grass,       "Grass Runoff",                  0.55, 0.72, 3.00, 0.006, 10.0, 0x3a6b3c},
    {SurfaceType::RumbleStrip, "Curb / Rumble Strip",           0.88, 0.92, 1.35, 0.016, 24.0, 0xd94e4e}
}};

} // namespace

const SurfaceProperties& getSurfaceProperties(SurfaceType type) {
    const int idx = static_cast<int>(type);
    if (idx >= 0 && idx < static_cast<int>(SURFACE_CATALOG.size())) {
        return SURFACE_CATALOG[static_cast<size_t>(idx)];
    }
    return SURFACE_CATALOG[0];
}

SurfaceType surfaceTypeFromString(const std::string& name) {
    if (name == "wet_asphalt") return SurfaceType::WetAsphalt;
    if (name == "gravel") return SurfaceType::Gravel;
    if (name == "ice") return SurfaceType::Ice;
    if (name == "grass") return SurfaceType::Grass;
    if (name == "rumble_strip") return SurfaceType::RumbleStrip;
    return SurfaceType::DryAsphalt;
}

std::string surfaceTypeToString(SurfaceType type) {
    switch (type) {
        case SurfaceType::DryAsphalt:  return "dry_asphalt";
        case SurfaceType::WetAsphalt:  return "wet_asphalt";
        case SurfaceType::Gravel:      return "gravel";
        case SurfaceType::Ice:         return "ice";
        case SurfaceType::Grass:       return "grass";
        case SurfaceType::RumbleStrip: return "rumble_strip";
    }
    return "dry_asphalt";
}

void SurfaceSystem::registerColliderSurface(int colliderHandle, SurfaceType surfaceType, bool isDriveable) {
    m_colliderSurfaces[colliderHandle] = surfaceType;
    m_colliderDriveable[colliderHandle] = isDriveable;
}

bool SurfaceSystem::isColliderDriveable(int colliderHandle) const {
    if (colliderHandle >= 0) {
        const auto it = m_colliderDriveable.find(colliderHandle);
        if (it != m_colliderDriveable.end()) {
            return it->second;
        }
    }
    return true;
}

const SurfaceProperties& SurfaceSystem::getSurfaceForCollider(int colliderHandle) const {
    if (colliderHandle >= 0) {
        const auto it = m_colliderSurfaces.find(colliderHandle);
        if (it != m_colliderSurfaces.end()) {
            return getSurfaceProperties(it->second);
        }
    }
    return getSurfaceProperties(m_defaultSurface);
}

void SurfaceSystem::setDefaultSurface(SurfaceType surfaceType) {
    m_defaultSurface = surfaceType;
}

void SurfaceSystem::clear() {
    m_colliderSurfaces.clear();
    m_colliderDriveable.clear();
}

} // namespace torquelab
