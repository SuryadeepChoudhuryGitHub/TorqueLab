#pragma once
/**
 * Torque Lab Native — glTF 2.0 / GLB Binary Vehicle Model Loader.
 *
 * Implements a robust, production-grade 3D model importer for Blender-exported vehicles.
 * Supports:
 * - Direct loading of .glb (binary) and .gltf files via cgltf
 * - Node hierarchy extraction, TRS local transforms, and scene graphs
 * - Extraction of mesh primitives into OpenGL VAO/VBO/EBO geometry (VertexPNU)
 * - Intelligent semantic role mapping (Chassis, Wheel_FL, Wheel_FR, Wheel_RL, Wheel_RR,
 *   BrakeCalipers, SteeringWheel, and studio plane exclusions)
 * - Optional manifest JSON support for user-defined per-car name mappings
 * - Physically Based Material (PBR) metallic/roughness and transparent glass classification
 * - Automatic wheel pivot computation and chassis alignment to physics center of mass
 */

#include <array>
#include <memory>
#include <string>
#include <vector>
#include "physics/Math3D.h"
#include "renderer/Mesh.h"
#include "renderer/Shader.h"
#include "vehicle/VehicleConfig.h"

namespace torquelab {

enum class ModelPartRole {
    Chassis,         // Main chassis / body shell / doors / hood / trim
    Wheel_FL,        // Front-Left wheel assembly (rotates + steers)
    Wheel_FR,        // Front-Right wheel assembly (rotates + steers)
    Wheel_RL,        // Rear-Left wheel assembly (rotates with drive torque)
    Wheel_RR,        // Rear-Right wheel assembly (rotates with drive torque)
    BrakeCaliper_FL, // Front-Left brake caliper (steers, does not rotate)
    BrakeCaliper_FR, // Front-Right brake caliper (steers, does not rotate)
    BrakeCaliper_RL, // Rear-Left brake caliper (non-steering, does not rotate)
    BrakeCaliper_RR, // Rear-Right brake caliper (non-steering, does not rotate)
    SteeringWheel,   // Interior steering wheel (rotates around steering axis)
    Ignore           // Studio ground planes, lights, cameras, etc.
};

struct LoadedModelPart {
    std::string name;
    Mesh mesh;
    Mat4 localBindMatrix{Mat4::identity()};
    Vec3 localPosition{0.0, 0.0, 0.0};
    Quat localRotation{0.0, 0.0, 0.0, 1.0};
    Vec3 localScale{1.0, 1.0, 1.0};
    MaterialProps material;
    ModelPartRole role{ModelPartRole::Chassis};
    bool isBrakeLight{false};
    bool isReverseLight{false};
    bool isTransparent{false};

    // For wheel components: offset relative to the calculated wheel rotation pivot
    Vec3 offsetFromWheelCenter{0.0, 0.0, 0.0};
    Quat wheelLocalRotation{0.0, 0.0, 0.0, 1.0};
    Vec3 wheelLocalScale{1.0, 1.0, 1.0};
};

struct LoadedVehicleModel {
    std::string modelName{"BMW M4 Coupe"};
    std::string filePath;
    bool loadedSuccessfully{false};

    // Identified wheel pivot centers in the model's coordinate frame
    std::array<Vec3, 4> modelWheelCenters{
        Vec3{ 0.77, 0.36,  1.53}, // FL
        Vec3{-0.77, 0.36,  1.53}, // FR
        Vec3{ 0.77, 0.36, -1.23}, // RL
        Vec3{-0.77, 0.36, -1.23}  // RR
    };
    std::array<bool, 4> wheelsIdentified{false, false, false, false};

    // Model bounding box (overall and chassis only)
    Vec3 aabbMin{0.0, 0.0, 0.0};
    Vec3 aabbMax{0.0, 0.0, 0.0};
    Vec3 chassisCenterOffset{0.0, 0.0, 0.0}; // Offset between model origin and physics CoM

    // Parts grouped by role for fast runtime rendering
    std::vector<LoadedModelPart> chassisParts;
    std::array<std::vector<LoadedModelPart>, 4> wheelParts; // 0=FL, 1=FR, 2=RL, 3=RR
    std::array<std::vector<LoadedModelPart>, 4> caliperParts;
    std::vector<LoadedModelPart> steeringWheelParts;

    size_t totalVertices{0};
    size_t totalTriangles{0};
    std::vector<std::string> diagnosticWarnings;
};

class ModelLoaderGLB {
public:
    /**
     * Loads a 3D vehicle model from a .glb or .gltf file and binds nodes to semantic vehicle roles.
     * Searches relative to project paths and supports an optional JSON manifest.
     */
    static bool loadFromFile(
        const std::string& glbFilePath,
        LoadedVehicleModel& outModel,
        const std::string& manifestFilePath = ""
    );

    /**
     * Attempts to find a vehicle model file across standard project asset directories.
     */
    static std::string resolveModelPath(const std::string& relPath);
};

} // namespace torquelab
