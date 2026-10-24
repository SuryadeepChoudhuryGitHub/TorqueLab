#include "vehicle/VehicleConfig.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace torquelab {

namespace {

std::string trim(const std::string& str) {
    const auto first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

std::string extractString(const std::string& line, const std::string& key) {
    const std::string search = "\"" + key + "\"";
    const auto pos = line.find(search);
    if (pos == std::string::npos) return "";
    const auto col = line.find(':', pos + search.size());
    if (col == std::string::npos) return "";
    const auto q1 = line.find('\"', col);
    if (q1 == std::string::npos) return "";
    const auto q2 = line.find('\"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return line.substr(q1 + 1, q2 - q1 - 1);
}

double extractDouble(const std::string& line, const std::string& key, double defaultVal = 0.0) {
    const std::string search = "\"" + key + "\"";
    const auto pos = line.find(search);
    if (pos == std::string::npos) return defaultVal;
    const auto col = line.find(':', pos + search.size());
    if (col == std::string::npos) return defaultVal;
    std::string valStr = line.substr(col + 1);
    const auto comma = valStr.find(',');
    if (comma != std::string::npos) valStr = valStr.substr(0, comma);
    const auto brace = valStr.find_first_of("}\r\n");
    if (brace != std::string::npos) valStr = valStr.substr(0, brace);
    valStr = trim(valStr);
    try {
        return std::stod(valStr);
    } catch (...) {
        return defaultVal;
    }
}

} // namespace

VehicleConfig createProceduralCoupeConfig() {
    VehicleConfig cfg{};
    cfg.id = "procedural_coupe";
    cfg.displayName = "Procedural Sports Coupe";
    cfg.visualType = VehicleVisualType::Procedural;
    cfg.visualOffset = Vec3{0.0, 0.0, 0.0};
    cfg.collisionShapeType = CollisionShapeType::Box;
    cfg.mass = 1500.0;
    cfg.wheelbase = 2.70;
    cfg.trackWidth = 1.60;
    cfg.chassisHalfExtents = Vec3{0.88, 0.36, 2.18};
    cfg.frontWeightDistribution = 0.52;
    cfg.wheelRadius = 0.32;
    cfg.wheelWidth = 0.235;
    cfg.wheelInertia = 1.15;
    cfg.frontSpringStiffness = 35000.0;
    cfg.rearSpringStiffness = 32500.0;
    cfg.frontDampingCompression = 3600.0;
    cfg.frontDampingRebound = 4800.0;
    cfg.rearDampingCompression = 3400.0;
    cfg.rearDampingRebound = 4500.0;
    cfg.frontAntiRollStiffness = 16000.0;
    cfg.rearAntiRollStiffness = 12000.0;
    cfg.tireFrictionCoeff = 1.10;
    cfg.idleRpm = 850.0;
    cfg.maxRpm = 7000.0;
    cfg.finalDriveRatio = 3.73;
    cfg.gearRatios = {-3.20, 0.0, 3.45, 2.15, 1.52, 1.18, 0.96, 0.79};
    cfg.maxBrakeTorque = 3600.0;
    cfg.brakeFrontBias = 0.64;
    cfg.handbrakeTorque = 2600.0;
    cfg.dragCoefficient = 0.31;
    cfg.frontalArea = 2.15;
    cfg.downforceCoefficient = 0.18;
    cfg.rollingResistanceCoeff = 0.014;
    return cfg;
}

VehicleConfig createBmwM4Config() {
    VehicleConfig cfg{};
    cfg.id = "bmw_m4";
    cfg.displayName = "BMW M4 Coupe";
    cfg.visualType = VehicleVisualType::ImportedGlb;
    cfg.modelGlbPath = "model/BMW_M4.glb";
    cfg.modelManifestPath = "model/BMW_M4_manifest.json";
    cfg.visualOffset = Vec3{0.0, -0.477, -0.1475};
    cfg.collisionShapeType = CollisionShapeType::TaperedCompound;
    cfg.noseTaperWidthFraction = 0.78;
    cfg.rearTaperWidthFraction = 0.84;

    // Factory M4 Specs: ~1600 kg curb weight, 52.5% front weight distribution
    cfg.mass = 1600.0;
    cfg.comHeightOffset = 0.0;
    cfg.frontWeightDistribution = 0.525;
    cfg.inertiaTensor = Vec3{2600.0, 2800.0, 720.0};

    // Chassis geometry: 2.76m wheelbase, 1.58m track width, length 4.67m, width 1.87m
    cfg.wheelbase = 2.76;
    cfg.trackWidth = 1.58;
    cfg.chassisHalfExtents = Vec3{0.92, 0.35, 2.33};
    cfg.suspensionMountHeight = 0.10;
    cfg.wheelRadius = 0.33; // 275/35 R19 & 285/30 R20 Michelin Pilot Super Sport
    cfg.wheelWidth = 0.265;
    cfg.wheelInertia = 1.25;

    // Adaptive M suspension rates
    cfg.suspensionRestLength = 0.38;
    cfg.suspensionMinLength = 0.15;
    cfg.suspensionMaxLength = 0.48;
    cfg.frontSpringStiffness = 38000.0;
    cfg.rearSpringStiffness = 35500.0;
    cfg.frontDampingCompression = 3900.0;
    cfg.frontDampingRebound = 5200.0;
    cfg.rearDampingCompression = 3700.0;
    cfg.rearDampingRebound = 4900.0;
    cfg.bumpStopStiffness = 190000.0;
    cfg.frontAntiRollStiffness = 18000.0;
    cfg.rearAntiRollStiffness = 14000.0;

    // Tire parameters (UHP Performance Tires)
    cfg.tireFrictionCoeff = 1.15;
    cfg.tireLongitudinalStiffness = 13.5;
    cfg.tireLateralStiffness = 12.2;
    cfg.tireLoadSensitivity = 0.11;
    cfg.nominalTireLoad = 3924.0;
    cfg.tireForceApplicationHeightOffset = 0.06;

    // Steering
    cfg.maxSteeringAngle = 0.58;
    cfg.steeringSpeed = 2.8;
    cfg.steeringReturnSpeed = 3.6;
    cfg.steeringSpeedSensitivity = 0.030;
    cfg.ackermannFactor = 0.88;

    // BMW S55/S58 3.0L Twin-Turbo Inline-6 (431 hp, 550 N*m flat from 1850 - 5500 RPM)
    cfg.drivetrainLayout = DrivetrainLayout::RWD;
    cfg.engineTorqueMultiplier = 1.0;
    cfg.torqueCurve = {
        {800.0,  240.0},
        {1500.0, 480.0},
        {1850.0, 550.0},
        {3500.0, 550.0},
        {5500.0, 550.0},
        {6500.0, 480.0},
        {7200.0, 410.0},
        {7500.0, 100.0}
    };
    cfg.idleRpm = 850.0;
    cfg.maxRpm = 7200.0;
    cfg.autoUpshiftRpm = 6800.0;
    cfg.autoDownshiftRpm = 2600.0;
    cfg.engineInertia = 0.26;
    cfg.engineBrakingTorque = 105.0;

    // 6-Speed M Transmission & M-Differential (3.46 final drive)
    cfg.gearRatios = {-3.71, 0.0, 4.11, 2.32, 1.54, 1.18, 1.00, 0.85};
    cfg.finalDriveRatio = 3.46;
    cfg.drivetrainEfficiency = 0.90;
    cfg.differentialLockingCoeff = 55.0; // Active M Differential locking
    cfg.shiftDuration = 0.12;

    // M Compound Brakes (380mm front / 370mm rear discs)
    cfg.maxBrakeTorque = 4200.0;
    cfg.brakeFrontBias = 0.65;
    cfg.handbrakeTorque = 2800.0;

    // Aerodynamics
    cfg.airDensity = 1.225;
    cfg.dragCoefficient = 0.34;
    cfg.frontalArea = 2.23;
    cfg.downforceCoefficient = 0.22;
    cfg.rollingResistanceCoeff = 0.013;

    return cfg;
}

bool saveVehicleConfigToJson(const VehicleConfig& cfg, const std::string& filePath) {
    std::ofstream out(filePath);
    if (!out.is_open()) return false;

    out << "{\n";
    out << "  \"id\": \"" << cfg.id << "\",\n";
    out << "  \"display_name\": \"" << cfg.displayName << "\",\n";
    out << "  \"visual_type\": \"" << (cfg.visualType == VehicleVisualType::ImportedGlb ? "ImportedGlb" : "Procedural") << "\",\n";
    out << "  \"model_glb_path\": \"" << cfg.modelGlbPath << "\",\n";
    out << "  \"model_manifest_path\": \"" << cfg.modelManifestPath << "\",\n";
    out << "  \"visual_offset\": [" << cfg.visualOffset.x << ", " << cfg.visualOffset.y << ", " << cfg.visualOffset.z << "],\n";
    out << "  \"collision_shape_type\": \"" << (cfg.collisionShapeType == CollisionShapeType::TaperedCompound ? "TaperedCompound" : "Box") << "\",\n";
    out << "  \"nose_taper_width_fraction\": " << cfg.noseTaperWidthFraction << ",\n";
    out << "  \"rear_taper_width_fraction\": " << cfg.rearTaperWidthFraction << ",\n";

    out << "  \"mass\": " << cfg.mass << ",\n";
    out << "  \"com_height_offset\": " << cfg.comHeightOffset << ",\n";
    out << "  \"front_weight_distribution\": " << cfg.frontWeightDistribution << ",\n";
    out << "  \"wheelbase\": " << cfg.wheelbase << ",\n";
    out << "  \"track_width\": " << cfg.trackWidth << ",\n";
    out << "  \"chassis_half_extents\": [" << cfg.chassisHalfExtents.x << ", " << cfg.chassisHalfExtents.y << ", " << cfg.chassisHalfExtents.z << "],\n";
    out << "  \"wheel_radius\": " << cfg.wheelRadius << ",\n";
    out << "  \"wheel_width\": " << cfg.wheelWidth << ",\n";
    out << "  \"wheel_inertia\": " << cfg.wheelInertia << ",\n";

    out << "  \"front_spring_stiffness\": " << cfg.frontSpringStiffness << ",\n";
    out << "  \"rear_spring_stiffness\": " << cfg.rearSpringStiffness << ",\n";
    out << "  \"front_damping_compression\": " << cfg.frontDampingCompression << ",\n";
    out << "  \"front_damping_rebound\": " << cfg.frontDampingRebound << ",\n";
    out << "  \"rear_damping_compression\": " << cfg.rearDampingCompression << ",\n";
    out << "  \"rear_damping_rebound\": " << cfg.rearDampingRebound << ",\n";
    out << "  \"front_anti_roll_stiffness\": " << cfg.frontAntiRollStiffness << ",\n";
    out << "  \"rear_anti_roll_stiffness\": " << cfg.rearAntiRollStiffness << ",\n";

    out << "  \"tire_friction_coeff\": " << cfg.tireFrictionCoeff << ",\n";
    out << "  \"drivetrain_layout\": \"" << (cfg.drivetrainLayout == DrivetrainLayout::RWD ? "RWD" : cfg.drivetrainLayout == DrivetrainLayout::FWD ? "FWD" : "AWD") << "\",\n";
    out << "  \"idle_rpm\": " << cfg.idleRpm << ",\n";
    out << "  \"max_rpm\": " << cfg.maxRpm << ",\n";
    out << "  \"final_drive_ratio\": " << cfg.finalDriveRatio << ",\n";
    out << "  \"max_brake_torque\": " << cfg.maxBrakeTorque << ",\n";
    out << "  \"handbrake_torque\": " << cfg.handbrakeTorque << ",\n";
    out << "  \"drag_coefficient\": " << cfg.dragCoefficient << ",\n";
    out << "  \"frontal_area\": " << cfg.frontalArea << ",\n";
    out << "  \"downforce_coefficient\": " << cfg.downforceCoefficient << "\n";
    out << "}\n";

    return true;
}

bool loadVehicleConfigFromJson(VehicleConfig& outCfg, const std::string& filePath) {
    std::ifstream in(filePath);
    if (!in.is_open()) return false;

    std::string line;
    while (std::getline(in, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '{' || trimmed[0] == '}') continue;

        const std::string idStr = extractString(trimmed, "id");
        if (!idStr.empty()) outCfg.id = idStr;

        const std::string nameStr = extractString(trimmed, "display_name");
        if (!nameStr.empty()) outCfg.displayName = nameStr;

        const std::string visStr = extractString(trimmed, "visual_type");
        if (visStr == "ImportedGlb") outCfg.visualType = VehicleVisualType::ImportedGlb;
        else if (visStr == "Procedural") outCfg.visualType = VehicleVisualType::Procedural;

        const std::string glbStr = extractString(trimmed, "model_glb_path");
        if (!glbStr.empty()) outCfg.modelGlbPath = glbStr;

        const std::string manStr = extractString(trimmed, "model_manifest_path");
        if (!manStr.empty()) outCfg.modelManifestPath = manStr;

        const std::string colStr = extractString(trimmed, "collision_shape_type");
        if (colStr == "Box") outCfg.collisionShapeType = CollisionShapeType::Box;
        else if (colStr == "TaperedCompound") outCfg.collisionShapeType = CollisionShapeType::TaperedCompound;

        if (trimmed.find("\"mass\"") != std::string::npos) outCfg.mass = extractDouble(trimmed, "mass", outCfg.mass);
        if (trimmed.find("\"com_height_offset\"") != std::string::npos) outCfg.comHeightOffset = extractDouble(trimmed, "com_height_offset", outCfg.comHeightOffset);
        if (trimmed.find("\"front_weight_distribution\"") != std::string::npos) outCfg.frontWeightDistribution = extractDouble(trimmed, "front_weight_distribution", outCfg.frontWeightDistribution);
        if (trimmed.find("\"wheelbase\"") != std::string::npos) outCfg.wheelbase = extractDouble(trimmed, "wheelbase", outCfg.wheelbase);
        if (trimmed.find("\"track_width\"") != std::string::npos) outCfg.trackWidth = extractDouble(trimmed, "track_width", outCfg.trackWidth);
        if (trimmed.find("\"wheel_radius\"") != std::string::npos) outCfg.wheelRadius = extractDouble(trimmed, "wheel_radius", outCfg.wheelRadius);
        if (trimmed.find("\"wheel_width\"") != std::string::npos) outCfg.wheelWidth = extractDouble(trimmed, "wheel_width", outCfg.wheelWidth);
        if (trimmed.find("\"front_spring_stiffness\"") != std::string::npos) outCfg.frontSpringStiffness = extractDouble(trimmed, "front_spring_stiffness", outCfg.frontSpringStiffness);
        if (trimmed.find("\"rear_spring_stiffness\"") != std::string::npos) outCfg.rearSpringStiffness = extractDouble(trimmed, "rear_spring_stiffness", outCfg.rearSpringStiffness);
        if (trimmed.find("\"front_damping_compression\"") != std::string::npos) outCfg.frontDampingCompression = extractDouble(trimmed, "front_damping_compression", outCfg.frontDampingCompression);
        if (trimmed.find("\"front_damping_rebound\"") != std::string::npos) outCfg.frontDampingRebound = extractDouble(trimmed, "front_damping_rebound", outCfg.frontDampingRebound);
        if (trimmed.find("\"rear_damping_compression\"") != std::string::npos) outCfg.rearDampingCompression = extractDouble(trimmed, "rear_damping_compression", outCfg.rearDampingCompression);
        if (trimmed.find("\"rear_damping_rebound\"") != std::string::npos) outCfg.rearDampingRebound = extractDouble(trimmed, "rear_damping_rebound", outCfg.rearDampingRebound);
        if (trimmed.find("\"front_anti_roll_stiffness\"") != std::string::npos) outCfg.frontAntiRollStiffness = extractDouble(trimmed, "front_anti_roll_stiffness", outCfg.frontAntiRollStiffness);
        if (trimmed.find("\"rear_anti_roll_stiffness\"") != std::string::npos) outCfg.rearAntiRollStiffness = extractDouble(trimmed, "rear_anti_roll_stiffness", outCfg.rearAntiRollStiffness);
        if (trimmed.find("\"tire_friction_coeff\"") != std::string::npos) outCfg.tireFrictionCoeff = extractDouble(trimmed, "tire_friction_coeff", outCfg.tireFrictionCoeff);
        if (trimmed.find("\"final_drive_ratio\"") != std::string::npos) outCfg.finalDriveRatio = extractDouble(trimmed, "final_drive_ratio", outCfg.finalDriveRatio);
        if (trimmed.find("\"max_brake_torque\"") != std::string::npos) outCfg.maxBrakeTorque = extractDouble(trimmed, "max_brake_torque", outCfg.maxBrakeTorque);
        if (trimmed.find("\"drag_coefficient\"") != std::string::npos) outCfg.dragCoefficient = extractDouble(trimmed, "drag_coefficient", outCfg.dragCoefficient);
        if (trimmed.find("\"frontal_area\"") != std::string::npos) outCfg.frontalArea = extractDouble(trimmed, "frontal_area", outCfg.frontalArea);
        if (trimmed.find("\"downforce_coefficient\"") != std::string::npos) outCfg.downforceCoefficient = extractDouble(trimmed, "downforce_coefficient", outCfg.downforceCoefficient);
    }
    return true;
}

} // namespace torquelab
