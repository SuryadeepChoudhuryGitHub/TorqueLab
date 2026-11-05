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
    // Accurate model-matched track widths: tucks flush inside fenders (chassis half-width 0.88m)
    cfg.trackWidth = 1.36;
    cfg.frontTrackWidth = 1.34;
    cfg.rearTrackWidth = 1.38;
    cfg.chassisHalfExtents = Vec3{0.88, 0.36, 2.18};
    cfg.frontWeightDistribution = 0.52;
    cfg.suspensionMountHeight = 0.15;
    cfg.suspensionRestLength = 0.32;
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
    cfg.tireNominalPressureKpa = 220.0;
    cfg.frontTirePressureKpa = 220.0;
    cfg.rearTirePressureKpa = 220.0;
    cfg.tireRimRadius = 0.22;
    cfg.frontTireRimRadius = 0.22;
    cfg.rearTireRimRadius = 0.22;
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
    cfg.wheelCollisionFidelity = WheelCollisionFidelity::Balanced;
    cfg.enableStationaryHold = true;
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

    // Chassis geometry: 2.76m wheelbase, authentic 1.427m model track width
    cfg.wheelbase = 2.76;
    cfg.trackWidth = 1.427;
    cfg.frontTrackWidth = 1.427;
    cfg.rearTrackWidth = 1.427;
    cfg.chassisHalfExtents = Vec3{0.92, 0.35, 2.33};
    cfg.suspensionMountHeight = 0.10;
    cfg.wheelRadius = 0.297;
    cfg.frontWheelRadius = 0.297;
    cfg.rearWheelRadius = 0.297;
    cfg.wheelWidth = 0.238;
    cfg.frontWheelWidth = 0.238;
    cfg.rearWheelWidth = 0.238;
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
    cfg.tireNominalPressureKpa = 220.0;
    cfg.frontTirePressureKpa = 220.0;
    cfg.rearTirePressureKpa = 230.0;
    cfg.tireRimRadius = 0.235;
    cfg.frontTireRimRadius = 0.235;
    cfg.rearTireRimRadius = 0.235;
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

VehicleConfig createDodgeChallengerConfig() {
    VehicleConfig cfg{};
    cfg.id = "dodge_challenger";
    cfg.displayName = "1970 Dodge Challenger R/T";
    cfg.visualType = VehicleVisualType::ImportedGlb;
    cfg.modelGlbPath = "model/Dodge_Challenger.glb";
    cfg.modelManifestPath = "model/Dodge_Challenger_manifest.json";
    cfg.visualOffset = Vec3{0.0, 0.0, 0.0};
    cfg.collisionShapeType = CollisionShapeType::TaperedCompound;
    cfg.noseTaperWidthFraction = 0.82;
    cfg.rearTaperWidthFraction = 0.86;

    // Authentic 1970 Dodge Challenger R/T 426 Hemi specs: ~1680 kg curb weight
    cfg.mass = 1680.0;
    cfg.comHeightOffset = 0.0;
    cfg.frontWeightDistribution = 0.54; // Cast iron 426 Hemi V8 up front
    cfg.inertiaTensor = Vec3{2850.0, 3100.0, 780.0};

    // Authentic E-body geometry: 110.0 in (2.794 m) wheelbase, 1.56 m track width
    cfg.wheelbase = 2.794;
    cfg.trackWidth = 1.56;
    cfg.chassisHalfExtents = Vec3{0.965, 0.38, 2.43}; // Length 4.86m, Width 1.93m
    cfg.suspensionMountHeight = 0.10;
    cfg.wheelRadius = 0.32; // F70-14 / G60-15 bias ply muscle tires
    cfg.wheelWidth = 0.225;
    cfg.wheelInertia = 1.20;

    // Heavy-duty front torsion bars + rear multi-leaf springs (authentic 70s muscle car feel)
    cfg.suspensionRestLength = 0.38;
    cfg.suspensionMinLength = 0.14;
    cfg.suspensionMaxLength = 0.49;
    cfg.frontSpringStiffness = 32000.0;
    cfg.rearSpringStiffness = 29500.0;
    cfg.frontDampingCompression = 3300.0;
    cfg.frontDampingRebound = 4500.0;
    cfg.rearDampingCompression = 3000.0;
    cfg.rearDampingRebound = 4200.0;
    cfg.bumpStopStiffness = 175000.0;
    cfg.frontAntiRollStiffness = 14000.0;
    cfg.rearAntiRollStiffness = 9000.0; // Soft rear ARB produces authentic rear squat on hard launch

    // Vintage muscle car bias-ply / radial tires (high slip angle, progressive breakaway)
    cfg.tireFrictionCoeff = 1.08;
    cfg.tireNominalPressureKpa = 210.0;
    cfg.frontTirePressureKpa = 210.0;
    cfg.rearTirePressureKpa = 210.0;
    cfg.tireRimRadius = 0.20;
    cfg.frontTireRimRadius = 0.20;
    cfg.rearTireRimRadius = 0.20;
    cfg.tireLongitudinalStiffness = 11.8;
    cfg.tireLateralStiffness = 10.5;
    cfg.tireLoadSensitivity = 0.12;
    cfg.nominalTireLoad = 4120.0;
    cfg.tireForceApplicationHeightOffset = 0.05;

    // Steering (power recirculating ball)
    cfg.maxSteeringAngle = 0.55;
    cfg.steeringSpeed = 2.4;
    cfg.steeringReturnSpeed = 3.2;
    cfg.steeringSpeedSensitivity = 0.028;
    cfg.ackermannFactor = 0.85;

    // Chrysler 426 Hemi V8 (425 hp @ 5000 RPM, 490 lb-ft / 664 N*m @ 4000 RPM)
    cfg.drivetrainLayout = DrivetrainLayout::RWD;
    cfg.engineTorqueMultiplier = 1.0;
    cfg.torqueCurve = {
        {600.0,  320.0},
        {800.0,  390.0},
        {1500.0, 520.0},
        {2500.0, 610.0},
        {4000.0, 664.0}, // Peak torque
        {5000.0, 610.0}, // Peak horsepower
        {5800.0, 520.0},
        {6500.0, 360.0},
        {6800.0,  80.0}
    };
    cfg.idleRpm = 750.0;
    cfg.maxRpm = 6500.0;
    cfg.autoUpshiftRpm = 5600.0;
    cfg.autoDownshiftRpm = 2000.0;
    cfg.engineInertia = 0.38; // Heavy iron V8 rotating mass
    cfg.engineBrakingTorque = 120.0;

    // 4-Speed A-833 manual transmission & Dana 60 Sure-Grip rear axle (3.54:1)
    cfg.gearRatios = {-2.66, 0.0, 2.66, 1.91, 1.39, 1.00};
    cfg.finalDriveRatio = 3.54;
    cfg.drivetrainEfficiency = 0.88;
    cfg.differentialLockingCoeff = 65.0; // Dana 60 Sure-Grip mechanical LSD
    cfg.shiftDuration = 0.18;

    // Heavy-duty braking (11-inch front discs / 10-inch rear drums)
    cfg.maxBrakeTorque = 3600.0;
    cfg.brakeFrontBias = 0.62;
    cfg.handbrakeTorque = 2800.0;

    // Aerodynamics (wide muscle coupe)
    cfg.airDensity = 1.225;
    cfg.dragCoefficient = 0.44; // Classic 1970 muscle car aerodynamics
    cfg.frontalArea = 2.35;
    cfg.downforceCoefficient = 0.08;
    cfg.rollingResistanceCoeff = 0.015;

    return cfg;
}

VehicleConfig createHummerConfig() {
    VehicleConfig cfg{};
    cfg.id = "hummer";
    cfg.displayName = "Hummer H2";
    cfg.visualType = VehicleVisualType::ImportedGlb;
    cfg.modelGlbPath = "model/HUMMER.glb";
    cfg.modelManifestPath = "model/HUMMER_manifest.json";
    cfg.visualOffset = Vec3{0.0442, -0.0067, -0.0025};
    cfg.collisionShapeType = CollisionShapeType::TaperedCompound;
    cfg.noseTaperWidthFraction = 0.88;
    cfg.rearTaperWidthFraction = 0.92;

    // Authentic Hummer H2 curb weight: ~2900 kg (6400 lbs)
    cfg.mass = 2900.0;
    cfg.comHeightOffset = 0.08;
    cfg.frontWeightDistribution = 0.53; // Vortec 6.2L V8 and heavy transfer case
    cfg.inertiaTensor = Vec3{5200.0, 5800.0, 1850.0};

    // Authentic full-size SUV geometry derived from model bounds
    cfg.wheelbase = 3.128;
    cfg.trackWidth = 1.731;
    cfg.frontTrackWidth = 1.731;
    cfg.rearTrackWidth = 1.731;
    cfg.chassisHalfExtents = Vec3{1.03, 0.58, 2.40}; // Length 4.80m, Width 2.06m
    cfg.suspensionMountHeight = 0.22;
    cfg.wheelRadius = 0.430; // 34-inch all-terrain tire (315/70R17)
    cfg.frontWheelRadius = 0.430;
    cfg.rearWheelRadius = 0.430;
    cfg.wheelWidth = 0.320;
    cfg.frontWheelWidth = 0.320;
    cfg.rearWheelWidth = 0.320;
    cfg.wheelInertia = 2.40;

    // Heavy-duty long-travel independent front suspension + 5-link rear solid axle
    cfg.suspensionRestLength = 0.46;
    cfg.suspensionMinLength = 0.16;
    cfg.suspensionMaxLength = 0.58;
    cfg.frontSpringStiffness = 54000.0;
    cfg.rearSpringStiffness = 48000.0;
    cfg.frontDampingCompression = 4800.0;
    cfg.frontDampingRebound = 7200.0;
    cfg.rearDampingCompression = 4200.0;
    cfg.rearDampingRebound = 6500.0;
    cfg.bumpStopStiffness = 260000.0;
    cfg.frontAntiRollStiffness = 16000.0;
    cfg.rearAntiRollStiffness = 12000.0;

    // High-volume pneumatic all-terrain / mud-terrain tires
    cfg.tireFrictionCoeff = 1.02;
    cfg.tireNominalPressureKpa = 250.0;
    cfg.frontTirePressureKpa = 250.0;
    cfg.rearTirePressureKpa = 260.0;
    cfg.tireRimRadius = 0.22; // 17-inch rims
    cfg.frontTireRimRadius = 0.22;
    cfg.rearTireRimRadius = 0.22;
    cfg.tireLongitudinalStiffness = 11.0;
    cfg.tireLateralStiffness = 9.8;
    cfg.tireLoadSensitivity = 0.10;
    cfg.nominalTireLoad = 7200.0;
    cfg.tireForceApplicationHeightOffset = 0.08;

    // Power recirculating-ball steering
    cfg.maxSteeringAngle = 0.54;
    cfg.steeringSpeed = 2.2;
    cfg.steeringReturnSpeed = 3.0;
    cfg.steeringSpeedSensitivity = 0.032;
    cfg.ackermannFactor = 0.82;

    // GM Vortec 6200 6.2L V8 (393 hp @ 5700 RPM, 565 N*m @ 4300 RPM)
    cfg.drivetrainLayout = DrivetrainLayout::AWD; // Full-time AWD
    cfg.engineTorqueMultiplier = 1.0;
    cfg.torqueCurve = {
        {600.0,  320.0},
        {1000.0, 420.0},
        {2000.0, 490.0},
        {3000.0, 535.0},
        {4300.0, 565.0}, // Peak torque
        {5200.0, 550.0},
        {5700.0, 495.0}, // Peak horsepower (~393 hp)
        {6200.0, 380.0},
        {6500.0, 100.0}
    };
    cfg.idleRpm = 650.0;
    cfg.maxRpm = 6200.0;
    cfg.autoUpshiftRpm = 5500.0;
    cfg.autoDownshiftRpm = 1800.0;
    cfg.engineInertia = 0.42; // Heavy rotating V8 mass + torque converter
    cfg.engineBrakingTorque = 135.0;

    // Heavy-Duty 6-Speed Automatic Transmission (Hydra-Matic 6L80) & 4.10:1 axle ratio
    cfg.gearRatios = {-3.064, 0.0, 4.027, 2.364, 1.532, 1.152, 0.852, 0.667};
    cfg.finalDriveRatio = 4.10;
    cfg.drivetrainEfficiency = 0.84;
    cfg.differentialLockingCoeff = 75.0; // Heavy-duty locking center & axle diffs
    cfg.shiftDuration = 0.25;

    // Heavy-duty braking (4-wheel vented discs with ABS)
    cfg.maxBrakeTorque = 4800.0;
    cfg.brakeFrontBias = 0.64;
    cfg.handbrakeTorque = 3600.0;

    // Aerodynamics (massive upright frontal area & knobby rolling resistance)
    cfg.airDensity = 1.225;
    cfg.dragCoefficient = 0.54;
    cfg.frontalArea = 3.65;
    cfg.downforceCoefficient = 0.02;
    cfg.rollingResistanceCoeff = 0.018;

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
    out << "  \"front_track_width\": " << cfg.frontTrackWidth << ",\n";
    out << "  \"rear_track_width\": " << cfg.rearTrackWidth << ",\n";
    out << "  \"chassis_half_extents\": [" << cfg.chassisHalfExtents.x << ", " << cfg.chassisHalfExtents.y << ", " << cfg.chassisHalfExtents.z << "],\n";
    out << "  \"wheel_radius\": " << cfg.wheelRadius << ",\n";
    out << "  \"front_wheel_radius\": " << cfg.frontWheelRadius << ",\n";
    out << "  \"rear_wheel_radius\": " << cfg.rearWheelRadius << ",\n";
    out << "  \"wheel_width\": " << cfg.wheelWidth << ",\n";
    out << "  \"front_wheel_width\": " << cfg.frontWheelWidth << ",\n";
    out << "  \"rear_wheel_width\": " << cfg.rearWheelWidth << ",\n";
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
    out << "  \"downforce_coefficient\": " << cfg.downforceCoefficient << ",\n";
    out << "  \"wheel_collision_fidelity\": " << static_cast<int>(cfg.wheelCollisionFidelity) << ",\n";
    out << "  \"enable_stationary_hold\": " << (cfg.enableStationaryHold ? "true" : "false") << "\n";
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
        if (trimmed.find("\"front_track_width\"") != std::string::npos) outCfg.frontTrackWidth = extractDouble(trimmed, "front_track_width", outCfg.frontTrackWidth);
        if (trimmed.find("\"rear_track_width\"") != std::string::npos) outCfg.rearTrackWidth = extractDouble(trimmed, "rear_track_width", outCfg.rearTrackWidth);
        if (trimmed.find("\"wheel_radius\"") != std::string::npos) outCfg.wheelRadius = extractDouble(trimmed, "wheel_radius", outCfg.wheelRadius);
        if (trimmed.find("\"front_wheel_radius\"") != std::string::npos) outCfg.frontWheelRadius = extractDouble(trimmed, "front_wheel_radius", outCfg.frontWheelRadius);
        if (trimmed.find("\"rear_wheel_radius\"") != std::string::npos) outCfg.rearWheelRadius = extractDouble(trimmed, "rear_wheel_radius", outCfg.rearWheelRadius);
        if (trimmed.find("\"wheel_width\"") != std::string::npos) outCfg.wheelWidth = extractDouble(trimmed, "wheel_width", outCfg.wheelWidth);
        if (trimmed.find("\"front_wheel_width\"") != std::string::npos) outCfg.frontWheelWidth = extractDouble(trimmed, "front_wheel_width", outCfg.frontWheelWidth);
        if (trimmed.find("\"rear_wheel_width\"") != std::string::npos) outCfg.rearWheelWidth = extractDouble(trimmed, "rear_wheel_width", outCfg.rearWheelWidth);
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
        if (trimmed.find("\"wheel_collision_fidelity\"") != std::string::npos) {
            const int fVal = static_cast<int>(extractDouble(trimmed, "wheel_collision_fidelity", 1.0));
            outCfg.wheelCollisionFidelity = static_cast<WheelCollisionFidelity>(std::clamp(fVal, 0, 2));
        }
        if (trimmed.find("\"enable_stationary_hold\"") != std::string::npos) {
            outCfg.enableStationaryHold = (trimmed.find("true") != std::string::npos);
        }
    }
    return true;
}

} // namespace torquelab
