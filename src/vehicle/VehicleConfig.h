#pragma once
/**
 * Complete Vehicle Physical & Visual Configuration Specification (SI Units).
 *
 * Supports decoupled visual models (Imported GLB vs Procedural),
 * vehicle-appropriate collision geometry (TaperedCompound vs Box),
 * and JSON persistence.
 */

#include <array>
#include <string>
#include <vector>
#include "physics/Math3D.h"

namespace torquelab {

enum class DrivetrainLayout {
    FWD,
    RWD,
    AWD
};

enum class VehicleVisualType {
    ImportedGlb,
    Procedural
};

enum class CollisionShapeType {
    TaperedCompound,
    Box
};

enum class WheelCollisionFidelity {
    Performance = 0, // Fast 5-probe single-plane query, minimal CPU overhead
    Balanced = 1,    // 9-probe 3-slice circumferential volume, accurate for curbs & steps
    Detailed = 2     // 17-probe 5-slice multi-angle circumferential hull + sidewall queries
};

struct TorqueCurvePoint {
    double rpm{0.0};
    double torqueNm{0.0};
};

struct VehicleConfig {
    // --- Identification & Visual Asset Binding ---
    std::string id{"procedural_coupe"};
    std::string displayName{"Procedural Sports Coupe"};
    VehicleVisualType visualType{VehicleVisualType::Procedural};
    std::string modelGlbPath{"model/BMW_M4.glb"};
    std::string modelManifestPath{"model/BMW_M4_manifest.json"};
    Vec3 visualOffset{0.0, 0.0, 0.0}; // Offset between physics chassis origin and visual model origin

    // --- Collision Representation ---
    CollisionShapeType collisionShapeType{CollisionShapeType::TaperedCompound};
    double noseTaperWidthFraction{0.78}; // Front bumper tip width relative to mid-body
    double rearTaperWidthFraction{0.84}; // Rear bumper tip width relative to mid-body

    // --- Mass & Inertia ---
    double mass{1500.0};                    // Total vehicle mass (kg)
    double comHeightOffset{0.0};            // Vertical Center-of-Mass offset relative to chassis origin (m)
    double frontWeightDistribution{0.52};   // Static front weight distribution fraction [0.35 .. 0.65]
    Vec3 inertiaTensor{2450.0, 2650.0, 680.0}; // Principal moments of inertia (kg*m^2): x=pitch, y=yaw, z=roll

    // --- Chassis & Wheel Geometry ---
    double wheelbase{2.70};                 // Distance between front and rear axles (m)
    double trackWidth{1.60};                // Lateral distance between left and right wheels (m)
    double frontTrackWidth{1.60};           // Front axle track width (m)
    double rearTrackWidth{1.60};            // Rear axle track width (m)
    Vec3 chassisHalfExtents{0.88, 0.36, 2.18}; // Collision half-extents (m): 1.76m wide, 0.72m tall, 4.36m long
    double suspensionMountHeight{0.10};     // Height of upper suspension mount in chassis frame (m)
    double wheelRadius{0.32};               // Tire outer radius (m)
    double frontWheelRadius{0.32};          // Front tire outer radius (m)
    double rearWheelRadius{0.32};           // Rear tire outer radius (m)
    double wheelWidth{0.235};               // Tire width (m)
    double frontWheelWidth{0.235};          // Front tire width (m)
    double rearWheelWidth{0.235};           // Rear tire width (m)
    double wheelInertia{1.15};              // Rotational inertia of a single wheel assembly (kg*m^2)
    bool hasExplicitWheelMounts{false};     // True if exact mount positions are derived from 3D model vertices
    std::array<Vec3, 4> explicitWheelMountsLocal{
        Vec3{ 0.80, 0.10,  1.35},
        Vec3{-0.80, 0.10,  1.35},
        Vec3{ 0.80, 0.10, -1.35},
        Vec3{-0.80, 0.10, -1.35}
    };
    std::array<Vec3, 4> explicitWheelCentersLocal{
        Vec3{ 0.80, -0.28,  1.35},
        Vec3{-0.80, -0.28,  1.35},
        Vec3{ 0.80, -0.28, -1.35},
        Vec3{-0.80, -0.28, -1.35}
    };

    // --- Suspension & Anti-Roll Bars ---
    double suspensionRestLength{0.38};      // Uncompressed spring rest length (m)
    double suspensionMinLength{0.15};       // Minimum length at full bump-stop compression (m)
    double suspensionMaxLength{0.48};       // Maximum length at full droop extension (m)
    double frontSpringStiffness{35000.0};   // Front spring rate k_f (N/m)
    double rearSpringStiffness{32500.0};    // Rear spring rate k_r (N/m)
    double frontDampingCompression{3600.0}; // Front bound damping c_f_comp (N*s/m)
    double frontDampingRebound{4800.0};     // Front rebound damping c_f_reb (N*s/m)
    double rearDampingCompression{3400.0};  // Rear bound damping c_r_comp (N*s/m)
    double rearDampingRebound{4500.0};      // Rear rebound damping c_r_reb (N*s/m)
    double bumpStopStiffness{180000.0};     // Progressive bump-stop stiffness when bottoming out (N/m)
    double frontAntiRollStiffness{16000.0}; // Front anti-roll bar stiffness (N/m)
    double rearAntiRollStiffness{12000.0};  // Rear anti-roll bar stiffness (N/m)

    // --- Tire Model (Pacejka-lite + Friction Circle + Load Sensitivity) ---
    double tireFrictionCoeff{1.10};         // Nominal peak tire friction coefficient on dry asphalt (mu_0)
    double tireLongitudinalStiffness{13.0}; // Longitudinal slip stiffness factor B_x
    double tireLateralStiffness{11.5};      // Cornering stiffness factor B_y
    double tireLoadSensitivity{0.12};       // Load sensitivity coefficient
    double nominalTireLoad{3700.0};         // Reference vertical load per tire F_z0 (N)
    double tireForceApplicationHeightOffset{0.06}; // Height above ground contact where forces act (m)

    // --- Deformable Pneumatic Tyre Specification ---
    double tireNominalPressureKpa{220.0};   // Standard cold inflation pressure (kPa)
    double frontTirePressureKpa{220.0};     // Front axle inflation pressure (kPa)
    double rearTirePressureKpa{220.0};      // Rear axle inflation pressure (kPa)
    double tireRimRadius{0.22};             // Rigid wheel rim radius (m)
    double frontTireRimRadius{0.22};        // Front wheel rim radius (m)
    double rearTireRimRadius{0.22};         // Rear wheel rim radius (m)
    double tireRadialStiffnessBase{180000.0}; // Base radial carcass stiffness at nominal pressure (N/m)
    double tireRadialStiffnessPressureCoeff{0.48}; // Radial stiffness pressure coefficient (N/(m*Pa))
    double tireLateralStiffnessBase{140000.0};     // Base lateral carcass stiffness (N/m)
    double tireLongitudinalStiffnessBase{190000.0}; // Base longitudinal carcass stiffness (N/m)

    // --- Steering System ---
    double maxSteeringAngle{0.60};          // Maximum front wheel steering angle at low speed (rad, ~34.4 deg)
    double steeringSpeed{2.6};              // Steering actuator rate when turning (rad/s)
    double steeringReturnSpeed{3.4};        // Self-centering return rate (rad/s)
    double steeringSpeedSensitivity{0.032}; // Speed sensitivity factor reducing max steer angle at high speed (s/m)
    double ackermannFactor{0.85};           // Ackermann geometry blend (0 = parallel steer, 1 = pure Ackermann)

    // --- Drivetrain, Engine & Transmission ---
    DrivetrainLayout drivetrainLayout{DrivetrainLayout::RWD};
    double engineTorqueMultiplier{1.0};     // Runtime tuning multiplier on engine torque curve
    std::vector<TorqueCurvePoint> torqueCurve{
        {800.0,  190.0},
        {1500.0, 260.0},
        {2500.0, 335.0},
        {3500.0, 380.0},
        {4500.0, 405.0},
        {5500.0, 390.0},
        {6500.0, 350.0},
        {7000.0, 280.0},
        {7300.0, 80.0}
    };
    double idleRpm{850.0};
    double maxRpm{7000.0};
    double autoUpshiftRpm{6400.0};
    double autoDownshiftRpm{2400.0};
    double engineInertia{0.24};             // Effective engine flywheel inertia (kg*m^2)
    double engineBrakingTorque{95.0};       // Peak closed-throttle engine braking torque at redline (N*m)
    // Gear ratios: index 0 = Reverse, index 1 = Neutral, index 2..7 = 1st..6th gears
    std::vector<double> gearRatios{-3.20, 0.0, 3.45, 2.15, 1.52, 1.18, 0.96, 0.79};
    double finalDriveRatio{3.73};
    double drivetrainEfficiency{0.88};
    double differentialLockingCoeff{45.0};  // Viscous limited-slip differential locking coefficient (N*m / (rad/s))
    double shiftDuration{0.14};             // Gear shift torque interruption duration (s)

    // --- Brakes ---
    double maxBrakeTorque{3600.0};          // Total maximum service brake torque capacity per axle scaling (N*m)
    double brakeFrontBias{0.64};            // Fraction of service brake torque sent to front wheels [0.5 .. 0.8]
    double handbrakeTorque{2600.0};         // Maximum mechanical handbrake torque per rear wheel (N*m)

    // --- Aerodynamics & Resistances ---
    double airDensity{1.225};               // Air density rho (kg/m^3)
    double dragCoefficient{0.31};           // Aerodynamic drag coefficient C_d
    double frontalArea{2.15};               // Vehicle frontal area A (m^2)
    double downforceCoefficient{0.18};      // Aerodynamic downforce coefficient C_L
    double rollingResistanceCoeff{0.014};   // Baseline tire rolling resistance coefficient C_rr

    // --- Wheel Physics Fidelity & Stationary Stability ---
    WheelCollisionFidelity wheelCollisionFidelity{WheelCollisionFidelity::Balanced};
    bool enableStationaryHold{true};         // Coulomb stiction & zero-creep holding when stopped
};

/** Built-in vehicle presets */
VehicleConfig createProceduralCoupeConfig();
VehicleConfig createBmwM4Config();
VehicleConfig createDodgeChallengerConfig();

inline VehicleConfig createDefaultVehicleConfig() {
    return createProceduralCoupeConfig();
}

/** JSON persistence */
bool saveVehicleConfigToJson(const VehicleConfig& cfg, const std::string& filePath);
bool loadVehicleConfigFromJson(VehicleConfig& outCfg, const std::string& filePath);

/** Helper to access per-wheel radius (supporting front/rear staggered fitment) */
inline double getWheelRadius(const VehicleConfig& config, int wheelIndex) {
    if (wheelIndex < 2) {
        return (config.frontWheelRadius > 0.05) ? config.frontWheelRadius : config.wheelRadius;
    }
    return (config.rearWheelRadius > 0.05) ? config.rearWheelRadius : config.wheelRadius;
}

/** Helper to access per-wheel width (supporting front/rear staggered fitment) */
inline double getWheelWidth(const VehicleConfig& config, int wheelIndex) {
    if (wheelIndex < 2) {
        return (config.frontWheelWidth > 0.05) ? config.frontWheelWidth : config.wheelWidth;
    }
    return (config.rearWheelWidth > 0.05) ? config.rearWheelWidth : config.wheelWidth;
}

/** Helper to access per-wheel rigid rim radius */
inline double getTireRimRadius(const VehicleConfig& config, int wheelIndex) {
    if (wheelIndex < 2) {
        return (config.frontTireRimRadius > 0.05) ? config.frontTireRimRadius : config.tireRimRadius;
    }
    return (config.rearTireRimRadius > 0.05) ? config.rearTireRimRadius : config.tireRimRadius;
}

/** Helper to access per-wheel inflation pressure (in kPa) */
inline double getTirePressureKpa(const VehicleConfig& config, int wheelIndex) {
    if (wheelIndex < 2) {
        return (config.frontTirePressureKpa > 10.0) ? config.frontTirePressureKpa : config.tireNominalPressureKpa;
    }
    return (config.rearTirePressureKpa > 10.0) ? config.rearTirePressureKpa : config.tireNominalPressureKpa;
}

/** Local mount positions */
inline std::array<Vec3, 4> getWheelMountPositionsLocal(const VehicleConfig& config) {
    if (config.hasExplicitWheelMounts) {
        return config.explicitWheelMountsLocal;
    }
    const double halfTrackFront = (config.frontTrackWidth > 0.0 ? config.frontTrackWidth : config.trackWidth) * 0.5;
    const double halfTrackRear  = (config.rearTrackWidth > 0.0  ? config.rearTrackWidth  : config.trackWidth) * 0.5;
    const double halfWheelbase  = config.wheelbase * 0.5;
    const double y = config.suspensionMountHeight;
    return {
        Vec3{+halfTrackFront, y, +halfWheelbase}, // 0: FL
        Vec3{-halfTrackFront, y, +halfWheelbase}, // 1: FR
        Vec3{+halfTrackRear,  y, -halfWheelbase}, // 2: RL
        Vec3{-halfTrackRear,  y, -halfWheelbase}  // 3: RR
    };
}

inline Vec3 getCenterOfMassLocal(const VehicleConfig& config) {
    const double zOffset = (config.frontWeightDistribution - 0.5) * config.wheelbase;
    return Vec3{0.0, config.comHeightOffset, zOffset};
}

inline double computeEquilibriumChassisHeight(const VehicleConfig& config) {
    const double totalStiffness = 2.0 * (config.frontSpringStiffness + config.rearSpringStiffness);
    const double avgCompression = (config.mass * GRAVITY) / totalStiffness;
    const double eqLength = clamp(
        config.suspensionRestLength - avgCompression,
        config.suspensionMinLength,
        config.suspensionMaxLength
    );
    return eqLength + config.wheelRadius - config.suspensionMountHeight;
}

} // namespace torquelab
