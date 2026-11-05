#pragma once
/**
 * Torque Lab Native — Universal Vehicle Asset Validation System.
 *
 * Provides comprehensive, automated diagnostic auditing of imported 3D vehicle assets:
 * - Geometric integrity: degenerate triangles, vertex normal sanity, tangent orthonormalization,
 *   inverted winding detection, and automotive bounding box proportions.
 * - Material & Transparency validation: bodywork opacity enforcement, glass isolation,
 *   alpha mode sanity, double-sided culling verification, and missing texture reporting.
 * - Hierarchy & Kinematic alignment: 4-quadrant wheel identification, wheelbase/track width
 *   boundaries, wheel radius sanity, caliper/steering wheel semantic role mapping.
 * - Produces formatted diagnostic reports suitable for CI/CLI diagnostics and live UI display.
 */

#include <string>
#include <vector>
#include "renderer/ModelLoaderGLB.h"

namespace torquelab {

struct ValidationIssue {
    enum class Severity {
        Info,
        Warning,
        Error
    };

    Severity severity{Severity::Info};
    std::string category; // "Geometry", "Material", "Hierarchy", "Wheels"
    std::string message;
};

struct VehicleAssetValidationReport {
    std::string vehicleName;
    std::string filePath;
    bool passed{false};

    // Geometry Metrics
    size_t totalVertices{0};
    size_t totalTriangles{0};
    size_t totalParts{0};
    size_t chassisPartCount{0};
    size_t wheelPartCount{0};
    size_t degenerateTriangles{0};
    size_t missingNormalVertices{0};
    size_t invertedWindingParts{0};
    Vec3 boundsExtents{0, 0, 0};
    bool boundsProportionsPlausible{false};

    // Material Metrics
    size_t uniqueMaterialsCount{0};
    size_t opaquePartCount{0};
    size_t transparentPartCount{0};
    size_t doubleSidedPartCount{0};
    size_t bodyTransparencyLeaks{0};
    size_t unassignedMaterialParts{0};
    size_t missingTextureCount{0};

    // Wheel & Hierarchy Metrics
    bool allFourWheelsIdentified{false};
    double wheelbase{0.0};
    double trackWidth{0.0};
    double averageWheelRadius{0.0};
    bool wheelbasePlausible{false};
    bool trackWidthPlausible{false};
    bool wheelRadiusPlausible{false};
    bool calipersIdentified{false};
    bool steeringWheelIdentified{false};

    std::vector<ValidationIssue> issues;

    // Formatted textual report
    std::string formattedReport;
};

class VehicleAssetValidator {
public:
    /**
     * Runs full battery of diagnostic tests against an imported vehicle model.
     */
    static VehicleAssetValidationReport validate(const LoadedVehicleModel& model);

    /**
     * Formats the validation report into a standard multi-line diagnostic string.
     */
    static std::string generateReportString(const VehicleAssetValidationReport& report);

    /**
     * Prints the diagnostic report to stdout.
     */
    static void printReport(const VehicleAssetValidationReport& report);
};

} // namespace torquelab
