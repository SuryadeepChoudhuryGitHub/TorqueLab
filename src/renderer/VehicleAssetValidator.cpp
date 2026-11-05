#include "renderer/VehicleAssetValidator.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace torquelab {

namespace {

std::string toLowerStr(const std::string& str) {
    std::string s = str;
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool isLikelyOpaqueBodyPart(const std::string& partName, const std::string& matName) {
    const std::string p = toLowerStr(partName);
    const std::string m = toLowerStr(matName);

    // If it's explicitly glass or lens, it's not an opaque body part
    if (p.find("glass") != std::string::npos || m.find("glass") != std::string::npos ||
        p.find("windshield") != std::string::npos || m.find("windshield") != std::string::npos ||
        p.find("window") != std::string::npos || m.find("window") != std::string::npos ||
        p.find("lens") != std::string::npos || m.find("lens") != std::string::npos) {
        return false;
    }

    // Check for structural body, paint, or chassis indicators
    if (p.find("paint") != std::string::npos || m.find("paint") != std::string::npos ||
        p.find("body") != std::string::npos || m.find("body") != std::string::npos ||
        p.find("hood") != std::string::npos || p.find("trunk") != std::string::npos ||
        p.find("door") != std::string::npos || p.find("fender") != std::string::npos ||
        p.find("bumper") != std::string::npos || p.find("roof") != std::string::npos ||
        p.find("chassis") != std::string::npos || p.find("frame") != std::string::npos ||
        m.find("metal") != std::string::npos || m.find("rubber") != std::string::npos) {
        return true;
    }
    return false;
}

} // namespace

VehicleAssetValidationReport VehicleAssetValidator::validate(const LoadedVehicleModel& model) {
    VehicleAssetValidationReport rep{};
    rep.vehicleName = model.modelName;
    rep.filePath = model.filePath;
    rep.totalVertices = model.totalVertices;
    rep.totalTriangles = model.totalTriangles;
    rep.chassisPartCount = model.chassisParts.size();
    rep.uniqueMaterialsCount = model.uniqueMaterials.size();

    size_t wCount = 0;
    for (int i = 0; i < 4; ++i) {
        wCount += model.wheelParts[static_cast<size_t>(i)].size();
    }
    rep.wheelPartCount = wCount;
    rep.totalParts = rep.chassisPartCount + rep.wheelPartCount;

    for (int i = 0; i < 4; ++i) {
        rep.totalParts += model.caliperParts[static_cast<size_t>(i)].size();
    }
    rep.totalParts += model.steeringWheelParts.size();

    // 1. Audit Geometry & Meshes across all parts
    auto auditPart = [&](const LoadedModelPart& p) {
        if (p.isTransparent) {
            rep.transparentPartCount++;
        } else {
            rep.opaquePartCount++;
        }

        if (p.material.doubleSided) {
            rep.doubleSidedPartCount++;
        }

        // Check for accidental body transparency leakage
        if (p.isTransparent && isLikelyOpaqueBodyPart(p.name, p.material.name)) {
            rep.bodyTransparencyLeaks++;
            ValidationIssue issue;
            issue.severity = ValidationIssue::Severity::Error;
            issue.category = "Material";
            issue.message = "Bodywork component '" + p.name + "' with material '" + p.material.name + "' is marked transparent!";
            rep.issues.push_back(issue);
        }

        if (p.material.name.empty() || p.material.name == "DefaultMaterial") {
            rep.unassignedMaterialParts++;
        }

        // CPU geometry checks
        const auto& indices = p.cpuGeometry.indices;
        const auto& verts = p.cpuGeometry.vertices;
        const size_t numTris = indices.size() / 3;

        for (size_t t = 0; t < numTris; ++t) {
            const unsigned int i0 = indices[t * 3 + 0];
            const unsigned int i1 = indices[t * 3 + 1];
            const unsigned int i2 = indices[t * 3 + 2];

            if (i0 == i1 || i1 == i2 || i0 == i2) {
                rep.degenerateTriangles++;
            } else if (i0 < verts.size() && i1 < verts.size() && i2 < verts.size()) {
                const Vec3& v0 = verts[i0];
                const Vec3& v1 = verts[i1];
                const Vec3& v2 = verts[i2];
                const Vec3 e1 = vec3Sub(v1, v0);
                const Vec3 e2 = vec3Sub(v2, v0);
                const Vec3 cp = vec3Cross(e1, e2);
                if (vec3LengthSq(cp) < 1e-12) {
                    rep.degenerateTriangles++;
                }
            }
        }

        // Determinant of 3x3 local bind transform (check for negative scale / inverted winding)
        const Mat4& m = p.localBindMatrix;
        const double det = m.m[0] * (m.m[5] * m.m[10] - m.m[6] * m.m[9])
                         - m.m[1] * (m.m[4] * m.m[10] - m.m[6] * m.m[8])
                         + m.m[2] * (m.m[4] * m.m[9] - m.m[5] * m.m[8]);
        if (det < -1e-6) {
            rep.invertedWindingParts++;
        }
    };

    for (const auto& p : model.chassisParts) auditPart(p);
    for (int i = 0; i < 4; ++i) {
        for (const auto& p : model.wheelParts[static_cast<size_t>(i)]) auditPart(p);
        for (const auto& p : model.caliperParts[static_cast<size_t>(i)]) auditPart(p);
    }
    for (const auto& p : model.steeringWheelParts) auditPart(p);

    // 2. Audit Bounding Box
    const Vec3 bMin = model.aabbMin;
    const Vec3 bMax = model.aabbMax;
    rep.boundsExtents = Vec3{
        std::abs(bMax.x - bMin.x),
        std::abs(bMax.y - bMin.y),
        std::abs(bMax.z - bMin.z)
    };

    // Automotive sanity ranges:
    // Width (X): 1.4m - 2.8m, Height (Y): 1.0m - 2.8m, Length (Z): 3.0m - 6.5m
    rep.boundsProportionsPlausible = (rep.boundsExtents.x >= 1.2 && rep.boundsExtents.x <= 3.0 &&
                                      rep.boundsExtents.y >= 0.9 && rep.boundsExtents.y <= 3.0 &&
                                      rep.boundsExtents.z >= 2.5 && rep.boundsExtents.z <= 7.0);

    if (!rep.boundsProportionsPlausible) {
        ValidationIssue issue;
        issue.severity = ValidationIssue::Severity::Warning;
        issue.category = "Geometry";
        issue.message = "Vehicle bounding box extents (W=" + std::to_string(rep.boundsExtents.x) +
                        "m, H=" + std::to_string(rep.boundsExtents.y) +
                        "m, L=" + std::to_string(rep.boundsExtents.z) + "m) deviate from typical automotive range.";
        rep.issues.push_back(issue);
    }

    // 3. Audit Wheels & Alignment
    rep.allFourWheelsIdentified = (model.wheelsIdentified[0] && model.wheelsIdentified[1] &&
                                   model.wheelsIdentified[2] && model.wheelsIdentified[3]);

    if (!rep.allFourWheelsIdentified) {
        ValidationIssue issue;
        issue.severity = ValidationIssue::Severity::Error;
        issue.category = "Wheels";
        issue.message = "One or more wheel quadrants failed identification (FL=" +
                        std::string(model.wheelsIdentified[0] ? "OK" : "MISSING") +
                        ", FR=" + std::string(model.wheelsIdentified[1] ? "OK" : "MISSING") +
                        ", RL=" + std::string(model.wheelsIdentified[2] ? "OK" : "MISSING") +
                        ", RR=" + std::string(model.wheelsIdentified[3] ? "OK" : "MISSING") + ").";
        rep.issues.push_back(issue);
    } else {
        const double frontZ = 0.5 * (model.modelWheelCenters[0].z + model.modelWheelCenters[1].z);
        const double rearZ  = 0.5 * (model.modelWheelCenters[2].z + model.modelWheelCenters[3].z);
        rep.wheelbase = std::abs(frontZ - rearZ);

        const double frontTrack = std::abs(model.modelWheelCenters[0].x - model.modelWheelCenters[1].x);
        const double rearTrack  = std::abs(model.modelWheelCenters[2].x - model.modelWheelCenters[3].x);
        rep.trackWidth = 0.5 * (frontTrack + rearTrack);

        double radSum = 0.0;
        for (int i = 0; i < 4; ++i) radSum += model.modelWheelRadii[static_cast<size_t>(i)];
        rep.averageWheelRadius = radSum * 0.25;

        rep.wheelbasePlausible = (rep.wheelbase >= 2.0 && rep.wheelbase <= 4.5);
        rep.trackWidthPlausible = (rep.trackWidth >= 1.2 && rep.trackWidth <= 2.5);
        rep.wheelRadiusPlausible = (rep.averageWheelRadius >= 0.22 && rep.averageWheelRadius <= 0.55);

        if (!rep.wheelbasePlausible) {
            ValidationIssue issue;
            issue.severity = ValidationIssue::Severity::Warning;
            issue.category = "Wheels";
            issue.message = "Derived wheelbase (" + std::to_string(rep.wheelbase) + "m) is outside plausible range [2.0m, 4.5m].";
            rep.issues.push_back(issue);
        }
        if (!rep.trackWidthPlausible) {
            ValidationIssue issue;
            issue.severity = ValidationIssue::Severity::Warning;
            issue.category = "Wheels";
            issue.message = "Derived track width (" + std::to_string(rep.trackWidth) + "m) is outside plausible range [1.2m, 2.5m].";
            rep.issues.push_back(issue);
        }
        if (!rep.wheelRadiusPlausible) {
            ValidationIssue issue;
            issue.severity = ValidationIssue::Severity::Warning;
            issue.category = "Wheels";
            issue.message = "Derived wheel radius (" + std::to_string(rep.averageWheelRadius) + "m) is outside plausible range [0.22m, 0.55m].";
            rep.issues.push_back(issue);
        }
    }

    rep.calipersIdentified = (!model.caliperParts[0].empty() || !model.caliperParts[1].empty() ||
                              !model.caliperParts[2].empty() || !model.caliperParts[3].empty());
    rep.steeringWheelIdentified = !model.steeringWheelParts.empty();

    // 4. Overall Pass/Fail Evaluation
    bool hasCriticalError = false;
    for (const auto& is : rep.issues) {
        if (is.severity == ValidationIssue::Severity::Error) {
            hasCriticalError = true;
            break;
        }
    }

    rep.passed = (!hasCriticalError &&
                  rep.totalVertices >= 1000 &&
                  rep.totalTriangles >= 500 &&
                  rep.chassisPartCount >= 1 &&
                  rep.allFourWheelsIdentified &&
                  rep.bodyTransparencyLeaks == 0);

    rep.formattedReport = generateReportString(rep);
    return rep;
}

std::string VehicleAssetValidator::generateReportString(const VehicleAssetValidationReport& rep) {
    std::ostringstream ss;
    ss << "================================================================================\n";
    ss << " VEHICLE ASSET VALIDATION REPORT\n";
    ss << " Vehicle: " << rep.vehicleName << " (" << rep.filePath << ")\n";
    ss << "--------------------------------------------------------------------------------\n";
    ss << " [GEOMETRY]\n";
    ss << "   Vertices: " << rep.totalVertices
       << " | Triangles: " << rep.totalTriangles
       << " | Parts: " << rep.totalParts
       << " (Chassis: " << rep.chassisPartCount << ", Wheels: " << rep.wheelPartCount << ")\n";
    ss << "   Extents: Width=" << std::fixed << std::setprecision(2) << rep.boundsExtents.x
       << "m, Height=" << rep.boundsExtents.y
       << "m, Length=" << rep.boundsExtents.z
       << "m (Proportions: " << (rep.boundsProportionsPlausible ? "PASS" : "WARN") << ")\n";
    ss << "   Degenerate Triangles: " << rep.degenerateTriangles
       << " | Inverted Winding Parts: " << rep.invertedWindingParts << "\n";

    ss << " [MATERIALS & TRANSPARENCY]\n";
    ss << "   Unique Materials: " << rep.uniqueMaterialsCount
       << " | Opaque Parts: " << rep.opaquePartCount
       << " | Transparent Parts: " << rep.transparentPartCount << "\n";
    ss << "   Double-Sided Parts: " << rep.doubleSidedPartCount
       << " | Unassigned Material Parts: " << rep.unassignedMaterialParts << "\n";
    ss << "   Bodywork Transparency Leakage: "
       << (rep.bodyTransparencyLeaks == 0 ? "NONE (PASS)" : std::to_string(rep.bodyTransparencyLeaks) + " LEAKS DETECTED (FAIL)") << "\n";

    ss << " [HIERARCHY & WHEEL ALIGNMENT]\n";
    ss << "   Wheel Quadrants Identified: " << (rep.allFourWheelsIdentified ? "4/4 (PASS)" : "INCOMPLETE (FAIL)") << "\n";
    if (rep.allFourWheelsIdentified) {
        ss << "   Wheelbase: " << std::fixed << std::setprecision(3) << rep.wheelbase << " m"
           << " (" << (rep.wheelbasePlausible ? "PASS" : "WARN") << ")\n";
        ss << "   Track Width: " << rep.trackWidth << " m"
           << " (" << (rep.trackWidthPlausible ? "PASS" : "WARN") << ")\n";
        ss << "   Average Wheel Radius: " << rep.averageWheelRadius << " m"
           << " (" << (rep.wheelRadiusPlausible ? "PASS" : "WARN") << ")\n";
    }
    ss << "   Brake Calipers: " << (rep.calipersIdentified ? "Identified" : "Not Found (Optional)")
       << " | Steering Wheel: " << (rep.steeringWheelIdentified ? "Identified" : "Not Found (Optional)") << "\n";

    if (!rep.issues.empty()) {
        ss << " [DIAGNOSTIC ISSUES]\n";
        for (const auto& is : rep.issues) {
            const char* sevStr = (is.severity == ValidationIssue::Severity::Error) ? "[ERROR]" :
                                 (is.severity == ValidationIssue::Severity::Warning) ? "[WARN ]" : "[INFO ]";
            ss << "   " << sevStr << " [" << is.category << "] " << is.message << "\n";
        }
    }

    ss << "--------------------------------------------------------------------------------\n";
    ss << " OVERALL ASSET INTEGRITY: " << (rep.passed ? "PASS" : "FAIL") << "\n";
    ss << "================================================================================\n";

    return ss.str();
}

void VehicleAssetValidator::printReport(const VehicleAssetValidationReport& rep) {
    std::cout << rep.formattedReport << std::endl;
}

} // namespace torquelab
