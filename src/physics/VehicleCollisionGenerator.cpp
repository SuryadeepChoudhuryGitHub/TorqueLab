#include "physics/VehicleCollisionGenerator.h"
#include "renderer/ModelLoaderGLB.h"
#include "vehicle/VehicleConfig.h"

#include <LinearMath/btConvexHullComputer.h>
#include <VHACD.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace torquelab {

namespace {

inline btVector3 toBtVec3(const Vec3& v) {
    return btVector3(
        static_cast<btScalar>(v.x),
        static_cast<btScalar>(v.y),
        static_cast<btScalar>(v.z)
    );
}

inline Vec3 fromBtVec3(const btVector3& v) {
    return Vec3{
        static_cast<double>(v.x()),
        static_cast<double>(v.y()),
        static_cast<double>(v.z())
    };
}

// Spatial 64-bit Morton-style hash key for voxel grid downsampling
uint64_t hashVoxel(int ix, int iy, int iz) {
    const uint64_t ux = static_cast<uint64_t>(ix + 10000) & 0x1FFFFF;
    const uint64_t uy = static_cast<uint64_t>(iy + 10000) & 0x1FFFFF;
    const uint64_t uz = static_cast<uint64_t>(iz + 10000) & 0x1FFFFF;
    return (ux << 42) | (uy << 21) | uz;
}

std::string extractJsonStr(const std::string& line, const std::string& key) {
    const auto kPos = line.find("\"" + key + "\"");
    if (kPos == std::string::npos) return "";
    const auto colon = line.find(':', kPos);
    if (colon == std::string::npos) return "";
    const auto qA = line.find('\"', colon);
    if (qA == std::string::npos) return "";
    const auto qB = line.find('\"', qA + 1);
    if (qB == std::string::npos) return "";
    return line.substr(qA + 1, qB - qA - 1);
}

double extractJsonNum(const std::string& line, const std::string& key, double defVal = 0.0) {
    const auto kPos = line.find("\"" + key + "\"");
    if (kPos == std::string::npos) return defVal;
    const auto colon = line.find(':', kPos);
    if (colon == std::string::npos) return defVal;
    try {
        size_t idx = 0;
        return std::stod(line.substr(colon + 1), &idx);
    } catch (...) {
        return defVal;
    }
}

} // namespace

std::string VehicleCollisionGenerator::getCacheFilePath(const std::string& cacheDir, const std::string& vehicleId) {
    std::string safeId = vehicleId;
    std::replace(safeId.begin(), safeId.end(), ' ', '_');
    std::replace(safeId.begin(), safeId.end(), '/', '_');
    std::replace(safeId.begin(), safeId.end(), '\\', '_');
    return cacheDir + safeId + "_collision.json";
}

std::vector<Vec3> VehicleCollisionGenerator::decimateVertices(
    const std::vector<Vec3>& inputVerts,
    int targetCount,
    const Vec3& aabbMin,
    const Vec3& aabbMax
) {
    if (inputVerts.size() <= static_cast<size_t>(targetCount)) {
        return inputVerts;
    }

    const Vec3 dim = vec3Sub(aabbMax, aabbMin);
    const double volume = std::max(0.1, dim.x * dim.y * dim.z);
    double cellSize = std::cbrt(volume / static_cast<double>(targetCount));
    cellSize = std::clamp(cellSize, 0.025, 0.20);

    std::unordered_map<uint64_t, Vec3> voxelGrid;
    voxelGrid.reserve(static_cast<size_t>(targetCount) * 2);

    for (const auto& v : inputVerts) {
        const int ix = static_cast<int>(std::floor((v.x - aabbMin.x) / cellSize));
        const int iy = static_cast<int>(std::floor((v.y - aabbMin.y) / cellSize));
        const int iz = static_cast<int>(std::floor((v.z - aabbMin.z) / cellSize));
        const uint64_t key = hashVoxel(ix, iy, iz);

        if (voxelGrid.find(key) == voxelGrid.end()) {
            voxelGrid[key] = v;
        }
    }

    std::vector<Vec3> result;
    result.reserve(voxelGrid.size());
    for (const auto& kv : voxelGrid) {
        result.push_back(kv.second);
    }

    return result;
}

bool VehicleCollisionGenerator::executeVHACD(
    const RawMeshGeometry& mesh,
    const CollisionPipelineSettings& settings,
    std::vector<ConvexDecompositionPiece>& outPieces,
    std::string& outDiagnostic
) {
    if (mesh.vertices.empty()) {
        outDiagnostic = "V-HACD Error: Empty vertex list";
        return false;
    }

    VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
    if (!vhacd) {
        outDiagnostic = "V-HACD Error: Failed to instantiate IVHACD";
        return false;
    }

    // Prepare point coordinates
    std::vector<double> points(mesh.vertices.size() * 3);
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        points[i * 3 + 0] = mesh.vertices[i].x;
        points[i * 3 + 1] = mesh.vertices[i].y;
        points[i * 3 + 2] = mesh.vertices[i].z;
    }

    // Prepare triangles (if indices empty, create sequential triplets)
    std::vector<int> triangles;
    if (!mesh.indices.empty()) {
        triangles.reserve(mesh.indices.size());
        for (uint32_t idx : mesh.indices) {
            triangles.push_back(static_cast<int>(idx));
        }
    } else {
        triangles.resize(mesh.vertices.size());
        for (size_t i = 0; i < triangles.size(); ++i) {
            triangles[i] = static_cast<int>(i);
        }
    }

    VHACD::IVHACD::Parameters params;
    params.m_resolution = 100000;
    params.m_depth = 12;
    params.m_concavity = settings.concavityTolerance;
    params.m_planeDownsampling = 4;
    params.m_convexhullDownsampling = 4;
    params.m_maxNumVerticesPerCH = static_cast<unsigned int>(settings.maxVerticesPerHull);
    params.m_minVolumePerCH = 0.001;
    params.m_convexhullApproximation = 1;
    params.m_oclAcceleration = 0; // Deterministic CPU evaluation

    const bool success = vhacd->Compute(
        points.data(),
        3,
        static_cast<unsigned int>(mesh.vertices.size()),
        triangles.data(),
        3,
        static_cast<unsigned int>(triangles.size() / 3),
        params
    );

    if (!success || vhacd->GetNConvexHulls() == 0) {
        vhacd->Clean();
        vhacd->Release();
        outDiagnostic = "V-HACD Compute failed or produced 0 hulls";
        return false;
    }

    const unsigned int numHulls = vhacd->GetNConvexHulls();
    outPieces.clear();
    outPieces.reserve(numHulls);

    for (unsigned int h = 0; h < numHulls; ++h) {
        VHACD::IVHACD::ConvexHull ch;
        vhacd->GetConvexHull(h, ch);

        if (ch.m_nPoints == 0) continue;

        // Compute centroid in model space
        Vec3 centroid{0.0, 0.0, 0.0};
        for (unsigned int p = 0; p < ch.m_nPoints; ++p) {
            centroid.x += ch.m_points[p * 3 + 0];
            centroid.y += ch.m_points[p * 3 + 1];
            centroid.z += ch.m_points[p * 3 + 2];
        }
        centroid = vec3Scale(centroid, 1.0 / static_cast<double>(ch.m_nPoints));

        ConvexDecompositionPiece piece;
        piece.centerModelSpace = centroid;
        piece.label = "VHACD_Piece_" + std::to_string(h);

        Vec3 minLocal{1e9, 1e9, 1e9};
        Vec3 maxLocal{-1e9, -1e9, -1e9};

        piece.vertices.reserve(ch.m_nPoints);
        for (unsigned int p = 0; p < ch.m_nPoints; ++p) {
            const Vec3 vLocal{
                ch.m_points[p * 3 + 0] - centroid.x,
                ch.m_points[p * 3 + 1] - centroid.y,
                ch.m_points[p * 3 + 2] - centroid.z
            };
            piece.vertices.push_back(vLocal);

            minLocal.x = std::min(minLocal.x, vLocal.x);
            minLocal.y = std::min(minLocal.y, vLocal.y);
            minLocal.z = std::min(minLocal.z, vLocal.z);
            maxLocal.x = std::max(maxLocal.x, vLocal.x);
            maxLocal.y = std::max(maxLocal.y, vLocal.y);
            maxLocal.z = std::max(maxLocal.z, vLocal.z);
        }

        piece.halfExtents = vec3Scale(vec3Sub(maxLocal, minLocal), 0.5);
        piece.volume = std::max(0.001, piece.halfExtents.x * piece.halfExtents.y * piece.halfExtents.z * 8.0);

        if (ch.m_triangles && ch.m_nTriangles > 0) {
            piece.indices.reserve(ch.m_nTriangles * 3);
            for (unsigned int t = 0; t < ch.m_nTriangles * 3; ++t) {
                piece.indices.push_back(static_cast<uint32_t>(ch.m_triangles[t]));
            }
        }

        outPieces.push_back(std::move(piece));
    }

    vhacd->Clean();
    vhacd->Release();

    outDiagnostic = "V-HACD generated " + std::to_string(outPieces.size()) + " convex pieces";
    return !outPieces.empty();
}

bool VehicleCollisionGenerator::executeSpatialClustering(
    const std::vector<Vec3>& vertices,
    const Vec3& aabbMin,
    const Vec3& aabbMax,
    int maxHulls,
    std::vector<ConvexDecompositionPiece>& outPieces,
    std::string& outDiagnostic
) {
    if (vertices.size() < 12) {
        outDiagnostic = "SpatialClustering Error: Insufficient vertices";
        return false;
    }

    const Vec3 dim = vec3Sub(aabbMax, aabbMin);
    const double zLen = dim.z;
    const double yLen = dim.y;
    const double beltlineY = aabbMin.y + 0.44 * yLen;

    // Define 6 natural vehicle body semantic zones:
    // 0: Front Bumper & Nose Tip (Z > 0.82 zLen)
    // 1: Hood & Front Fenders (0.55 zLen < Z <= 0.82 zLen, Y <= beltlineY + 0.12 yLen)
    // 2: Cabin Greenhouse & Roof (0.24 zLen < Z <= 0.62 zLen, Y > beltlineY)
    // 3: Central Passenger Tub Floor (0.22 zLen <= Z <= 0.56 zLen, Y <= beltlineY)
    // 4: Rear Deck & Fenders (0.12 zLen < Z <= 0.25 zLen)
    // 5: Rear Bumper & Diffuser (Z <= 0.12 zLen)

    std::array<std::vector<Vec3>, 6> clusters;
    const char* clusterLabels[6] = {
        "FrontBumper_Hull",
        "HoodFenders_Hull",
        "Greenhouse_Hull",
        "ChassisTub_Hull",
        "RearDeck_Hull",
        "RearBumper_Hull"
    };

    for (const auto& v : vertices) {
        const double relZ = (v.z - aabbMin.z);
        if (relZ > 0.82 * zLen) {
            clusters[0].push_back(v);
        } else if (relZ > 0.55 * zLen) {
            if (v.y > beltlineY + 0.10 * yLen) {
                clusters[2].push_back(v); // Windshield base
            } else {
                clusters[1].push_back(v); // Hood
            }
        } else if (relZ > 0.24 * zLen) {
            if (v.y > beltlineY) {
                clusters[2].push_back(v); // Greenhouse
            } else {
                clusters[3].push_back(v); // Tub floor
            }
        } else if (relZ > 0.12 * zLen) {
            clusters[4].push_back(v); // Rear deck
        } else {
            clusters[5].push_back(v); // Rear bumper
        }
    }

    outPieces.clear();

    for (size_t c = 0; c < 6; ++c) {
        const auto& cVerts = clusters[c];
        if (cVerts.size() < 6) continue;

        // Convert coordinates to double array for btConvexHullComputer
        std::vector<double> coords(cVerts.size() * 3);
        for (size_t i = 0; i < cVerts.size(); ++i) {
            coords[i * 3 + 0] = cVerts[i].x;
            coords[i * 3 + 1] = cVerts[i].y;
            coords[i * 3 + 2] = cVerts[i].z;
        }

        btConvexHullComputer computer;
        const btScalar shrink = static_cast<btScalar>(0.0);
        computer.compute(coords.data(), sizeof(double) * 3, static_cast<int>(cVerts.size()), shrink, 0.0f);

        if (computer.vertices.size() < 4) continue;

        // Compute centroid of the computed convex hull vertices
        Vec3 centroid{0.0, 0.0, 0.0};
        for (int i = 0; i < computer.vertices.size(); ++i) {
            centroid.x += static_cast<double>(computer.vertices[i].x());
            centroid.y += static_cast<double>(computer.vertices[i].y());
            centroid.z += static_cast<double>(computer.vertices[i].z());
        }
        centroid = vec3Scale(centroid, 1.0 / static_cast<double>(computer.vertices.size()));

        ConvexDecompositionPiece piece;
        piece.centerModelSpace = centroid;
        piece.label = clusterLabels[c];

        Vec3 minL{1e9, 1e9, 1e9};
        Vec3 maxL{-1e9, -1e9, -1e9};

        piece.vertices.reserve(static_cast<size_t>(computer.vertices.size()));
        for (int i = 0; i < computer.vertices.size(); ++i) {
            const Vec3 vL{
                static_cast<double>(computer.vertices[i].x()) - centroid.x,
                static_cast<double>(computer.vertices[i].y()) - centroid.y,
                static_cast<double>(computer.vertices[i].z()) - centroid.z
            };
            piece.vertices.push_back(vL);

            minL.x = std::min(minL.x, vL.x);
            minL.y = std::min(minL.y, vL.y);
            minL.z = std::min(minL.z, vL.z);
            maxL.x = std::max(maxL.x, vL.x);
            maxL.y = std::max(maxL.y, vL.y);
            maxL.z = std::max(maxL.z, vL.z);
        }

        piece.halfExtents = vec3Scale(vec3Sub(maxL, minL), 0.5);
        piece.halfExtents.x = std::max(0.04, piece.halfExtents.x);
        piece.halfExtents.y = std::max(0.04, piece.halfExtents.y);
        piece.halfExtents.z = std::max(0.04, piece.halfExtents.z);
        piece.volume = std::max(0.001, piece.halfExtents.x * piece.halfExtents.y * piece.halfExtents.z * 8.0);

        outPieces.push_back(std::move(piece));
        if (static_cast<int>(outPieces.size()) >= maxHulls) break;
    }

    outDiagnostic = "SpatialClustering generated " + std::to_string(outPieces.size()) + " topological hulls";
    return outPieces.size() >= 2;
}

bool VehicleCollisionGenerator::executeSingleConvexHull(
    const std::vector<Vec3>& vertices,
    std::vector<ConvexDecompositionPiece>& outPieces,
    std::string& outDiagnostic
) {
    if (vertices.size() < 6) {
        outDiagnostic = "SingleConvexHull Error: Insufficient vertices";
        return false;
    }

    std::vector<double> coords(vertices.size() * 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        coords[i * 3 + 0] = vertices[i].x;
        coords[i * 3 + 1] = vertices[i].y;
        coords[i * 3 + 2] = vertices[i].z;
    }

    btConvexHullComputer computer;
    computer.compute(coords.data(), sizeof(double) * 3, static_cast<int>(vertices.size()), 0.0f, 0.0f);

    if (computer.vertices.size() < 4) {
        outDiagnostic = "SingleConvexHull Error: btConvexHullComputer failed";
        return false;
    }

    Vec3 centroid{0.0, 0.0, 0.0};
    for (int i = 0; i < computer.vertices.size(); ++i) {
        centroid.x += static_cast<double>(computer.vertices[i].x());
        centroid.y += static_cast<double>(computer.vertices[i].y());
        centroid.z += static_cast<double>(computer.vertices[i].z());
    }
    centroid = vec3Scale(centroid, 1.0 / static_cast<double>(computer.vertices.size()));

    ConvexDecompositionPiece piece;
    piece.centerModelSpace = centroid;
    piece.label = "SingleChassis_ConvexHull";

    Vec3 minL{1e9, 1e9, 1e9};
    Vec3 maxL{-1e9, -1e9, -1e9};

    piece.vertices.reserve(static_cast<size_t>(computer.vertices.size()));
    for (int i = 0; i < computer.vertices.size(); ++i) {
        const Vec3 vL{
            static_cast<double>(computer.vertices[i].x()) - centroid.x,
            static_cast<double>(computer.vertices[i].y()) - centroid.y,
            static_cast<double>(computer.vertices[i].z()) - centroid.z
        };
        piece.vertices.push_back(vL);
        minL.x = std::min(minL.x, vL.x);
        minL.y = std::min(minL.y, vL.y);
        minL.z = std::min(minL.z, vL.z);
        maxL.x = std::max(maxL.x, vL.x);
        maxL.y = std::max(maxL.y, vL.y);
        maxL.z = std::max(maxL.z, vL.z);
    }

    piece.halfExtents = vec3Scale(vec3Sub(maxL, minL), 0.5);
    piece.halfExtents.x = std::max(0.04, piece.halfExtents.x);
    piece.halfExtents.y = std::max(0.04, piece.halfExtents.y);
    piece.halfExtents.z = std::max(0.04, piece.halfExtents.z);
    piece.volume = std::max(0.001, piece.halfExtents.x * piece.halfExtents.y * piece.halfExtents.z * 8.0);

    outPieces.clear();
    outPieces.push_back(std::move(piece));

    outDiagnostic = "SingleConvexHull generated 1 hull with " + std::to_string(computer.vertices.size()) + " vertices";
    return true;
}

void VehicleCollisionGenerator::executeBoundsFallback(
    const Vec3& aabbMin,
    const Vec3& aabbMax,
    std::vector<ConvexDecompositionPiece>& outPieces,
    std::string& outDiagnostic
) {
    const Vec3 dim = vec3Sub(aabbMax, aabbMin);
    const Vec3 center = vec3Scale(vec3Add(aabbMin, aabbMax), 0.5);
    const double hx = dim.x * 0.5;
    const double hy = dim.y * 0.5;
    const double hz = dim.z * 0.5;

    outPieces.clear();

    // 1. Lower Body Box
    {
        ConvexDecompositionPiece lowerPiece;
        lowerPiece.label = "Fallback_LowerBody";
        lowerPiece.centerModelSpace = Vec3{center.x, aabbMin.y + hy * 0.55, center.z};
        lowerPiece.halfExtents = Vec3{hx, hy * 0.55, hz};

        // 8 box vertices
        const double lx = lowerPiece.halfExtents.x;
        const double ly = lowerPiece.halfExtents.y;
        const double lz = lowerPiece.halfExtents.z;
        for (int sx : {-1, 1}) {
            for (int sy : {-1, 1}) {
                for (int sz : {-1, 1}) {
                    lowerPiece.vertices.push_back(Vec3{sx * lx, sy * ly, sz * lz});
                }
            }
        }
        lowerPiece.volume = lx * ly * lz * 8.0;
        outPieces.push_back(std::move(lowerPiece));
    }

    // 2. Greenhouse Cabin Box
    {
        ConvexDecompositionPiece cabinPiece;
        cabinPiece.label = "Fallback_Greenhouse";
        cabinPiece.centerModelSpace = Vec3{center.x, aabbMin.y + hy * 1.35, center.z - hz * 0.10};
        cabinPiece.halfExtents = Vec3{hx * 0.82, hy * 0.45, hz * 0.45};

        const double lx = cabinPiece.halfExtents.x;
        const double ly = cabinPiece.halfExtents.y;
        const double lz = cabinPiece.halfExtents.z;
        for (int sx : {-1, 1}) {
            for (int sy : {-1, 1}) {
                for (int sz : {-1, 1}) {
                    cabinPiece.vertices.push_back(Vec3{sx * lx, sy * ly, sz * lz});
                }
            }
        }
        cabinPiece.volume = lx * ly * lz * 8.0;
        outPieces.push_back(std::move(cabinPiece));
    }

    outDiagnostic = "Bounds fallback used (2 box shapes)";
}

bool VehicleCollisionGenerator::generateFromMesh(
    const RawMeshGeometry& chassisMesh,
    const std::string& vehicleId,
    const CollisionPipelineSettings& settings,
    GeneratedVehicleCollision& outCollision
) {
    const auto startTime = std::chrono::high_resolution_clock::now();

    outCollision.vehicleId = vehicleId;
    outCollision.strategyUsed = settings.strategy;
    outCollision.inputVertexCount = static_cast<int>(chassisMesh.vertices.size());
    outCollision.inputTriangleCount = static_cast<int>(chassisMesh.indices.size() / 3);
    outCollision.chassisAABBMin = chassisMesh.aabbMin;
    outCollision.chassisAABBMax = chassisMesh.aabbMax;
    outCollision.chassisDimensions = vec3Sub(chassisMesh.aabbMax, chassisMesh.aabbMin);
    outCollision.isFallback = false;

    if (chassisMesh.empty()) {
        executeBoundsFallback(Vec3{-0.9, 0.0, -2.3}, Vec3{0.9, 1.4, 2.3}, outCollision.pieces, outCollision.diagnosticMessage);
        outCollision.isFallback = true;
        outCollision.strategyUsed = CollisionAlgorithmStrategy::CompoundBoundsFallback;
        return true;
    }

    // Decimate high-poly visual vertices to uniform surface representation
    const std::vector<Vec3> decimated = decimateVertices(
        chassisMesh.vertices,
        settings.decimationTargetTriangles,
        chassisMesh.aabbMin,
        chassisMesh.aabbMax
    );

    RawMeshGeometry decimatedMesh;
    decimatedMesh.vertices = decimated;
    decimatedMesh.aabbMin = chassisMesh.aabbMin;
    decimatedMesh.aabbMax = chassisMesh.aabbMax;

    bool success = false;
    std::string diag;

    if (settings.strategy == CollisionAlgorithmStrategy::HierarchicalConvexDecomposition) {
        success = executeVHACD(decimatedMesh, settings, outCollision.pieces, diag);
        if (!success) {
            success = executeSpatialClustering(decimated, chassisMesh.aabbMin, chassisMesh.aabbMax, settings.maxConvexHulls, outCollision.pieces, diag);
            outCollision.strategyUsed = CollisionAlgorithmStrategy::SpatialClusteredHulls;
        }
    } else if (settings.strategy == CollisionAlgorithmStrategy::SpatialClusteredHulls) {
        success = executeSpatialClustering(decimated, chassisMesh.aabbMin, chassisMesh.aabbMax, settings.maxConvexHulls, outCollision.pieces, diag);
    } else if (settings.strategy == CollisionAlgorithmStrategy::SingleConvexHull) {
        success = executeSingleConvexHull(decimated, outCollision.pieces, diag);
    } else {
        // AutomaticDecomposition: Try spatial clustering first (instantaneous & watertight) or VHACD
        success = executeSpatialClustering(decimated, chassisMesh.aabbMin, chassisMesh.aabbMax, settings.maxConvexHulls, outCollision.pieces, diag);
        if (success && outCollision.pieces.size() >= 2) {
            outCollision.strategyUsed = CollisionAlgorithmStrategy::SpatialClusteredHulls;
        } else {
            success = executeVHACD(decimatedMesh, settings, outCollision.pieces, diag);
            if (success) {
                outCollision.strategyUsed = CollisionAlgorithmStrategy::HierarchicalConvexDecomposition;
            } else {
                success = executeSingleConvexHull(decimated, outCollision.pieces, diag);
                outCollision.strategyUsed = CollisionAlgorithmStrategy::SingleConvexHull;
            }
        }
    }

    if (!success || outCollision.pieces.empty()) {
        executeBoundsFallback(chassisMesh.aabbMin, chassisMesh.aabbMax, outCollision.pieces, diag);
        outCollision.strategyUsed = CollisionAlgorithmStrategy::CompoundBoundsFallback;
        outCollision.isFallback = true;
    }

    const auto endTime = std::chrono::high_resolution_clock::now();
    outCollision.generationTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    outCollision.diagnosticMessage = diag;

    outCollision.totalOutputHulls = static_cast<int>(outCollision.pieces.size());
    outCollision.totalOutputVertices = 0;
    for (const auto& p : outCollision.pieces) {
        outCollision.totalOutputVertices += static_cast<int>(p.vertices.size());
    }

    return true;
}

bool VehicleCollisionGenerator::generateFromModel(
    const LoadedVehicleModel& model,
    const CollisionPipelineSettings& settings,
    GeneratedVehicleCollision& outCollision
) {
    outCollision.vehicleId = model.modelName.empty() ? "custom_vehicle" : model.modelName;
    outCollision.sourceModelPath = model.filePath;
    outCollision.visualModelOffset = model.chassisCenterOffset;

    // Check disk cache first
    const std::string cachePath = getCacheFilePath(settings.cacheDirectory, outCollision.vehicleId);
    if (settings.enableCaching && loadFromCache(outCollision, cachePath)) {
        outCollision.loadedFromCache = true;
        std::cout << "[VehicleCollisionGenerator] Loaded cached collision geometry for '"
                  << outCollision.vehicleId << "' from " << cachePath
                  << " (" << outCollision.pieces.size() << " convex hulls)" << std::endl;
        return true;
    }

    // Automatically derive wheel dimensions and chassis metrics from model geometry
    double radiusSum = 0.0;
    double widthSum = 0.0;
    int wheelsMeasured = 0;

    for (size_t i = 0; i < 4; ++i) {
        const auto& wMesh = model.wheelMeshes[i];
        if (!wMesh.empty()) {
            const Vec3 center = vec3Add(model.modelWheelCenters[i], model.chassisCenterOffset);
            const double wheelDiameterY = wMesh.aabbMax.y - wMesh.aabbMin.y;
            const double wheelRadius = std::clamp(0.5 * wheelDiameterY, 0.20, 0.55);
            const double wheelWidth = std::clamp(wMesh.aabbMax.x - wMesh.aabbMin.x, 0.15, 0.45);

            radiusSum += wheelRadius;
            widthSum += wheelWidth;
            wheelsMeasured++;
            outCollision.derivedWheelCenters[i] = center;
        } else {
            outCollision.derivedWheelCenters[i] = vec3Add(model.modelWheelCenters[i], model.chassisCenterOffset);
        }
    }

    // Transform raw model collision mesh into Chassis Physics Space so that generated collision hulls
    // align 1:1 with the rendered visual chassis (offset relative to the vehicle's geometric center).
    RawMeshGeometry chassisPhysicsMesh;
    const Mat4 offsetMat = Mat4::fromTranslation(model.chassisCenterOffset);
    chassisPhysicsMesh.appendTransformed(model.chassisCollisionMesh, offsetMat);
    chassisPhysicsMesh.computeBounds();

    if (wheelsMeasured >= 2) {
        outCollision.derivedWheelRadius = radiusSum / static_cast<double>(wheelsMeasured);
        outCollision.derivedWheelWidth = widthSum / static_cast<double>(wheelsMeasured);
        outCollision.derivedWheelbase = std::abs(
            0.5 * (outCollision.derivedWheelCenters[0].z + outCollision.derivedWheelCenters[1].z) -
            0.5 * (outCollision.derivedWheelCenters[2].z + outCollision.derivedWheelCenters[3].z)
        );
        outCollision.derivedTrackWidth = std::abs(
            0.5 * (outCollision.derivedWheelCenters[0].x - outCollision.derivedWheelCenters[1].x +
                   outCollision.derivedWheelCenters[2].x - outCollision.derivedWheelCenters[3].x)
        );
        const double tireBottomY = outCollision.derivedWheelCenters[0].y - outCollision.derivedWheelRadius;
        outCollision.derivedGroundClearance = std::max(0.05, chassisPhysicsMesh.aabbMin.y - tireBottomY);
        outCollision.geometryDerivedSuccessfully = true;
    }

    // Generate collision hulls from chassis collision mesh in Chassis Physics Space
    const bool ok = generateFromMesh(
        chassisPhysicsMesh,
        outCollision.vehicleId,
        settings,
        outCollision
    );

    // Generate shape-accurate volumetric collision hulls for all 4 wheels and tyres
    static const char* wheelLabels[4] = {
        "WheelFL_Hull",
        "WheelFR_Hull",
        "WheelRL_Hull",
        "WheelRR_Hull"
    };

    for (size_t i = 0; i < 4; ++i) {
        const auto& wMesh = model.wheelMeshes[i];
        if (wMesh.vertices.empty()) continue;

        RawMeshGeometry wheelPhysMesh;
        wheelPhysMesh.appendTransformed(wMesh, offsetMat);
        wheelPhysMesh.computeBounds();

        if (wheelPhysMesh.vertices.size() < 6) continue;

        // In Chassis Physics Space, clamp the bottom vertices of the wheel hull by 3.5cm
        // so the wheel hull never fights the flat road contact patch while suspension is compressing,
        // but maintains 100% collision coverage on front, rear, sides, top, and outer tread!
        const Vec3& wCenter = outCollision.derivedWheelCenters[i];
        const double wRadius = (outCollision.derivedWheelRadius > 0.15) ? outCollision.derivedWheelRadius : 0.32;
        const double bottomClearanceY = wCenter.y - wRadius * 0.88;

        std::vector<Vec3> verts = wheelPhysMesh.vertices;
        for (auto& v : verts) {
            v.y = std::max(v.y, bottomClearanceY);
        }

        std::vector<double> coords(verts.size() * 3);
        for (size_t k = 0; k < verts.size(); ++k) {
            coords[k * 3 + 0] = verts[k].x;
            coords[k * 3 + 1] = verts[k].y;
            coords[k * 3 + 2] = verts[k].z;
        }

        btConvexHullComputer computer;
        computer.compute(coords.data(), sizeof(double) * 3, static_cast<int>(verts.size()), 0.0f, 0.0f);

        if (computer.vertices.size() < 4) continue;

        Vec3 centroid{0.0, 0.0, 0.0};
        for (int k = 0; k < computer.vertices.size(); ++k) {
            centroid.x += static_cast<double>(computer.vertices[k].x());
            centroid.y += static_cast<double>(computer.vertices[k].y());
            centroid.z += static_cast<double>(computer.vertices[k].z());
        }
        centroid = vec3Scale(centroid, 1.0 / static_cast<double>(computer.vertices.size()));

        ConvexDecompositionPiece wheelPiece;
        wheelPiece.centerModelSpace = centroid;
        wheelPiece.label = wheelLabels[i];

        Vec3 minL{1e9, 1e9, 1e9};
        Vec3 maxL{-1e9, -1e9, -1e9};

        wheelPiece.vertices.reserve(static_cast<size_t>(computer.vertices.size()));
        for (int k = 0; k < computer.vertices.size(); ++k) {
            const Vec3 vL{
                static_cast<double>(computer.vertices[k].x()) - centroid.x,
                static_cast<double>(computer.vertices[k].y()) - centroid.y,
                static_cast<double>(computer.vertices[k].z()) - centroid.z
            };
            wheelPiece.vertices.push_back(vL);

            minL.x = std::min(minL.x, vL.x);
            minL.y = std::min(minL.y, vL.y);
            minL.z = std::min(minL.z, vL.z);
            maxL.x = std::max(maxL.x, vL.x);
            maxL.y = std::max(maxL.y, vL.y);
            maxL.z = std::max(maxL.z, vL.z);
        }

        wheelPiece.halfExtents = vec3Scale(vec3Sub(maxL, minL), 0.5);
        wheelPiece.halfExtents.x = std::max(0.04, wheelPiece.halfExtents.x);
        wheelPiece.halfExtents.y = std::max(0.04, wheelPiece.halfExtents.y);
        wheelPiece.halfExtents.z = std::max(0.04, wheelPiece.halfExtents.z);
        wheelPiece.volume = std::max(0.001, wheelPiece.halfExtents.x * wheelPiece.halfExtents.y * wheelPiece.halfExtents.z * 8.0);

        outCollision.pieces.push_back(std::move(wheelPiece));
    }

    // Save to disk cache
    if (ok && settings.enableCaching) {
        std::filesystem::create_directories(settings.cacheDirectory);
        saveToCache(outCollision, cachePath);
    }

    return ok;
}

std::unique_ptr<btCompoundShape> VehicleCollisionGenerator::createBulletCompoundShape(
    const GeneratedVehicleCollision& collision,
    const Vec3& localCoM,
    std::vector<std::unique_ptr<btCollisionShape>>& outSubShapes
) {
    auto compound = std::make_unique<btCompoundShape>();
    outSubShapes.clear();

    for (const auto& piece : collision.pieces) {
        if (piece.vertices.size() < 4) continue;

        auto hull = std::make_unique<btConvexHullShape>();
        for (const auto& v : piece.vertices) {
            hull->addPoint(toBtVec3(v), false);
        }
        hull->recalcLocalAabb();
        hull->setMargin(static_cast<btScalar>(0.005)); // 5 mm clean skin margin for exact visual alignment

        btTransform localTrans;
        localTrans.setIdentity();
        // Position child hull in rigid body space relative to the vehicle Center of Mass
        localTrans.setOrigin(toBtVec3(vec3Sub(piece.centerModelSpace, localCoM)));

        compound->addChildShape(localTrans, hull.get());
        outSubShapes.push_back(std::move(hull));
    }

    return compound;
}

void VehicleCollisionGenerator::applyDerivedGeometryToConfig(
    const GeneratedVehicleCollision& collision,
    VehicleConfig& config
) {
    if (!collision.geometryDerivedSuccessfully) return;

    if (collision.derivedWheelbase > 1.5 && collision.derivedWheelbase < 4.0) {
        config.wheelbase = collision.derivedWheelbase;
    }
    if (collision.derivedTrackWidth > 1.2 && collision.derivedTrackWidth < 2.5) {
        config.trackWidth = collision.derivedTrackWidth;
    }
    if (collision.derivedWheelRadius > 0.20 && collision.derivedWheelRadius < 0.55) {
        config.wheelRadius = collision.derivedWheelRadius;
    }
    if (collision.derivedWheelWidth > 0.15 && collision.derivedWheelWidth < 0.45) {
        config.wheelWidth = collision.derivedWheelWidth;
    }

    // Update chassis half-extents from derived dimensions
    config.chassisHalfExtents = vec3Scale(collision.chassisDimensions, 0.5);
    config.visualOffset = collision.visualModelOffset;
}

bool VehicleCollisionGenerator::saveToCache(const GeneratedVehicleCollision& col, const std::string& cachePath) {
    std::ofstream file(cachePath);
    if (!file.is_open()) return false;

    std::string safePath = col.sourceModelPath;
    for (char& c : safePath) {
        if (c == '\\') c = '/';
    }

    file << "{\n";
    file << "  \"cache_version\": 5,\n";
    file << "  \"vehicle_id\": \"" << col.vehicleId << "\",\n";
    file << "  \"source_model_path\": \"" << safePath << "\",\n";
    file << "  \"strategy_used\": " << static_cast<int>(col.strategyUsed) << ",\n";
    file << "  \"strategy_name\": \"" << getStrategyName(col.strategyUsed) << "\",\n";
    file << "  \"geometry_derived\": " << (col.geometryDerivedSuccessfully ? "true" : "false") << ",\n";
    file << "  \"derived_wheelbase\": " << std::fixed << std::setprecision(4) << col.derivedWheelbase << ",\n";
    file << "  \"derived_track_width\": " << col.derivedTrackWidth << ",\n";
    file << "  \"derived_wheel_radius\": " << col.derivedWheelRadius << ",\n";
    file << "  \"derived_wheel_width\": " << col.derivedWheelWidth << ",\n";
    file << "  \"derived_ground_clearance\": " << col.derivedGroundClearance << ",\n";
    file << "  \"aabb_min\": [" << col.chassisAABBMin.x << ", " << col.chassisAABBMin.y << ", " << col.chassisAABBMin.z << "],\n";
    file << "  \"aabb_max\": [" << col.chassisAABBMax.x << ", " << col.chassisAABBMax.y << ", " << col.chassisAABBMax.z << "],\n";
    file << "  \"visual_offset\": [" << col.visualModelOffset.x << ", " << col.visualModelOffset.y << ", " << col.visualModelOffset.z << "],\n";
    file << "  \"pieces\": [\n";

    for (size_t p = 0; p < col.pieces.size(); ++p) {
        const auto& piece = col.pieces[p];
        file << "    {\n";
        file << "      \"label\": \"" << piece.label << "\",\n";
        file << "      \"center\": [" << piece.centerModelSpace.x << ", " << piece.centerModelSpace.y << ", " << piece.centerModelSpace.z << "],\n";
        file << "      \"half_extents\": [" << piece.halfExtents.x << ", " << piece.halfExtents.y << ", " << piece.halfExtents.z << "],\n";
        file << "      \"vertices\": [\n";
        for (size_t v = 0; v < piece.vertices.size(); ++v) {
            file << "        [" << piece.vertices[v].x << ", " << piece.vertices[v].y << ", " << piece.vertices[v].z << "]"
                 << (v + 1 < piece.vertices.size() ? "," : "") << "\n";
        }
        file << "      ]\n";
        file << "    }" << (p + 1 < col.pieces.size() ? "," : "") << "\n";
    }

    file << "  ]\n";
    file << "}\n";
    file.close();

    // Mirror cache file to alternate directory (workspace root vs build/Release) if it exists
    const std::string altDir = (cachePath.find("build") != std::string::npos)
        ? "model/cache/"
        : "build/Release/model/cache/";
    if (std::filesystem::exists(altDir)) {
        const std::string altPath = altDir + getCacheFilePath("", col.vehicleId);
        std::ofstream altFile(altPath);
        if (altFile.is_open()) {
            std::ifstream srcFile(cachePath);
            altFile << srcFile.rdbuf();
        }
    }

    return true;
}

bool VehicleCollisionGenerator::loadFromCache(GeneratedVehicleCollision& col, const std::string& cachePath) {
    std::ifstream file(cachePath);
    if (!file.is_open()) return false;

    std::string line;
    ConvexDecompositionPiece currentPiece;
    bool inPiece = false;
    bool inVertices = false;
    int cacheVersion = 0;

    while (std::getline(file, line)) {
        if (line.find("\"cache_version\"") != std::string::npos) {
            cacheVersion = static_cast<int>(extractJsonNum(line, "cache_version", 0));
        } else if (line.find("\"vehicle_id\"") != std::string::npos) {
            col.vehicleId = extractJsonStr(line, "vehicle_id");
        } else if (line.find("\"strategy_used\"") != std::string::npos) {
            col.strategyUsed = static_cast<CollisionAlgorithmStrategy>(static_cast<int>(extractJsonNum(line, "strategy_used", 0)));
        } else if (line.find("\"derived_wheelbase\"") != std::string::npos) {
            col.derivedWheelbase = extractJsonNum(line, "derived_wheelbase", 0.0);
            col.geometryDerivedSuccessfully = true;
        } else if (line.find("\"derived_track_width\"") != std::string::npos) {
            col.derivedTrackWidth = extractJsonNum(line, "derived_track_width", 0.0);
        } else if (line.find("\"derived_wheel_radius\"") != std::string::npos) {
            col.derivedWheelRadius = extractJsonNum(line, "derived_wheel_radius", 0.0);
        } else if (line.find("\"derived_wheel_width\"") != std::string::npos) {
            col.derivedWheelWidth = extractJsonNum(line, "derived_wheel_width", 0.0);
        } else if (line.find("\"derived_ground_clearance\"") != std::string::npos) {
            col.derivedGroundClearance = extractJsonNum(line, "derived_ground_clearance", 0.0);
        } else if (line.find("\"visual_offset\"") != std::string::npos) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                ss >> col.visualModelOffset.x >> comma >> col.visualModelOffset.y >> comma >> col.visualModelOffset.z;
            }
        } else if (line.find("\"aabb_min\"") != std::string::npos) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                ss >> col.chassisAABBMin.x >> comma >> col.chassisAABBMin.y >> comma >> col.chassisAABBMin.z;
            }
        } else if (line.find("\"aabb_max\"") != std::string::npos) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                ss >> col.chassisAABBMax.x >> comma >> col.chassisAABBMax.y >> comma >> col.chassisAABBMax.z;
                col.chassisDimensions = vec3Sub(col.chassisAABBMax, col.chassisAABBMin);
            }
        } else if (line.find("\"label\"") != std::string::npos) {
            currentPiece.label = extractJsonStr(line, "label");
            inPiece = true;
        } else if (line.find("\"center\"") != std::string::npos) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                ss >> currentPiece.centerModelSpace.x >> comma
                   >> currentPiece.centerModelSpace.y >> comma
                   >> currentPiece.centerModelSpace.z;
            }
        } else if (line.find("\"half_extents\"") != std::string::npos) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                ss >> currentPiece.halfExtents.x >> comma
                   >> currentPiece.halfExtents.y >> comma
                   >> currentPiece.halfExtents.z;
            }
        } else if (line.find("\"vertices\"") != std::string::npos) {
            inVertices = true;
            currentPiece.vertices.clear();
        } else if (inVertices) {
            const auto bA = line.find('[');
            const auto bB = line.find(']');
            if (bA != std::string::npos && bB != std::string::npos) {
                std::stringstream ss(line.substr(bA + 1, bB - bA - 1));
                char comma = ',';
                Vec3 pt{0, 0, 0};
                if (ss >> pt.x >> comma >> pt.y >> comma >> pt.z) {
                    currentPiece.vertices.push_back(pt);
                }
            }
            if (line.find(']') != std::string::npos && line.find('[') == std::string::npos) {
                inVertices = false;
            }
        }

        if (inPiece && !inVertices && line.find('}') != std::string::npos) {
            if (!currentPiece.vertices.empty()) {
                col.pieces.push_back(std::move(currentPiece));
                currentPiece = ConvexDecompositionPiece{};
            }
            inPiece = false;
        }
    }

    col.totalOutputHulls = static_cast<int>(col.pieces.size());
    col.totalOutputVertices = 0;
    for (const auto& p : col.pieces) {
        col.totalOutputVertices += static_cast<int>(p.vertices.size());
    }

    if (cacheVersion < 5) {
        return false; // Automatically invalidate older cache schemas
    }

    return !col.pieces.empty();
}

} // namespace torquelab
