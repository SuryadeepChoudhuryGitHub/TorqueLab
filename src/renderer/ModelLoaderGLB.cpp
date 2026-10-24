#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include "renderer/ModelLoaderGLB.h"
#include "renderer/Texture2D.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace torquelab {

namespace {

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

unsigned int parseHexColor(const std::string& hexStr, unsigned int defaultHex = 0xffffff) {
    if (hexStr.empty()) return defaultHex;
    try {
        size_t idx = 0;
        if (hexStr.rfind("0x", 0) == 0 || hexStr.rfind("0X", 0) == 0) {
            return static_cast<unsigned int>(std::stoul(hexStr.substr(2), &idx, 16));
        } else if (hexStr[0] == '#') {
            return static_cast<unsigned int>(std::stoul(hexStr.substr(1), &idx, 16));
        } else {
            return static_cast<unsigned int>(std::stoul(hexStr, &idx, 16));
        }
    } catch (...) {
        return defaultHex;
    }
}

// Simple key-value JSON reader for manifest configuration without bulky external dependencies
struct ManifestConfig {
    Vec3 chassisCenterOffset{0.0, -0.477, -0.1475};
    bool hasExplicitChassisCenterOffset{false};
    std::unordered_set<std::string> ignoreNodes;
    std::unordered_map<std::string, ModelPartRole> roleOverrides;
    std::unordered_map<std::string, MaterialProps> materialOverrides;
    std::unordered_map<std::string, std::string> textureMap;
    std::unordered_map<std::string, bool> isBrakeLightMap;
    std::unordered_map<std::string, bool> isTransparentMap;
};

std::string extractJsonString(const std::string& line, const std::string& key) {
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

float extractJsonFloat(const std::string& line, const std::string& key, float defVal) {
    const auto kPos = line.find("\"" + key + "\"");
    if (kPos == std::string::npos) return defVal;
    const auto colon = line.find(':', kPos);
    if (colon == std::string::npos) return defVal;
    try {
        size_t idx = 0;
        return std::stof(line.substr(colon + 1), &idx);
    } catch (...) {
        return defVal;
    }
}

bool extractJsonBool(const std::string& line, const std::string& key) {
    const auto kPos = line.find("\"" + key + "\"");
    if (kPos == std::string::npos) return false;
    const auto colon = line.find(':', kPos);
    if (colon == std::string::npos) return false;
    const std::string sub = line.substr(colon + 1, 12);
    return sub.find("true") != std::string::npos;
}

ModelPartRole parseRoleString(const std::string& str) {
    const std::string s = toLower(str);
    if (s == "wheel_fl" || s == "wheel_front_left" || s == "fl") return ModelPartRole::Wheel_FL;
    if (s == "wheel_fr" || s == "wheel_front_right" || s == "fr") return ModelPartRole::Wheel_FR;
    if (s == "wheel_rl" || s == "wheel_rear_left" || s == "rl") return ModelPartRole::Wheel_RL;
    if (s == "wheel_rr" || s == "wheel_rear_right" || s == "rr") return ModelPartRole::Wheel_RR;
    if (s == "caliper_fl" || s == "brakecaliper_fl") return ModelPartRole::BrakeCaliper_FL;
    if (s == "caliper_fr" || s == "brakecaliper_fr") return ModelPartRole::BrakeCaliper_FR;
    if (s == "caliper_rl" || s == "brakecaliper_rl") return ModelPartRole::BrakeCaliper_RL;
    if (s == "caliper_rr" || s == "brakecaliper_rr") return ModelPartRole::BrakeCaliper_RR;
    if (s == "steeringwheel" || s == "steering_wheel") return ModelPartRole::SteeringWheel;
    if (s == "ignore" || s == "exclude") return ModelPartRole::Ignore;
    return ModelPartRole::Chassis;
}

ManifestConfig loadManifest(const std::string& manifestPath) {
    ManifestConfig cfg{};
    std::ifstream file(manifestPath);
    if (!file.is_open()) {
        return cfg;
    }

    std::string line;
    std::string currentSection;

    while (std::getline(file, line)) {
        const auto start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        const auto end = line.find_last_not_of(" \t\r\n");
        const std::string trimmed = line.substr(start, end - start + 1);

        if (trimmed.find("\"ignore_nodes\"") != std::string::npos) {
            currentSection = "ignore";
            continue;
        }
        if (trimmed.find("\"roles\"") != std::string::npos) {
            currentSection = "roles";
            continue;
        }
        if (trimmed.find("\"material_overrides\"") != std::string::npos) {
            currentSection = "materials";
            continue;
        }
        if (trimmed.find("\"chassis_center_offset\"") != std::string::npos) {
            const auto openBracket = trimmed.find('[');
            const auto closeBracket = trimmed.find(']', openBracket);
            if (openBracket != std::string::npos && closeBracket != std::string::npos) {
                std::stringstream ss(trimmed.substr(openBracket + 1, closeBracket - openBracket - 1));
                std::string sx, sy, sz;
                if (std::getline(ss, sx, ',') && std::getline(ss, sy, ',') && std::getline(ss, sz, ',')) {
                    try {
                        cfg.chassisCenterOffset.x = std::stod(sx);
                        cfg.chassisCenterOffset.y = std::stod(sy);
                        cfg.chassisCenterOffset.z = std::stod(sz);
                        cfg.hasExplicitChassisCenterOffset = true;
                    } catch (...) {}
                }
            }
            continue;
        }

        if (currentSection == "ignore") {
            const auto q1 = trimmed.find('\"');
            if (q1 != std::string::npos) {
                const auto q2 = trimmed.find('\"', q1 + 1);
                if (q2 != std::string::npos) {
                    cfg.ignoreNodes.insert(trimmed.substr(q1 + 1, q2 - q1 - 1));
                }
            }
            if (trimmed.find(']') != std::string::npos) currentSection.clear();
        } else if (currentSection == "roles") {
            const auto q1 = trimmed.find('\"');
            if (q1 != std::string::npos) {
                const auto q2 = trimmed.find('\"', q1 + 1);
                const auto col = trimmed.find(':', q2);
                if (q2 != std::string::npos && col != std::string::npos) {
                    const auto q3 = trimmed.find('\"', col + 1);
                    const auto q4 = (q3 != std::string::npos) ? trimmed.find('\"', q3 + 1) : std::string::npos;
                    if (q4 != std::string::npos) {
                        const std::string nodeName = trimmed.substr(q1 + 1, q2 - q1 - 1);
                        const std::string roleStr = trimmed.substr(q3 + 1, q4 - q3 - 1);
                        cfg.roleOverrides[nodeName] = parseRoleString(roleStr);
                    }
                }
            }
            if (trimmed.find('}') != std::string::npos && trimmed.find('{') == std::string::npos) {
                currentSection.clear();
            }
        } else if (currentSection == "materials") {
            const auto q1 = trimmed.find('\"');
            if (q1 != std::string::npos) {
                const auto q2 = trimmed.find('\"', q1 + 1);
                const auto col = trimmed.find(':', q2);
                if (q2 != std::string::npos && col != std::string::npos) {
                    const std::string matKey = trimmed.substr(q1 + 1, q2 - q1 - 1);
                    const std::string rest = trimmed.substr(col + 1);

                    MaterialProps mat{};
                    mat.roughness = extractJsonFloat(rest, "roughness", 0.5f);
                    mat.metalness = extractJsonFloat(rest, "metalness", 0.1f);
                    mat.opacity = extractJsonFloat(rest, "opacity", 1.0f);

                    const std::string baseHex = extractJsonString(rest, "base_color_hex");
                    if (!baseHex.empty()) {
                        mat.baseColor = Color3::fromHex(parseHexColor(baseHex));
                    }
                    const std::string emissiveHex = extractJsonString(rest, "emissive_color_hex");
                    if (!emissiveHex.empty()) {
                        mat.emissiveColor = Color3::fromHex(parseHexColor(emissiveHex));
                        mat.emissiveIntensity = extractJsonFloat(rest, "emissive_intensity", 2.0f);
                    }

                    if (extractJsonBool(rest, "transparent") || mat.opacity < 0.99f) {
                        cfg.isTransparentMap[matKey] = true;
                    }
                    if (extractJsonBool(rest, "is_brake_light")) {
                        cfg.isBrakeLightMap[matKey] = true;
                    }

                    const std::string texStr = extractJsonString(rest, "texture");
                    if (!texStr.empty()) {
                        cfg.textureMap[matKey] = texStr;
                    }

                    cfg.materialOverrides[matKey] = mat;
                }
            }
            if (trimmed.find('}') != std::string::npos && trimmed.find('{') == std::string::npos) {
                currentSection.clear();
            }
        }
    }
    return cfg;
}

} // namespace

std::string ModelLoaderGLB::resolveModelPath(const std::string& relPath) {
    namespace fs = std::filesystem;
    const std::vector<fs::path> prefixes{
        fs::current_path(),
        fs::current_path() / "model",
        fs::current_path() / "..",
        fs::current_path() / ".." / "model",
        fs::current_path() / ".." / "..",
        fs::current_path() / ".." / ".." / "model"
    };

    for (const auto& p : prefixes) {
        const fs::path candidate = p / relPath;
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }
    return relPath;
}

bool ModelLoaderGLB::loadFromFile(
    const std::string& glbFilePath,
    LoadedVehicleModel& outModel,
    const std::string& manifestFilePath
) {
    outModel = LoadedVehicleModel{};
    outModel.filePath = resolveModelPath(glbFilePath);

    std::cout << "[ModelLoaderGLB] Loading vehicle model: " << outModel.filePath << std::endl;

    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, outModel.filePath.c_str(), &data);
    if (result != cgltf_result_success || !data) {
        std::cerr << "[ModelLoaderGLB] Error: Failed to parse GLB file: " << outModel.filePath << " (Code: " << result << ")" << std::endl;
        return false;
    }

    result = cgltf_load_buffers(&options, data, outModel.filePath.c_str());
    if (result != cgltf_result_success) {
        std::cerr << "[ModelLoaderGLB] Error: Failed to load GLB binary buffer for " << outModel.filePath << std::endl;
        cgltf_free(data);
        return false;
    }

    // Load optional manifest for overrides
    std::string resolvedManifest = manifestFilePath;
    if (resolvedManifest.empty()) {
        const std::string candidate1 = outModel.filePath.substr(0, outModel.filePath.find_last_of('.')) + "_manifest.json";
        const std::string candidate2 = resolveModelPath("BMW_M4_manifest.json");
        if (std::filesystem::exists(candidate1)) {
            resolvedManifest = candidate1;
        } else if (outModel.filePath.find("BMW_M4") != std::string::npos && std::filesystem::exists(candidate2)) {
            resolvedManifest = candidate2;
        }
    }
    ManifestConfig manifest = loadManifest(resolvedManifest);
    outModel.chassisCenterOffset = manifest.chassisCenterOffset;

    // Temporary list of all loaded parts before grouping
    std::vector<LoadedModelPart> rawParts;
    std::unordered_map<std::string, GLuint> textureCache;

    outModel.aabbMin = Vec3{+1e9, +1e9, +1e9};
    outModel.aabbMax = Vec3{-1e9, -1e9, -1e9};

    // Iterate through nodes in the glTF scene
    for (size_t n = 0; n < data->nodes_count; ++n) {
        const cgltf_node* node = &data->nodes[n];
        if (!node->mesh) continue;

        const std::string nodeName = node->name ? node->name : ("Node_" + std::to_string(n));
        const std::string lowerName = toLower(nodeName);

        // Check ignore filter
        if (manifest.ignoreNodes.find(nodeName) != manifest.ignoreNodes.end() ||
            lowerName.find("ground") != std::string::npos ||
            lowerName.find("plane.036") != std::string::npos || // Standalone floor plane
            lowerName == "plane") {
            continue;
        }

        // Extract Accumulated Scene World Transform (resolves parent node hierarchies, collections, armatures)
        cgltf_float worldMatrix[16];
        cgltf_node_transform_world(node, worldMatrix);

        Mat4 localBind;
        for (int i = 0; i < 16; ++i) localBind.m[i] = worldMatrix[i];
        const Vec3 localPos{localBind.m[12], localBind.m[13], localBind.m[14]};
        const Quat localRot = quatIdentity();
        const Vec3 localScale{1.0, 1.0, 1.0};

        // Process primitives inside mesh
        const cgltf_mesh* mesh = node->mesh;
        for (size_t p = 0; p < mesh->primitives_count; ++p) {
            const cgltf_primitive* prim = &mesh->primitives[p];
            if (prim->type != cgltf_primitive_type_triangles) continue;

            const cgltf_accessor* posAcc = nullptr;
            const cgltf_accessor* normAcc = nullptr;
            const cgltf_accessor* uvAcc = nullptr;

            for (size_t a = 0; a < prim->attributes_count; ++a) {
                const auto& attr = prim->attributes[a];
                if (attr.type == cgltf_attribute_type_position) posAcc = attr.data;
                else if (attr.type == cgltf_attribute_type_normal) normAcc = attr.data;
                else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uvAcc = attr.data;
            }

            if (!posAcc) continue;

            const size_t vertexCount = posAcc->count;
            std::vector<VertexPNU> vertices(vertexCount);

            for (size_t v = 0; v < vertexCount; ++v) {
                float pos[3]{0, 0, 0};
                cgltf_accessor_read_float(posAcc, v, pos, 3);
                vertices[v].px = pos[0];
                vertices[v].py = pos[1];
                vertices[v].pz = pos[2];

                if (normAcc) {
                    float norm[3]{0, 1, 0};
                    cgltf_accessor_read_float(normAcc, v, norm, 3);
                    vertices[v].nx = norm[0];
                    vertices[v].ny = norm[1];
                    vertices[v].nz = norm[2];
                } else {
                    vertices[v].nx = 0.0f;
                    vertices[v].ny = 1.0f;
                    vertices[v].nz = 0.0f;
                }

                if (uvAcc) {
                    float uv[2]{0, 0};
                    cgltf_accessor_read_float(uvAcc, v, uv, 2);
                    vertices[v].u = uv[0];
                    vertices[v].v = uv[1];
                } else {
                    vertices[v].u = 0.0f;
                    vertices[v].v = 0.0f;
                }

                // Accumulate model AABB in model coordinate space
                const Vec3 vWorld = mat4TransformPoint(localBind, Vec3{pos[0], pos[1], pos[2]});
                outModel.aabbMin.x = std::min(outModel.aabbMin.x, vWorld.x);
                outModel.aabbMin.y = std::min(outModel.aabbMin.y, vWorld.y);
                outModel.aabbMin.z = std::min(outModel.aabbMin.z, vWorld.z);
                outModel.aabbMax.x = std::max(outModel.aabbMax.x, vWorld.x);
                outModel.aabbMax.y = std::max(outModel.aabbMax.y, vWorld.y);
                outModel.aabbMax.z = std::max(outModel.aabbMax.z, vWorld.z);
            }

            std::vector<unsigned int> indices;
            if (prim->indices) {
                indices.resize(prim->indices->count);
                for (size_t i = 0; i < prim->indices->count; ++i) {
                    indices[i] = static_cast<unsigned int>(cgltf_accessor_read_index(prim->indices, i));
                }
            } else {
                indices.resize(vertexCount);
                for (size_t i = 0; i < vertexCount; ++i) indices[i] = static_cast<unsigned int>(i);
            }

            Mesh glMesh;
            glMesh.upload(vertices, indices);

            RawMeshGeometry cpuGeom;
            cpuGeom.vertices.reserve(vertices.size());
            for (const auto& vt : vertices) {
                cpuGeom.addVertex(Vec3{vt.px, vt.py, vt.pz});
            }
            cpuGeom.indices = indices;

            outModel.totalVertices += vertices.size();
            outModel.totalTriangles += indices.size() / 3;

            // Extract or customize material properties
            MaterialProps matProps{};
            std::string matName = "DefaultMaterial";
            bool isTransparent = false;
            bool isBrakeLight = false;

            if (prim->material) {
                matName = prim->material->name ? prim->material->name : "Material";
                if (lowerName.find("rim") != std::string::npos || lowerName.find("tyre") != std::string::npos) {
                    std::cout << "[MeshPrim] " << nodeName << " p=" << p << " mat=" << matName << " numVerts=" << vertexCount << std::endl;
                }
                if (prim->material->has_pbr_metallic_roughness) {
                    const auto& pbr = prim->material->pbr_metallic_roughness;
                    matProps.baseColor = Color3{pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
                    matProps.metalness = pbr.metallic_factor;
                    matProps.roughness = pbr.roughness_factor;
                    matProps.opacity = pbr.base_color_factor[3];
                }
                if (prim->material->has_emissive_strength) {
                    matProps.emissiveIntensity = prim->material->emissive_strength.emissive_strength;
                }
                if (prim->material->alpha_mode == cgltf_alpha_mode_blend) {
                    isTransparent = true;
                }
            }

            // Apply heuristic or manifest PBR material enhancements based on material names
            const std::string lowerMat = toLower(matName);
            if (manifest.materialOverrides.find(matName) != manifest.materialOverrides.end()) {
                const auto& ovr = manifest.materialOverrides[matName];
                matProps.roughness = ovr.roughness;
                matProps.metalness = ovr.metalness;
                if (ovr.baseColor.r != 1.0f || ovr.baseColor.g != 1.0f || ovr.baseColor.b != 1.0f) {
                    matProps.baseColor = ovr.baseColor;
                }
                if (ovr.opacity < 0.99f) matProps.opacity = ovr.opacity;
                if (ovr.emissiveIntensity > 0.0f) {
                    matProps.emissiveColor = ovr.emissiveColor;
                    matProps.emissiveIntensity = ovr.emissiveIntensity;
                }
            } else {
                if (lowerMat.find("paint") != std::string::npos || lowerMat.find("body") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0xdce0e6); // Alpine White
                    matProps.metalness = 0.70f;
                    matProps.roughness = 0.22f;
                } else if (lowerMat.find("glass") != std::string::npos || lowerMat.find("windscreen") != std::string::npos || lowerMat.find("window") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0x0f172a);
                    matProps.metalness = 0.15f;
                    matProps.roughness = 0.05f;
                    matProps.opacity = 0.32f;
                    matProps.castShadows = false;
                    isTransparent = true;
                } else if (lowerMat.find("rubber") != std::string::npos || lowerMat.find("tire") != std::string::npos || lowerMat.find("tyre") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0x1a1d24);
                    matProps.metalness = 0.04f;
                    matProps.roughness = 0.85f;
                } else if (lowerMat.find("rim") != std::string::npos || lowerMat.find("alloy") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0xd8dee9);
                    matProps.metalness = 0.85f;
                    matProps.roughness = 0.20f;
                } else if (lowerMat.find("rearlight") != std::string::npos || lowerMat.find("tailight") != std::string::npos || lowerMat.find("brakelight") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0x660808);
                    matProps.emissiveColor = Color3::fromHex(0xff1e1e);
                    matProps.emissiveIntensity = 0.40f;
                    matProps.roughness = 0.25f;
                    isBrakeLight = true;
                } else if (lowerMat.find("chrome") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0xf8fafc);
                    matProps.metalness = 0.98f;
                    matProps.roughness = 0.05f;
                } else if (lowerMat.find("piano_black") != std::string::npos || lowerMat.find("solid_black") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0x101318);
                    matProps.metalness = 0.20f;
                    matProps.roughness = 0.15f;
                } else if (lowerMat.find("interior") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0x202428);
                    matProps.metalness = 0.08f;
                    matProps.roughness = 0.75f;
                }
            }

            if (manifest.isBrakeLightMap.count(matName)) isBrakeLight = manifest.isBrakeLightMap[matName];
            if (manifest.isTransparentMap.count(matName)) isTransparent = manifest.isTransparentMap[matName];

            if (manifest.textureMap.count(matName)) {
                const std::string texRel = manifest.textureMap.at(matName);
                const std::string texPath = resolveModelPath(texRel);
                auto it = textureCache.find(texPath);
                if (it != textureCache.end()) {
                    matProps.textureId = it->second;
                } else {
                    auto tex = std::make_unique<Texture2D>(Texture2D::loadFromFile(texPath, true));
                    if (tex->isValid()) {
                        matProps.textureId = tex->getId();
                        textureCache[texPath] = tex->getId();
                        outModel.loadedTextures.push_back(std::move(tex));
                    }
                }
            }

            // Resolve Semantic Role
            ModelPartRole role = ModelPartRole::Chassis;
            if (manifest.roleOverrides.find(nodeName) != manifest.roleOverrides.end()) {
                role = manifest.roleOverrides[nodeName];
            } else {
                // Heuristic identification for arbitrary car models
                if (lowerName.find("steer") != std::string::npos) {
                    role = ModelPartRole::SteeringWheel;
                } else if (lowerName.find("caliper") != std::string::npos) {
                    if (localPos.z >= 0.0) {
                        role = (localPos.x >= 0.0) ? ModelPartRole::BrakeCaliper_FL : ModelPartRole::BrakeCaliper_FR;
                    } else {
                        role = (localPos.x >= 0.0) ? ModelPartRole::BrakeCaliper_RL : ModelPartRole::BrakeCaliper_RR;
                    }
                } else if (lowerName.find("wheel") != std::string::npos ||
                           lowerName.find("tire") != std::string::npos ||
                           lowerName.find("tyre") != std::string::npos ||
                           lowerName.find("rim") != std::string::npos) {
                    // Match spatial quadrant based on local position
                    if (localPos.z >= 0.0) {
                        role = (localPos.x >= 0.0) ? ModelPartRole::Wheel_FL : ModelPartRole::Wheel_FR;
                    } else {
                        role = (localPos.x >= 0.0) ? ModelPartRole::Wheel_RL : ModelPartRole::Wheel_RR;
                    }
                }
            }

            if (role == ModelPartRole::Ignore) continue;

            LoadedModelPart part{};
            part.name = nodeName;
            part.mesh = std::move(glMesh);
            part.cpuGeometry = std::move(cpuGeom);
            part.localBindMatrix = localBind;
            part.localPosition = localPos;
            part.localRotation = localRot;
            part.localScale = localScale;
            part.material = matProps;
            part.role = role;
            part.isBrakeLight = isBrakeLight;
            part.isTransparent = isTransparent;
            part.wheelLocalRotation = localRot;
            part.wheelLocalScale = localScale;

            rawParts.push_back(std::move(part));
        }
    }

    cgltf_free(data);

    if (rawParts.empty()) {
        std::cerr << "[ModelLoaderGLB] Error: No valid mesh primitives found in " << outModel.filePath << std::endl;
        return false;
    }

    // Compute wheel pivot centers per quadrant from assigned wheel parts and mesh geometry
    std::array<Vec3, 4> wheelPosSums{Vec3{0,0,0}, Vec3{0,0,0}, Vec3{0,0,0}, Vec3{0,0,0}};
    std::array<int, 4> wheelPartCounts{0, 0, 0, 0};

    for (const auto& p : rawParts) {
        int wIdx = -1;
        if (p.role == ModelPartRole::Wheel_FL) wIdx = 0;
        else if (p.role == ModelPartRole::Wheel_FR) wIdx = 1;
        else if (p.role == ModelPartRole::Wheel_RL) wIdx = 2;
        else if (p.role == ModelPartRole::Wheel_RR) wIdx = 3;

        if (wIdx >= 0) {
            wheelPosSums[static_cast<size_t>(wIdx)] = vec3Add(wheelPosSums[static_cast<size_t>(wIdx)], p.localPosition);
            wheelPartCounts[static_cast<size_t>(wIdx)]++;
        }
    }

    for (int i = 0; i < 4; ++i) {
        if (wheelPartCounts[static_cast<size_t>(i)] > 0) {
            outModel.modelWheelCenters[static_cast<size_t>(i)] = vec3Scale(
                wheelPosSums[static_cast<size_t>(i)],
                1.0 / static_cast<double>(wheelPartCounts[static_cast<size_t>(i)])
            );
            outModel.wheelsIdentified[static_cast<size_t>(i)] = true;
        } else {
            outModel.diagnosticWarnings.push_back("Warning: Wheel quadrant " + std::to_string(i) + " was not identified.");
        }
    }

    // If manifest did not explicitly define chassis_center_offset, derive it from the wheel centers:
    if (!manifest.hasExplicitChassisCenterOffset && outModel.wheelsIdentified[0] && outModel.wheelsIdentified[2]) {
        const double midZ = 0.5 * (outModel.modelWheelCenters[0].z + outModel.modelWheelCenters[2].z);
        const double midX = 0.5 * (outModel.modelWheelCenters[0].x + outModel.modelWheelCenters[1].x);
        const double wheelHubY = 0.5 * (outModel.modelWheelCenters[0].y + outModel.modelWheelCenters[1].y);
        const double deltaY = -(wheelHubY + 0.117);
        outModel.chassisCenterOffset = Vec3{-midX, deltaY, -midZ};
    }

    // Accumulate collision geometry for all parts of the car (including tyres, rims, calipers, and chassis)
    // and configure pre-computed sub-part relative transforms for wheel rendering.
    for (auto& p : rawParts) {
        int wIdx = -1;
        if (p.role == ModelPartRole::Wheel_FL) wIdx = 0;
        else if (p.role == ModelPartRole::Wheel_FR) wIdx = 1;
        else if (p.role == ModelPartRole::Wheel_RL) wIdx = 2;
        else if (p.role == ModelPartRole::Wheel_RR) wIdx = 3;

        if (wIdx >= 0) {
            const Vec3& pivot = outModel.modelWheelCenters[static_cast<size_t>(wIdx)];
            p.offsetFromWheelCenter = vec3Sub(p.localPosition, pivot);
            // Mathematically exact relative transform for rendering: T(-pivot) * localBindMatrix
            p.subPartLocalMatrix = Mat4::multiply(
                Mat4::fromTranslation(vec3Scale(pivot, -1.0)),
                p.localBindMatrix
            );

            // Extract wheel assembly vertices within physical wheel cylinder bounds (0.45m of hub)
            RawMeshGeometry wheelGeom;
            for (const auto& vLocal : p.cpuGeometry.vertices) {
                const Vec3 vModel = mat4TransformPoint(p.localBindMatrix, vLocal);
                if (vec3Length(vec3Sub(vModel, pivot)) <= 0.45) {
                    wheelGeom.addVertex(vModel);
                }
            }

            if (!wheelGeom.empty()) {
                outModel.wheelMeshes[static_cast<size_t>(wIdx)].appendTransformed(wheelGeom, Mat4::identity());
            }

            outModel.wheelParts[static_cast<size_t>(wIdx)].push_back(std::move(p));
        } else if (p.role == ModelPartRole::BrakeCaliper_FL) {
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.caliperParts[0].push_back(std::move(p));
        } else if (p.role == ModelPartRole::BrakeCaliper_FR) {
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.caliperParts[1].push_back(std::move(p));
        } else if (p.role == ModelPartRole::BrakeCaliper_RL) {
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.caliperParts[2].push_back(std::move(p));
        } else if (p.role == ModelPartRole::BrakeCaliper_RR) {
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.caliperParts[3].push_back(std::move(p));
        } else if (p.role == ModelPartRole::SteeringWheel) {
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.steeringWheelParts.push_back(std::move(p));
        } else {
            // Main chassis body, doors, hood, glass, bumpers, lights, interior, mirrors, trim
            outModel.chassisCollisionMesh.appendTransformed(p.cpuGeometry, p.localBindMatrix);
            outModel.chassisParts.push_back(std::move(p));
        }
    }

    // Compute chassis and wheel collision mesh bounds
    if (!outModel.chassisCollisionMesh.empty()) {
        outModel.chassisCollisionMesh.computeBounds();
        outModel.aabbMin = outModel.chassisCollisionMesh.aabbMin;
        outModel.aabbMax = outModel.chassisCollisionMesh.aabbMax;
    }
    for (int i = 0; i < 4; ++i) {
        if (!outModel.wheelMeshes[static_cast<size_t>(i)].empty()) {
            outModel.wheelMeshes[static_cast<size_t>(i)].computeBounds();
        }
    }

    outModel.loadedSuccessfully = true;

    std::cout << "[ModelLoaderGLB] Success! Loaded " << rawParts.size()
              << " parts (" << outModel.totalVertices << " verts, "
              << outModel.totalTriangles << " tris) from " << outModel.filePath << std::endl;
    std::cout << "  Chassis Parts: " << outModel.chassisParts.size()
              << " | Wheel Parts: FL=" << outModel.wheelParts[0].size()
              << ", FR=" << outModel.wheelParts[1].size()
              << ", RL=" << outModel.wheelParts[2].size()
              << ", RR=" << outModel.wheelParts[3].size() << std::endl;
    std::printf("  Model Axle Bounds: Front Z=+%.3f m, Rear Z=%.3f m (Wheelbase=%.3f m)\n",
                outModel.modelWheelCenters[0].z, outModel.modelWheelCenters[2].z,
                outModel.modelWheelCenters[0].z - outModel.modelWheelCenters[2].z);

    return true;
}

} // namespace torquelab
