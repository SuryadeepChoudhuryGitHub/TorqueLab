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
#include <map>
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
    std::string modelName;
    float scale{1.0f};
    float rotationYDeg{0.0f};
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
    if (s == "wheel_all" || s == "wheels" || s == "wheels_all" || s == "wheel_auto") return ModelPartRole::Wheel_All;
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

        if (trimmed.find("\"model_name\"") != std::string::npos) {
            const std::string mName = extractJsonString(trimmed, "model_name");
            if (!mName.empty()) cfg.modelName = mName;
            continue;
        }
        if (trimmed.find("\"scale\"") != std::string::npos) {
            cfg.scale = extractJsonFloat(trimmed, "scale", cfg.scale);
            continue;
        }
        if (trimmed.find("\"rotation_y_deg\"") != std::string::npos) {
            cfg.rotationYDeg = extractJsonFloat(trimmed, "rotation_y_deg", cfg.rotationYDeg);
            continue;
        }
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
    const std::string& manifestFilePath,
    bool loadGpuResources
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
    if (!manifest.modelName.empty()) {
        outModel.modelName = manifest.modelName;
    }
    outModel.chassisCenterOffset = manifest.chassisCenterOffset;

    Mat4 rootTransform = Mat4::identity();
    if (manifest.scale != 1.0f || manifest.rotationYDeg != 0.0f) {
        const double rad = degToRad(manifest.rotationYDeg);
        const Quat rotY = quatFromAxisAngle(Vec3{0, 1, 0}, rad);
        rootTransform = Mat4::fromTRS(Vec3{0, 0, 0}, rotY, Vec3{manifest.scale, manifest.scale, manifest.scale});
    }

    // Temporary list of all loaded parts before grouping
    std::vector<LoadedModelPart> rawParts;
    std::unordered_map<std::string, GLuint> textureCache;
    std::map<std::pair<const void*, bool>, GLuint> embeddedTextureCache;

    auto resolveAndLoadGltfTexture = [&](const cgltf_texture_view& view, bool isSRGB) -> GLuint {
        if (!view.texture) return 0;
        const cgltf_texture* tex = view.texture;
        if (!tex->image) return 0;
        const cgltf_image* img = tex->image;

        // Check embedded bufferView first (GLB binary chunks)
        if (img->buffer_view && img->buffer_view->buffer && img->buffer_view->buffer->data) {
            const auto cacheKey = std::make_pair(static_cast<const void*>(img), isSRGB);
            auto it = embeddedTextureCache.find(cacheKey);
            if (it != embeddedTextureCache.end()) {
                return it->second;
            }
            const auto* bv = img->buffer_view;
            const uint8_t* bufData = static_cast<const uint8_t*>(bv->buffer->data) + bv->offset;
            const int bufSize = static_cast<int>(bv->size);
            auto loadedTex = std::make_unique<Texture2D>(Texture2D::loadFromMemory(bufData, bufSize, isSRGB));
            if (loadedTex->isValid()) {
                const GLuint texId = loadedTex->getId();
                embeddedTextureCache[cacheKey] = texId;
                outModel.loadedTextures.push_back(std::move(loadedTex));
                return texId;
            }
        } else if (img->uri && strlen(img->uri) > 0) {
            // External image file (PNG / JPEG)
            const std::string uriStr(img->uri);
            if (uriStr.rfind("data:", 0) != 0) {
                std::filesystem::path modelDir = std::filesystem::path(outModel.filePath).parent_path();
                std::filesystem::path resolvedPath = modelDir / uriStr;
                std::string finalPath = resolvedPath.string();
                if (!std::filesystem::exists(finalPath)) {
                    finalPath = ModelLoaderGLB::resolveModelPath(uriStr);
                }
                const std::string cacheKey = finalPath + (isSRGB ? "_srgb" : "_linear");
                auto it = textureCache.find(cacheKey);
                if (it != textureCache.end()) {
                    return it->second;
                }
                auto loadedTex = std::make_unique<Texture2D>(Texture2D::loadFromFile(finalPath, isSRGB));
                if (loadedTex->isValid()) {
                    const GLuint texId = loadedTex->getId();
                    textureCache[cacheKey] = texId;
                    outModel.loadedTextures.push_back(std::move(loadedTex));
                    return texId;
                }
            }
        }
        return 0;
    };

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

        Mat4 rawNodeBind;
        for (int i = 0; i < 16; ++i) rawNodeBind.m[i] = worldMatrix[i];
        const Mat4 localBind = Mat4::multiply(rootTransform, rawNodeBind);
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
            const cgltf_accessor* tanAcc = nullptr;

            for (size_t a = 0; a < prim->attributes_count; ++a) {
                const auto& attr = prim->attributes[a];
                if (attr.type == cgltf_attribute_type_position) posAcc = attr.data;
                else if (attr.type == cgltf_attribute_type_normal) normAcc = attr.data;
                else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uvAcc = attr.data;
                else if (attr.type == cgltf_attribute_type_tangent) tanAcc = attr.data;
            }

            if (!posAcc) continue;

            Vec3 primMin{+1e9, +1e9, +1e9};
            Vec3 primMax{-1e9, -1e9, -1e9};

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

                if (tanAcc) {
                    float tanVal[4]{1, 0, 0, 1};
                    cgltf_accessor_read_float(tanAcc, v, tanVal, 4);
                    vertices[v].tx = tanVal[0];
                    vertices[v].ty = tanVal[1];
                    vertices[v].tz = tanVal[2];
                    vertices[v].tw = tanVal[3];
                } else {
                    vertices[v].tx = 1.0f;
                    vertices[v].ty = 0.0f;
                    vertices[v].tz = 0.0f;
                    vertices[v].tw = 1.0f;
                }

                // Accumulate model AABB in model coordinate space
                const Vec3 vWorld = mat4TransformPoint(localBind, Vec3{pos[0], pos[1], pos[2]});
                primMin.x = std::min(primMin.x, vWorld.x);
                primMin.y = std::min(primMin.y, vWorld.y);
                primMin.z = std::min(primMin.z, vWorld.z);
                primMax.x = std::max(primMax.x, vWorld.x);
                primMax.y = std::max(primMax.y, vWorld.y);
                primMax.z = std::max(primMax.z, vWorld.z);

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

            // If glTF source had no tangent vectors, compute smooth orthogonal tangents and bitangent signs
            if (!tanAcc) {
                Mesh::computeTangents(vertices, indices);
            }

            Mesh glMesh;
            if (loadGpuResources) {
                glMesh.upload(vertices, indices);
            }

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
            bool isReverseLight = false;

            if (prim->material) {
                const cgltf_material* mat = prim->material;
                matName = mat->name ? mat->name : "Material";
                matProps.name = matName;

                if (mat->has_pbr_metallic_roughness) {
                    const auto& pbr = mat->pbr_metallic_roughness;
                    matProps.baseColor = Color3{pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
                    matProps.metalness = pbr.metallic_factor;
                    matProps.roughness = pbr.roughness_factor;
                    matProps.opacity = pbr.base_color_factor[3];

                    if (loadGpuResources && pbr.base_color_texture.texture) {
                        matProps.textureId = resolveAndLoadGltfTexture(pbr.base_color_texture, true);
                    }
                    if (loadGpuResources && pbr.metallic_roughness_texture.texture) {
                        matProps.metallicRoughnessMapId = resolveAndLoadGltfTexture(pbr.metallic_roughness_texture, false);
                    }
                }

                // Normal texture map (Linear)
                if (loadGpuResources && mat->normal_texture.texture) {
                    matProps.normalMapId = resolveAndLoadGltfTexture(mat->normal_texture, false);
                    matProps.normalScale = (mat->normal_texture.scale != 0.0f) ? mat->normal_texture.scale : 1.0f;
                }

                // Ambient Occlusion texture map (Linear)
                if (loadGpuResources && mat->occlusion_texture.texture) {
                    matProps.occlusionMapId = resolveAndLoadGltfTexture(mat->occlusion_texture, false);
                    matProps.occlusionStrength = (mat->occlusion_texture.scale != 0.0f) ? mat->occlusion_texture.scale : 1.0f;
                }

                // Emissive color factor & texture map (sRGB)
                matProps.emissiveColor = Color3{mat->emissive_factor[0], mat->emissive_factor[1], mat->emissive_factor[2]};
                if (matProps.emissiveColor.r > 0.001f || matProps.emissiveColor.g > 0.001f || matProps.emissiveColor.b > 0.001f) {
                    matProps.emissiveIntensity = 1.0f;
                }
                if (mat->has_emissive_strength) {
                    matProps.emissiveIntensity = mat->emissive_strength.emissive_strength;
                }
                if (loadGpuResources && mat->emissive_texture.texture) {
                    matProps.emissiveMapId = resolveAndLoadGltfTexture(mat->emissive_texture, true);
                    if (matProps.emissiveIntensity <= 0.0f) matProps.emissiveIntensity = 1.0f;
                }

                // Alpha blending & masking
                if (mat->alpha_mode == cgltf_alpha_mode_blend) {
                    matProps.alphaMode = AlphaMode::Blend;
                    isTransparent = true;
                } else if (mat->alpha_mode == cgltf_alpha_mode_mask) {
                    matProps.alphaMode = AlphaMode::Mask;
                    matProps.alphaCutoff = mat->alpha_cutoff;
                } else {
                    matProps.alphaMode = AlphaMode::Opaque;
                }

                matProps.doubleSided = (mat->double_sided != 0);
                matProps.unlit = (mat->unlit != 0);
            }

            const std::string lowerMat = toLower(matName);

            // Manifest overrides take priority when explicitly defined
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
            } else if (!prim->material || !prim->material->has_pbr_metallic_roughness) {
                // Heuristic fallback ONLY if the model had no authentic glTF PBR material values
                if (lowerMat.find("paint") != std::string::npos || lowerMat.find("body") != std::string::npos) {
                    matProps.baseColor = Color3::fromHex(0xdce0e6);
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

            // Semantic vehicle reactive lights detection
            if (lowerMat.find("rearlight") != std::string::npos ||
                lowerMat.find("tailight") != std::string::npos ||
                lowerMat.find("brakelight") != std::string::npos ||
                lowerMat.find("rear_lights") != std::string::npos) {
                isBrakeLight = true;
            }
            if (lowerMat.find("reverse_light") != std::string::npos) {
                isReverseLight = true;
            }

            if (manifest.isBrakeLightMap.count(matName)) isBrakeLight = manifest.isBrakeLightMap[matName];
            if (manifest.isTransparentMap.count(matName)) isTransparent = manifest.isTransparentMap[matName];

            // Explicit manifest texture overrides (e.g. decal composites)
            if (loadGpuResources && manifest.textureMap.count(matName)) {
                const std::string texRel = manifest.textureMap.at(matName);
                const std::string texPath = resolveModelPath(texRel);
                const std::string cacheKey = texPath + "_srgb";
                auto it = textureCache.find(cacheKey);
                if (it != textureCache.end()) {
                    matProps.textureId = it->second;
                } else {
                    auto tex = std::make_unique<Texture2D>(Texture2D::loadFromFile(texPath, true));
                    if (tex->isValid()) {
                        matProps.textureId = tex->getId();
                        textureCache[cacheKey] = tex->getId();
                        outModel.loadedTextures.push_back(std::move(tex));
                    }
                }
            }

            // Store unique material for runtime inspection & UI diagnostics
            bool matAlreadyRecorded = false;
            for (const auto& existingMat : outModel.uniqueMaterials) {
                if (existingMat.name == matProps.name) {
                    matAlreadyRecorded = true;
                    break;
                }
            }
            if (!matAlreadyRecorded) {
                outModel.uniqueMaterials.push_back(matProps);
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
                           lowerName.find("rim") != std::string::npos ||
                           lowerName.find("hubcap") != std::string::npos) {
                    // Check if geometry spans multiple quadrants (merged wheel mesh across left/right or front/rear)
                    if ((primMin.x < -0.35 && primMax.x > 0.35) || (primMin.z < -0.35 && primMax.z > 0.35)) {
                        role = ModelPartRole::Wheel_All;
                    } else {
                        if (localPos.z >= 0.0) {
                            role = (localPos.x >= 0.0) ? ModelPartRole::Wheel_FL : ModelPartRole::Wheel_FR;
                        } else {
                            role = (localPos.x >= 0.0) ? ModelPartRole::Wheel_RL : ModelPartRole::Wheel_RR;
                        }
                    }
                }
            }

            if (role == ModelPartRole::Ignore) continue;

            if (role == ModelPartRole::Wheel_All) {
                // Partition triangles into 4 independent wheel assemblies:
                // 0=FL (Z>=0, X>=0), 1=FR (Z>=0, X<0), 2=RL (Z<0, X>=0), 3=RR (Z<0, X<0)
                std::array<std::vector<unsigned int>, 4> quadIndices;
                const size_t numTris = indices.size() / 3;
                for (size_t t = 0; t < numTris; ++t) {
                    const unsigned int i0 = indices[t * 3 + 0];
                    const unsigned int i1 = indices[t * 3 + 1];
                    const unsigned int i2 = indices[t * 3 + 2];

                    const Vec3 p0 = mat4TransformPoint(localBind, Vec3{vertices[i0].px, vertices[i0].py, vertices[i0].pz});
                    const Vec3 p1 = mat4TransformPoint(localBind, Vec3{vertices[i1].px, vertices[i1].py, vertices[i1].pz});
                    const Vec3 p2 = mat4TransformPoint(localBind, Vec3{vertices[i2].px, vertices[i2].py, vertices[i2].pz});

                    const Vec3 c = vec3Scale(vec3Add(vec3Add(p0, p1), p2), 1.0 / 3.0);
                    int qIdx = 0;
                    if (c.z >= 0.0) {
                        qIdx = (c.x >= 0.0) ? 0 : 1;
                    } else {
                        qIdx = (c.x >= 0.0) ? 2 : 3;
                    }
                    quadIndices[static_cast<size_t>(qIdx)].push_back(i0);
                    quadIndices[static_cast<size_t>(qIdx)].push_back(i1);
                    quadIndices[static_cast<size_t>(qIdx)].push_back(i2);
                }

                static const ModelPartRole quadRoles[4] = {
                    ModelPartRole::Wheel_FL,
                    ModelPartRole::Wheel_FR,
                    ModelPartRole::Wheel_RL,
                    ModelPartRole::Wheel_RR
                };
                static const char* quadSuffix[4] = {"_FL", "_FR", "_RL", "_RR"};

                for (int q = 0; q < 4; ++q) {
                    const auto& qIdxs = quadIndices[static_cast<size_t>(q)];
                    if (qIdxs.empty()) continue;

                    std::vector<VertexPNU> qVerts;
                    std::vector<unsigned int> qRemappedIndices;
                    std::unordered_map<unsigned int, unsigned int> remap;
                    qVerts.reserve(qIdxs.size());
                    qRemappedIndices.reserve(qIdxs.size());

                    Vec3 qPosSum{0.0, 0.0, 0.0};
                    for (unsigned int oldIdx : qIdxs) {
                        auto it = remap.find(oldIdx);
                        if (it == remap.end()) {
                            const unsigned int newIdx = static_cast<unsigned int>(qVerts.size());
                            remap[oldIdx] = newIdx;
                            qVerts.push_back(vertices[oldIdx]);
                            const Vec3 vW = mat4TransformPoint(localBind, Vec3{vertices[oldIdx].px, vertices[oldIdx].py, vertices[oldIdx].pz});
                            qPosSum = vec3Add(qPosSum, vW);
                            qRemappedIndices.push_back(newIdx);
                        } else {
                            qRemappedIndices.push_back(it->second);
                        }
                    }

                    const Vec3 qCenter = qVerts.empty() ? localPos : vec3Scale(qPosSum, 1.0 / static_cast<double>(qVerts.size()));

                    Mesh qMesh;
                    if (loadGpuResources) {
                        qMesh.upload(qVerts, qRemappedIndices);
                    }

                    RawMeshGeometry qCpu;
                    qCpu.vertices.reserve(qVerts.size());
                    for (const auto& vt : qVerts) {
                        qCpu.addVertex(Vec3{vt.px, vt.py, vt.pz});
                    }
                    qCpu.indices = qRemappedIndices;

                    LoadedModelPart qPart{};
                    qPart.name = nodeName + quadSuffix[q];
                    qPart.mesh = std::move(qMesh);
                    qPart.cpuGeometry = std::move(qCpu);
                    qPart.localBindMatrix = localBind;
                    qPart.localPosition = qCenter;
                    qPart.localRotation = quatIdentity();
                    qPart.localScale = Vec3{1.0, 1.0, 1.0};
                    qPart.material = matProps;
                    qPart.role = quadRoles[q];
                    qPart.isBrakeLight = isBrakeLight;
                    qPart.isReverseLight = isReverseLight;
                    qPart.isTransparent = isTransparent;
                    qPart.wheelLocalRotation = quatIdentity();
                    qPart.wheelLocalScale = Vec3{1.0, 1.0, 1.0};

                    rawParts.push_back(std::move(qPart));
                }
                continue;
            }

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
            part.isReverseLight = isReverseLight;
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
            const auto& wBounds = outModel.wheelMeshes[static_cast<size_t>(i)];
            const Vec3 truePivot = vec3Scale(vec3Add(wBounds.aabbMin, wBounds.aabbMax), 0.5);
            const double trueRadius = 0.5 * (wBounds.aabbMax.y - wBounds.aabbMin.y);
            const double trueWidth = std::max(0.12, wBounds.aabbMax.x - wBounds.aabbMin.x);

            outModel.modelWheelCenters[static_cast<size_t>(i)] = truePivot;
            outModel.modelWheelRadii[static_cast<size_t>(i)] = trueRadius;
            outModel.modelWheelWidths[static_cast<size_t>(i)] = trueWidth;
            outModel.wheelsIdentified[static_cast<size_t>(i)] = true;

            // Re-align sub-part relative transforms to the exact geometric center of the wheel assembly
            for (auto& wp : outModel.wheelParts[static_cast<size_t>(i)]) {
                wp.offsetFromWheelCenter = vec3Sub(wp.localPosition, truePivot);
                wp.subPartLocalMatrix = Mat4::multiply(
                    Mat4::fromTranslation(vec3Scale(truePivot, -1.0)),
                    wp.localBindMatrix
                );
            }
        }
    }

    // Update chassis center offset if not explicitly provided in manifest
    if (!manifest.hasExplicitChassisCenterOffset && outModel.wheelsIdentified[0] && outModel.wheelsIdentified[2]) {
        const double midZ = 0.5 * (outModel.modelWheelCenters[0].z + outModel.modelWheelCenters[2].z);
        const double midX = 0.5 * (outModel.modelWheelCenters[0].x + outModel.modelWheelCenters[1].x);
        const double wheelHubY = 0.5 * (outModel.modelWheelCenters[0].y + outModel.modelWheelCenters[1].y);
        const double deltaY = -(wheelHubY + 0.117);
        outModel.chassisCenterOffset = Vec3{-midX, deltaY, -midZ};
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
