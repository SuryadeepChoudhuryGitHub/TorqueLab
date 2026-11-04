#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vPosLightSpace;
in vec3 vWorldTangent;
in float vTangentW;

out vec4 FragColor;

// Environment, Camera & Atmospheric Lighting
uniform vec3 uCameraPos;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uSkyColor;
uniform vec3 uGroundColor;
uniform vec3 uFogColor;
uniform float uFogDensity;
uniform float uExposure;
uniform float uIblIntensity;
uniform int uDiagnosticMode; // 0=Final PBR, 1=Albedo, 2=Metallic, 3=Roughness, 4=Normals, 5=AO, 6=Emissive, 7=Tangents, 8=Environment Reflections
uniform int uQualityPreset;  // 0=Low, 1=Medium, 2=High, 3=Maximum
uniform float uShadowTexelSize; // 1.0 / 2048.0 or 1.0 / 1024.0

// Material Properties (SI metric / glTF 2.0 PBR standard)
uniform vec3 uBaseColor;
uniform vec3 uEmissiveColor;
uniform float uEmissiveIntensity;
uniform float uRoughness;
uniform float uMetalness;
uniform float uOpacity;
uniform float uNormalScale;
uniform float uOcclusionStrength;
uniform int uAlphaMode; // 0=Opaque, 1=Mask, 2=Blend
uniform float uAlphaCutoff;
uniform int uDoubleSided;
uniform int uReceiveShadows;
uniform int uShadowsEnabled;
uniform int uUnlit;
uniform vec2 uUvScale;

// Texture Samplers
uniform sampler2D uAlbedoMap;             // Unit 0 (sRGB)
uniform int uHasAlbedoMap;
uniform int uUseTexture;                  // Backward compatibility
uniform sampler2D uShadowMap;             // Unit 1 (Depth)
uniform sampler2D uEnvironmentMap;        // Unit 2 (HDR Panoramic Equirectangular Map)
uniform sampler2D uNormalMap;             // Unit 3 (Linear)
uniform int uHasNormalMap;
uniform sampler2D uMetallicRoughnessMap;   // Unit 4 (G=Roughness, B=Metallic) (Linear)
uniform int uHasMetallicRoughnessMap;
uniform sampler2D uOcclusionMap;          // Unit 5 (R=AO) (Linear)
uniform int uHasOcclusionMap;
uniform sampler2D uEmissiveMap;            // Unit 6 (sRGB)
uniform int uHasEmissiveMap;

const float PI = 3.141592653589793;

float computeShadowPCF(vec4 posLightSpace, vec3 normal, vec3 lightDir) {
    vec3 projCoords = posLightSpace.xyz / posLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 || projCoords.y < 0.0 || projCoords.y > 1.0) {
        return 0.0;
    }

    float currentDepth = projCoords.z;
    float bias = max(0.0018 * (1.0 - dot(normal, lightDir)), 0.00045);
    float texelSize = (uShadowTexelSize > 0.0) ? uShadowTexelSize : (1.0 / 2048.0);

    if (uQualityPreset >= 3) {
        // Maximum: High-quality 8-tap Poisson disk filter for soft, antialiased contact shadows
        const vec2 poissonDisk[8] = vec2[8](
            vec2(-0.7071,  0.7071),
            vec2( 0.7071, -0.7071),
            vec2(-0.7071, -0.7071),
            vec2( 0.7071,  0.7071),
            vec2(-0.3826,  0.9238),
            vec2( 0.9238, -0.3826),
            vec2(-0.9238, -0.3826),
            vec2( 0.3826,  0.9238)
        );
        float shadow = 0.0;
        for (int i = 0; i < 8; ++i) {
            float pcfDepth = texture(uShadowMap, projCoords.xy + poissonDisk[i] * texelSize).r;
            shadow += (currentDepth - bias > pcfDepth) ? 1.0 : 0.0;
        }
        return shadow * 0.125;
    } else if (uQualityPreset == 2) {
        // High: 4-tap Poisson rotated grid (exact previous baseline)
        const vec2 offsets[4] = vec2[4](
            vec2(-0.75, -0.25),
            vec2( 0.25, -0.75),
            vec2(-0.25,  0.75),
            vec2( 0.75,  0.25)
        );
        float shadow = 0.0;
        for (int i = 0; i < 4; ++i) {
            float pcfDepth = texture(uShadowMap, projCoords.xy + offsets[i] * texelSize).r;
            shadow += (currentDepth - bias > pcfDepth) ? 1.0 : 0.0;
        }
        return shadow * 0.25;
    } else {
        // Medium: Fast 2-tap PCF
        float pcf1 = texture(uShadowMap, projCoords.xy + vec2(-0.5, -0.5) * texelSize).r;
        float pcf2 = texture(uShadowMap, projCoords.xy + vec2( 0.5,  0.5) * texelSize).r;
        float s1 = (currentDepth - bias > pcf1) ? 1.0 : 0.0;
        float s2 = (currentDepth - bias > pcf2) ? 1.0 : 0.0;
        return (s1 + s2) * 0.5;
    }
}

// GGX / Trowbridge-Reitz normal distribution function
float distributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
    return a2 / max(denom, 0.0000001);
}

// Schlick-GGX single-direction geometry obstruction / shadowing
float geometrySchlickGGX(float NdotV, float roughness) {
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

// Smith combined geometric attenuation
float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = geometrySchlickGGX(NdotV, roughness);
    float ggx1 = geometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

// Fresnel-Schlick reflection factor
vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Roughness-dependent Fresnel-Schlick for IBL
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Karis / Lazarov polynomial environment BRDF approximation
vec2 envBRDFApprox(float roughness, float NoV) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
    return vec2(-1.04, 0.04) * a004 + r.zw;
}

// Convert 3D reflection or normal direction to 2D equirectangular UV
vec2 sampleEquirectangular(vec3 v) {
    vec2 uv = vec2(atan(v.x, v.z), asin(clamp(v.y, -1.0, 1.0)));
    uv *= vec2(0.15915494309, 0.31830988618); // 1 / (2*PI), 1 / PI
    uv += 0.5;
    return uv;
}

// Photographic ACES filmic tone reproduction curve
vec3 acesToneMap(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main() {
    vec2 uv = vTexCoord * uUvScale;

    // 1. Resolve Albedo & Base Opacity
    vec3 albedo = uBaseColor;
    float alpha = uOpacity;
    if (uHasAlbedoMap == 1 || uUseTexture == 1) {
        vec4 texCol = texture(uAlbedoMap, uv);
        // If texture has non-opaque alpha (e.g. decal or badge), composite decal over baseColor
        // If texture is fully opaque (texCol.a == 1.0), albedo is tinted by baseColor
        if (texCol.a < 0.999 && uAlphaMode == 0) {
            albedo = mix(uBaseColor, texCol.rgb, texCol.a);
        } else {
            albedo *= texCol.rgb;
            if (uAlphaMode != 0) {
                alpha *= texCol.a;
            }
        }
    }

    // Alpha Cutoff / Masking
    if (uAlphaMode == 1 && alpha < uAlphaCutoff) {
        discard;
    }

    // 2. Resolve Material Roughness & Metalness (glTF standard: Green=Roughness, Blue=Metallic)
    float roughness = clamp(uRoughness, 0.04, 1.0);
    float metalness = clamp(uMetalness, 0.0, 1.0);
    if (uHasMetallicRoughnessMap == 1) {
        vec4 mrSample = texture(uMetallicRoughnessMap, uv);
        roughness = clamp(roughness * mrSample.g, 0.04, 1.0);
        metalness = clamp(metalness * mrSample.b, 0.0, 1.0);
    }

    // 3. Resolve Ambient Occlusion (glTF standard: Red channel)
    float ao = 1.0;
    if (uHasOcclusionMap == 1) {
        float rawAo = texture(uOcclusionMap, uv).r;
        ao = 1.0 + uOcclusionStrength * (rawAo - 1.0);
        ao = clamp(ao, 0.0, 1.0);
    }

    // 4. Resolve Emissive Color & Texture Map
    vec3 emissive = uEmissiveColor * uEmissiveIntensity;
    if (uHasEmissiveMap == 1) {
        vec3 emMap = texture(uEmissiveMap, uv).rgb;
        emissive *= emMap;
    }

    // 5. Construct Orthonormal Tangent Space (TBN) & Resolve Shading Normal
    vec3 N = normalize(vWorldNormal);
    if (uDoubleSided == 1 && !gl_FrontFacing) {
        N = -N;
    }
    vec3 T = normalize(vWorldTangent - dot(vWorldTangent, N) * N);
    vec3 B = cross(N, T) * (vTangentW < 0.0 ? -1.0 : 1.0);
    mat3 TBN = mat3(T, B, N);

    if (uHasNormalMap == 1) {
        vec3 tangentNormal = texture(uNormalMap, uv).xyz * 2.0 - 1.0;
        tangentNormal.xy *= uNormalScale;
        N = normalize(TBN * tangentNormal);
    }

    // 6. Camera View & Reflection Vectors
    vec3 V = normalize(uCameraPos - vWorldPos);
    vec3 R = reflect(-V, N);

    // Diagnostic Inspection Modes (Direct Debug Overrides)
    if (uDiagnosticMode == 1) { FragColor = vec4(albedo, 1.0); return; }
    if (uDiagnosticMode == 2) { FragColor = vec4(vec3(metalness), 1.0); return; }
    if (uDiagnosticMode == 3) { FragColor = vec4(vec3(roughness), 1.0); return; }
    if (uDiagnosticMode == 4) { FragColor = vec4(N * 0.5 + 0.5, 1.0); return; }
    if (uDiagnosticMode == 5) { FragColor = vec4(vec3(ao), 1.0); return; }
    if (uDiagnosticMode == 6) { FragColor = vec4(emissive, 1.0); return; }
    if (uDiagnosticMode == 7) { FragColor = vec4(T * 0.5 + 0.5, 1.0); return; }
    if (uDiagnosticMode == 8) {
        vec3 refColor = textureLod(uEnvironmentMap, sampleEquirectangular(R), roughness * 6.0).rgb;
        FragColor = vec4(refColor, 1.0);
        return;
    }

    vec3 finalLinearColor;

    if (uUnlit == 1) {
        finalLinearColor = albedo + emissive;
    } else {
        // Base reflectivity for non-metals (dielectrics = 0.04) vs metals (tinted by albedo)
        vec3 F0 = mix(vec3(0.04), albedo, metalness);

        // 7. Direct Directional Light (Sun)
        vec3 L = normalize(uSunDir);
        vec3 H = normalize(L + V);
        float NdotL = max(dot(N, L), 0.0);
        float NdotV = max(dot(N, V), 0.001);

        float NDF = distributionGGX(N, H, roughness);
        float G   = geometrySmith(N, V, L, roughness);
        vec3  F   = fresnelSchlick(max(dot(H, V), 0.0), F0);

        vec3 numerator    = NDF * G * F;
        float denominator = max(4.0 * NdotV * NdotL, 0.001);
        vec3 specularBRDF = numerator / denominator;

        vec3 kS = F;
        vec3 diffuseColor = albedo * (1.0 - metalness * 0.75);
        vec3 directDiffuse = (vec3(1.0) - kS) * diffuseColor * uSunColor * NdotL;
        vec3 directSpecular = specularBRDF * uSunColor * NdotL;

        float shadow = 0.0;
        if (uShadowsEnabled != 0 && uReceiveShadows != 0) {
            shadow = computeShadowPCF(vPosLightSpace, N, L);
        }

        vec3 Lo = (directDiffuse + directSpecular) * (1.0 - shadow * 0.72);

        // 8. Image-Based Lighting (IBL: Specular Environment Reflections + Diffuse Irradiance)
        float hemiWeight = N.y * 0.5 + 0.5;
        vec3 hemiAmbient = mix(uGroundColor, uSkyColor, hemiWeight) * 0.52;
        vec3 ambientIBL;

        if (uQualityPreset == 0) {
            // Preset 0 (Low): Fast hemispherical ambient fallback (skips expensive textureLod lookups)
            ambientIBL = hemiAmbient * diffuseColor * ao;
        } else {
            const float maxLod = 6.0;
            vec3 prefilteredColor = textureLod(uEnvironmentMap, sampleEquirectangular(R), roughness * maxLod).rgb;
            vec2 envBRDF = envBRDFApprox(roughness, NdotV);
            vec3 F_IBL = fresnelSchlickRoughness(NdotV, F0, roughness);
            vec3 specularIBL = prefilteredColor * (F_IBL * envBRDF.x + envBRDF.y);

            vec3 diffuseIrradiance = textureLod(uEnvironmentMap, sampleEquirectangular(N), maxLod).rgb;
            vec3 ambientDiffuse = (diffuseIrradiance * 0.75 + hemiAmbient * 0.45) * diffuseColor;

            float iblMult = (uIblIntensity > 0.0) ? uIblIntensity : 1.0;
            if (uQualityPreset == 1) {
                iblMult *= 0.85; // Medium: subtle performance balance
            }
            ambientIBL = (ambientDiffuse + specularIBL * 1.15) * ao * iblMult;
        }

        finalLinearColor = ambientIBL + Lo + emissive;
    }

    // 9. Tone Mapping & Exposure Control
    // The Narkowicz ACES fit directly transforms linear HDR into non-linear sRGB display space.
    // (Applying an additional pow(x, 1/2.2) creates double gamma correction, which washes out contrast and desaturates colors).
    float exposure = (uExposure > 0.0) ? uExposure : 1.0;
    vec3 tonemapped = acesToneMap(finalLinearColor * exposure);

    // 10. Clean Outdoor Daytime Distance Perspective
    // Foreground and racetrack (dist < 400m) are strictly 100% fog-free and crystal clear.
    // Beyond 400m, distant terrain smoothly transitions into the blue sky horizon.
    float dist = length(vWorldPos - uCameraPos);
    float fogDist = max(dist - 400.0, 0.0);
    float fogFactor = exp(-pow(0.0010 * fogDist, 2.0));
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    vec3 fogToneMapped = acesToneMap(uFogColor * exposure);
    vec3 finalColor = mix(fogToneMapped, tonemapped, fogFactor);

    FragColor = vec4(finalColor, alpha);
}
