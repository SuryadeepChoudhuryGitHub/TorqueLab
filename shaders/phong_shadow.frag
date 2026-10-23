#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vPosLightSpace;

out vec4 FragColor;

uniform vec3 uCameraPos;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uSkyColor;
uniform vec3 uGroundColor;
uniform vec3 uFogColor;
uniform float uFogDensity;

uniform vec3 uBaseColor;
uniform vec3 uEmissiveColor;
uniform float uEmissiveIntensity;
uniform float uRoughness;
uniform float uMetalness;
uniform float uOpacity;
uniform int uUseTexture;
uniform vec2 uUvScale;
uniform int uReceiveShadows;
uniform int uShadowsEnabled;
uniform int uUnlit;

uniform sampler2D uAlbedoMap;
uniform sampler2D uShadowMap;

float computeShadowPCF(vec4 posLightSpace, vec3 normal, vec3 lightDir) {
    vec3 projCoords = posLightSpace.xyz / posLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 || projCoords.y < 0.0 || projCoords.y > 1.0) {
        return 0.0;
    }

    float currentDepth = projCoords.z;
    float bias = max(0.0018 * (1.0 - dot(normal, lightDir)), 0.00045);

    const float texelSize = 1.0 / 2048.0;
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
}

vec3 acesToneMap(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main() {
    vec3 albedo = uBaseColor;
    if (uUseTexture != 0) {
        vec3 texColor = texture(uAlbedoMap, vTexCoord * uUvScale).rgb;
        albedo *= texColor;
    }

    vec3 color;
    if (uUnlit != 0) {
        color = albedo + uEmissiveColor * uEmissiveIntensity;
    } else {
        vec3 N = normalize(vWorldNormal);
        vec3 V = normalize(uCameraPos - vWorldPos);
        vec3 L = normalize(uSunDir);
        vec3 H = normalize(L + V);

        // Hemisphere ambient lighting (matching Three.js HemisphereLight)
        float hemiWeight = N.y * 0.5 + 0.5;
        vec3 hemiAmbient = mix(uGroundColor, uSkyColor, hemiWeight) * 0.52;

        // Diffuse & metallic/roughness specular
        float NdotL = max(dot(N, L), 0.0);
        vec3 diffuse = albedo * (1.0 - uMetalness * 0.65) * NdotL;

        float shininess = mix(140.0, 8.0, clamp(uRoughness, 0.05, 1.0));
        float specStrength = mix(0.95, 0.08, clamp(uRoughness, 0.05, 1.0));
        vec3 specTint = mix(vec3(1.0), albedo, uMetalness);
        float spec = pow(max(dot(N, H), 0.0), shininess) * specStrength * NdotL;
        vec3 specular = specTint * spec;

        // Subtle sky reflection on metallic/glossy surfaces (car paint & glass)
        float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);
        vec3 envReflect = mix(uGroundColor, uSkyColor, clamp(reflect(-V, N).y * 0.5 + 0.5, 0.0, 1.0));
        vec3 reflection = envReflect * (uMetalness * 0.28 + fresnel * (1.0 - uRoughness) * 0.22);

        float shadow = 0.0;
        if (uShadowsEnabled != 0 && uReceiveShadows != 0) {
            shadow = computeShadowPCF(vPosLightSpace, N, L);
        }

        vec3 directLight = (diffuse + specular) * uSunColor * (1.0 - shadow * 0.72);
        vec3 emissive = uEmissiveColor * uEmissiveIntensity;

        color = hemiAmbient * albedo + reflection + directLight + emissive;
        color = acesToneMap(color * 1.08);
    }

    // Exponential squared atmospheric fog (matching Three.js FogExp2)
    float dist = length(vWorldPos - uCameraPos);
    float fogFactor = exp(-pow(uFogDensity * dist, 2.0));
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    color = mix(uFogColor, color, fogFactor);

    FragColor = vec4(color, uOpacity);
}
