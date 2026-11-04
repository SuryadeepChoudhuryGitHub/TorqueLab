#version 330 core
in vec3 vWorldDir;
out vec4 FragColor;

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uZenithColor;
uniform vec3 uMidSkyColor;
uniform vec3 uHorizonColor;
uniform vec3 uGroundHazeColor;
uniform float uExposure;
uniform int uQualityPreset; // 0=Low, 1=Medium, 2=High, 3=Maximum

const float PI = 3.141592653589793;

// Photographic ACES filmic tone reproduction curve (identical to scene shader)
vec3 acesToneMap(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// High-frequency interleaved gradient noise for anti-banding dithering
float getDither(vec2 coord) {
    return fract(52.9829189 * fract(dot(coord, vec2(0.06711056, 0.00583715))));
}

void main() {
    vec3 dir = normalize(vWorldDir);
    vec3 sunDir = normalize(uSunDir);
    float cosTheta = dot(dir, sunDir);
    float dy = dir.y;

    vec3 skyColor;

    if (uQualityPreset == 0) {
        // Preset 0 (Low): Fast smooth 2-stop daylight sky gradient
        float t = clamp(dy * 1.6, 0.0, 1.0);
        skyColor = mix(uHorizonColor, uZenithColor, t);
    } else {
        // Atmospheric Rayleigh & Mie Scattering
        if (dy >= 0.0) {
            float h = clamp(dy, 0.0, 1.0);
            
            // Optical depth falloff from horizon to zenith
            vec3 atmosColor;
            if (h < 0.28) {
                float k = h / 0.28;
                atmosColor = mix(uHorizonColor, uMidSkyColor, k);
            } else {
                float k = (h - 0.28) / 0.72;
                atmosColor = mix(uMidSkyColor, uZenithColor, k);
            }

            // Forward Rayleigh scattering: sky hemisphere facing the sun is slightly brighter
            float rayleighPhase = 0.75 * (1.0 + cosTheta * cosTheta);
            atmosColor += uSunColor * 0.045 * rayleighPhase * (1.0 - h * 0.45);

            skyColor = atmosColor;
        } else {
            // Below horizon: smooth blend into ground haze (eliminates seams)
            float t = clamp(-dy * 3.5, 0.0, 1.0);
            skyColor = mix(uHorizonColor, uGroundHazeColor, t);
        }

        // Solar Disc & Mie Corona
        if (cosTheta > 0.0) {
            // Crisp physical solar disc (approx 1.2 degrees angular diameter)
            float disc = smoothstep(0.99965, 0.99990, cosTheta);
            
            // Multi-octave solar aureole (corona)
            float corona1 = pow(max(cosTheta, 0.0), 32.0) * 0.55;
            float corona2 = pow(max(cosTheta, 0.0), 8.0) * 0.22;
            
            if (uQualityPreset >= 3) {
                // Maximum: High dynamic range solar core & forward Mie haze
                float corona3 = pow(max(cosTheta, 0.0), 256.0) * 2.2;
                skyColor += (uSunColor * 14.0 * disc + (corona1 + corona2 + corona3) * uSunColor * 1.5);
            } else {
                // High / Medium: Standard solar disc & corona
                skyColor += (uSunColor * 8.0 * disc + (corona1 + corona2) * uSunColor * 1.2);
            }
        }
    }

    // Photographic Exposure & ACES Tone Mapping
    // The Narkowicz ACES fit directly transforms linear HDR into non-linear sRGB display space.
    // (Applying an additional pow(x, 1/2.2) creates double gamma correction, which washes out contrast and desaturates colors).
    float exposure = (uExposure > 0.0) ? uExposure : 1.0;
    vec3 tonemapped = acesToneMap(skyColor * exposure);

    vec3 srgbOutput = tonemapped;

    // Anti-Banding Dithering (smooth 8-bit sky reproduction without banding)
    if (uQualityPreset >= 2) {
        float dither = (getDither(gl_FragCoord.xy) - 0.5) / 255.0;
        srgbOutput += vec3(dither);
    }

    FragColor = vec4(clamp(srgbOutput, 0.0, 1.0), 1.0);
}
