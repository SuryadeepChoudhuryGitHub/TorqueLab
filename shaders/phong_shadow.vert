#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec4 aTangent; // (tx, ty, tz, tw)

uniform mat4 uModel;
uniform mat4 uViewProj;
uniform mat4 uLightSpaceMatrix;

// Physically Based Pneumatic Tyre Deformation Uniforms
uniform int uTireDeformEnabled;        // 1 = tyre rubber, 0 = rigid
uniform int uWheelInContact;           // 1 = in physical contact, 0 = airborne
uniform vec3 uWheelCenterWorld;        // Physical wheel hub center in world space
uniform vec3 uWheelAxleWorld;          // Wheel spin axle unit vector in world space
uniform vec3 uGroundPointWorld;        // Contact point on road surface in world space
uniform vec3 uGroundNormalWorld;       // Contact normal (pointing upwards into tyre)
uniform float uTireOuterRadius;        // Nominal outer radius R_0
uniform float uTireRimRadius;          // Rigid rim boundary R_rim (strictly protected from deformation)
uniform float uTireWidth;              // Section width W_0
uniform float uRadialDeformation;      // delta_r (m, vertical radial squash)
uniform float uLateralDeformation;     // delta_y (m, lateral carcass deflection)
uniform float uLongitudinalDeformation;// delta_x (m, longitudinal carcass shear)
uniform float uSidewallBulge;          // delta_w (m, Poisson ratio sidewall expansion)
uniform float uPatchAngleSpan;         // Angular half-extent of deformation zone (rad)

// Backward compatibility local uniforms
uniform vec3 uTireContactDirLocal;
uniform vec3 uTireAxleLocal;

out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec2 vTexCoord;
out vec4 vPosLightSpace;
out vec3 vWorldTangent;
out float vTangentW;

void main() {
    vec4 worldPos4 = uModel * vec4(aPos, 1.0);
    vec3 worldPos = worldPos4.xyz;

    mat3 normalMatrix = transpose(inverse(mat3(uModel)));
    vec3 worldNormal = normalize(normalMatrix * aNormal);
    vec3 worldTangent = normalize(mat3(uModel) * aTangent.xyz);

    // Apply physically based tyre deformation exclusively to rubber tyre vertices outside rim radius
    if (uTireDeformEnabled != 0 && uWheelInContact != 0 && uRadialDeformation > 0.0001) {
        // 1. Vector from physical wheel center to this vertex in World Space
        vec3 rHub = worldPos - uWheelCenterWorld;
        vec3 uAxle = normalize(uWheelAxleWorld);
        float xAxle = dot(rHub, uAxle);
        vec3 rRadial = rHub - uAxle * xAxle;
        float r = length(rRadial);

        // Strict Rim Protection: vertices within or at rim radius experience ZERO deformation
        if (r > uTireRimRadius + 0.002) {
            float eta = clamp((r - (uTireRimRadius + 0.002)) / max(0.005, uTireOuterRadius - (uTireRimRadius + 0.002)), 0.0, 1.0);
            float smoothEta = eta * eta * (3.0 - 2.0 * eta);

            // Unit radial vector in wheel disk plane
            vec3 uRadial = rRadial / max(0.001, r);

            // 2. Exact Perpendicular Distance to Ground Contact Plane
            // dGround < 0 means the undeformed vertex extends below ground (penetrating)
            vec3 uNormal = normalize(uGroundNormalWorld);
            float dGround = dot(worldPos - uGroundPointWorld, uNormal);

            // Angle around wheel hub relative to contact downward direction (-uNormal)
            vec3 uContactDown = -uNormal;
            float cosTheta = clamp(dot(uRadial, uContactDown), -1.0, 1.0);
            float theta = acos(cosTheta);

            // Contact patch angular span with smooth viscoelastic blend shoulder
            float angleSpan = max(0.12, uPatchAngleSpan);
            float blendSpan = angleSpan * 1.50;

            if (theta < blendSpan) {
                // Smooth contact zone weight
                float contactWeight = 0.0;
                if (theta < angleSpan) {
                    float t = theta / angleSpan;
                    contactWeight = 1.0 - t * t * (3.0 - 2.0 * t);
                } else {
                    float t = (theta - angleSpan) / (blendSpan - angleSpan);
                    float edgeWeight = (1.0 - t * t * (3.0 - 2.0 * t));
                    contactWeight = edgeWeight * 0.18;
                }

                // 3. Ground Penetration Elimination & Soft Viscoelastic Flattening:
                // If penetrating (dGround < 0), lift by exactly -dGround so vertex sits flush on the road surface
                // Add subtle curvature rounding at the edge of the patch
                float penDepth = max(0.0, -dGround);
                float edgeRounding = (1.0 - cosTheta) * 0.06 * uRadialDeformation * contactWeight;
                float liftAmount = (penDepth + edgeRounding) * smoothEta;
                vec3 deltaFlatten = uNormal * liftAmount;

                // 4. Realistic Toroidal Sidewall Outward Bulge (Poisson expansion):
                // Peaks at mid-sidewall (eta ~ 0.55), zero at rim bead (eta = 0), smooth at shoulder
                float bulgeProfile = sin(pow(eta, 0.75) * 3.14159265) * (1.0 - 0.20 * eta);
                vec3 deltaBulge = uAxle * (sign(xAxle) * uSidewallBulge * bulgeProfile * contactWeight);

                // 5. Lateral Carcass Cornering Shear (delta_y)
                vec3 deltaLat = uAxle * (uLateralDeformation * eta * contactWeight);

                // 6. Longitudinal Traction / Braking Shear Windup (delta_x)
                vec3 uForward = normalize(cross(uNormal, uAxle));
                vec3 deltaLong = uForward * (uLongitudinalDeformation * eta * contactWeight);

                worldPos = worldPos + deltaFlatten + deltaBulge + deltaLat + deltaLong;

                // 7. Update Surface Normal Vector for Flawless PBR Shading & Reflections:
                // In flattened contact patch, tread normal aligns with ground normal
                float flattenInfluence = clamp(penDepth / max(0.008, uRadialDeformation), 0.0, 1.0) * smoothEta;
                worldNormal = normalize(mix(worldNormal, uNormal, flattenInfluence * 0.92));
                // Along bulging sidewall, normal tilts outward
                worldNormal = normalize(worldNormal + uAxle * (sign(xAxle) * uSidewallBulge * 14.0 * bulgeProfile * contactWeight));
            }
        }
    }

    vWorldPos = worldPos;
    vWorldNormal = worldNormal;
    vWorldTangent = worldTangent;
    vTangentW = aTangent.w;
    vTexCoord = aTexCoord;
    vPosLightSpace = uLightSpaceMatrix * vec4(worldPos, 1.0);
    gl_Position = uViewProj * vec4(worldPos, 1.0);
}
