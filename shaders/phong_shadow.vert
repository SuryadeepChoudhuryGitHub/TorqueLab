#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 uModel;
uniform mat4 uViewProj;
uniform mat4 uLightSpaceMatrix;

out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec2 vTexCoord;
out vec4 vPosLightSpace;

void main() {
    vec4 worldPos4 = uModel * vec4(aPos, 1.0);
    vWorldPos = worldPos4.xyz;
    mat3 normalMatrix = transpose(inverse(mat3(uModel)));
    vWorldNormal = normalize(normalMatrix * aNormal);
    vTexCoord = aTexCoord;
    vPosLightSpace = uLightSpaceMatrix * worldPos4;
    gl_Position = uViewProj * worldPos4;
}
