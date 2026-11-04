#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldDir;

void main() {
    vWorldDir = aPos;
    // Strip camera translation so the sky dome is always centered on the camera
    mat4 rotView = mat4(mat3(uView));
    vec4 clipPos = uProj * rotView * vec4(aPos, 1.0);
    // Put at far plane (z = w gives NDC z = 1.0)
    gl_Position = clipPos.xyww;
}
