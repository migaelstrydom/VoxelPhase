#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// A fall's sheet: two vertices at each point of its arc, pushed apart
// across it by the width the discharge now gives, spreading as it drops.
layout(location = 0) in vec3 inCentre;
layout(location = 1) in vec3 inSide;
layout(location = 2) in vec3 inPath;   // (across, seconds from the lip, share of the way down)

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (half width, strength, spread, clock)
    vec4 tile;   // unused
} pc;

layout(location = 0) out vec3 fragWorldPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out float fragAcross;
layout(location = 3) out float fragTime;
layout(location = 4) out float fragAlong;
layout(location = 5) flat out vec2 fragBody;   // (strength, clock)

void main() {
    float halfWidth = pc.body.x * (1.0 + pc.body.z * inPath.z);
    vec3 position = inCentre + inSide * inPath.x * halfWidth;
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragWorldPos = position;
    fragNormal = normalize(cross(inSide, vec3(0.0, 1.0, 0.0)));
    fragAcross = inPath.x;
    fragTime = inPath.y;
    fragAlong = inPath.z;
    fragBody = pc.body.yw;
}
