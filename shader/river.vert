#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// A reach's surface: cross-sections along its centreline, each vertex at the
// section's bed plus the design depth, scaled to the reach's discharge now.
layout(location = 0) in vec2 inXz;
layout(location = 1) in vec3 inSection;  // (bed, design depth, distance down the reach)
layout(location = 2) in vec2 inFlow;     // velocity at the design discharge

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (depth scale, tail, front, clock)
    vec4 tile;   // (speed scale, unused, unused, unused)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragAlong;
layout(location = 6) flat out vec2 fragWetRange;
layout(location = 7) out vec4 fragSwell;

void main() {
    float surface = inSection.x + inSection.y * pc.body.x;
    vec3 position = vec3(inXz.x, surface, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = vec3(0.0, 1.0, 0.0);
    fragSwell = vec4(0.0);
    fragWorldPos = position;
    fragLayer = -1;
    fragTileOrigin = vec2(0.0);
    fragFlow = inFlow * pc.tile.x;
    fragAlong = inSection.z;
    fragWetRange = pc.body.yz;
}
