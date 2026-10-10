#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// A river surface: each vertex at the water's own height, carrying its depth
// (negative over a bank) and its flow. The fragment shader cuts the surface
// where the depth falls below the shore depth, so a shoreline is a contour.
layout(location = 0) in vec2 inXz;
layout(location = 1) in vec2 inWater;  // (surface, depth)
layout(location = 2) in vec2 inFlow;   // depth-averaged velocity, m/s

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (shore depth, unused, unused, clock)
    vec4 tile;   // unused by a river surface
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragWetDepth;
layout(location = 6) flat out vec2 fragWetRange;
layout(location = 7) out vec4 fragSwell;

void main() {
    vec3 position = vec3(inXz.x, inWater.x, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = vec3(0.0, 1.0, 0.0);
    fragSwell = vec4(0.0);
    fragWorldPos = position;
    fragLayer = -1;
    fragTileOrigin = vec2(0.0);
    fragFlow = inFlow;
    fragWetDepth = inWater.y;
    fragWetRange = vec2(pc.body.x, 1.0e9);
}
