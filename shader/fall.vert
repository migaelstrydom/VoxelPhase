#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// A fall's sheet: two vertices at each point of its arc, pushed apart
// across it by the width the discharge now gives, spreading as it drops.
// The arc runs to the ground; the sheet is lifted to leave the surface it
// leaves from, less so down the arc to not at all at the ground, and cut off
// where it enters the water below (§7.9).
layout(location = 0) in vec3 inCentre;
layout(location = 1) in vec3 inSide;
layout(location = 2) in vec3 inPath;   // (across, seconds from the lip, share of the arc)

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (half width, strength, spread, clock)
    vec4 tile;   // (lift, top, cut, aeration)
} pc;

layout(location = 0) out vec3 fragWorldPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out float fragAcross;
layout(location = 3) out float fragTime;
layout(location = 4) out float fragAlong;
layout(location = 5) flat out vec4 fragBody;   // (strength, clock, cut, aeration)

void main() {
    vec3 centre = inCentre + vec3(0.0, pc.tile.x * (1.0 - inPath.z), 0.0);
    // Share of the way down the part that shows, from the top to the cut.
    float along = clamp((pc.tile.y - centre.y) / max(pc.tile.y - pc.tile.z, 1e-3), 0.0, 1.0);
    float halfWidth = pc.body.x * (1.0 + pc.body.z * along);
    vec3 position = centre + inSide * inPath.x * halfWidth;
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragWorldPos = position;
    fragNormal = normalize(cross(inSide, vec3(0.0, 1.0, 0.0)));
    fragAcross = inPath.x;
    fragTime = inPath.y;
    fragAlong = along;
    fragBody = vec4(pc.body.yw, pc.tile.zw);
}
