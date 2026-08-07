// Per-frame scene uniforms shared by all lit geometry pipelines (set 0, binding 0).
//
// Layout must match `SceneUbo` in src/rendering/frame.rs. Every member is
// vec4-aligned so the Rust and GLSL views agree without padding surprises.

#ifndef SCENE_GLSL
#define SCENE_GLSL

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 view;
    mat4 proj;
    /// World space to the sun's clip space, for shadow map lookups.
    mat4 light_view_proj;
    /// xyz = camera world position, w unused.
    vec4 camera_pos;
    /// xyz = normalized direction from surface towards the sun, w = intensity.
    vec4 sun_direction;
    /// rgb = linear sun colour, w unused.
    vec4 sun_colour;
    /// rgb = linear ambient fill colour, w unused.
    vec4 ambient_colour;
    /// x = one shadow map texel in UV, y = normal offset in world units,
    /// z = shadow strength (0 leaves everything lit), w unused.
    vec4 shadow_params;
} scene;

#endif // SCENE_GLSL
