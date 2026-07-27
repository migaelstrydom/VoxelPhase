// Per-frame scene uniforms shared by all lit geometry pipelines (set 0, binding 0).
//
// Layout must match `SceneUbo` in src/rendering/frame.rs. Every member is
// vec4-aligned so the Rust and GLSL views agree without padding surprises.

#ifndef SCENE_GLSL
#define SCENE_GLSL

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 view;
    mat4 proj;
    /// xyz = camera world position, w unused.
    vec4 camera_pos;
    /// xyz = normalized direction from surface towards the sun, w = intensity.
    vec4 sun_direction;
    /// rgb = linear sun colour, w unused.
    vec4 sun_colour;
    /// rgb = linear ambient fill colour, w unused.
    vec4 ambient_colour;
} scene;

#endif // SCENE_GLSL
