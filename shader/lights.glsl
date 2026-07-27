// Per-frame point light set (set 0, binding 1).
//
// Layout must match `LightUbo` in src/rendering/frame.rs. The `MAX_ACTIVE_LIGHTS`
// value below is checked against the Rust constant by a unit test in that file;
// changing one without the other corrupts the tail of the array.
//
// std140: `PointLight` is two vec4s (32 bytes), which is already the array
// element stride, so no per-element padding is needed. `count` is a uint at
// offset 0 and the array must begin at 16, hence the explicit padding.

#ifndef LIGHTS_GLSL
#define LIGHTS_GLSL

// `PointLight` itself lives in lighting.glsl beside the other light types, so
// that shaders can shade a point light without pulling in this uniform block.
#include "lighting.glsl"

const int MAX_ACTIVE_LIGHTS = 16;

layout(std140, set = 0, binding = 1) uniform LightUbo {
    /// Number of live entries in `lights`. Entries beyond it are zeroed, but
    /// looping past it still costs shading time — bound the loop with this.
    uint count;
    uint _padding0;
    uint _padding1;
    uint _padding2;
    PointLight lights[MAX_ACTIVE_LIGHTS];
} light_set;

#endif // LIGHTS_GLSL
