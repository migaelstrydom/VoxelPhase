// The probe face being captured (set 2, binding 0 of the capture pipeline).
//
// Layout must match `GpuProbeFace` in src/rendering/reflection/faces.rs.

#ifndef PROBE_FACE_GLSL
#define PROBE_FACE_GLSL

layout(set = 2, binding = 0) uniform ProbeFace {
    /// World space to this face's clip space.
    mat4 view_proj;
    /// xyz = the probe's centre, the eye every fragment is viewed from.
    /// w unused.
    vec4 eye;
} probe_face;

#endif // PROBE_FACE_GLSL
