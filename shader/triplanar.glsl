// World-space triplanar texture projection.
//
// Marching-cubes terrain has no texture coordinates to sample by: the surface is
// an isosurface with no parameterisation, and destruction re-meshes it, so any
// UV assignment would have to be recomputed per crater. Such a surface is
// textured by world position instead — sampled once per axis plane and blended
// by how much the surface faces each.
//
// The CPU side of the decision is `src/rendering/triplanar.rs`; a scale of zero
// there means the fragment shader samples vertex texture coordinates as usual.

#ifndef TRIPLANAR_GLSL
#define TRIPLANAR_GLSL

/// Weights for the three axis projections, normalised to sum to one.
///
/// `sharpness` is the exponent on each axis component: it decides how quickly a
/// surface commits to the plane it most nearly faces. The normalisation is
/// guarded because a normal of zero length — which degenerate mesh triangles do
/// produce — would otherwise divide by zero and turn the fragment black.
vec3 triplanarWeights(vec3 normal, float sharpness) {
    vec3 weights = pow(abs(normal), vec3(sharpness));
    return weights / max(weights.x + weights.y + weights.z, 1e-4);
}

/// Sample `tex` by world position, blended across the three axis planes.
///
/// Each plane is addressed by the two axes it spans, so the plane a floor faces
/// is sampled by (x, z) and a wall facing along X by (z, y). `scale` is in
/// texture repeats per world unit.
///
/// No coordinate is flipped by the normal's sign, so opposite faces sample the
/// texture mirrored. That is invisible for the isotropic noise terrain uses and
/// would need revisiting for a texture with a readable direction to it.
vec4 triplanarSample(
    sampler2D tex,
    vec3 world_pos,
    vec3 normal,
    float scale,
    float sharpness
) {
    vec3 weights = triplanarWeights(normal, sharpness);
    vec3 projected = world_pos * scale;

    vec4 x_plane = texture(tex, projected.zy);
    vec4 y_plane = texture(tex, projected.xz);
    vec4 z_plane = texture(tex, projected.xy);

    return x_plane * weights.x + y_plane * weights.y + z_plane * weights.z;
}

#endif // TRIPLANAR_GLSL
