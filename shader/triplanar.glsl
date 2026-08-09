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

/// What one sample of the packed surface field yields.
struct TriplanarSurface {
    /// Albedo wash, from the field's R channel.
    float wash;
    /// Geometric normal perturbed by the field's detail normal, in world space.
    vec3 normal;
};

/// Sample the packed surface field: albedo wash and detail normal in one pass.
///
/// The field stores the wash in R and a tangent-space detail normal's xy in GB
/// (`src/terrain/surface.rs`), so this costs the same three texture reads as
/// `triplanarSample` and returns both. Adding a second texture for the normals
/// would have tripled the sample count on the largest mesh in the game.
///
/// Detail normals are combined by the *whiteout* blend rather than by averaging
/// three world-space normals. Averaging flattens: where two planes both
/// contribute, their perturbations partly cancel and a 45° surface ends up
/// visibly smoother than the flat ground beside it. Whiteout adds each plane's
/// tangent-space slope to the geometric normal before the blend, so slopes
/// accumulate instead of competing.
///
/// `strength` scales the perturbation. Zero returns the geometric normal
/// exactly, which is what a material with no microstructure should get.
TriplanarSurface triplanarSurface(
    sampler2D tex,
    vec3 world_pos,
    vec3 normal,
    float scale,
    float sharpness,
    float strength
) {
    vec3 weights = triplanarWeights(normal, sharpness);
    vec3 projected = world_pos * scale;

    vec4 x_plane = texture(tex, projected.zy);
    vec4 y_plane = texture(tex, projected.xz);
    vec4 z_plane = texture(tex, projected.xy);

    TriplanarSurface result;
    result.wash = x_plane.r * weights.x + y_plane.r * weights.y + z_plane.r * weights.z;

    // Decode each plane's tangent-space slope. The mip chain averages these
    // towards zero as the texture minifies, which is what fades the detail out
    // with distance — no explicit LOD blend is needed.
    vec2 slope_x = (x_plane.gb * 2.0 - 1.0) * strength;
    vec2 slope_y = (y_plane.gb * 2.0 - 1.0) * strength;
    vec2 slope_z = (z_plane.gb * 2.0 - 1.0) * strength;

    // Each plane's uv axes map to the world axes it is addressed by, matching
    // the swizzles the samples were taken with above.
    vec3 detail_x = vec3(slope_x + normal.zy, normal.x).zyx;
    vec3 detail_y = vec3(slope_y + normal.xz, normal.y).xzy;
    vec3 detail_z = vec3(slope_z + normal.xy, normal.z).xyz;

    result.normal = normalize(
        detail_x * weights.x + detail_y * weights.y + detail_z * weights.z);
    return result;
}

#endif // TRIPLANAR_GLSL
