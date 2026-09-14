// Microstructure from the shared grain atlas.
//
// The atlas packs two grains into one texture's four channels — RG is stone,
// BA is directional fibre — so selecting one costs a swizzle rather than a
// second sample.
// See src/rendering/grain.rs for how it is generated and why.
//
// Terrain does not come through here. Its detail normal arrives packed beside
// its albedo in a single triplanar read (triplanar.glsl), which is cheaper than
// a separate lookup on the largest mesh in the game.

#ifndef GRAIN_GLSL
#define GRAIN_GLSL

#include "triplanar.glsl"

/// This grain's tangent-space slope at a point in the atlas.
///
/// `layer` selects the channel pair: 0 is stone (RG), 1 is wood (BA). The
/// stored bytes are signed values biased into the unit range, so they come back
/// through the same `value * 2 - 1` the terrain field uses.
vec2 grainSlope(sampler2D atlas, vec2 uv, uint layer) {
    vec4 packed = texture(atlas, uv);
    vec2 encoded = layer == 0u ? packed.rg : packed.ba;
    return encoded * 2.0 - 1.0;
}

/// Perturb a normal by grain addressed with the mesh's texture coordinates.
///
/// For grain with a direction the mesh already knows — fibre runs along the
/// plank and fracture runs through the block, and a projection would run
/// either along whichever axis the face happens to point at.
///
/// The mesh's UVs are treated as a tangent frame whose u axis is world-x-ish
/// and v axis world-z-ish, which is the same approximation the rest of the
/// engine's untangented meshes make. It is good enough for a perturbation of a
/// few degrees and wrong enough that a strongly directional grain on a curved
/// UV-mapped surface will lean; the surfaces this is used on are flat-faced.
vec3 grainByUv(
    sampler2D atlas,
    vec3 normal,
    vec2 uv,
    uint layer,
    float scale,
    float strength
) {
    vec2 slope = grainSlope(atlas, uv * scale, layer) * strength;

    // Build a tangent frame around the geometric normal. `up` is chosen away
    // from the normal so the cross product cannot collapse on a vertical face.
    vec3 up = abs(normal.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(up, normal));
    vec3 bitangent = cross(normal, tangent);

    return normalize(normal + tangent * slope.x + bitangent * slope.y);
}

/// Perturb a normal by grain projected from the mesh's own frame.
///
/// Both the sampling position and the plane weights come from model space, so
/// the pattern is fixed to the object: a menhir that topples carries its grain
/// with it, rather than sliding through a world-fixed field. The perturbed
/// normal is built in model space and rotated out by `model_to_world`.
///
/// Combined with the whiteout blend rather than by averaging three world-space
/// normals, for the reason `triplanarSurface` documents: averaging lets two
/// planes' perturbations cancel, so a 45-degree face ends up visibly smoother
/// than the flat one beside it.
vec3 grainObjectSpace(
    sampler2D atlas,
    vec3 world_normal,
    mat3 model_to_world,
    vec3 model_pos,
    vec3 model_normal,
    uint layer,
    float scale,
    float strength,
    float sharpness
) {
    vec3 weights = triplanarWeights(model_normal, sharpness);
    vec3 projected = model_pos * scale;

    vec2 slope_x = grainSlope(atlas, projected.zy, layer) * strength;
    vec2 slope_y = grainSlope(atlas, projected.xz, layer) * strength;
    vec2 slope_z = grainSlope(atlas, projected.xy, layer) * strength;

    // Each plane's uv axes map to the model axes it was addressed by, matching
    // the swizzles the samples were taken with.
    vec3 detail_x = vec3(slope_x + model_normal.zy, model_normal.x).zyx;
    vec3 detail_y = vec3(slope_y + model_normal.xz, model_normal.y).xzy;
    vec3 detail_z = vec3(slope_z + model_normal.xy, model_normal.z).xyz;

    vec3 perturbed = detail_x * weights.x + detail_y * weights.y + detail_z * weights.z;

    vec3 result = model_to_world * perturbed;

    // A degenerate model matrix (a fully flattened scale) would leave nothing
    // to normalise; keep the geometric normal rather than emitting a NaN.
    return length(result) > 1e-4 ? normalize(result) : world_normal;
}

#endif // GRAIN_GLSL
