// Per-draw surface material parameters.
//
// Only an index travels in the push constants; the parameters themselves live
// in the frame's surface table, a storage buffer at set 0, binding 3. Push
// constants are guaranteed to be only 128 bytes and the model matrix plus the
// colour override already spend 80 of them, which left no room for the
// material system to grow. See src/rendering/surface_buffer.rs.
//
// The `GpuSurface` layout must match the struct of that name in
// src/rendering/material.rs, and the push-constant offset must match the
// fragment range in src/rendering/pipeline.rs.

#ifndef MATERIAL_GLSL
#define MATERIAL_GLSL

layout(push_constant) uniform MaterialPushConstants {
    /// The draw's model matrix, shared with the vertex stage. The fragment
    /// stage reads its rotation to carry an object-space grain perturbation
    /// into world space.
    layout(offset = 0) mat4 model;
    /// Flat colour replacing all shading when a > 0 (debug wireframe overlay).
    layout(offset = 64) vec4 colour_override;
    /// Which entry of the surface table this draw shades with.
    uint surface_index;
} material;

/// One surface's shading parameters. Whole vec4s exactly: std430 rounds a
/// struct's stride up to its alignment, so a layout that is not a multiple of
/// 16 bytes would silently read every entry after the first from the wrong
/// offset.
struct GpuSurface {
    /// rgb = linear emissive colour, at its authored magnitude,
    /// w = scale bringing that colour to the authored emissive luminance.
    /// Use materialEmissive() rather than reading rgb directly — every
    /// consumer, including the rim term, wants the luminance-normalised
    /// product so brightness stays consistent across hues.
    vec4 emissive;
    /// x = roughness, y = metallic, z = rim strength, w = rim power.
    vec4 surface;
    /// x = albedo triplanar scale in texture repeats per world unit,
    /// y = blend sharpness, z = grain scale, w = grain strength.
    vec4 projection;
    /// x = SurfaceSource flags (see surface_source.glsl), y = grain index,
    /// zw spare.
    uvec4 control;
    /// x = opacity viewed head-on (1 = opaque), y = reflectance at normal
    /// incidence, derived from the material's refractive index. zw spare.
    /// See src/rendering/transparency/optics.rs.
    vec4 optics;
    /// x = depth of the relief in the diffuse alpha, in texture coordinates.
    /// yzw spare. See src/rendering/material.rs `with_relief`.
    vec4 detail;
};

layout(std430, set = 0, binding = 3) readonly buffer SurfaceTable {
    GpuSurface surfaces[];
} surface_table;

/// The rotation (and scale) taking this draw's model space into world space.
///
/// Uses `mat3(model)` rather than its inverse transpose, matching what the
/// vertex shader does to normals: exact for uniform scaling, and the meshes
/// this is applied to are uniformly scaled.
mat3 materialModelToWorld() { return mat3(material.model); }

/// This draw's surface parameters.
GpuSurface materialSurface() {
    return surface_table.surfaces[material.surface_index];
}

float materialRoughness() { return materialSurface().surface.x; }
float materialMetallic()  { return materialSurface().surface.y; }
float materialRimStrength() { return materialSurface().surface.z; }
float materialRimPower()    { return materialSurface().surface.w; }

float materialTriplanarScale()     { return materialSurface().projection.x; }
float materialTriplanarSharpness() { return materialSurface().projection.y; }
float materialGrainScale()         { return materialSurface().projection.z; }
float materialGrainStrength()      { return materialSurface().projection.w; }

/// Which shading inputs this surface asks for. See surface_source.glsl.
uint materialSource() { return materialSurface().control.x; }

/// Which layer of the grain atlas this surface's microstructure comes from.
uint materialGrainLayer() { return materialSurface().control.y; }

/// Fraction of the pixel this surface claims when viewed head-on, before the
/// Fresnel gain at grazing angles. 1 for everything but glass and ice.
float materialOpacity() { return materialSurface().optics.x; }

/// This surface's reflectance at normal incidence, from its refractive index.
float materialReflectance() { return materialSurface().optics.y; }

/// How deep the relief in the diffuse alpha runs, in texture coordinates.
float materialReliefDepth() { return materialSurface().detail.x; }

/// Emissive radiance added independently of incoming light.
///
/// The scale is computed CPU-side (`Emission::radiance_scale`) so that this
/// product always has the authored luminance, whatever the hue.
vec3 materialEmissive() {
    GpuSurface s = materialSurface();
    return s.emissive.rgb * s.emissive.w;
}

#endif // MATERIAL_GLSL
