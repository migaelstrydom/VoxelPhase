// Per-draw surface material parameters, delivered via fragment push constants.
//
// Layout must match `SurfaceParams` in src/rendering/material.rs, and the
// declared offset must match the fragment push-constant range in
// src/rendering/pipeline.rs.

#ifndef MATERIAL_GLSL
#define MATERIAL_GLSL

layout(push_constant) uniform MaterialPushConstants {
    /// Flat colour replacing all shading when a > 0 (debug wireframe overlay).
    layout(offset = 64) vec4 colour_override;
    /// rgb = linear emissive colour, at its authored magnitude,
    /// w = scale bringing that colour to the authored emissive luminance.
    /// Use materialEmissive() rather than reading rgb directly — every
    /// consumer, including the rim term, wants the luminance-normalised
    /// product so brightness stays consistent across hues.
    vec4 emissive;
    /// x = roughness, y = metallic, z = rim strength, w = rim power.
    vec4 surface;
} material;

float materialRoughness() { return material.surface.x; }
float materialMetallic()  { return material.surface.y; }
float materialRimStrength() { return material.surface.z; }
float materialRimPower()    { return material.surface.w; }

/// Emissive radiance added independently of incoming light.
///
/// The scale is computed CPU-side (`Emission::radiance_scale`) so that this
/// product always has the authored luminance, whatever the hue.
vec3 materialEmissive() { return material.emissive.rgb * material.emissive.w; }

#endif // MATERIAL_GLSL
