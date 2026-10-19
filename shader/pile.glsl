// A pile: a surface of standing fibre, shaded as a layer of thin strands.
//
// Grass is not a solid face, and lit as one it reads as green felt. A layer of
// blades answers light in three ways a face does not:
//
//   sun ahead of the eye  ──▶ transmission: the blades are thin, so light comes
//                             through them and they glow yellow-green; most of
//                             all along crests seen against the sun
//   seen edge-on          ──▶ sheen: the eye sees the blades' sides instead of
//                             the dark gaps between them, so the layer goes
//                             pale and soft towards its silhouettes
//   seen from above       ──▶ depth: the eye looks down into the gaps between
//                             the blades, which are in their own shade
//
// Sheen and depth trade light rather than add it: the layer is darker face-on
// and paler edge-on. Grass is authored against the scene's light level, and a
// term that only adds pushes its saturated green up the tonemap's shoulder.
//
// How much of a surface is pile is `VoxelMaterial::pile`, carried per vertex.
// Everything here is added on top of the surface's own response, scaled by it.

#ifndef PILE_GLSL
#define PILE_GLSL

#include "lighting.glsl"
#include "environment.glsl"

/// Width of the sheen lobe ("Charlie" distribution alpha). Lower gathers the
/// sheen into a thinner rim at the silhouette; higher spreads it across the
/// slope.
const float PILE_SHEEN_ROUGHNESS = 0.35;

/// Brightness of the sun's sheen.
const float PILE_SHEEN_STRENGTH = 1.0;

/// Brightness of the sky's sheen: the pale cast a grazing view of the layer
/// takes from the whole dome, sun or no sun.
const float PILE_SKY_SHEEN_STRENGTH = 0.4;

/// Brightness of sunlight coming through the blades.
const float PILE_TRANSMISSION_STRENGTH = 4.0;

/// How tightly transmission gathers around looking straight into the sun.
const float PILE_TRANSMISSION_FOCUS = 2.0;

/// How far past the terminator transmission reaches. Blades stand up out of
/// the surface, so a slope turned slightly away from the sun still has blade
/// tips in it; 0 would stop the glow exactly where the face goes dark.
const float PILE_TRANSMISSION_WRAP = 0.4;

/// Share of its light a pile seen straight down returns: the rest falls into
/// the gaps between the blades. Edge-on it returns all of it.
const float PILE_FACE_ON_SHADE = 0.8;

/// Colour of light reflected off the blades' sides: the albedo lifted and
/// desaturated, as a waxy blade seen edge-on is.
vec3 pileSheenColour(vec3 albedo) {
    return sqrt(albedo);
}

/// Colour of light that has passed through a blade: the albedo's hue at full
/// value. A leaf filters light, so what comes through is saturated rather than
/// dark.
vec3 pileTransmittedColour(vec3 albedo) {
    return albedo / max(max(albedo.r, max(albedo.g, albedo.b)), 1e-3);
}

/// "Charlie" sheen distribution (Estevez and Kulla): fibres scatter most where
/// the half vector lies across them, so it grows towards grazing.
float pileSheenDistribution(float n_dot_h) {
    float inverse_alpha = 1.0 / PILE_SHEEN_ROUGHNESS;
    float sin2 = max(1.0 - n_dot_h * n_dot_h, 0.0078125);
    return (2.0 + inverse_alpha) * pow(sin2, inverse_alpha * 0.5) / (2.0 * SKY_PI);
}

/// Ashikhmin's visibility term for a fibre layer: no masking to speak of.
float pileSheenVisibility(float n_dot_l, float n_dot_v) {
    return 1.0 / (4.0 * (n_dot_l + n_dot_v - n_dot_l * n_dot_v));
}

/// How much of the surface's own light a pile returns towards the eye, in
/// [PILE_FACE_ON_SHADE, 1]: the shade of the gaps between the blades.
float pileDepthShade(SurfaceSample surface, float pile) {
    float n_dot_v = clamp(dot(surface.normal, surface.view_dir), 0.0, 1.0);
    return mix(1.0, mix(1.0, PILE_FACE_ON_SHADE, n_dot_v), pile);
}

/// A pile's response to the sun, on top of the surface's own: sheen and light
/// through the blades. Before the sun's shadow, which the
/// caller applies as it does to the rest of the sun's light.
vec3 pileSunResponse(SurfaceSample surface, DirectionalLight sun, float pile) {
    if (pile <= 0.0) {
        return vec3(0.0);
    }
    vec3 n = surface.normal;
    vec3 v = surface.view_dir;
    vec3 l = sun.direction;
    vec3 radiance = sun.colour * sun.intensity;

    float n_dot_l = dot(n, l);
    float n_dot_v = max(dot(n, v), 1e-3);
    float lit = max(n_dot_l, 0.0);

    float n_dot_h = max(dot(n, normalize(l + v)), 0.0);
    vec3 sheen = pileSheenColour(surface.albedo) * PILE_SHEEN_STRENGTH
               * pileSheenDistribution(n_dot_h) * pileSheenVisibility(lit, n_dot_v) * lit;

    float into_sun = pow(max(dot(v, -l), 0.0), PILE_TRANSMISSION_FOCUS);
    // Light reaches the eye through the blades only where it enters them from
    // behind, near the terminator, and is seen across the blades rather than
    // down into them: the glow gathers on crests and silhouettes. Ground the
    // sun shines straight down on sends what comes through into the soil, and
    // a slope seen face-on keeps its modelling.
    float wrapped = clamp((n_dot_l + PILE_TRANSMISSION_WRAP) / (1.0 + PILE_TRANSMISSION_WRAP), 0.0, 1.0);
    float backlit = (1.0 - lit) * (1.0 - lit);
    float across = (1.0 - n_dot_v) * (1.0 - n_dot_v);
    vec3 transmission = pileTransmittedColour(surface.albedo) * PILE_TRANSMISSION_STRENGTH
                      * into_sun * wrapped * backlit * across;

    return radiance * pile * (sheen + transmission);
}

/// A pile's sheen under the sky, on top of the surface's own environment
/// light. Occluded like the sky irradiance it comes from.
vec3 pileSkyResponse(SurfaceSample surface, vec3 sun_dir, float pile) {
    if (pile <= 0.0) {
        return vec3(0.0);
    }
    float n_dot_v = max(dot(surface.normal, surface.view_dir), 0.0);
    float grazing = pow(1.0 - n_dot_v, 5.0);
    return pileSheenColour(surface.albedo) * environmentIrradiance(surface.normal, sun_dir)
         * PILE_SKY_SHEEN_STRENGTH * grazing * surface.occlusion * pile;
}

#endif // PILE_GLSL
