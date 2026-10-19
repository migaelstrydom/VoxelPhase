// Aerial perspective: distance fades a surface toward the sky behind it.
//
// Air between the eye and a surface scatters skylight into the view ray, so a
// far cliff is paler than a near one and the eye can order them. Without it a
// cliff at 300 m is as crisp as one at 30 m, and the sea ends in a hard line
// where the far plane cuts it off.
//
// The haze is the sky's own radiance at the horizon on the ray's bearing,
// from the same `skyRadiance` the sky pass draws. A surface hazed out entirely
// is therefore the colour of the sky beside it, which is what lets the sea
// dissolve into the horizon with no seam. The sky pass draws the same colour
// below the horizon, where the ray runs past the far plane.
//
// Everything here is radiance in and radiance out, like the rest of the
// lighting: exposure and tonemapping stay in the post chain.

#ifndef AERIAL_GLSL
#define AERIAL_GLSL

#include "sky_model.glsl"

/// How sharply the haze closes in around the visibility distance.
///
/// 1 is plain exponential haze, which cannot keep the middle distance clear
/// and still close before the far plane: tuned to hide the sea's edge, it turns
/// the whole sea milky. Higher keeps the play space and the island crisp and
/// spends the haze on the last few hundred metres.
const float AERIAL_FALLOFF = 3.0;

/// How much of a surface at `distance` metres the haze covers, in [0, 1),
/// where it covers 1 - 1/e (about 63%) at `visibility` metres.
float aerialCoverage(float distance, float visibility) {
    return 1.0 - exp(-pow(distance / visibility, AERIAL_FALLOFF));
}

/// The direction whose sky radiance the haze takes on a view ray.
///
/// Rays below the horizon take the horizon's colour on the same bearing: the
/// haze in front of a distant sea is the air at the horizon, not the dim ground
/// tone `skyScattering` settles to underneath it.
vec3 aerialSkyDirection(vec3 view_ray) {
    return normalize(vec3(view_ray.x, max(view_ray.y, 0.0), view_ray.z));
}

/// Radiance of the haze along a view ray.
vec3 aerialRadiance(vec3 view_ray, vec3 sun_dir) {
    return skyRadiance(aerialSkyDirection(view_ray), sun_dir);
}

/// `radiance` leaving a surface at `world_pos`, as the eye sees it through the
/// air between them. `camera` is the eye's position in xyz and the visibility
/// distance in w, as the scene uniforms carry them; a visibility of 0 is no haze
/// at all (`SceneLighting::aerial_visibility`).
vec3 applyAerialPerspective(vec3 radiance, vec3 world_pos, vec4 camera, vec3 sun_dir) {
    vec3 to_surface = world_pos - camera.xyz;
    float distance = length(to_surface);
    if (camera.w <= 0.0 || distance <= 0.0) {
        return radiance;
    }
    vec3 haze = aerialRadiance(to_surface / distance, sun_dir);
    return mix(radiance, haze, aerialCoverage(distance, camera.w));
}

#endif // AERIAL_GLSL
