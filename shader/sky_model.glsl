// The sky as a function of direction, in linear HDR radiance.
//
// Shared by the sky pass, which evaluates it once per screen pixel, and by lit
// geometry, which evaluates it along reflection and hemisphere directions to
// light surfaces from their surroundings. Because the sky is an analytic
// function of direction there is no cubemap to render, prefilter or keep in
// step — a reflection probe is just another call to `skyRadiance`.
//
// Everything here returns radiance, never display colour. Tonemapping and
// gamma belong to the post chain, which is the only thing that knows the
// scene's exposure; a shader that resolved its own would both double-tonemap
// and cap itself below the bloom threshold.

#ifndef SKY_MODEL_GLSL
#define SKY_MODEL_GLSL

const float SKY_PI = 3.14159265359;

const vec3 RAYLEIGH_COEFF = vec3(5.8e-6, 13.5e-6, 33.1e-6);

/// Angular radius of the sun in radians (about 0.53 degrees across).
const float SUN_ANGULAR_RADIUS = 0.00925;

/// Scales the scattering model's arbitrary units into scene radiance.
///
/// The model's own output is a shaping curve rather than a physical quantity,
/// so one number sets where the whole sky sits relative to the exposure the
/// post chain applies. Raising it brightens the sky and everything the sky
/// lights, since geometry samples this same function.
///
/// It cannot be tuned independently of how the sky looks on screen: a mirror
/// reflecting the sky has to match the sky beside it, so one number sets both
/// the backdrop's brightness and the fill light every surface receives. Balance
/// a scene with the sun's intensity against this, rather than by splitting it.
const float SKY_RADIANCE_SCALE = 1.2;

/// Radiance of the solar disc itself.
///
/// Above any exposure the scene will use, which is the point: the sun reads as
/// a source rather than a white circle because it clears the bloom threshold
/// and the bloom filter spreads it. Chasing that look by growing the disc
/// instead just produces a big flat blob.
///
/// This is the dial for the sun's *apparent* size, far more than the disc
/// radius below is. Bloom spreads brightness, so overdriving the radiance
/// inflates the glow around the sun until it dominates the sky.
const float SUN_DISC_RADIANCE = 24.0;

/// How much wider than life the disc is drawn.
///
/// The real sun is about half a degree across, which at normal fields of view
/// is a few pixels and shimmers as the camera turns. A little larger holds
/// together; much larger stops reading as a disc at all.
const float SUN_DISC_WIDENING = 2.0;

float rayleighPhase(float cos_theta) {
    return (3.0 / (16.0 * SKY_PI)) * (1.0 + cos_theta * cos_theta);
}

/// Henyey-Greenstein phase function for Mie scattering.
float miePhase(float cos_theta, float g) {
    float g2 = g * g;
    float denom = 1.0 + g2 - 2.0 * g * cos_theta;
    return (1.0 / (4.0 * SKY_PI)) * (1.0 - g2) / (denom * sqrt(denom));
}

/// Sky colour from atmospheric scattering, before the sun disc is added.
///
/// Continuous over the whole sphere, including below the horizon, where it
/// settles to a dim ground tone. That continuity is what lets reflection and
/// ambient queries pass any direction in without a special case.
vec3 skyScattering(vec3 ray_dir, vec3 sun_dir) {
    float sun_dot = dot(ray_dir, sun_dir);
    float sun_height = sun_dir.y;

    // The exponent controls how tightly the pale band hugs the horizon. Low
    // values bleed it far up the dome and leave the whole sky milky, which also
    // washes out everything the sky lights.
    float horizon_factor = pow(1.0 - abs(ray_dir.y), 6.0);

    vec3 zenith_colour = vec3(0.16, 0.38, 0.92);
    vec3 horizon_colour = vec3(0.55, 0.76, 0.95);
    vec3 sunset_colour = vec3(1.0, 0.6, 0.3);

    float sun_influence = max(0.0, sun_height);
    float sunset_factor = smoothstep(0.0, 0.2, sun_height)
                        * (1.0 - smoothstep(0.2, 0.5, sun_height))
                        * horizon_factor;

    vec3 rayleigh_colour = RAYLEIGH_COEFF * rayleighPhase(sun_dot) * 3000.0;
    vec3 mie_colour = vec3(1.0, 0.95, 0.85) * miePhase(sun_dot, 0.76) * 0.03
                    * max(0.0, sun_height + 0.1);

    vec3 colour = mix(zenith_colour, horizon_colour, horizon_factor);
    colour = mix(colour, sunset_colour, sunset_factor * 0.5);
    colour += rayleigh_colour * sun_influence;
    colour += mie_colour;
    colour *= 1.0 + 0.2 * sun_influence;

    float below_horizon = smoothstep(0.0, -0.1, ray_dir.y);
    colour = mix(colour, vec3(0.15, 0.2, 0.25), below_horizon);

    return colour;
}

/// Pale brightening in the band just above and below the horizon.
vec3 horizonHaze(vec3 ray_dir, vec3 sun_dir) {
    float horizon_factor = pow(1.0 - abs(ray_dir.y), 12.0);

    vec3 haze_colour = vec3(0.85, 0.9, 0.95);

    // Warmer looking towards the sun, judged on the horizontal bearing alone so
    // the warmth wraps the horizon rather than tracking the sun's elevation.
    float bearing = max(0.0, dot(normalize(vec3(ray_dir.x, 0.0, ray_dir.z)),
                                 normalize(vec3(sun_dir.x, 0.0, sun_dir.z))));
    haze_colour = mix(haze_colour, vec3(1.0, 0.95, 0.85), bearing * 0.3);

    return haze_colour * horizon_factor * 0.08;
}

/// Sky radiance along a direction, excluding the solar disc.
///
/// This is what surfaces should sample. The disc is left out because direct
/// sunlight already reaches geometry as a directional light, and a surface that
/// also reflected the disc would count the sun twice.
vec3 skyRadiance(vec3 ray_dir, vec3 sun_dir) {
    return (skyScattering(ray_dir, sun_dir) + horizonHaze(ray_dir, sun_dir))
         * SKY_RADIANCE_SCALE;
}

/// The solar disc and the glow around it, for the sky pass only.
vec3 sunDiscRadiance(vec3 ray_dir, vec3 sun_dir) {
    float cos_angle = clamp(dot(ray_dir, sun_dir), -1.0, 1.0);
    float angle = acos(cos_angle);

    float disc_radius = SUN_ANGULAR_RADIUS * SUN_DISC_WIDENING;
    float soft_edge = SUN_ANGULAR_RADIUS * 0.5;

    float disc = 1.0 - smoothstep(disc_radius - soft_edge, disc_radius, angle);
    vec3 radiance = vec3(1.0, 0.97, 0.92) * disc * SUN_DISC_RADIANCE;

    float inner_glow = 1.0 - smoothstep(disc_radius, disc_radius * 3.0, angle);
    radiance += vec3(1.0, 0.8, 0.5) * inner_glow * SUN_DISC_RADIANCE * 0.06;

    float corona = 1.0 - smoothstep(disc_radius, disc_radius * 10.0, angle);
    radiance += vec3(1.0, 0.6, 0.3) * corona * SUN_DISC_RADIANCE * 0.015;

    float atmospheric_glow = pow(max(0.0, cos_angle), 8.0);
    radiance += vec3(1.0, 0.7, 0.4) * atmospheric_glow * 0.45
              * max(0.0, sun_dir.y + 0.2);

    return radiance;
}

#endif // SKY_MODEL_GLSL
