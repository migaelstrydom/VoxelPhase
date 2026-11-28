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
/// inflates the glow around the sun until it dominates the sky. Bloom holds
/// every pixel to `PostProcessConfig::bloom_ceiling` (32), set just above
/// this: raising one means raising the other.
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

/// How far the sun has sunk towards the horizon: 0 for a sun above about 43
/// degrees, which leaves the sky its noon colours, rising to 1 for a sun below
/// about 24.
float lowSun(float sun_height) {
    return 1.0 - smoothstep(0.40, 0.68, sun_height);
}

/// How nearly a ray looks along the sun's compass bearing: 1 towards it, 0 at
/// right angles and beyond. Judged on the horizontal alone, so what it shades
/// wraps the horizon rather than tracking the sun's elevation.
float sunBearing(vec3 ray_dir, vec3 sun_dir) {
    vec2 ray = ray_dir.xz / max(length(ray_dir.xz), 1e-4);
    vec2 sun = sun_dir.xz / max(length(sun_dir.xz), 1e-4);
    return max(0.0, dot(ray, sun));
}

/// The sky under a low sun: a gold horizon on the sun's side, rose round the
/// rest of it, and a softer blue overhead.
const vec3 LOW_SUN_ZENITH = vec3(0.20, 0.38, 0.80);
const vec3 LOW_SUN_HORIZON_TOWARDS = vec3(1.05, 0.74, 0.40);
const vec3 LOW_SUN_HORIZON_AWAY = vec3(0.78, 0.64, 0.66);

/// How tightly the pale band hugs the horizon, as the exponent of its falloff
/// with height: at noon, and under a low sun, whose glow climbs the dome.
const float HORIZON_BAND_NOON = 8.0;
const float HORIZON_BAND_LOW_SUN = 3.5;

/// Sky colour from atmospheric scattering, before the sun disc is added.
///
/// Continuous over the whole sphere, including below the horizon, where it
/// settles to a dim ground tone. That continuity is what lets reflection and
/// ambient queries pass any direction in without a special case.
vec3 skyScattering(vec3 ray_dir, vec3 sun_dir) {
    float sun_dot = dot(ray_dir, sun_dir);
    float sun_height = sun_dir.y;

    float low_sun = lowSun(sun_height);

    // The exponent controls how tightly the pale band hugs the horizon. Low
    // values bleed it far up the dome and leave the whole sky milky, which also
    // washes out everything the sky lights.
    float horizon_factor = pow(1.0 - abs(ray_dir.y),
                               mix(HORIZON_BAND_NOON, HORIZON_BAND_LOW_SUN, low_sun));

    vec3 zenith_colour = vec3(0.16, 0.38, 0.92);
    // Saturated rather than pale. A gameplay camera looks along the horizon
    // almost all the time, so this band is most of the sky that is ever on
    // screen, and a washed-out one gives the greens and reds in front of it
    // nothing to sit against. It also has to stay darker than sunlit ground for
    // the ground to read as lit.
    vec3 horizon_colour = vec3(0.40, 0.66, 0.96);
    vec3 sunset_colour = vec3(1.0, 0.6, 0.3);

    vec3 low_sun_horizon = mix(LOW_SUN_HORIZON_AWAY, LOW_SUN_HORIZON_TOWARDS,
                               sunBearing(ray_dir, sun_dir));
    zenith_colour = mix(zenith_colour, LOW_SUN_ZENITH, low_sun);
    horizon_colour = mix(horizon_colour, low_sun_horizon, low_sun);

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

    // Warmer looking towards the sun.
    haze_colour = mix(haze_colour, vec3(1.0, 0.95, 0.85),
                      sunBearing(ray_dir, sun_dir) * 0.3);

    return haze_colour * horizon_factor * 0.05;
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
