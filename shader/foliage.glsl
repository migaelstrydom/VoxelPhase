// A surface lit as a mass of leaves (`SurfaceSource::FOLIAGE`).
//
// Three things set foliage apart from a solid of the same colour, and the
// mesh can carry none of them:
//
//   leaf normal   individual leaves, finer than any vertex, tilted every
//                 which way: a normal perturbed by noise at leaf scale, fixed
//                 to the clump's own frame so it does not swim as it moves
//   wrap          light reaching round into the clump through the gaps
//                 between leaves: diffuse that fades past the terminator
//                 instead of stopping at it
//   transmission  the sun through thin leaves at the clump's edge, seen
//                 from the shadowed side: the glow round a backlit bush

#ifndef FOLIAGE_GLSL
#define FOLIAGE_GLSL

// Leaves per metre along any line through the clump.
const float LEAF_FREQUENCY = 26.0;
// How far a leaf's normal tilts from the clump's, at most.
const float LEAF_TILT = 0.55;
// How far past the terminator diffuse light reaches, as a share of the
// hemisphere: 0 is a solid's, 1 lights the whole clump.
const float LEAF_WRAP = 0.45;
// How tightly the transmitted glow gathers round the sun's direction.
const float TRANSMISSION_POWER = 6.0;
// How much of the sun comes through a leaf's edge.
const float TRANSMISSION_STRENGTH = 0.9;
// Share of the sun's wrap and transmission let through where the shadow map
// says the clump shadows itself.
const float FOLIAGE_SELF_SHADOW = 0.4;
// Colour of light through a leaf, relative to its albedo: warmer and
// lighter, as chlorophyll passes yellow-green.
const vec3 TRANSMISSION_TINT = vec3(1.6, 1.8, 0.9);

float foliageHash(vec3 p) {
    p = fract(p * 0.3183099 + vec3(0.71, 0.113, 0.419));
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

// Trilinear value noise.
float foliageNoise(vec3 p) {
    vec3 cell = floor(p);
    vec3 f = fract(p);
    vec3 w = f * f * (3.0 - 2.0 * f);
    float n000 = foliageHash(cell);
    float n100 = foliageHash(cell + vec3(1, 0, 0));
    float n010 = foliageHash(cell + vec3(0, 1, 0));
    float n110 = foliageHash(cell + vec3(1, 1, 0));
    float n001 = foliageHash(cell + vec3(0, 0, 1));
    float n101 = foliageHash(cell + vec3(1, 0, 1));
    float n011 = foliageHash(cell + vec3(0, 1, 1));
    float n111 = foliageHash(cell + vec3(1, 1, 1));
    return mix(
        mix(mix(n000, n100, w.x), mix(n010, n110, w.x), w.y),
        mix(mix(n001, n101, w.x), mix(n011, n111, w.x), w.y),
        w.z);
}

// The clump's normal tilted as the leaf at `model_pos` faces: by the
// noise's gradient across the surface, carried into the world by
// `model_to_world`.
vec3 leafNormal(vec3 normal, vec3 model_pos, mat3 model_to_world) {
    vec3 p = model_pos * LEAF_FREQUENCY;
    const float h = 0.35;
    float n = foliageNoise(p);
    vec3 gradient = vec3(
        foliageNoise(p + vec3(h, 0, 0)) - n,
        foliageNoise(p + vec3(0, h, 0)) - n,
        foliageNoise(p + vec3(0, 0, h)) - n) / h;
    vec3 world_gradient = model_to_world * gradient;
    vec3 across = world_gradient - normal * dot(world_gradient, normal);
    return normalize(normal - across * LEAF_TILT);
}

// The sun's diffuse on leaves beyond what a solid's `max(n·l, 0)` gives:
// the wrapped share past the terminator.
float leafWrapExtra(vec3 normal, vec3 light_dir) {
    float n_dot_l = dot(normal, light_dir);
    float wrapped = max((n_dot_l + LEAF_WRAP) / (1.0 + LEAF_WRAP), 0.0);
    return max(wrapped - max(n_dot_l, 0.0), 0.0);
}

// Sun through the leaves towards the eye: strongest looking towards the
// sun, at the clump's edge where the leaves are thinnest.
vec3 leafTransmission(vec3 albedo, vec3 normal, vec3 view_dir, vec3 light_dir, vec3 radiance) {
    float towards_sun = pow(max(dot(view_dir, -light_dir), 0.0), TRANSMISSION_POWER);
    float edge = 1.0 - max(dot(normal, view_dir), 0.0);
    return albedo * TRANSMISSION_TINT * radiance
         * (towards_sun * edge * edge * TRANSMISSION_STRENGTH);
}

#endif // FOLIAGE_GLSL
