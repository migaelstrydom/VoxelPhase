#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#include "tonemap.glsl"

layout(location = 0) in vec4 fragColor;
layout(location = 1) in float fragLife;   // 0 = just born, 1 = about to die
layout(location = 2) in vec2 fragUV;
layout(location = 3) in vec4 fragShape;   // (rotation, additive, billow, seed)

layout(location = 0) out vec4 outColor;

// Matches the vertex stage's two matrices, which occupy the first 128 bytes.
layout(push_constant) uniform PushConstants {
    layout(offset = 128) float exposure;
} pc;

// How many times the noise repeats across the billboard. Low: the puff should
// be shaped by a couple of large lobes, not by a field of speckles.
const float BILLOW_FREQUENCY = 1.9;

// Fraction of the particle eaten away by erosion by the end of its life. Fully
// billowed particles dissolve into wisps rather than fading as a whole disc,
// which is what a cloud thinning out actually looks like.
const float MAX_EROSION = 0.55;

// Softness of the eroded edge, as a fraction of density. Too sharp reads as a
// cut-out; too soft and the erosion stops being visible at all.
const float EDGE_SOFTNESS = 0.22;

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

// Bilinear value noise. Cheaper than gradient noise and indistinguishable once
// it is only being used to tear the edge off a billboard.
float valueNoise(vec2 p) {
    vec2 cell = floor(p);
    vec2 f = fract(p);
    vec2 weight = f * f * (3.0 - 2.0 * f);

    float a = hash21(cell);
    float b = hash21(cell + vec2(1.0, 0.0));
    float c = hash21(cell + vec2(0.0, 1.0));
    float d = hash21(cell + vec2(1.0, 1.0));

    return mix(mix(a, b, weight.x), mix(c, d, weight.x), weight.y);
}

// Two octaves: one for the overall lobes, one for the ragged edge. A third
// costs another four hashes per fragment and an explosion is the one moment
// the frame can least afford them.
float fbm(vec2 p) {
    return 0.70 * valueNoise(p) + 0.30 * valueNoise(p * 2.4 + 17.0);
}

void main() {
    float rotation = fragShape.x;
    float additive = fragShape.y;
    float billow = fragShape.z;
    float seed = fragShape.w;

    // Rotate the mask, not the quad: the billboard stays camera-facing while
    // its shape spins, which costs two trig calls instead of a second basis.
    vec2 centered = fragUV * 2.0 - 1.0;
    float s = sin(rotation);
    float c = cos(rotation);
    vec2 p = vec2(c * centered.x - s * centered.y, s * centered.x + c * centered.y);

    // Soft circular falloff — the shape every particle starts from.
    float disc = 1.0 - smoothstep(0.25, 1.0, length(p));

    // Corners of the quad, and every particle drawn as a clean disc, leave
    // before paying for the noise. An explosion is exactly when the frame can
    // least afford eight hashes it will discard, and both branches are uniform
    // across a whole billboard, so they cost nothing to take.
    if (disc <= 0.0) {
        discard;
    }

    float mask = disc;
    if (billow > 0.0) {
        // Break the disc up into lobes. `seed` decorrelates overlapping puffs,
        // so a burst does not read as one shape stamped repeatedly.
        float noise = fbm(p * BILLOW_FREQUENCY + seed * 31.7);
        float density = mix(disc, disc * (0.35 + 1.3 * noise), billow);

        // Erosion rises with age: the thinnest parts of the puff go first and
        // it comes apart, rather than the whole billboard dimming uniformly.
        float erosion = billow * fragLife * MAX_EROSION;
        mask = clamp((density - erosion) / max(EDGE_SOFTNESS, 1.0 - erosion), 0.0, 1.0);
    }

    float alpha = fragColor.a * mask;
    if (alpha < 0.004) {
        discard;
    }

    // Premultiplied output, so one pipeline draws both glowing and occluding
    // particles: colour is always added, and the alpha written back is what
    // the destination is attenuated by. Emitting zero alpha therefore adds
    // light without darkening anything behind it.
    //
    // How additive a particle is falls out of how bright it currently is, not
    // just how it was authored: a fireball puff that has cooled to dark soot
    // stops adding light and starts blocking it, with no second effect and no
    // handover to arrange.
    float glow = clamp(max(fragColor.r, max(fragColor.g, fragColor.b)), 0.0, 1.0);
    float blend = additive * glow;

    // Effect ramps author colour well above 1.0, and what that has always
    // meant on screen is a clipped one: the hot end of a fireball reads white
    // because each channel saturates, which is the bleach a filmic curve would
    // otherwise have to provide. The scene's tonemap preserves hue instead, so
    // handing it the raw ramp value keeps the chroma and the core comes out
    // orange — the same fire, wrong temperature.
    //
    // So the clip is applied here, where the authored intent is, and the result
    // converted to the radiance that resolves back to it. The particle looks
    // the way it was tuned; being in the scene target is what lets the glass in
    // front of it tint it.
    vec3 display = min(fragColor.rgb, vec3(1.0));
    vec3 radiance = sceneRadianceFor(display, pc.exposure);

    outColor = vec4(radiance * alpha, alpha * (1.0 - blend));
}
