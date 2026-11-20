#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "ripple.glsl"
#include "swell.glsl"
#include "chop.glsl"
#include "aerial.glsl"

// --- Tuning constants ---
const float REFRACTION_STRENGTH = 0.01;
const float SSS_INTENSITY = 1.9;
// Extinction of light along a ray through the water body, per metre and per
// channel. Red goes first, then green: what lies under the water turns from
// its own colour through turquoise and teal to the water's own blue as the
// ray through the water lengthens. Red goes at half the rate of real water's:
// an eye adapts to the light around it and does not see the full shift, and
// at the real rate a red fish a metre down reads green, a white floor reads as
// a swimming pool's and a blast's flash on it turns cyan.
const vec3  WATER_EXTINCTION = vec3(0.12, 0.06, 0.05);
// Light the water body scatters back towards the eye, where the ray through it
// is long enough that nothing behind shows: deep water's own colour.
const vec3  WATER_SCATTER = vec3(0.02, 0.08, 0.17);
const vec3  SSS_COLOR = vec3(0.15, 0.5, 0.4);
// How long the noise is carried by the flow before it starts over, s. Two
// phases half a period apart cross-fade, each faded out as it starts over,
// so the noise is never carried further than a period's flow: carried for
// ever, the small currents of still water would shear it into swirls.
const float FLOW_PERIOD = 2.0;
// Steepest a ripple may tilt the normal: past this the surface is noise.
const float MAX_RIPPLE_SLOPE = 1.0;
// How fast falling water's two layers of streaks run down it, m/s.
const float SHEET_FALL_SPEED = 4.0;
const float SHEET_FALL_SPEED_FINE = 6.5;
// How far water falls clear and glassy before it is white water, m.
const float SHEET_GLASS_DROP = 0.8;
// How far above its foot falling water breaks up into the spray it lands in,
// m.
const float SHEET_FOOT_FADE = 0.7;
// How thick falling glass is when it is two-thirds opaque, m.
const float SHEET_GLASS_THICKNESS = 0.15;
// How far light wraps round a falling curtain past its terminator: white
// water scatters it, so its shadowed side is only dimmer.
const float SHEET_WRAP = 0.4;
// How much of the light falling on the far side of a curtain comes through
// it: seen from under, white water is lit from above.
const float SHEET_THROUGH = 0.8;
// Most of the sky a falling sheet mirrors where it is seen edge on. Its face
// is ruffled; mirrored whole, the line along which it turns away from the eye
// is drawn across the fall as a bright wire.
const float SHEET_MAX_MIRROR = 0.25;
// How far the ropes of white water stand out of a curtain, as the slope
// their streaks tilt its face by.
const float SHEET_RELIEF = 0.9;
// Most a patch of foam where falling water lands covers what is under it.
const float PLUNGE_OPACITY = 0.8;
// Pour depth at which falling water is a full curtain, not a few strands, m.
const float SHEET_FULL_POUR = 0.12;
// Whether the pipeline only lays the depth of falling water where it is
// solid enough to hide what is behind it, before the sheets are blended:
// then the nearest solid fall hides those behind it, whatever order they
// are drawn in. Clearer water is blended over what is behind it unhidden.
layout(constant_id = 0) const bool SHEET_DEPTHS = false;
// How opaque falling water must be to hide another fall behind it.
const float SHEET_SOLID = 0.6;
// Share of the way from a fall's middle out to its frayed end past which it
// frays away.
const float FRAY_SHARE = 0.5;
// Foam the water carries (`fragAir` on the surface): how far past covering
// the water it must lie to cover it whole, and over what share of the
// billows' range its patches' edges fade.
const float RIDING_FOAM_FULL = 1.2;
const float RIDING_FOAM_EDGE = 0.4;
// Depths of water over which it comes to carry its foam, m.
const vec2 RIDING_FOAM_DEPTH = vec2(0.03, 0.2);
// Water churned full of air under its foam: its colour, and how far the
// water under the thickest foam is turned to it.
const vec3 AERATED_COLOR = vec3(0.62, 0.86, 0.88);
const float AERATED_SHARE = 0.4;
// Gravity, m/s²: how long falling water has taken to drop as far as it has.
const float FALL_GRAVITY = 9.81;
// How far the head and the tail of a fall's water are torn up and down it, s
// of falling, and over how long of it each fades in.
const float FLYING_RAGGED = 0.16;
const float FLYING_EDGE = 0.03;
// How long behind its head a new fall's water is still broken white, s.
const float FLYING_HEAD = 0.3;

// --- Soft edges ---
// Length of view ray through the water over which a surface fades in where
// it meets what is behind it, m: the waterline against a bank, a body, the
// thin film of an advancing front.
const float CONTACT_SOFTNESS = 0.04;
// Ground over which a surface fades in past its shore depth, m, and the
// least wet depth it fades over: a shoreline across a cell is a contour of
// depths interpolated over its triangles, so it would otherwise show their
// edges. Measured on the ground, it is as soft over deep water running into
// a bank as over a thin film. A film thinner than the least reads as wet
// ground rather than a surface: on a steep face its cells' quads would
// stand out from the slope as slivers.
const float SHORE_SOFTNESS = 0.12;
const float SHORE_SOFTNESS_DEPTH = 0.04;
// How far noise moves both, as a share of their width: enough to lose the
// triangles in a wandering, lapping edge.
const float EDGE_WANDER = 1.2;
// Ground over which the edge carries a frill of foam, m: where the water
// thins out over a beach or an advancing front, not up a wall.
const float EDGE_FOAM_REACH = 0.35;
// Slopes of the ground under the water, rise over run, over which its edge
// stops frothing: a beach froths, a wall does not.
const vec2 FROTHING_SLOPE = vec2(0.4, 1.0);
// Depths of water, as the field has them, over which a surface stops taking
// its depth from what is behind it, m.
const vec2 GROUND_TRUSTED_DEPTH = vec2(0.15, 0.3);
// How far the ground behind a surface the field has shallow may lie below
// the field's depth before the surface is taken to stand in the air, m,
// fading out over the second: a film on a bank steeper than its column can
// draw, seen against the slope below it.
const vec2 LIFTED_GAP = vec2(0.05, 0.12);
// Wet depth under which a surface is near enough an edge to wander and carry
// its foam, m: past it neither is ever seen however steep the shore, so
// neither is worked out.
const float FOAMY_DEPTH = 0.6;
// Speeds over which running water turns to lace of white water, m/s.
const vec2 WHITE_WATER_SPEED = vec2(1.2, 3.5);
// How far white water's lace is drawn out along the current.
const float LACE_STRETCH = 3.0;
// Fastest the flow carries the noise and foam, m/s. A pattern carried by a
// flow that differs from pixel to pixel is sheared by the difference times
// the period; past this speed water reads as fast by its streaks, and the
// shear would only tear them.
const float MAX_CARRY_SPEED = 1.5;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragWorldPos;
// The ripple tile under this fragment, or -1 where none is awake, and the
// tile's corner.
layout(location = 2) flat in int fragLayer;
layout(location = 3) flat in vec2 fragTileOrigin;
// A river surface's flow velocity, its depth of water here and the depths it
// is drawn over; zero flow and an unbounded range for still water.
layout(location = 4) in vec2 fragFlow;
layout(location = 5) in float fragWetDepth;
layout(location = 6) flat in vec2 fragWetRange;
// The swell under this fragment: (amplitude with its shore fade, phase,
// clock, the wind sea's peak wavelength from wind.glsl); zero amplitude where
// there is none.
layout(location = 7) in vec4 fragSwell;
// Whether this is a tile's own fine grid, which covers the whole tile and is
// masked to its body's columns.
layout(location = 8) flat in int fragMasked;
// On falling water: how far below its lip and its whole fall, m; which side
// of its curtain's middle, -1 to 1 between the ends of a fall's lip; and how
// thick the water is, m. On the foam where it lands the first is its fall
// and 1 more and where across the foam, -1 behind its foot to 1 at its outer
// edge. Zero on a surface. There `fragFlow` is where its water left its lip,
// `fragFace` the way it flies, level, `fragFray` how far an end of the fall
// frays out, and `fragAir` how much air it had taken in as it left its lip.
layout(location = 9) in vec4 fragSheet;
layout(location = 10) in float fragFray;
layout(location = 11) in vec2 fragFace;
layout(location = 12) in float fragAir;
// On falling water, how long ago its fall began and how long ago it stopped
// pouring, s (negative while it pours): only the stretch of it its water
// has reached, and has not yet left, is drawn.
layout(location = 13) in vec2 fragLife;

// The scene beyond the water, copied before the water is drawn (sampled at
// offset UVs for refraction)
layout(set = 0, binding = 0) uniform sampler2D colorSampler;

// Its depth, copied with it (sampled at both current and refracted UVs)
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 160) vec4 cameraPos;     // xyz, w = aerial visibility
    layout(offset = 176) vec4 sunDir;
    layout(offset = 192) vec4 projParams;    // (near, far, time, unused)
    layout(offset = 208) vec4 screenParams;  // (width, height, unused, unused)
} fpc;

layout(location = 0) out vec4 outColor;

// Linearize a [0,1] depth buffer value to a view-space distance.
float linearizeDepth(float d, float near, float far) {
    return near * far / (far - d * (far - near));
}

// Where the scene behind a water surface lies, seen through the point
// `surface` of it `surfaceDist` from the eye (view depth) at `uv`.
vec3 behindSurface(vec2 uv, vec3 surface, float surfaceDist, float near, float far) {
    float behindDist = linearizeDepth(texture(depthSampler, uv).r, near, far);
    return fpc.cameraPos.xyz + (surface - fpc.cameraPos.xyz) * (behindDist / surfaceDist);
}

// How steeply the ground behind a surface rises, rise over run, from the
// depth copy one pixel either way along each screen axis. Each axis takes
// the side nearer in depth, so across the silhouette of a body or a bank in
// front of the water it reads the ground on its own side, never the jump.
// The surface itself is smooth, so its neighbours come from its derivatives.
float groundSlope(vec2 uv, vec3 surface, float surfaceDist, vec3 behind, float near, float far) {
    vec2 pixel = 1.0 / fpc.screenParams.xy;
    float steepest = 0.0;
    for (int axis = 0; axis < 2; axis++) {
        vec2 step = axis == 0 ? vec2(pixel.x, 0.0) : vec2(0.0, pixel.y);
        vec3 surfaceStep = axis == 0 ? dFdx(surface) : dFdy(surface);
        float distStep = axis == 0 ? dFdx(surfaceDist) : dFdy(surfaceDist);
        vec3 ahead = behindSurface(uv + step, surface + surfaceStep, surfaceDist + distStep, near, far);
        vec3 back = behindSurface(uv - step, surface - surfaceStep, surfaceDist - distStep, near, far);
        vec3 toward = distance(ahead, behind) < distance(back, behind) ? ahead - behind : behind - back;
        steepest = max(steepest, abs(toward.y) / max(length(toward.xz), 1.0e-4));
    }
    return steepest;
}

// Hash-based pseudo-random for procedural noise (no texture needed).
vec2 hash2(vec2 p) {
    p = vec2(dot(p, vec2(127.1, 311.7)),
             dot(p, vec2(269.5, 183.3)));
    return fract(sin(p) * 43758.5453);
}

// Value noise with smooth interpolation.
float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);

    float a = dot(hash2(i + vec2(0.0, 0.0)), vec2(1.0));
    float b = dot(hash2(i + vec2(1.0, 0.0)), vec2(1.0));
    float c = dot(hash2(i + vec2(0.0, 1.0)), vec2(1.0));
    float d = dot(hash2(i + vec2(1.0, 1.0)), vec2(1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Two-octave FBM for richer detail.
float fbm(vec2 p) {
    float v = 0.0;
    v += 0.5 * valueNoise(p);
    v += 0.25 * valueNoise(p * 2.0 + vec2(1.7, 9.2));
    return v;
}

// Foam's lace: the borders between drifting cells, as bubbles crowd. One at
// the borders, falling to zero within `width` of them.
float foamLace(vec2 p, float time, float width) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    float nearest = 8.0;
    float second = 8.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            vec2 g = vec2(float(x), float(y));
            vec2 o = hash2(i + g);
            o = 0.5 + 0.45 * sin(time * 0.8 + 6.2831 * o);
            vec2 r = g + o - f;
            float d = dot(r, r);
            if (d < nearest) {
                second = nearest;
                nearest = d;
            } else if (d < second) {
                second = d;
            }
        }
    }
    return 1.0 - smoothstep(0.0, width, sqrt(second) - sqrt(nearest));
}

// Lace drawn out along the current. A frame turned to the flow at every
// pixel would carry the pattern round the world's origin wherever the flow
// turns, and stretch it from there wherever it speeds up: over a breach it
// drew a map's contours. So the streaks are laid in fixed frames an eighth
// of a turn apart, all at one stretch, and the two nearest the flow fade
// into each other.
float streakedLace(vec2 p, vec2 downstream, float time) {
    const float EIGHTH = 0.78539816;
    // The flow's bearing in eighths of a turn; a streak has no head or tail,
    // so half a turn on is the same frame.
    float bearing = mod(atan(downstream.y, downstream.x) / EIGHTH, 4.0);
    float first = floor(bearing);
    float blend = smoothstep(0.25, 0.75, bearing - first);
    float lace = 0.0;
    for (int k = 0; k < 2; k++) {
        float weight = k == 0 ? 1.0 - blend : blend;
        if (weight <= 0.0) {
            continue;
        }
        float frame = mod(first + float(k), 4.0);
        float angle = frame * EIGHTH;
        vec2 along = vec2(cos(angle), sin(angle));
        vec2 laid = vec2(dot(p, along) / LACE_STRETCH, dot(p, vec2(-along.y, along.x)));
        lace += weight * foamLace(laid + 5.7 * frame, time, 0.4);
    }
    return lace;
}

// Sample a normal perturbation from procedural noise at a given position.
// Uses central differences on the noise heightfield.
vec3 proceduralNormal(vec2 pos, float scale, float strength) {
    float eps = 0.1;
    float h  = fbm(pos * scale);
    float hx = fbm((pos + vec2(eps, 0.0)) * scale);
    float hz = fbm((pos + vec2(0.0, eps)) * scale);
    float dx = (hx - h) / eps * strength;
    float dz = (hz - h) / eps * strength;
    return normalize(vec3(-dx, 1.0, -dz));
}

// Where the scene seen through the water at `screenUV` is looked up, for
// something `opticalDepth` metres of view ray beyond the surface. Nothing
// under 0.5 m shifts: at that depth the offset would be too small to clear
// an object's silhouette, and would only fringe it.
vec2 refractedLookup(vec2 screenUV, vec3 N, float opticalDepth) {
    float refractionDepth = smoothstep(0.5, 2.0, opticalDepth) * 3.0;
    vec2 offset = N.xz * REFRACTION_STRENGTH * refractionDepth;
    return clamp(screenUV + offset, vec2(0.001), vec2(0.999));
}

// The streaks of falling water whose water left its lip at `lip`, flying the
// way `facing` does, at height `y`: a coarse layer and a fine one, each
// falling at its own speed. They hang from the lip, so they run straight down
// a curtain however it is thrown, and are laid along whichever of the world's
// axes the curtain runs along (both, faded into each other, for one running
// between them), so they run on from each fall to the next round any lip.
vec2 sheetStreaks(vec2 lip, vec2 facing, float y, float time) {
    vec2 weight = facing * facing;
    weight /= max(weight.x + weight.y, 1.0e-4);
    // How far along the lip one pixel covers: seen edge on, a curtain's
    // streaks are finer than its pixels, and are left out before they alias.
    vec2 grain = fwidth(lip);
    vec2 streaks = vec2(0.0);
    for (int axis = 0; axis < 2; axis++) {
        // A curtain facing along x runs along z, and the other way about.
        float share = axis == 0 ? weight.x : weight.y;
        if (share < 0.03) {
            continue;
        }
        float along = axis == 0 ? lip.y : lip.x + 31.7;
        float pixel = axis == 0 ? grain.y : grain.x;
        vec2 laid = vec2(
            fbm(vec2(along * 6.0, (y + time * SHEET_FALL_SPEED) * 0.35)),
            fbm(vec2(along * 14.0 + 3.1, (y + time * SHEET_FALL_SPEED_FINE) * 0.7))) / 1.5;
        vec2 blurred = smoothstep(vec2(0.25), vec2(0.6), pixel * vec2(6.0, 14.0));
        streaks += share * mix(laid, vec2(0.5), blurred);
    }
    return streaks;
}

// The face normal `N` tilted by the slope of a height `h` laid over the
// surface, from its screen-space derivatives (Mikkelsen 2010): relief with
// no second look at the height.
vec3 embossed(vec3 N, float h, float scale) {
    vec3 across = dFdx(fragWorldPos);
    vec3 down = dFdy(fragWorldPos);
    vec3 r1 = cross(down, N);
    vec3 r2 = cross(N, across);
    float area = dot(across, r1);
    vec3 slope = sign(area) * (dFdx(h) * r1 + dFdy(h) * r2);
    return normalize(abs(area) * N - scale * slope);
}

// How much of a fall's water is there `seconds` into its fall, 0 to 1: none
// ahead of its first water, which has fallen only as long as the fall has
// run, and none behind its last, which left the lip when it stopped. Each
// end is `ragged` (-0.5 to 0.5) by a share of a second, so the water's head
// and tail are torn, not cut.
float flying(float seconds, float ragged) {
    float torn = ragged * FLYING_RAGGED;
    float head = fragLife.x - seconds + torn;
    float tail = seconds - fragLife.y + torn;
    return smoothstep(0.0, FLYING_EDGE, head) * smoothstep(0.0, FLYING_EDGE, tail);
}

// Falling water at a fragment, as its look and how much it hides both take
// it.
struct Sheet {
    // How far it has fallen from its lip, and how far its foot lies below
    // it, m.
    float drop;
    float fall;
    // Where across its lip it lies, 0 in the middle to 1 at an end.
    float across;
    // The face seen: the pipeline culls the one turned away.
    vec3 face;
    // Its streaks: the coarse layer, and both layers blended into ropes.
    float coarse;
    float strands;
    // How long its water has been falling, s.
    float seconds;
    // How white it is, 0 for glass to 1 for white water.
    float aerated;
    // How full a curtain it pours, and how much of its glass's depth it
    // has, 0 to 1.
    float pour;
    float body;
};

Sheet sheetHere(float time) {
    Sheet s;
    s.drop = max(fragSheet.x, 0.0);
    s.fall = max(fragSheet.y, 1.0e-3);
    s.across = abs(fragSheet.z);
    s.face = normalize(fragNormal);
    vec2 facing = dot(fragFace, fragFace) > 1.0e-6 ? normalize(fragFace) : vec2(1.0, 0.0);
    vec2 streaks = sheetStreaks(fragFlow, facing, fragWorldPos.y, time);
    s.coarse = streaks.x;
    s.strands = 0.6 * streaks.x + 0.4 * streaks.y;

    // It takes in air as it falls; a jet bursts out white already, and so
    // does the head of water that has only just begun to fall.
    s.seconds = sqrt(2.0 * s.drop / FALL_GRAVITY);
    float head = 1.0 - smoothstep(0.0, FLYING_HEAD, fragLife.x - s.seconds);
    float air = max(clamp(fragAir, 0.0, 1.0), head);
    // Its under face springs clear of the ledge torn and white: only its
    // top slides over the lip as glass.
    bool under = s.face.y < -1.0e-3;
    air = under ? 1.0 : air;
    s.aerated = 1.0 - (1.0 - air) * (1.0 - smoothstep(0.0, SHEET_GLASS_DROP, s.drop));
    s.pour = smoothstep(0.0, SHEET_FULL_POUR, fragWetDepth);
    // Thick glass is deep green and hides what is behind it.
    float thickness = max(fragSheet.w, 0.0);
    s.body = 1.0 - exp(-thickness / SHEET_GLASS_THICKNESS);
    return s;
}

// How much of what is behind falling water it hides, 0 to 1: its alpha,
// with `fresnel` of the sky mirrored in its glass, `behindDepth` m in front
// of what is behind it.
float sheetCover(Sheet s, float fresnel, float behindDepth) {
    // Glass is clear where it is thin and deep where it is thick; white
    // water is ropes with air between them, closing up where much pours and
    // where the body is thick.
    float glassAlpha = min(mix(0.25, 0.96, s.body) + 0.25 * fresnel, 1.0) * mix(0.4, 1.0, s.pour);
    float full = max(s.pour, s.body);
    float strandAlpha = smoothstep(0.62 - 0.4 * full, 0.82 - 0.35 * full, s.strands + 0.12 * s.aerated);
    float alpha = mix(glassAlpha, mix(0.92, 0.98, s.body) * strandAlpha, s.aerated);
    // It breaks up over the white water at its foot.
    alpha *= smoothstep(0.0, SHEET_FOOT_FADE * (0.5 + s.coarse), s.fall - s.drop);
    // A frayed end thins out into strands and is gone.
    float frayed = s.across + (s.coarse - 0.5) * FRAY_SHARE;
    alpha *= 1.0 - fragFray * smoothstep(1.0 - FRAY_SHARE, 1.0, frayed);
    // Where it runs against rock, it fades into it rather than cutting it.
    alpha *= smoothstep(0.0, 0.12, behindDepth);
    alpha *= flying(s.seconds, s.strands - 0.5);
    return alpha;
}

// Water falling over an edge, a body with a top face and an under face:
// clear glass where it leaves the lip, as deep as it is thick; white water in
// ropes as it falls and takes in air; breaking up over the pool at its foot.
// Blended over what is drawn, by alpha.
vec4 shadeSheet(vec3 noiseN, vec3 V, vec3 L, float behindDepth, float time) {
    Sheet s = sheetHere(time);

    // Lit by its own shape, so the curl over the lip and the round of the
    // curtain show; its ropes stand out of it as it turns white (the coarse
    // ones: the fine layer's would comb its highlights into hairs), and the
    // noise ruffles it.
    vec3 sheetN = embossed(s.face, s.coarse, SHEET_RELIEF * s.aerated);
    sheetN = normalize(sheetN + 0.35 * vec3(noiseN.x, 0.0, noiseN.z));
    float lit = dot(sheetN, L);
    float wrapped = clamp((max(lit, -SHEET_THROUGH * lit) + SHEET_WRAP) / (1.0 + SHEET_WRAP), 0.0, 1.0);
    float skyward = 0.5 + 0.5 * abs(sheetN.y);
    vec3 H = normalize(L + V);
    float NdotH = max(dot(sheetN, H), 0.0);
    float fresnel = min(0.02 + 0.98 * pow(1.0 - max(dot(sheetN, V), 0.0), 5.0), SHEET_MAX_MIRROR);
    vec3 sky = mix(vec3(0.6, 0.7, 0.8), vec3(0.25, 0.4, 0.7), max(reflect(-V, sheetN).y, 0.0));
    vec3 tint = mix(vec3(0.10, 0.30, 0.32), vec3(0.03, 0.15, 0.17), s.body);
    vec3 glass = mix(tint, sky, 0.1 + 0.6 * fresnel)
        + vec3(1.0, 0.95, 0.8) * pow(NdotH, 96.0) * 0.6;
    // White water is brightest on its ropes and bluer in the hollows between.
    vec3 white = mix(vec3(0.62, 0.78, 0.9), vec3(0.94, 0.97, 1.0), smoothstep(0.25, 0.7, s.strands))
        * (0.5 + 0.42 * wrapped + 0.15 * skyward)
        + vec3(1.0, 0.95, 0.8) * pow(NdotH, 24.0) * 0.15;
    vec3 color = mix(glass, white, s.aerated * (0.6 + 0.4 * s.strands));
    return vec4(color, sheetCover(s, fresnel, behindDepth));
}

// Foam churned up where falling water lands, lying on the water round its
// foot: `fragSheet.x` past the fall by 1 and where across the patch it lies,
// from -1 behind the foot, under the fall, to 1 at its outer edge. White and
// boiling at the foot, breaking into patches and thinning out to nothing at
// its edges.
vec4 shadePlunge(vec3 L, float time) {
    float outward = abs(fragSheet.x - fragSheet.y - 1.0);
    vec2 xz = fragWorldPos.xz;
    // The foam is carried out from the foot as it breaks up.
    vec2 drift = fragFace * (0.6 * time) + vec2(0.31, -0.27) * time;
    float billows = fbm(xz * 1.7 - drift) / 1.5;
    float froth = fbm(xz * 5.5 - drift * 1.8 + 3.7) / 1.5;
    // Solid at the foot, in billows further out, gone before its edge, which
    // the billows carry in and out.
    float churn = 1.0 - outward;
    float cover = smoothstep(0.0, 0.6, 0.9 * churn + 1.3 * (billows - 0.5) - 0.05);
    cover *= 1.0 - smoothstep(0.45, 0.95, outward + 0.5 * (billows - 0.5));
    float pour = smoothstep(0.0, SHEET_FULL_POUR, fragWetDepth);
    float alpha = cover * pour * mix(0.55, 1.0, froth) * PLUNGE_OPACITY;
    // The ends of a fall's foam thin out too.
    float frayed = abs(fragSheet.z) + (billows - 0.5) * FRAY_SHARE;
    alpha *= 1.0 - fragFray * smoothstep(1.0 - FRAY_SHARE, 1.0, frayed);
    // There is foam only while water is landing.
    alpha *= flying(sqrt(2.0 * fragSheet.y / FALL_GRAVITY), billows - 0.5);
    vec3 color = vec3(0.92, 0.96, 1.0) * (0.74 + 0.2 * max(L.y, 0.0) + 0.16 * froth);
    return vec4(color, alpha);
}

void main() {
    // How much ground a pixel covers, taken before any fragment is discarded.
    float footprint = max(length(dFdx(fragWorldPos.xz)), length(dFdy(fragWorldPos.xz)));

    // A tile's fine grid covers its whole 8 m square; only its body's
    // columns are this body's water.
    if (fragMasked != 0 && isnan(rippleFloorAt(fragLayer, fragWorldPos.xz - fragTileOrigin))) {
        discard;
    }

    bool sheet = fragSheet.y > 0.0;
    float time = fpc.projParams.z;
    float near = fpc.projParams.x;
    float far = fpc.projParams.y;
    vec2 screenUV = gl_FragCoord.xy / fpc.screenParams.xy;

    // How far below the surface whatever is behind this pixel lies, m: the
    // ground's own depth, to the pixel, where the field's is a column's.
    float terrainDepthRaw = texture(depthSampler, screenUV).r;
    float waterDepthRaw = gl_FragCoord.z;
    float terrainDist = linearizeDepth(terrainDepthRaw, near, far);
    float waterDist = linearizeDepth(waterDepthRaw, near, far);
    vec3 behindPos = fpc.cameraPos.xyz + (fragWorldPos - fpc.cameraPos.xyz) * (terrainDist / waterDist);
    float groundDepth = fragWorldPos.y - behindPos.y;

    // Laying falling water's depth needs only how much it hides, and that
    // at its least: without the sky its glass mirrors, which only adds to
    // it. Water pouring past its foot is foam lying on the pool, which
    // hides no other fall.
    if (SHEET_DEPTHS) {
        bool poured = fragWetDepth >= fragWetRange.x && fragWetDepth <= fragWetRange.y;
        float behind = max(terrainDist - waterDist, 0.0);
        if (fragSheet.x > fragSheet.y || !poured || sheetCover(sheetHere(time), 0.0, behind) < SHEET_SOLID) {
            discard;
        }
        return;
    }
    float fieldSlope = fwidth(fragWetDepth) / max(footprint, 1.0e-4);
    float slope = groundSlope(screenUV, fragWorldPos, waterDist, behindPos, near, far);

    // Where the field has the water shallow, it is as deep as what is behind
    // it, where that is shallower still: a beach's edge, to the pixel. Water
    // the field has deep is taken at its word: what stands shallow in it is
    // a body, not the ground, and stays under the water; and where deep water
    // meets a wall, the wall cuts it, softened only by the contact below. A
    // skirt reads the field's depth at its cell's edge.
    float trust = 1.0 - smoothstep(GROUND_TRUSTED_DEPTH.x, GROUND_TRUSTED_DEPTH.y, fragWetDepth);
    bool byGround = !sheet && groundDepth < fragWetDepth && trust > 0.0;
    float wetDepth = byGround ? mix(fragWetDepth, groundDepth, trust) : fragWetDepth;
    // How fast the water thins across the ground, per metre of it.
    float thinning = byGround ? mix(fieldSlope, slope, trust) : fieldSlope;
    // Shallow water lies on the ground: where what is behind it lies well
    // below the field's depth, its quad stands in the air over a drop its
    // column cannot draw, and there is no water there.
    float lifted = sheet ? 0.0
        : trust * smoothstep(LIFTED_GAP.x, LIFTED_GAP.y, groundDepth - fragWetDepth);

    // A river surface is drawn only where its water is deep enough: past its
    // shore depth it fades in over a wandering edge, not a cut along its
    // triangles. A sheet's depth is how deep it pours; it is cut where
    // nothing pours, and thins out below a full curtain on its own.
    // Only water near an edge wanders; deep water against a wall fades into
    // it straight.
    bool nearEdge = wetDepth - fragWetRange.x < FOAMY_DEPTH;
    float wander = nearEdge
        ? fbm(fragWorldPos.xz * 2.3 + vec2(time * 0.21, -time * 0.17)) / 1.5 - 0.5
        : 0.0;
    if (wetDepth > fragWetRange.y) {
        discard;
    }
    float shoreWidth = max(SHORE_SOFTNESS_DEPTH,
                           thinning * SHORE_SOFTNESS);
    float shore = sheet
        ? step(fragWetRange.x, wetDepth)
        : smoothstep(0.0, shoreWidth,
                     wetDepth - fragWetRange.x + wander * EDGE_WANDER * shoreWidth)
          * (1.0 - lifted);
    if (shore <= 0.0) {
        discard;
    }

    // --- The chop, and the falls' noise ---
    // Running water carries its detail downstream: the chop and foam are
    // carried with the flow, in two phases that start over in turn.
    vec2 worldXZ = fragWorldPos.xz;
    // Falling water carries where it left its lip in place of a flow.
    vec2 flow = sheet ? vec2(0.0) : fragFlow;
    float speed = length(flow);
    float phases[2] = float[2](fract(time / FLOW_PERIOD), fract(time / FLOW_PERIOD + 0.5));
    float firstWeight = 1.0 - abs(1.0 - 2.0 * phases[0]);
    // Faster water runs rougher, and the wind gusts and lulls in patches.
    float gust = smoothstep(0.6, 1.4, valueNoise(worldXZ / CHOP_GUST_SIZE - CHOP_GUST_DRIFT * time));
    float roughness = (1.0 + 0.2 * min(speed, 2.0)) * mix(CHOP_GUST_RANGE.x, CHOP_GUST_RANGE.y, gust);
    bool flowing = speed > 1.0e-3;
    float peak = chopPeak(fragSwell.w, speed);
    vec2 chopSlope = !sheet && !flowing ? chopSlopeAt(worldXZ, roughness, peak, footprint) : vec2(0.0);
    vec3 noiseN1 = vec3(0.0);
    float whiteNoise = 0.0;
    float lace = 0.0;
    // Foam is drawn out along the current where it runs white, and lies as
    // it falls where the water is slow.
    vec2 downstream = speed > 1.0e-3 ? flow / speed : vec2(1.0, 0.0);
    float streaked = smoothstep(0.6 * WHITE_WATER_SPEED.x, WHITE_WATER_SPEED.x, speed);
    vec2 carrying = flow * min(1.0, MAX_CARRY_SPEED / max(speed, 1.0e-3));
    // Lace is dear and only shows where water runs fast or thins out to an
    // edge; open water goes without.
    // A film has no foam to carry: it is what is left when the foam's water
    // has run out from under it.
    float riding = sheet ? 0.0
        : clamp(fragAir, 0.0, 1.0) * smoothstep(RIDING_FOAM_DEPTH.x, RIDING_FOAM_DEPTH.y, wetDepth);
    bool foamy = !sheet && (speed > WHITE_WATER_SPEED.x || nearEdge || riding > 0.02);
    for (int p = 0; p < 2; p++) {
        // Each phase starts over at its own offset, so the restart is not
        // seen as the same pattern coming back.
        vec2 carried = -carrying * phases[p] * FLOW_PERIOD + vec2(0.5 * float(p), 0.37 * float(p));
        float weight = p == 0 ? firstWeight : 1.0 - firstWeight;
        if (sheet) {
            noiseN1 += weight * proceduralNormal(worldXZ + carried + vec2(time * 0.3, time * 0.2), 0.5, 0.07 + 0.015 * min(speed, 2.0));
        } else if (flowing) {
            // No offset of its own: the waves already differ from one restart
            // to the next, and as the current dies away both phases must come
            // to the still water's chop, or a seam shows where it stops.
            vec2 drifted = worldXZ - carrying * phases[p] * FLOW_PERIOD;
            chopSlope += weight * chopSlopeAt(drifted, roughness, peak, footprint);
        }
        whiteNoise += weight * fbm(worldXZ * 2.0 + carried * 2.0);
        if (foamy) {
            vec2 laid = (worldXZ + carried) * 2.2;
            if (streaked < 1.0) {
                lace += weight * (1.0 - streaked) * foamLace(laid, time, 0.4);
            }
            if (streaked > 0.0) {
                lace += weight * streaked * streakedLace(laid, downstream, time);
            }
        }
    }

    // The ripples' normal is read per pixel where a tile is awake; the
    // swell's slope is added to it.
    vec3 rippleN = vec3(0.0, 1.0, 0.0);
    if (fragLayer >= 0) {
        float extent = float(RIPPLE_COLUMNS) * RIPPLE_COLUMN;
        vec2 local = clamp(worldXZ - fragTileOrigin, vec2(0.0), vec2(extent));
        vec2 slope = rippleGradientAt(fragLayer, local);
        float steepness = length(slope);
        slope *= steepness > MAX_RIPPLE_SLOPE ? MAX_RIPPLE_SLOPE / steepness : 1.0;
        rippleN = normalize(vec3(-slope.x, 1.0, -slope.y));
    }
    vec2 swellSlope = swellSlopeAt(worldXZ, fragSwell.z, fragSwell.x, fragSwell.y, footprint);
    vec3 vertexN = normalize(vec3(
        rippleN.x / rippleN.y - swellSlope.x,
        1.0,
        rippleN.z / rippleN.y - swellSlope.y
    ));

    // The chop's slope on top of the vertex normal's.
    vec3 N = normalize(vec3(
        vertexN.x / vertexN.y - chopSlope.x,
        1.0,
        vertexN.z / vertexN.y - chopSlope.y
    ));

    vec3 V = normalize(fpc.cameraPos.xyz - fragWorldPos);

    vec3 L = normalize(fpc.sunDir.xyz);


    // --- Volumetric depth from depth buffer ---
    float opticalDepth = max(terrainDist - waterDist, 0.0);

    float depthFactor = 1.0 - exp(-opticalDepth * 0.06);

    // --- Screen-space refraction ---

    // Offset UV by the water normal's XZ. How far something under the water
    // appears to shift depends on its own depth below the surface, not on the
    // depth behind this pixel: a shallow object over a deep floor barely
    // moves. A first lookup is offset by the depth behind this pixel; where it
    // lands on something shallower, the offset is cut down to that depth and
    // looked up again. Otherwise the floor around a shallow object reaches
    // across onto it, and the object shows twice: in place, and refracted
    // around itself.
    vec2 firstUV = refractedLookup(screenUV, N, opticalDepth);
    float firstDist = linearizeDepth(texture(depthSampler, firstUV).r, near, far);
    float firstOpticalDepth = max(firstDist - waterDist, 0.0);
    vec2 refractedUV = refractedLookup(screenUV, N, min(opticalDepth, firstOpticalDepth));

    // Reject refraction offsets that land on pixels above the water surface
    // (e.g. a character's head poking out). If the depth at the refracted UV
    // is closer than the water fragment, fall back to the undistorted UV.
    float refractedDepthRaw = texture(depthSampler, refractedUV).r;
    if (refractedDepthRaw < waterDepthRaw) {
        refractedUV = screenUV;
        refractedDepthRaw = terrainDepthRaw;
    }

    vec3 terrainColor = texture(colorSampler, refractedUV).rgb;

    float refractedTerrainDist = linearizeDepth(refractedDepthRaw, near, far);
    float refractedOpticalDepth = max(refractedTerrainDist - waterDist, 0.0);

    // What survives the water between the eye and what is behind it, per
    // channel, along the ray to the refracted point (the one shown).
    vec3 transmittance = exp(-WATER_EXTINCTION * refractedOpticalDepth);

    // --- Fresnel reflectance (Schlick approximation) ---
    float F0 = 0.02;
    float NdotV = max(dot(N, V), 0.0);
    float fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    // The sky the surface mirrors, from the sky model the sky pass draws. A
    // reflected ray that dips below the horizon on a wave's back sees the
    // horizon, as `aerialRadiance` does, not the ground tone under it.
    vec3 R = reflect(-V, N);
    vec3 reflectionColor = aerialRadiance(R, L);

    // The water's own light (only this gets diffuse lighting): what the body
    // scatters back, taking each channel's place as that channel is absorbed.
    float NdotL = max(dot(N, L), 0.0);
    vec3 scattered = WATER_SCATTER * (0.5 + 0.5 * NdotL);
    vec3 waterBody = terrainColor * transmittance + scattered * (1.0 - transmittance);

    // Blend water body with reflection via Fresnel.
    vec3 color = mix(waterBody, reflectionColor, fresnel);

    // --- Specular highlight (Blinn-Phong) ---
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    float spec = pow(NdotH, 256.0);
    vec3 sunColor = vec3(1.0, 0.95, 0.8);
    color += sunColor * spec * 0.8;

    // --- Subsurface scattering approximation ---
    float sss = pow(max(dot(V, -L), 0.0), 4.0) * (1.0 - depthFactor) * SSS_INTENSITY;
    color += SSS_COLOR * sss;

    if (sheet) {
        // Past its foot, falling water is the foam where it lands.
        vec4 fall = fragSheet.x > fragSheet.y ? shadePlunge(L, time)
                                              : shadeSheet(noiseN1, V, L, opticalDepth, time);
        outColor = vec4(applyAerialPerspective(fall.rgb, fragWorldPos, fpc.cameraPos, L), fall.a);
        return;
    }

    // --- Foam: a frill along the edge, lace where a river runs fast ---
    // Both are lace broken into patches, so neither reads as a net.
    float patches = smoothstep(0.3, 0.8, whiteNoise / 1.5);
    // Patches of froth, laced inside: never the lace's net on its own.
    float froth = patches * (0.45 + 0.55 * lace);
    float foamDepth = shoreWidth / SHORE_SOFTNESS * EDGE_FOAM_REACH;
    float edge = 1.0 - smoothstep(0.0, foamDepth,
                                  wetDepth - fragWetRange.x + wander * foamDepth);
    // Water thinning up a wall is no beach: only a gentle shore froths.
    float edgeFoam = edge * mix(0.15, 0.5, froth) * (1.0 - smoothstep(FROTHING_SLOPE.x, FROTHING_SLOPE.y, thinning));
    float whiteWater = smoothstep(WHITE_WATER_SPEED.x, WHITE_WATER_SPEED.y, speed) * froth * 0.6;
    // Foam the water carries from where it was made lies in billows that
    // close up where it is thick and break into flecks as it thins, over
    // water turned pale by the air churned into it.
    float billows = smoothstep(0.25, 0.75, whiteNoise / 1.5);
    float cover = smoothstep(0.0, RIDING_FOAM_EDGE, riding * RIDING_FOAM_FULL - (1.0 - billows));
    // Laced inside where it is thin, never the lace's net on its own, and
    // closing up where it lies thick.
    float thick = smoothstep(0.5, 1.0, riding);
    float ridingFoam = cover * mix(0.3 + 0.7 * lace, 0.5 + 0.4 * lace, thick);
    vec3 foamColor = vec3(0.9, 0.95, 1.0) * (0.8 + 0.2 * NdotL);
    // The pale water follows the billows too: by the foam's amount alone
    // its edge is the straight edge of the cells that hold it.
    float churned = smoothstep(0.1, 1.0, riding * (0.5 + billows));
    color = mix(color, AERATED_COLOR * (0.55 + 0.45 * NdotL), AERATED_SHARE * churned);
    float foamFactor = max(max(edgeFoam, whiteWater), ridingFoam);
    color = mix(color, foamColor, foamFactor);

    // --- Soft edges: fade into what is behind where the water thins ---
    // Against a bank or a body, and across a thin film, the view ray
    // crosses little water.
    float contact = smoothstep(0.0, CONTACT_SOFTNESS,
                               opticalDepth + wander * EDGE_WANDER * CONTACT_SOFTNESS);
    float coverage = shore * contact;
    if (coverage < 0.004) {
        discard;
    }
    vec3 behind = texture(colorSampler, screenUV).rgb;
    // Only the water's own colour: what is behind it was hazed when it was drawn.
    color = applyAerialPerspective(color, fragWorldPos, fpc.cameraPos, L);

    // Scene radiance, like `colorSampler`: the water is drawn into the HDR
    // scene target and resolved with everything around it.
    outColor = vec4(mix(behind, color, coverage), 1.0);
}
