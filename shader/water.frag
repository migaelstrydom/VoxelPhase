#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "ripple.glsl"
#include "swell.glsl"

// --- Tuning constants ---
const float REFRACTION_STRENGTH = 0.01;
const float SSS_INTENSITY = 1.9;
const vec3  SSS_COLOR = vec3(0.15, 0.5, 0.4);
// How long the noise is carried by the flow before it starts over, s. Two
// phases half a period apart cross-fade, each faded out as it starts over,
// so the noise is never carried further than a period's flow: carried for
// ever, the small currents of still water would shear it into swirls.
const float FLOW_PERIOD = 2.0;
// Steepest a ripple may tilt the normal: past this the surface is noise.
const float MAX_RIPPLE_SLOPE = 1.0;
// How fast a falling sheet's two layers of streaks run down it, m/s.
const float SHEET_FALL_SPEED = 4.0;
const float SHEET_FALL_SPEED_FINE = 6.5;
// How far a sheet falls clear and glassy before it is white water, m.
const float SHEET_GLASS_DROP = 1.2;
// How far above its foot a sheet frays out into the pool, and how far below
// its lip it frays in, m: a sheet's ends are edges in mid-air wherever the
// fall steps down a column at a time.
const float SHEET_FOOT_FADE = 0.9;
const float SHEET_LIP_FADE = 0.25;
// How thick falling glass is when it is two-thirds opaque, m.
const float SHEET_GLASS_THICKNESS = 0.15;
// How far light wraps round a falling curtain past its terminator: white
// water scatters it, so its shadowed side is only dimmer.
const float SHEET_WRAP = 0.4;
// Gravity, m/s².
const float GRAVITY = 9.81;
// Most a patch of foam where falling water lands covers what is under it.
const float PLUNGE_OPACITY = 0.9;
// Pour depth at which a sheet is a full curtain, not a few strands, m.
const float SHEET_FULL_POUR = 0.12;
// Share of a sheet's half-width that overlaps its neighbour, to fade: its
// `SHEET_OVERLAP` past the end of half a column's edge.
const float SHEET_OVERLAP_SHARE = 0.2 / 0.45;

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
// Wet depth under which a surface is near enough an edge to wander and carry
// its foam, m: past it neither is ever seen however steep the shore, so
// neither is worked out.
const float FOAMY_DEPTH = 0.6;
// Speeds over which running water turns to lace of white water, m/s.
const vec2 WHITE_WATER_SPEED = vec2(1.2, 3.5);

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
// clock, unused); zero amplitude where there is none.
layout(location = 7) in vec4 fragSwell;
// Whether this is a tile's own fine grid, which covers the whole tile and is
// masked to its body's columns.
layout(location = 8) flat in int fragMasked;
// On a falling sheet: how far below its lip and its whole fall, m; where
// along its edge it hangs from, m; and across the sheet, -1 to 1 between the
// ends of its widened edge. Zero on a surface.
layout(location = 9) in vec4 fragSheet;
layout(location = 10) in float fragFray;

// The scene beyond the water, copied before the water is drawn (sampled at
// offset UVs for refraction)
layout(set = 0, binding = 0) uniform sampler2D colorSampler;

// Its depth, copied with it (sampled at both current and refracted UVs)
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 160) vec4 cameraPos;
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

// A sheet of water falling over an edge: clear glass where it leaves the
// lip, white water in streaks as it falls and takes in air, thinning out
// into mist over the pool at its foot. Blended over what is drawn, by alpha.
vec4 shadeSheet(vec3 noiseN, vec3 V, vec3 L, float behindDepth, float time) {
    float drop = max(fragSheet.x, 0.0);
    float fall = max(fragSheet.y, 1.0e-3);
    // The face seen: the pipeline culls the one turned away.
    vec3 face = normalize(fragNormal);
    // Along the edge, as it hangs from its lip and the same on every quad of
    // it, so the streaks fall straight and run on from one to the next.
    float along = fragSheet.z;
    float y = fragWorldPos.y;
    float coarse = fbm(vec2(along * 6.0, (y + time * SHEET_FALL_SPEED) * 0.35)) / 1.5;
    float fine = fbm(vec2(along * 14.0 + 3.1, (y + time * SHEET_FALL_SPEED_FINE) * 0.7)) / 1.5;
    float strands = 0.6 * coarse + 0.4 * fine;

    float aerated = smoothstep(0.0, SHEET_GLASS_DROP, drop);
    float pour = smoothstep(0.0, SHEET_FULL_POUR, fragWetDepth);

    // How thick the tongue is here: the flow over its lip keeps its rate as
    // it speeds up, so it thins as it falls (`nappe.rs`), from about 1 m/s
    // off the lip. Thick glass is deep green and hides what is behind it.
    float thickness = fragWetDepth * inversesqrt(1.0 + 2.0 * GRAVITY * drop);
    float body = 1.0 - exp(-thickness / SHEET_GLASS_THICKNESS);

    // Lit by its own shape, so the curl over the lip and the round of the
    // curtain show; the streaks and the noise only ruffle it.
    vec3 sheetN = normalize(face + vec3(0.0, 0.25 * (strands - 0.5), 0.0) + 0.35 * vec3(noiseN.x, 0.0, noiseN.z));
    float wrapped = clamp((dot(sheetN, L) + SHEET_WRAP) / (1.0 + SHEET_WRAP), 0.0, 1.0);
    float skyward = 0.5 + 0.5 * sheetN.y;
    vec3 H = normalize(L + V);
    float NdotH = max(dot(sheetN, H), 0.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - max(dot(sheetN, V), 0.0), 5.0);
    vec3 sky = mix(vec3(0.6, 0.7, 0.8), vec3(0.25, 0.4, 0.7), max(reflect(-V, sheetN).y, 0.0));
    vec3 tint = mix(vec3(0.10, 0.30, 0.32), vec3(0.03, 0.15, 0.17), body);
    vec3 glass = mix(tint, sky, 0.1 + 0.6 * fresnel)
        + vec3(1.0, 0.95, 0.8) * pow(NdotH, 96.0) * 0.6;
    vec3 white = vec3(0.92, 0.96, 1.0) * (0.42 + 0.45 * wrapped + 0.15 * skyward + 0.2 * strands)
        + vec3(1.0, 0.95, 0.8) * pow(NdotH, 24.0) * 0.15;
    vec3 color = mix(glass, white, aerated * (0.6 + 0.4 * strands));

    // Glass is clear where it is thin and deep where it is thick; white
    // water is strands with air between them, sparser where little pours.
    float glassAlpha = mix(0.25, 0.75, body) + 0.25 * fresnel;
    float strandAlpha = smoothstep(0.62 - 0.3 * pour, 0.82 - 0.25 * pour, strands + 0.12 * aerated);
    float alpha = mix(glassAlpha * mix(0.4, 1.0, pour), 0.92 * strandAlpha, aerated);
    // It frays in below its lip and out over the white water at its foot.
    alpha *= smoothstep(0.0, SHEET_FOOT_FADE * (0.5 + coarse), fall - drop);
    alpha *= smoothstep(0.0, SHEET_LIP_FADE * (0.5 + coarse), drop);
    // The ends of a fall fray out; where a sheet joins the next it runs on.
    float across = abs(fragSheet.w) + (coarse - 0.5) * SHEET_OVERLAP_SHARE;
    alpha *= 1.0 - fragFray * smoothstep(1.0 - SHEET_OVERLAP_SHARE, 1.0, across);
    // Where it runs against rock, it fades into it rather than cutting it.
    alpha *= smoothstep(0.0, 0.12, behindDepth);
    return vec4(color, alpha);
}

// Foam churned up where falling water lands, lying on the water out from
// the foot: `fragSheet.x` past the fall by the share of its reach. White and
// boiling under the fall, breaking into patches and thinning out to nothing
// at its edge.
vec4 shadePlunge(vec3 L, float time) {
    float share = clamp(fragSheet.x - fragSheet.y, 0.0, 1.0);
    vec2 xz = fragWorldPos.xz;
    vec2 drift = vec2(0.31, -0.27) * time;
    float patches = fbm(xz * 2.3 + drift) / 1.5;
    float bubbles = foamLace(xz * 6.0 + drift * 1.5, time * 1.6, 0.35);
    // Solid under the fall, in patches further out, gone before its edge.
    float churn = 1.0 - share;
    float cover = smoothstep(0.15, 0.6, 0.8 * patches + 0.7 * churn - 0.3);
    cover *= 1.0 - smoothstep(0.7, 0.95, share);
    float pour = smoothstep(0.0, SHEET_FULL_POUR, fragWetDepth);
    float alpha = cover * pour * (0.75 + 0.25 * bubbles) * PLUNGE_OPACITY;
    // The ends of a fall fray out here too.
    float across = abs(fragSheet.w) + (patches - 0.5) * SHEET_OVERLAP_SHARE;
    alpha *= 1.0 - fragFray * smoothstep(1.0 - SHEET_OVERLAP_SHARE, 1.0, across);
    vec3 color = vec3(0.92, 0.96, 1.0) * (0.78 + 0.22 * max(L.y, 0.0) + 0.1 * bubbles);
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
                     wetDepth - fragWetRange.x + wander * EDGE_WANDER * shoreWidth);
    if (shore <= 0.0) {
        discard;
    }

    // --- Animated normal mapping (dual-layer procedural noise) ---
    // Running water carries its detail downstream: the noise scrolls with the
    // flow, over a drift of its own.
    vec2 worldXZ = fragWorldPos.xz;
    float speed = length(fragFlow);
    float phases[2] = float[2](fract(time / FLOW_PERIOD), fract(time / FLOW_PERIOD + 0.5));
    float firstWeight = 1.0 - abs(1.0 - 2.0 * phases[0]);
    vec3 noiseN1 = vec3(0.0);
    vec3 noiseN2 = vec3(0.0);
    float whiteNoise = 0.0;
    float lace = 0.0;
    // Foam is drawn out along the current: its cells are laid in the flow's
    // frame, stretched downstream as the water runs faster.
    vec2 downstream = speed > 1.0e-3 ? fragFlow / speed : vec2(1.0, 0.0);
    float stretch = 1.0 + min(speed, 3.0);
    mat2 alongFlow = mat2(downstream.x / stretch, -downstream.y,
                          downstream.y / stretch, downstream.x);
    // Lace is dear and only shows where water runs fast or thins out to an
    // edge; open water goes without.
    bool foamy = !sheet && (speed > WHITE_WATER_SPEED.x || nearEdge);
    for (int p = 0; p < 2; p++) {
        // Each phase starts over at its own offset, so the restart is not
        // seen as the same pattern coming back.
        vec2 carried = -fragFlow * phases[p] * FLOW_PERIOD + vec2(0.5 * float(p), 0.37 * float(p));
        float weight = p == 0 ? firstWeight : 1.0 - firstWeight;
        noiseN1 += weight * proceduralNormal(worldXZ + carried + vec2(time * 0.3, time * 0.2), 0.5, 0.07 + 0.015 * min(speed, 2.0));
        noiseN2 += weight * proceduralNormal(worldXZ + carried * 1.3 + vec2(-time * 0.15, time * 0.25), 1.0, 0.04 + 0.015 * min(speed, 2.0));
        whiteNoise += weight * fbm(worldXZ * 2.0 + carried * 2.0);
        if (foamy) {
            lace += weight * foamLace(alongFlow * (worldXZ + carried) * 2.2, time, 0.4);
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

    // Blend noise normals with the vertex normal.
    vec3 N = normalize(vec3(
        vertexN.x + noiseN1.x + noiseN2.x,
        vertexN.y,
        vertexN.z + noiseN1.z + noiseN2.z
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

    // Depth-based absorption: red absorbs fastest (realistic turquoise tint).
    // Uses the optical depth at the refracted UV for correct absorption.
    vec3 refractedTerrain = terrainColor * exp(-vec3(0.04, 0.01, 0.02) * refractedOpticalDepth);

    // Deep water blend factor at the refracted point (matches the terrain we're showing).
    float refractedDepthFactor = 1.0 - exp(-refractedOpticalDepth * 0.06);

    // --- Fresnel reflectance (Schlick approximation) ---
    float F0 = 0.02;
    float NdotV = max(dot(N, V), 0.0);
    float fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    // Sky/reflection color approximated from reflected direction.
    vec3 R = reflect(-V, N);
    float skyFactor = max(R.y, 0.0);
    vec3 horizonColor = vec3(0.6, 0.7, 0.8);
    vec3 zenithColor  = vec3(0.25, 0.4, 0.7);
    vec3 reflectionColor = mix(horizonColor, zenithColor, skyFactor);

    // Deep water color (only this gets diffuse lighting).
    vec3 deepColor = vec3(0.02, 0.12, 0.22);
    float NdotL = max(dot(N, L), 0.0);
    vec3 litDeepColor = deepColor * (0.5 + 0.5 * NdotL);

    // Blend refracted terrain toward deep water as optical depth increases.
    // Uses refracted depth factor so the blend matches the terrain being shown.
    vec3 waterBody = mix(refractedTerrain, litDeepColor, refractedDepthFactor);

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
        outColor = fragSheet.x > fragSheet.y ? shadePlunge(L, time)
                                             : shadeSheet(noiseN1, V, L, opticalDepth, time);
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
    float foamFactor = max(edgeFoam, whiteWater);
    vec3 foamColor = vec3(0.9, 0.95, 1.0) * (0.8 + 0.2 * NdotL);
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

    // Scene radiance, like `colorSampler`: the water is drawn into the HDR
    // scene target and resolved with everything around it.
    outColor = vec4(mix(behind, color, coverage), 1.0);
}
