// Relief carried in a diffuse texture's alpha: height, not coverage.
//
// A pattern that cuts into the surface it colours — the cracks and pits of
// weathered stone — bakes how far it cut into the alpha of the same texture
// (src/rendering/pattern/layer.rs, `Pattern::relief_depth`). Reading it back
// costs two extra taps of a texture the fragment was sampling anyway, and
// needs no tangents from the mesh: the frame the height is differentiated in
// comes from the screen-space derivatives of position and texture coordinate.

#ifndef RELIEF_GLSL
#define RELIEF_GLSL

/// The steepest a relief slope may tilt the normal, as rise over run.
///
/// A crack one texel wide and several deep is a near-vertical wall, and a
/// normal tilted that far faces away from the camera and shades black along
/// every crack edge. Capped, a crack still has a lit lip and a shaded one.
const float RELIEF_MAX_SLOPE = 2.5;

/// Perturb `normal` by the height stored in `tex`'s alpha.
///
/// `height` is the alpha already sampled at `uv`; `depth` is how many texture
/// coordinates deep the full range of the alpha runs. The height is
/// differentiated across one texel *of the mip level being sampled*, so a
/// receding surface takes its slope from the same filtered field it takes its
/// colour from and does not sparkle.
vec3 reliefNormal(
    sampler2D tex,
    vec2 uv,
    float height,
    vec3 normal,
    vec3 world_pos,
    float depth
) {
    // The surface's own frame, from how position and texture coordinate change
    // across a pixel: the world-space gradients of u and v along the surface.
    vec3 dp_dx = dFdx(world_pos);
    vec3 dp_dy = dFdy(world_pos);
    vec2 duv_dx = dFdx(uv);
    vec2 duv_dy = dFdy(uv);

    vec3 r1 = cross(dp_dy, normal);
    vec3 r2 = cross(normal, dp_dx);
    float det = dot(dp_dx, r1);
    if (abs(det) < 1e-14) {
        return normal;
    }
    vec3 grad_u = (duv_dx.x * r1 + duv_dy.x * r2) / det;
    vec3 grad_v = (duv_dx.y * r1 + duv_dy.y * r2) / det;

    // Unit directions of increasing u and v. The meshes that carry relief lay
    // their texture flat on each face at one scale in both directions, so the
    // two share a length and dividing by it turns a slope in texture space
    // into the same slope in the world.
    float stretch = inversesqrt(max(max(dot(grad_u, grad_u), dot(grad_v, grad_v)), 1e-20));
    vec3 along_u = grad_u * stretch;
    vec3 along_v = grad_v * stretch;

    float lod = textureQueryLod(tex, uv).y;
    float texel = exp2(lod) / float(textureSize(tex, 0).x);
    float height_u = texture(tex, uv + vec2(texel, 0.0)).a;
    float height_v = texture(tex, uv + vec2(0.0, texel)).a;

    vec2 slope = vec2(height_u - height, height_v - height) * (depth / texel);
    float steepness = length(slope);
    if (steepness > RELIEF_MAX_SLOPE) {
        slope *= RELIEF_MAX_SLOPE / steepness;
    }

    // A surface leans away from its uphill direction.
    return normalize(normal - along_u * slope.x - along_v * slope.y);
}

/// How much of the sky a point at this relief height still sees.
///
/// The bottom of a crack is open only to the sliver of sky straight above it;
/// the per-vertex occlusion the mesh carries is far too coarse to know that.
float reliefOcclusion(float height) {
    return mix(0.65, 1.0, height);
}

#endif // RELIEF_GLSL
