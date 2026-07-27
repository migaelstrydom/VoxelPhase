// HDR → LDR conversion, shared by the composite pass and any shader that has
// to resolve HDR scene colour on its own (e.g. water refraction, which samples
// the HDR scene target but writes to the LDR swapchain).

#ifndef TONEMAP_GLSL
#define TONEMAP_GLSL

/// Narkowicz's fit of the ACES filmic curve.
///
/// Compresses unbounded HDR radiance into [0,1] with a filmic shoulder, so
/// bright emissive surfaces desaturate towards white instead of clipping to a
/// flat primary colour.
vec3 tonemapACES(vec3 colour) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((colour * (a * colour + b)) / (colour * (c * colour + d) + e), 0.0, 1.0);
}

/// Perceptual luminance of a linear colour (Rec. 709 primaries).
float luminance(vec3 colour) {
    return dot(colour, vec3(0.2126, 0.7152, 0.0722));
}

/// The same ACES fit applied to a single scalar, for curving luminance alone.
float tonemapACESScalar(float x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

/// Below this luminance a colour carries no usable hue, so rescaling chroma by
/// it would amplify noise. Such pixels fall back to the per-channel curve,
/// which is well behaved approaching black.
const float TONEMAP_MIN_LUMINANCE = 1e-4;

/// Hue-preserving tonemap: curve the luminance, then rescale the original
/// colour to match, instead of curving each channel separately.
///
/// Per-channel ACES compresses a bright channel harder than a dim one, which
/// pulls saturated colours towards white — the desaturation is the point of the
/// filmic look, but it means anything bright enough to bloom is also bleached.
/// Scaling by `curve(l) / l` moves the colour along its own hue line instead.
///
/// Achromatic input is unaffected: for a grey colour the luminance equals the
/// channel value, so the rescale reduces to the scalar curve — the same result
/// the per-channel path gives. Sky and neutral stone therefore look identical
/// either way, whatever the strength.
vec3 tonemapACESHuePreserving(vec3 colour) {
    float l = luminance(colour);
    if (l <= TONEMAP_MIN_LUMINANCE) {
        return tonemapACES(colour);
    }

    float lt = tonemapACESScalar(l);
    vec3 scaled = colour * (lt / l);

    // Holding chroma fixed can push a channel past 1.0 — the display cannot
    // show that colour at that brightness. Something has to give: either the
    // colour desaturates towards grey, or it darkens. Scaling the whole triple
    // down keeps the channel ratios exactly, so the hue survives intact and the
    // pixel is merely dimmer than its luminance asked for.
    //
    // The alternative — blending towards the tonemapped luminance by the least
    // amount that fits — keeps the brightness but spends saturation, and it
    // spends most where chroma is highest. Measured on the test orbs it made
    // the orange one *less* saturated than plain per-channel ACES (0.23 against
    // 0.41), which defeats the point of the function. Rejected on that evidence.
    //
    // Consequence worth knowing: a colour treated this way never bleaches to
    // white however bright it gets, which is not how film behaves. That is what
    // the strength dial is for — blending back towards the per-channel curve
    // restores the filmic bleach.
    float peak = max(max(scaled.r, scaled.g), scaled.b);
    if (peak > 1.0) {
        scaled /= peak;
    }

    return clamp(scaled, 0.0, 1.0);
}

/// Resolve HDR colour to display range, blending between the per-channel ACES
/// curve and the hue-preserving one.
///
/// `hue_preservation` 0 is exactly the per-channel curve, 1 is fully
/// hue-preserving. Every shader that resolves HDR must call this with the same
/// value, or surfaces that tonemap themselves (water) will not match the
/// composited scene around them.
vec3 tonemapScene(vec3 colour, float hue_preservation) {
    vec3 per_channel = tonemapACES(colour);
    if (hue_preservation <= 0.0) {
        return per_channel;
    }
    return mix(per_channel, tonemapACESHuePreserving(colour), hue_preservation);
}

#endif // TONEMAP_GLSL
