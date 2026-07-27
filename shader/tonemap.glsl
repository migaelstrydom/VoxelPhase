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

#endif // TONEMAP_GLSL
