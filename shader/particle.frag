#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec4 fragColor;
layout(location = 1) in float fragLife;
layout(location = 2) in vec2 fragUV;

layout(location = 0) out vec4 outColor;

void main() {
    // Calculate distance from center for circular particles
    vec2 centered = fragUV * 2.0 - 1.0;
    float dist = length(centered);

    // Soft circular falloff
    float alpha = 1.0 - smoothstep(0.5, 1.0, dist);

    // Apply base color with alpha
    outColor = vec4(fragColor.rgb, fragColor.a * alpha);

    // Discard fully transparent pixels
    if (outColor.a < 0.01) {
        discard;
    }
}
