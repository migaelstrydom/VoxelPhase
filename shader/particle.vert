#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Vertex inputs
layout(location = 0) in vec3 inCenter;    // Particle center in world space
layout(location = 1) in vec2 inCorner;    // Billboard corner (-1,-1 to 1,1)
layout(location = 2) in float inSize;     // Particle size
layout(location = 3) in vec4 inColor;     // RGBA color
layout(location = 4) in float inLife;     // Normalized lifetime

// Push constants for matrices
layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
} pc;

// Outputs to fragment shader
layout(location = 0) out vec4 fragColor;
layout(location = 1) out float fragLife;
layout(location = 2) out vec2 fragUV;

void main() {
    // Transform center to view space
    vec4 viewCenter = pc.view * vec4(inCenter, 1.0);

    // Expand billboard in view space (camera-facing)
    // Right vector is (1,0,0) and up vector is (0,1,0) in view space
    vec3 viewPos = viewCenter.xyz + vec3(inCorner * inSize, 0.0);

    // Project to clip space
    gl_Position = pc.proj * vec4(viewPos, 1.0);

    // Pass to fragment shader
    fragColor = inColor;
    fragLife = inLife;
    fragUV = inCorner * 0.5 + 0.5;  // Convert -1..1 to 0..1
}
