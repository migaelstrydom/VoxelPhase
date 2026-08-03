#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Vertex inputs
layout(location = 0) in vec3 inCenter;    // Particle center in world space
layout(location = 1) in vec2 inCorner;    // Billboard corner (-1,-1 to 1,1)
layout(location = 2) in float inSize;     // Particle size
layout(location = 3) in vec4 inColor;     // RGBA color
layout(location = 4) in float inLife;     // Normalized lifetime
layout(location = 5) in vec3 inMotion;    // World-space smear vector (0 = round)

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

    // Expand the billboard in view space, where the camera's right and up are
    // simply x and y.
    //
    // A particle carrying a smear vector is drawn along it instead of
    // axis-aligned: the quad's local up follows the direction the particle
    // travels as it appears on screen, and grows by how far it travels. A
    // stream of these reads as streaks rather than as a dotted line. The
    // component of motion towards the camera is dropped, since it foreshortens
    // to nothing on screen anyway.
    vec2 smear = (mat3(pc.view) * inMotion).xy;
    float smearLength = length(smear);

    vec2 alongAxis = vec2(0.0, 1.0);
    vec2 acrossAxis = vec2(1.0, 0.0);
    if (smearLength > 1e-5) {
        alongAxis = smear / smearLength;
        acrossAxis = vec2(alongAxis.y, -alongAxis.x);
    }

    vec2 offset = acrossAxis * (inCorner.x * inSize)
                + alongAxis * (inCorner.y * (inSize + smearLength));
    vec3 viewPos = viewCenter.xyz + vec3(offset, 0.0);

    // Project to clip space
    gl_Position = pc.proj * vec4(viewPos, 1.0);

    // Pass to fragment shader. The UV still spans the quad, so the fragment
    // shader's circular falloff stretches along with it into an ellipse.
    fragColor = inColor;
    fragLife = inLife;
    fragUV = inCorner * 0.5 + 0.5;  // Convert -1..1 to 0..1
}
