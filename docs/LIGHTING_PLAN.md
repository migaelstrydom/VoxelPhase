# Lighting Plan

High-level roadmap for the game's lighting system. Each stage below is a self-contained feature with visible value on its own; later stages assume earlier ones are in place. Per-stage integration design (shader layout, resource management, pass ordering) is deferred to individual design docs.

## Current state

- Directional sun light (one), sourced from `SkyRenderer` so shaded geometry and
  the visible sun disc agree. Colour, intensity and ambient live in `SceneLighting`.
- Point lights (`src/lighting/`): a `PointLight` component, a `LightCollector`
  that picks the frame's most relevant lights, and an `ActiveLights` resource
  uploaded to a uniform buffer at set 0, binding 1.
- Blinn-Phong diffuse + specular with Schlick Fresnel, shared via
  `shader/lighting.glsl` and applied to sun and point lights alike. Per-material
  roughness / metallic / emissive / rim (`SurfaceFinish`, `Emission`) delivered
  as fragment push constants.
- HDR scene target (`R16G16B16A16_SFLOAT`) resolved by `src/rendering/post/`:
  bright pass → separable blur → tonemap composite, with the bloom itself screen
  blended in a final pass after all transparent geometry.
- Procedural sky (`sky.vert/frag`).
- No shadows, no AO.

**Stage status:** 1 not started · **2 done** · **3 done** · **4 done** ·
5–7 not started.

Known gaps from stage 3. The 16-light cap is *global per frame*, not per
fragment: when more than sixteen lights are relevant the collector drops whole
lights rather than degrading gracefully, and a light crossing the cap boundary
will pop. Neither is worth handling until scenes routinely carry more than
sixteen lights; clustered shading is the upgrade path, and the component, the
collector and the BRDF all survive that change — only the upload and the shader
loop are replaced. Point lights cast no shadows, so they light through walls.
Only `triangle.frag` reads them, which covers terrain and models; water,
particles, fire and sky are unlit by them. `Emission` (a surface that looks
bright) and `PointLight` (a thing that lights its surroundings) are deliberately
separate concerns, so an object that should do both carries both and their
colours are kept in step by hand.

Known gaps from stage 4: only the opaque pass is HDR. Water, particles, fire and
the overlay render after the composite, straight onto the LDR swapchain, so they
do not *generate* bloom and are not tonemapped (water resolves HDR itself, using
the same exposure and curve as the composite to stay consistent). Moving the
transparent pass into the HDR target would need a separate copy of the opaque
result for water refraction to sample.

Also from stage 4: the tonemap can preserve hue instead of desaturating towards
white, on a strength dial (`PostProcessConfig::hue_preservation`, 0 = per-channel
ACES, 1 = fully hue-preserving). Saturated emissive surfaces keep their colour
where per-channel ACES bleaches them. The trade is that a hue-preserved colour
never bleaches to white however bright it gets, which is not how film behaves —
dialling back towards 0 is what buys the filmic look back. Emissive brightness
(`Emission::strength`) is expressed as luminance rather than as a multiplier on
colour, so one number means the same brightness at any hue and is directly
comparable against the bloom threshold. `PointLight::intensity` follows the
same convention, so a light's colour and its cast brightness can be tuned
independently.

Because the bloom overlay runs last, halos are drawn over transparent geometry —
correct for water, but it also means bright halos tint the debug text overlay.
Splitting the transparent pass so the overlay draws after the bloom would fix
that if it becomes annoying.

## Staged roadmap

### Stage 1 — Baked SDF Ambient Occlusion

Darken crevices and recesses in the terrain where ambient light is partially blocked by nearby geometry. Computed at mesh-generation time by sampling the SDF (or raymarching it) in a hemisphere around each vertex normal and storing an occlusion factor as a per-vertex attribute. Zero runtime cost after bake; naturally smooth because the surface is smooth.

Biggest readability win for the terrain and fully self-contained — no changes to the lighting pipeline, just a new vertex attribute consumed by the fragment shader.

### Stage 2 — Specular BRDF + Emissive Materials

Upgrade the material model from pure Lambert to a diffuse + specular BRDF (Blinn-Phong is sufficient; GGX if we want to go PBR later). Exposes two new per-material knobs: roughness (sharp vs broad highlight) and metallic-ness (colored vs white highlight).

Add an emissive color term so glowing objects (magical orbs, fire cores, UI-relevant props) render bright regardless of incoming light. Emissive is independent of whether the object also acts as a light source — that's Stage 3.

Scope is a shader-local change plus material-data plumbing.

### Stage 3 — Point Lights (Small Fixed Cap)

Support a modest number of point lights (position, color, range, intensity) — e.g. torches, fire, the glowing orb. No clustered / Forward+ machinery at this stage.

A single global per-frame light set is used rather than a per-object list: a terrain chunk spans many lights, so choosing lights per object gives neighbouring chunks different light sets and a visible discontinuity at the seam. One capped, scored, deterministically ordered array is uploaded per frame and every lit fragment loops over it, bounded by the live count.

Spotlights can slot into the same infrastructure later if needed.

### Stage 4 — HDR Pipeline + Tonemapping + Bloom

Once multiple light sources with real intensities exist, rendering into an 8-bit framebuffer clips highlights and destroys the sense of brightness. Switch the main color target to a floating-point format, add a tonemap pass (ACES or Reinhard) that compresses HDR → LDR, and add a bloom pass that blurs bright-pass extraction to make emissive surfaces and bright lights feel luminous.

These three effects belong together — each one alone has limited value, but as a group they define the "modern lit" look.

### Stage 5 — Blob Shadow for the Player

Projected decal: a soft circular shadow texture projected straight down from the player onto whatever geometry sits beneath. Drapes correctly over our smooth marching-cubes terrain (unlike a flat quad). Extremely cheap, buys us a functional shadow under the character without any shadow-map infrastructure.

Optional / skippable: if we go straight to Stage 6, we can omit this. The reason to do it anyway is that shadows are the biggest readability feature for a platformer (players judge jumps from their shadow), and staging the cheap version first de-risks the expensive one.

### Stage 6 — Single Shadow Map for the Sun

Render the scene from the sun's point of view into a depth texture; in the main pass, transform each shaded fragment into light space and compare depths to decide if it's shadowed. Covers all casters and receivers — player, props, terrain, moving platforms — and handles self-shadowing naturally.

One shadow map (no cascades) is the baseline. Expect blocky edges and limited coverage area; those are acceptable starting points. Filtering (PCF) can be added as a small follow-up.

### Stage 7 — Cascaded Shadow Maps (CSM)

Upgrade Stage 6 from a single shadow map to several, each covering a different slice of the view frustum (near / mid / far). Gives crisp shadows near the camera while still covering distant geometry. Adds per-cascade render passes, cascade selection in the fragment shader, and cascade-boundary blending.

Deferred until single-shadow-map quality becomes a visible problem.

## Deliberately out of scope (for now)

- **Global illumination** (voxel GI, lightmaps, probes). AO covers the most valuable portion of GI's visual contribution.
- **Point-light shadows** (cube shadow maps per light). Expensive and rarely load-bearing for a platformer.
- **Screen-space reflections**, **volumetric lighting**, **subsurface scattering**. All real features but not core to the platformer's readability.
- **Deferred rendering / Forward+ / clustered shading**. The current forward path is sufficient for the light counts we're planning.

## Dependency ordering

```
Stage 1 (AO)            ─── independent
Stage 2 (BRDF + emissive) ─── independent
Stage 3 (point lights) ─── benefits from Stage 2
Stage 4 (HDR + tonemap + bloom) ─── benefits from Stages 2 & 3
Stage 5 (blob shadow) ─── independent, optional
Stage 6 (shadow map) ─── independent; supersedes Stage 5
Stage 7 (CSM) ─── depends on Stage 6
```

Stages 1 and 2 can be done in either order. Stage 4 only pays off once there are bright light sources to tonemap.
