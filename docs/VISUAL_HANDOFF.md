# Visual work — where things stand

Written 2026-08-07, updated at the end of the session that landed sun shadows.
Read this, then `VISUAL_DIRECTION.md` for the backlog and `LIGHTING_PLAN.md` for
occlusion.

## Start here

**Look at the game and confirm the shadows.** The bench has no marching-cubes
terrain in it, and terrain shadowing itself under a low sun is where depth-map
bias fails if it is going to. The two dials are `ShadowVolume`'s
`normal_offset_texels` (receiver side; raise it if surfaces stripe themselves)
and `DEPTH_BIAS_SLOPE` in `shadow/pipeline.rs` (caster side). Raising either too
far detaches a shadow from the thing casting it, which is the failure that
undoes the whole feature — the `shadows` bench scene exists to show both edges
of that trade at once.

**Then: ambient occlusion.** `LIGHTING_PLAN.md` stage 1. Shadows resolve the sun
but nothing occludes the *sky*, which is now a real fill light arriving from the
whole hemisphere. Creases, undersides and the ground next to a wall all still
receive full sky. It is the same complaint one level down, and it is what the
honest discounts in `environment.glsl` (`SKY_IRRADIANCE_FACTOR`,
`GROUND_ALBEDO`) are standing in for.

A useful cheap follow-up now that shadows exist: a **black point in the grade**
(`VISUAL_DIRECTION.md` §6.1). There is finally something in the frame that
*should* be dark, so the anchor has something to anchor.

Two dials were tried against the original "lit by a hospital light" complaint
and both were largely dead ends — don't repeat them:

- **Tonemap hue preservation barely does anything** at the current light level.
  Bleaching only happens on the tonemap's shoulder and almost nothing reaches
  it now. Run `grade_sweep` and look across a row; the cells are nearly
  identical. It mattered only while the scene was overexposed.
- **Warming the sun helps a little**, and past a mild warmth it tints the whole
  frame instead of separating key from fill. Worth maybe one line of default
  change, not worth a project.

## What changed this session

Sun shadow mapping — `LIGHTING_PLAN.md` stage 6, which has the design notes.
`src/rendering/shadow/`, a `shadows` bench scene, and shadow framing exposed on
`SceneEnvironment` so a scene can retune or disable it.

## What changed the session before

Commits `026bb72` through `91cc129`.

- `FrameOutput` trait (`src/rendering/target/`) splits presentation from
  per-frame render targets, so the same pipeline draws to a window or to an
  image in memory. `SwapchainOutput` and `OffscreenOutput`.
- `visual_bench` renders scenes through the real pipeline to a PNG contact
  sheet, with no window. See below.
- Environment lighting: `shader/sky_model.glsl` (sky as linear HDR radiance),
  `shader/environment.glsl` (hemisphere irradiance + sky reflection with a
  split-sum BRDF fit). Metals work now; roughness reads across its whole range.
- `TextureHandle` actually refcounts. It derived `Clone` with a `Drop` that
  released unconditionally, so any clone freed the texture out from under every
  other handle.

## Traps worth not re-breaking

**A comparison sampler has to be immutable in the descriptor set layout.** Metal
takes the comparison function from the sampler state, not from a descriptor
write, so MoltenVK reports `mutableComparisonSamplers = FALSE` and rejects one
written at runtime. This is why `ShadowMap` is built *before* `GraphicsPipeline`
in `Renderer::new` — the layout is built around the sampler. The descriptor
write then supplies only the image view.

**Judge shadow bias at a low sun, never a high one.** At 60° everything looks
fine at any bias; the whole trade only becomes visible near the horizon, where
one shadow texel covers a long stretch of ground. That is why the `shadows`
scene sweeps the sun down to 7° instead of stopping somewhere flattering.

**Never let a shader resolve its own HDR.** `sky.frag` used to apply a Reinhard
curve and a gamma encode before writing into the linear HDR target. Its output
could never exceed 1.0, so after exposure it never reached the bloom threshold
and the sun could not bloom at any authored brightness — that was the whole of
why it "lacked lustre". The post chain owns exposure and tonemapping.

**Absolute light level is pinned by the content, not by taste.** Raising total
illumination on a sunlit surface from ~1.0 to ~2.4 looked fine on every bench
scene and washed the game out completely. Level albedos are saturated primaries
near full value, authored against the old dim lighting, so extra light pushes
them onto the tonemap's shoulder and they go pale and electric rather than
bright. Get contrast from the key-to-fill *ratio* (~5:1) and leave the absolute
level alone. Raising it means re-authoring albedos, which is a deliberate
project.

**A directional light needs a floor on its specular lobe width.** A point at
infinity narrows its highlight without limit; below a pixel it vanishes, and
polished ends up looking flatter than satin. `SUN_SPECULAR_ROUGHNESS_FLOOR`.

**`SKY_RADIANCE_SCALE` sets the sky's on-screen brightness *and* the fill it
casts.** Those cannot be split for the specular path — a mirror has to agree
with the sky beside it. The diffuse path can be discounted honestly
(`SKY_IRRADIANCE_FACTOR`, `GROUND_ALBEDO`), standing in for occlusion that AO
will eventually compute properly. Revisit both when AO lands.

**Don't ring-sample the sky for rough reflections.** Too few taps to afford per
fragment, and each crosses the horizon at a different roughness, which shows up
as concentric banding on rough metal. Lerp towards an analytic hemisphere
average instead.

**The sun's apparent size is its radiance, not its radius.** Bloom spreads
brightness, so overdriving `SUN_DISC_RADIANCE` inflates the glow until it
dominates the sky. That dial, not `SUN_DISC_WIDENING`, is the one to reach for.

## Using the bench

```bash
cargo run --bin visual_bench -- --list
cargo run --bin visual_bench -- palette --out /tmp/palette.png --columns 2
cargo run --bin visual_bench -- shadows --out /tmp/shadows.png --columns 3
```

Scenes live in `src/rendering/visual_bench/scenes/`, registered in
`registry.rs`. Sweeps (a parameter ladder as one sheet) are the reason the tool
beats a windowed viewer — you cannot be in twelve places in parameter space at
once. Keep scenes deterministic or before-and-after comparison is meaningless.

**Judge exposure, saturation and grade on `palette`, never on `props` or
`material_grid`.** Those are muted mid-value albedos: a comfortable place for a
renderer to sit and a misleading one to tune in. `palette` is saturated level
colours on bright terrain, which is where clipping actually shows. It pulls its
grass straight from `VoxelMaterial::Grass` — a hand-written copy had already
drifted lighter than the real thing.

The game window cannot be launched from an agent shell, so the bench is the only
way an assistant sees its own rendering changes. Ask the owner for a screenshot
to confirm anything that matters; the bench has misled once already, and it
misled by being too flattering.
