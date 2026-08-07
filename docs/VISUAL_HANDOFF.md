# Visual work — where things stand

Written 2026-08-07, at the end of the session that landed environment lighting.
Read this, then `VISUAL_DIRECTION.md` for the backlog and `LIGHTING_PLAN.md` for
shadows and occlusion.

## Start here

**Sun shadows.** `LIGHTING_PLAN.md` stages 1 and 5–7. This is the single
highest-value change left and everything below is secondary to it.

The renderer has no shadows and no ambient occlusion of any kind. Nothing
occludes anything, so illumination is even everywhere and every object floats
above ground it has no relationship to. The owner described the result as "lit
by a hospital light", which is exactly right: uniform light with no shadow
structure is what that looks like.

Two dials were tried against that complaint and both were largely dead ends —
don't repeat them:

- **Tonemap hue preservation barely does anything** at the current light level.
  Bleaching only happens on the tonemap's shoulder and almost nothing reaches
  it now. Run `grade_sweep` and look across a row; the cells are nearly
  identical. It mattered only while the scene was overexposed.
- **Warming the sun helps a little**, and past a mild warmth it tints the whole
  frame instead of separating key from fill. Worth maybe one line of default
  change, not worth a project.

A useful cheap follow-up once shadows exist: a **black point in the grade**
(§6.1). Nothing currently renders below about mid-grey, so the frame has no
anchor even where it should be dark.

## What changed this session

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
