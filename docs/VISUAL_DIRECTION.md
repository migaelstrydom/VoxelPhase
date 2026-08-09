# Visual Direction

An idea backlog, not a plan. Nothing here is committed to or scheduled. Each
entry records the intent and the trap, because the trap is the part that gets
forgotten between the conversation and the attempt. Implementation approach is
deliberately left to whoever picks the item up.

Lighting infrastructure items (ambient occlusion, sun shadows) live in
[LIGHTING_PLAN.md](LIGHTING_PLAN.md) and are only cross-referenced here — this
doc is about the *look*, that one is about the pipeline.

## The target look

Crisp, shiny, exaggerated realism. The reference point is Astro Bot: objects
read as injection-moulded plastic and painted metal, lit like a studio product
shot, with saturated colour and hard clean highlights. "Too realistic" for a
whimsical platformer, on purpose.

Where we diverge: this game exists to show off its physics engine. So the look
should not merely be pretty, it should make the simulation *legible* — you
should be able to see that a thing is heavy, that a surface is slippery, that a
crate is resting rather than floating. Every idea below is judged against both
goals, and the ones that serve both come first.

## Tooling

`cargo run --bin visual_bench -- <scene>` renders scenes headlessly through the
real pipeline and writes a labelled PNG contact sheet. It is the feedback loop
for everything in this document: it needs no window, and its *sweep* scenes
render a whole parameter ladder as one image, which is how material and lighting
values actually get chosen.

Scenes live in `src/rendering/visual_bench/scenes/`. Add one there and register
it in `registry.rs`. Determinism is a hard requirement — fixed camera, fixed
sun, fixed geometry — or before/after comparison is worthless.

## Current state

- Blinn-Phong diffuse + specular, Schlick Fresnel, per-material roughness /
  metallic / emissive / rim (`shader/lighting.glsl`, `shader/material.glsl`).
- One directional sun, up to 16 point lights, flat constant ambient colour.
- HDR target, ACES tonemap with a hue-preservation dial, bloom.
- Procedural sky with atmospheric scattering.
- **No shadows. No ambient occlusion. No environment reflection of any kind.**

That last line is the single biggest reason the render doesn't look like the
reference. A surface reads as shiny because it reflects its surroundings, not
because it has a tight specular highlight. Right now a surface can only be
bright where a light happens to be; everywhere else it falls back to flat
ambient, which is a matte grey lie.

The `material_grid` bench scene makes this concrete, and it is worth running
before starting §1 so the improvement is measurable:

- **Roughness barely reads.** Across a 0.05 → 1.0 sweep a dielectric sphere
  changes only by the size of one small highlight dot. A "polished" surface and
  a matte one are nearly indistinguishable, because the specular lobe covers a
  few pixels and there is nothing else for gloss to show up in.
- **At roughness 0.05 the highlight almost vanishes**, making the mirror end of
  the sweep look *less* shiny than the middle. The lobe has narrowed below a
  pixel; with an environment term it would be showing a sharp reflection of the
  sky instead.
- **Metals are black.** A metal has no diffuse response and takes no ambient, so
  with nothing to reflect it renders as a black disc with one bright spot. Metal
  is not a usable material in this renderer today.

### Since resolved by §1.1 and §1.2

All three symptoms above are fixed. The sweep now reads monotonically from
mirror to matte, and metals show a sky-and-ground reflection with a horizon in
it. Two things had to change beyond adding the environment terms:

- **The sky was resolving its own HDR.** `sky.frag` applied a Reinhard curve and
  a gamma encode before writing into the linear HDR target, so its output could
  never exceed 1.0 and, after exposure, never reached the bloom threshold. The
  sun *could not* bloom however bright it was authored — which is the whole of
  why it "lacked lustre". It now writes linear radiance and lets the post chain
  resolve, as everything else does.
- **The sun needed angular size.** A directional light is a point at infinity,
  so its highlight narrows without limit and below a pixel it disappears — the
  reason polished looked flatter than satin. `SUN_SPECULAR_ROUGHNESS_FLOOR`
  gives it a minimum lobe width, set much wider than the real sun on purpose.

Balance is now sun-dominant: the sky supplies fill at `SKY_RADIANCE_SCALE`, the
sun keys well above it, and the flat ambient constant is nearly zero because the
sky does that job with direction. `SKY_RADIANCE_SCALE` sets the sky's on-screen
brightness *and* the fill it casts, and cannot be split — a mirror has to agree
with the sky next to it. Tune the ratio with sun intensity.

**Absolute light level is pinned by the content, not by taste.** The first
balance attempt raised total illumination on a sunlit surface from about 1.0 to
about 2.4 and looked fine on every bench scene — then washed the game out
completely. Level albedos are saturated primaries already close to full value,
authored against the old dim lighting, so extra illumination pushes them onto
the tonemap's shoulder and they go pale and electric instead of bright. Contrast
now comes from the key-to-fill *ratio* (about 5:1) while the absolute level
stays near where it was. Raising it means re-authoring albedos, which is a
deliberate project, not a side effect.

This is the "everything downstream is tuned against the lighting environment"
warning below, and content is downstream too.

**The bench scenes were the reason this was missed.** They were built from
muted, mid-value albedos — a comfortable place for a renderer to sit and a
misleading one to tune in. `palette` exists to fix that: saturated level colours
on bright terrain, which is where clipping and oversaturation actually show up.
Judge any exposure or tonemap change there first.

Still true after §1: nothing casts a shadow, so objects float; the ground and
sky sit at similar values with no aerial perspective between them.

---

## 1. Environment lighting — the plastic look

The highest value-per-effort cluster in this document. All three items are
changes to the shared lighting headers: no new render passes, no new resources,
and they improve terrain, props, grenades, explosions and the player character
simultaneously.

### 1.1 Specular environment reflection

Reflect the view vector about the surface normal and evaluate the sky along it,
weighted by Fresnel and blurred toward the sky's average colour as roughness
rises. Grazing angles on every object pick up a bright sky-coloured sheen.

Leans on: the sky already being an **analytic function** rather than a texture,
so this needs no cubemap, no probe capture and no extra pass — just the sky
colour evaluation factored into a header both shaders can include.

Likely to go wrong: the sky function is not cheap, and this evaluates it for
every lit fragment. If that bites, the mitigation is a coarse precomputed
representation (a handful of spherical-harmonic coefficients, or a small
cubemap refreshed when the sun moves) behind the same interface. Also, the
reflection knows nothing about occlusion — a surface deep inside a cave will
reflect open sky. AO is the mitigation, which is one reason these want doing
near each other.

### 1.2 Hemisphere ambient

Replace the flat constant ambient with a blend between sky colour from above
and a ground-bounce colour from below, chosen by the normal's vertical
component. Upward faces go cool and skylit, downward faces go warm and earthy.

Small change, disproportionate effect: it is what stops unlit surfaces reading
as flat grey, and it gives shape to everything the sun doesn't reach.

Likely to go wrong: nothing much. Worth doing even if 1.1 is deferred.

### 1.3 Clearcoat

A second specular lobe at a fixed low roughness layered over the base one, so a
surface can be a rough coloured diffuse underneath with a hard glassy glaze on
top. This is *the* toy-plastic ingredient — it's what separates painted plastic
from bare plastic, and lacquered wood from raw wood.

Likely to go wrong: it costs a material parameter and it is easy to apply
globally as a cheap gloss boost, which makes everything look uniformly wet.
Clearcoat is a statement that a surface has been *coated*; things that haven't
been — rock, dirt, cloth, chalk — must not have it, or the contrast that makes
it valuable disappears.

---

## 2. Material identity as physics readout

The strongest idea available to us, and the one no other game has, because no
other game is built around this particular showpiece.

Let surface appearance **encode physical parameters**, so the player learns to
predict the simulation by looking at it:

| Physics property | Reads as |
|---|---|
| High restitution | Deep saturated rubber, broad soft highlight, heavy clearcoat |
| High density | Dark, faintly metallic, tight highlight, low-frequency surface detail |
| Low friction | Near-mirror chrome or ice, roughness near zero |
| High friction | Chalky matte, visible grit, no clearcoat |
| Fracturable | Glazed ceramic outside, raw matte interior revealed on the fracture faces |

The mechanism that matters: derive a spawnable's `SurfaceFinish` from its
collider material rather than authoring the two independently. Visual variety
then comes for free with every new object, and the two can never drift out of
agreement.

Companion item: **per-instance variation.** A small hash-driven jitter on albedo
and roughness, so a pile of identical crates stops looking cloned. Physics debris
piles are exactly where cloning is most visible.

Likely to go wrong: the mapping is a lossy projection of many physics parameters
onto few visual ones, and it will sometimes fight art direction — a level may
want a black rubber ball and a black steel ball to look different despite the
table saying otherwise. Treat the derivation as a *default* that authored
material data can override, not as a law.

Also worth knowing: this only pays off if the material palette is legible in the
first place, which means resisting the urge to give every prop a unique bespoke
finish. Few, distinct, memorable finishes beat many similar ones.

### Landed 2026-08-09: the derivation and most of the wiring

`PhysicalSurface` (`src/rendering/physical_finish.rs`) maps friction,
restitution and density onto roughness and metallic. Friction sets the base
roughness, restitution pulls it towards gloss, density alone decides metal.
`physics_finish` on the visual bench shoots one sphere per archetype using the
real derivation and prints the numbers under each tile.

`src/app/spawnables/shared/finish.rs` is the seam that stops the two drifting:
a spawnable declares one `PhysicalSurface`, the collider takes
`with_physical_surface` and the material takes `with_derived_finish`. Around
twenty spawnables use it — the box family, the primitives, dolos, menhir,
trilithon, table, jack, jenga, domino, banana, fence post, hex prism, beach
ball, pendulum.

Three things learned by doing it, none of them obvious from the table above:

- **Bounce has to override grip, not average with it.** The two both want to own
  roughness. Averaging put a rubber ball and satin wood within 0.05 roughness of
  each other, which is invisible; letting restitution win separates them, and
  it is the cue a player acts on anyway.
- **The "broad soft highlight" for rubber is not available.** That is clearcoat
  (§1.3), and without it the only rubber cue is gloss — hence
  `ELASTIC_GLOSS_ROUGHNESS` at 0.18 rather than the mid-roughness the table
  implies. Revisit this when clearcoat lands.
- **Game densities are gameplay values, not physical ones.** A crate is
  50 kg/m³ and a "heavy" crate 150, a tenth of real timber, so nothing in the
  box family can ever read as metal however metallic its texture is. Props that
  were authored with real densities (menhir 2700, dolos 2400, pendulum frame
  7800, jack 7800) read correctly. Fixing the box family means changing physics
  a level is tuned against, and is a separate decision.

The multi-part structures followed the same day: play wheel, plank bridge,
seesaw, voussoir arch, house and temple. Only **trampoline** is deliberately
unwired — its bed carries the high restitution, so the derivation would make
canvas glossy, and that is the one case where the mapping is literally correct
and visually wrong.

Wiring them turned up a fourth instance of the same theme, and the sharpest one:

- **A whole structure usually shares one collider material.** House stone,
  brick and slate are all 1800 / 0.7 / 0.1, so all three render identically at
  roughness 0.56, and the temple was the same until marble was split out.

The temple split is worth reading before doing the same to the house, because
what stopped it was not taste:

- Marble is now its own surface (2700 / 0.4 / 0.05, roughness **0.36**) against
  the stylobate's rubble (2400 / 1.5 / 0.05, roughness **0.96**). Columns,
  entablature, pediments and roof are marble; only the steps are rubble.
- **The roof pitch puts a hard floor under how slick marble can be.** The roof
  panels are sloped slabs held by friction alone, so a panel needs
  `mu >= pediment_h / eave_dist`. The pediment used to be `half_w / PHI`, which
  approached a 31.7° roof demanding mu >= 0.62 — steeper than the friction angle
  of stone, and enough to pin marble at satin.
- **The proportion was the bug.** Doric gables are shallow: the Parthenon's is
  about 0.22 of its half-width, roughly 13°, where `half_w / PHI` is 0.618. The
  golden ratio was being applied to the one dimension the Greeks did not apply
  it to, which made the temple read gothic *and* created the physics problem.
  `DORIC_PEDIMENT_RATIO` = 0.22 fixes both: the roof now needs only mu >= 0.22
  at any width, so marble sits at a real dressed-stone 0.4 and renders polished.
- Real temples do not need joints for this, which is why none were added. Their
  roofs were **timber rafters carrying small overlapping tiles**, not stone
  slabs, and their iron clamps in lead resist spreading rather than sliding.
  Three tests in `temple.rs` hold the invariant, including one asserting the
  gable stays under 15°.
- **The arch's abutments land in the half-metal band** at metallic 0.16, because
  their density is doubled for stability rather than because they are metal.
  Masonry at 4000 kg/m³ is a gameplay value crossing a threshold set for real
  ones. It is subtle enough to leave, but it is the failure the narrow band was
  supposed to make rare, and a second offender would mean density is the wrong
  metal signal in a game whose densities are tuned rather than measured.

Still open here: **per-instance variation**, the fracturable row of the table
(interior material on fracture faces), and the trampoline.

---

## 3. Grounding — shadows and occlusion

Cross-reference, not new work: [LIGHTING_PLAN.md](LIGHTING_PLAN.md) Stage 1
(baked terrain AO, designed in [BAKED_AO_DESIGN.md](BAKED_AO_DESIGN.md)) and
Stages 5–7 (blob shadow, sun shadow map, cascades).

Recorded here because their *visual* importance is easy to underrate relative to
the flashier items. Nothing reads as a solid object in a real place without a
shadow anchoring it to the ground, and no stack of physics debris has weight
without darkening in the crevices where the pieces meet. Between them they are
worth more than every terrain and sky item in this document.

Two additions to what the lighting plan already covers:

- **Contact hardening.** A shadow that is sharp where the caster touches the
  ground and softens with separation. Cheap to approximate, and it is what
  communicates *how far above the floor* a tumbling object is — directly
  valuable for a physics showcase, and for judging a jump.
- **Screen-space contact shadows / AO for dynamic bodies.** Baked terrain AO
  covers the static world, but the moving objects are the point of this game.
  Some runtime term is needed for the occlusion *between* dynamic bodies, and
  between a body and the ground it is resting on.

---

## 4. Terrain

The problem is not that the noise texture is simple. It is that the material
doesn't respond to the geometry, so a cliff and a floor are the same substance
at different angles.

### 4.1 Geometry-driven material variation

Triplanar projection, with the material selected and blended by properties of the
surface itself: slope picks the substance (rock on cliffs, growth on flats),
curvature drives cavity darkening in concavities and wear on convex ridges.

Leans on: the fact that the terrain is generated, so slope and curvature are
already available or cheaply derivable at mesh time.

Likely to go wrong: blend thresholds tuned against one piece of terrain look
wrong on the next. The blend wants to be smooth and its parameters want to be
authorable per level rather than hard-coded.

### 4.2 Detail normals at close range

Microstructure on the marching-cubes surface so it has something for the new
environment specular to catch. Without this, item 1.1 makes terrain look like
smooth polished plastic, which is exactly wrong for rock.

### 4.3 Fresh destruction reveals interior material

Terrain carved out by an explosion should expose a different material to the
weathered outer surface — bright raw rock inside the crater against dull
weathered rock outside.

This is close to free: the destruction system already knows what it just carved.
It is a large amount of storytelling for one material parameter, it makes every
grenade permanently legible in the level afterwards, and it is squarely on the
physics-showcase theme.

Likely to go wrong: needs some notion of "age" if fresh damage should weather
over time, and it must survive the chunk re-mesh that destruction already
triggers. Simplest version — permanent, binary, never weathers — is probably
enough and should be tried first.

---

## 5. Sky and atmosphere

### 5.1 The sun

It lacks lustre because a bright disc is not a sun. Three things fix it:

- Push the disc's radiance far above the bloom threshold and let **bloom** do the
  work, rather than trying to make the disc itself look bright.
- **Limb darkening** across the disc, so it has a surface rather than being a
  flat circle.
- A **streak or anamorphic flare** in the bloom pass.

Likely to go wrong: a radiance high enough to look right may misbehave in the
tonemap or blow out the bright pass for everything else on screen. The bloom
threshold and the sun radiance are one tuning problem, not two.

### 5.2 Clouds

Layered scrolling noise on the sky dome with a strong forward-scattering silver
lining where the sun is behind them. The silver lining is the part that makes
clouds read as whimsical rather than as flat grey shapes — it is not an optional
polish detail, it is the effect.

Likely to go wrong: clouds are a well-known time sink and it is easy to end up
raymarching volumetrics. The cheap layered version is very probably enough for a
platformer where the camera rarely looks up for long.

### 5.3 Aerial perspective

Fade distant geometry toward the sky colour along the view ray. Costs almost
nothing and is one of the strongest "this is a modern renderer" signals there
is, because it is what makes *distance* legible — without it, a far cliff and a
near one are equally crisp and the eye can't order them.

---

## 6. Full-frame post

### 6.1 Grade and crispness

A colour grade (lift/gamma/gain, or a LUT) to lock the palette, a light sharpen,
a subtle vignette, and chromatic aberration confined to the frame edges. Grade
plus sharpen is most of what "crisp" actually means in practice.

Likely to go wrong: a grade is a global multiplier on every decision made
elsewhere, so it wants to be settled *before* materials are hand-tuned, or every
material gets tuned twice.

### 6.2 Simulation-reactive post

The genuinely on-theme category. Post-processing that responds to the physics
rather than sitting statically on top of it:

- Radial blur and a slight FOV punch scaled by player speed.
- A chromatic-aberration pulse and shake on heavy impacts, driven by the same
  impulse magnitude the physics engine already computes.
- Brief desaturation and recovery on the explosion flash, riding the blast light
  that already exists.

Leans on: the physics engine already knowing all of these quantities precisely.
The hook is the impulse, which the grenade detonation rule already reads.

Likely to go wrong: every one of these is nausea-adjacent and all of them want
to be subtler than first instinct suggests, and individually disableable.

### 6.3 Deliberately not doing

- **Depth of field** — hurts platformers, where you need to read the whole frame.
- **Heavy motion blur** — fights the crispness that is the entire target look.
- **Film grain** — same reason, more so.

---

## Suggested ordering

Not a schedule, just the order that maximises visible change per unit of work.

1. **Environment lighting** (§1) — one session, no new passes, transforms every
   pixel on screen including the existing work that already looks good.
2. **Sun shadows** (§3 / lighting plan stages 5–6).
3. **Material identity from physics** (§2).
4. **Ambient occlusion** (§3 / lighting plan stage 1, plus dynamic-body AO).
5. **Terrain** (§4).
6. **Sky, sun, clouds, aerial perspective** (§5).
7. **Grade and reactive post** (§6).

Item 1 is first because everything downstream is tuned against the lighting
environment, and doing it late means re-tuning all of it.
