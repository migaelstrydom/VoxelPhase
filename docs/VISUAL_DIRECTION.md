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
- Procedural sky with atmospheric scattering, written as linear HDR radiance.
- Sky-based environment lighting: hemisphere irradiance plus a specular sky
  reflection on a split-sum BRDF fit (`shader/environment.glsl`, §1.1 and §1.2).
- A single uncascaded sun shadow map (`src/rendering/shadow/`, lighting plan
  stage 6). Only the sun is shadowed — the sky's contribution arrives from the
  whole hemisphere and needs occlusion a shadow map cannot express.
- Baked per-vertex ambient occlusion on terrain (`src/terrain/ao.rs`), consumed
  by `shadeEnvironment` and `shadeAmbient`.
- Surface finish derived from collider physics on most spawnables (§2).

### What is still missing

- **No clearcoat** (§1.3), which is why rubber has no cue but gloss.
- **No occlusion on or between dynamic bodies.** The AO bake covers terrain
  only; every other vertex carries 1.0, which is the neutral value and the
  honest one. The moving objects are the point of this game, and they are the
  ones with no contact darkening.
- **One shadow cascade**, so the map's resolution is spread across the whole
  frustum fit (lighting plan stage 7).
- **Terrain's surface structure stops at the material level** — triplanar
  projection, detail normals and a physics-derived finish have landed (§4.1-4.3);
  slope zoning, grass sheen and fresh-cut destruction have not.

### How it looked before §1, and what fixed it

Retained because it is the measurement that justified the work, and because two
of the fixes are easy to re-break.

The renderer had no shadows, no ambient occlusion and no environment reflection
of any kind, which was the single biggest reason it did not look like the
reference. A surface reads as shiny because it reflects its surroundings, not
because it has a tight specular highlight; with nothing to reflect, a surface
could only be bright where a light happened to be, and everywhere else fell back
to flat ambient, which is a matte grey lie.

The `material_grid` bench scene made it concrete:

- **Roughness barely reads.** Across a 0.05 → 1.0 sweep a dielectric sphere
  changes only by the size of one small highlight dot. A "polished" surface and
  a matte one are nearly indistinguishable, because the specular lobe covers a
  few pixels and there is nothing else for gloss to show up in.
- **At roughness 0.05 the highlight almost vanishes**, making the mirror end of
  the sweep look *less* shiny than the middle. The lobe has narrowed below a
  pixel; with an environment term it would be showing a sharp reflection of the
  sky instead.
- **Metals are black.** A metal has no diffuse response and takes no ambient, so
  with nothing to reflect it rendered as a black disc with one bright spot.
  Metal was not a usable material.

**§1.1 and §1.2 resolved all three.** The sweep now reads monotonically from
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

The sun shadow map and the terrain AO bake have since closed the grounding half
of this. What remains from the same family: dynamic bodies neither receive nor
cast occlusion onto each other, and the ground and sky still sit at similar
values with no aerial perspective between them (§5.3).

---

## 1. Environment lighting — the plastic look

**1.1 and 1.2 have landed; 1.3 is the only item outstanding.**

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

Stage 1 and Stage 6 have landed, and Stage 5 was skipped as superseded. What is
left from the lighting plan is Stage 7 (cascades); what is left here is the two
items below, and the second of them is now the largest grounding gap in the
renderer.

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

### The register terrain is aiming at

Terrain is **exaggerated realism**, not the moulded plastic the props are heading
towards. Real materials behaving characteristically, pushed past life: grass
that is aggressively green and visibly furry, rock with a glint and a legible
grain, chalk that is chalkier than chalk. The props can read as manufactured
because they are; a hillside cannot, and giving it a manufactured finish is how
terrain ends up looking like a mould of a landscape rather than a landscape.

This is a correction to how §4 was originally written. The first draft asked for
cavity darkening, wear on ridges, and weathered-versus-fresh rock — the
vocabulary of accumulated history, borrowed from photoreal naturalism. Terrain
should look like a *material*, not like a material's biography. Where an item
below survives from that draft, it survives on different grounds.

### What terrain actually does today

Worth stating precisely, because two of these are easy to get wrong from memory:

- **Albedo is a flat constant per material.** `VoxelMaterial::color()`
  (`src/terrain/voxel.rs`) returns one RGB per material; marching cubes copies it
  from the solid corner of each edge and interpolates. Rock is 0.5 grey
  everywhere.
- **There is a texture, and it does very little.** `TerrainWorld::from_segments`
  generates a 512² five-octave noise texture and binds it. Its values are
  remapped to **[0.85, 1.0]** greyscale — a ±7.5 % multiplier on albedo, and
  nothing else. It modulates no normal and no roughness.
- **Its projection is top-down only.** Terrain vertices carry
  `tex_coords = (pos.x * 0.1, pos.z * 0.1)` (`src/terrain/mesh_octree.rs`), so on
  a vertical face the texture is smeared into infinite vertical streaks. This is
  a live defect, not a future concern; it has gone unnoticed because the
  modulation is too faint to see either way.
- **Roughness and metallic are per-draw.** They arrive as a fragment push
  constant (`shader/material.glsl`), so the entire terrain mesh has exactly one
  roughness. Every prop in the game now varies its finish; the world does not.
- **The only physics signal terrain carries is toughness.** `VoxelMaterial` has
  colour and `toughness()` and nothing else — no friction, no restitution, no
  density. Grass 1, sand 1, dirt 2, ite 3, limestone 4, rock 5, slate 8, bedrock
  indestructible.

So the summary line "the material doesn't respond to the geometry" was true but
aimed low. The deeper problem is that terrain has no surface *structure* at all:
one flat colour, one gloss, and a faint grey wash projected from above.

### The central idea

**Perceived roughness and BRDF roughness are different channels, and terrain is
missing the one that matters.** A rock glints because thousands of micro-facets
each catch the sun at a different angle. That is normal detail. The roughness
scalar cannot produce it — roughness is the statistical summary of detail you
have chosen *not* to model, so turning it up removes glint rather than adding it.

Three channels contribute to a surface reading as rough, very unequally:

| Channel | What it buys | Status |
|---|---|---|
| Albedo variation | Weakest; reads as dirt rather than as texture | The noise texture, at ±7.5 % grey |
| **Normal detail** | Glint, grit, legible grain — the whole effect | **Does not exist** |
| Roughness variation | Chalky versus slick, as a class | One value for all terrain |

Detail normals are therefore the core of §4, not a polish pass appended to it,
and everything else here is either their delivery mechanism or a modulation of
them.

### 4.1 Triplanar projection

Project world position along all three axes, sample each, blend by the squared
normal components. Prerequisite for every other item in this section: it is the
only way to get a coherent surface parameterisation onto an isosurface that has
no UVs and that gets re-meshed by every grenade.

Leans on: nothing new. The sampling point is world position, which the fragment
shader already has.

Likely to go wrong: it triples texture sampling cost on the largest mesh in the
game, and the blend exponent is a real tuning parameter — too low and the three
projections ghost against each other, too high and the transition bands narrow
into visible seams on 45° faces.

Worth noting that this is a fix, not a feature. Landing it alone will make cliffs
stop streaking and will otherwise look almost identical, because the thing being
projected is a ±7.5 % grey wash. Judge it on cliffs, and do not expect it to
justify itself on its own.

#### Landed

`TriplanarProjection` (`src/rendering/triplanar.rs`) rides in `SurfaceParams`
as a scale and a blend sharpness; a zero scale means the mesh has its own UVs
and `triangle.frag` samples them as before, so nothing but terrain changed
path. The sampling is `shader/triplanar.glsl`. Terrain's own view of its
appearance — the noise texture's parameters and the projection it is addressed
by — moved to `src/terrain/surface.rs`, which the game and the bench now both
call, because the bench was drawing terrain untextured and so could not have
shown this defect at all.

Three things learned:

- **The scale had to match the UV it replaced** (0.1 repeats per world unit,
  which is the Y plane of the projection), or the change would have silently
  re-scaled every level's floor while claiming to only fix cliffs. Guarded by a
  test.
- **The defect was invisible at the authored contrast.** A ±7.5 % grey wash
  hides its own smearing; the streaks only become obvious with the noise
  temporarily amplified, which is how the before/after was confirmed. Any future
  judgement of terrain surface work should amplify first and restore after.
- **Push constants are nearly full.** The projection took the range to 120 bytes
  of the 128 Vulkan guarantees, so §4.2 and §4.3 have 8 bytes left and the next
  parameter after that needs a different delivery. A test now asserts the
  budget rather than leaving it to fail at pipeline creation on whichever device
  sits at the minimum.

### 4.2 Detail normals, coupled to roughness

Perturb the shading normal with procedural high-frequency detail delivered
through the triplanar projection, so terrain has microstructure for the sun and
the §1.1 environment specular to break up against. This is what "rocky glint"
and "visible roughness" actually are.

**The coupling is the design, and it is not optional.** High-frequency normals
under a narrow specular lobe produce specular aliasing — sparkle that crawls and
fireflies as the camera moves, which is the single most legible symptom of a
cheap renderer and fights the crispness the whole document is chasing. The fix
is to treat normal detail and roughness as the same parameter at two scales:
where normal variance within a pixel is high, roughness rises to absorb it
(Toksvig / LEAN-style filtered normals). Detail that has shrunk below a pixel
stops being geometry and becomes gloss.

That coupling also solves distance for free — a cliff glints up close and goes
correctly matte far away with no hand-authored LOD fade — which is the reason to
build it as one system rather than landing detail normals and tuning roughness
after. Tuned separately, the two will be re-tuned against each other forever.

Likely to go wrong, beyond the aliasing: the detail has to be derived in the
fragment shader rather than baked into vertices. Terrain vertex buffers are 52
bytes per vertex and re-uploaded every frame, so new vertex attributes cost
bandwidth on the biggest mesh in the game — see the per-frame upload item in
TODO.md, which this would make worse.

#### Landed

The detail normal is packed into the surface texture's spare channels — R holds
the albedo wash it always held, GB hold a tangent-space normal derived from the
same height field — so the three triplanar samples terrain already paid for now
deliver both. **The whole feature costs no additional texture reads.** Relief
strength comes from §4.3's hardness, so chalk is rounded and rock is sharp, and
roughness is widened by the screen-space variance of the final shading normal
(`filteredRoughness` in `shader/lighting.glsl`).

- **Value noise cannot be differentiated.** The first attempt reused the
  engine's existing fbm and printed the lattice across every surface as
  axis-aligned banding. Value noise interpolates a random *value* per lattice
  point, which forces every lattice line to be an extremum, so its slope field
  is full of structure the value field hides. Gradient noise was added alongside
  it (`perlin_2d_periodic`) rather than replacing it, because terrain generation
  is built on the old one and swapping its field would reshape every level.
  Quintic fade rather than smoothstep, for the same reason: smoothstep's second
  derivative is discontinuous and creases the normals.
- **Whiteout blending, not averaging.** Averaging three world-space perturbed
  normals lets the planes cancel where two of them contribute, so a 45° surface
  comes out visibly smoother than the flat ground beside it.
- **The specular filtering earns its place.** It cuts pixel-to-pixel contrast on
  a rocky crater floor by about 15%, and touches almost nothing else — 0.1% of
  the pixels on the prop sheet, confined to silhouettes, where the ceiling keeps
  it from putting a dull rim around every object. The distance ladder in the new
  `terrain_detail` sheet goes grain → gloss → smooth with no authored fade.
- **A still frame cannot prove the aliasing is gone.** Crawling is a property of
  motion. The far tiles being speckle-free is necessary evidence, not
  sufficient; the sufficient test needs the game window.

**Exposed, not caused: sloped ground is faintly terraced.** The bands survive
zeroing the detail strength, so they are the generator's rather than the
renderer's; the relief only makes them legible. Genuinely flat ground is clean —
it is *slopes* that band, which is the signature of a surface stepping between
lattice planes rather than of general mottling. Tracked in
[TODO.md](TODO.md#terraced-slopes-in-generated-terrain). Worth knowing about
before §4.4, whose slope threshold would track the same artifact.

### 4.3 Surface character from toughness

Derive terrain's finish from `VoxelMaterial::toughness()` the way a prop's finish
is derived from its collider (§2), so the same rule holds in the world as on the
objects: what you can see predicts what will happen.

Soft, low-toughness materials read chalky and matte with soft-edged detail; hard,
high-toughness rock reads dense, tight-grained and glinty; bedrock reads slick
and near-black, which it already half does by colour alone.

This is the strongest item in §4 on the project's own terms. Destruction is the
showpiece, toughness is the parameter that decides how destruction goes, and a
player currently **cannot see it** — grass and rock differ by a flat colour
somebody chose. Being able to look at a wall and know whether a grenade goes
through it is worth more than any amount of surface grit.

Requires per-fragment roughness, which means moving roughness off the push
constant for the terrain path. That is a pipeline change rather than a shader
tweak, and it is the enabling work for §4.2 as well.

Likely to go wrong: toughness is a gameplay-tuned number, not a measured one, so
the same trap §2 hit with box densities applies — a level that retunes toughness
silently retunes the look. Keep the mapping monotonic and coarse, so that only
large toughness differences produce visible ones.

If a richer derivation is wanted later, the move is to give `VoxelMaterial` the
same `PhysicalSurface` the props use. That means terrain friction becoming
per-material in the physics engine, which is a physics change with gameplay
consequences, and should not be smuggled in as part of a rendering item.

#### Landed

`VoxelMaterial::hardness()` normalises toughness to 0–1 through
`t / (t + 4)`, marching cubes carries it per vertex from the same corner the
colour comes from, and `shader/surface_character.glsl` maps it to roughness
between 1.0 (chalk) and 0.45 (dense stone). The bench sheet is
`terrain_finish`: a shallow dish blown through a stack of all seven
destructible layers plus bedrock, so every material is exposed at once.

Three things worth carrying forward.

- **Indestructible is the limit of the curve, not a special case.** A saturating
  `t / (t + k)` puts bedrock at exactly 1.0 as toughness grows without bound,
  which means there is no branch to keep in step and no way for a new material
  to land outside the range the shader expects.
- **No new vertex attribute was needed.** §4.1 left terrain's texture
  coordinates dead — a world-projected mesh never reads them — so hardness went
  in that channel, and the projection scale the shader already branches on picks
  which meaning applies. The vertex stayed at 52 bytes, which matters because
  terrain buffers are the largest mesh in the game and are re-uploaded every
  frame.
- **The claim above — that this "delivers chalky-versus-slick immediately" — was
  optimistic.** Measured against a control with the map flattened, it changes
  34 % of the dish's pixels by a mean of 2.2/255, concentrated correctly on the
  hard rings and rising monotonically inwards. But pushing the hard endpoint
  from 0.45 all the way to 0.10 moves the image by only a further 1.8/255: the
  dial is near-saturated. A dielectric reflects about 4 % of a fairly uniform
  sky, and a broad smooth surface gives one narrow band where the sun's
  half-vector lines up, so there is very little for a tighter lobe to catch.
  Roughness on its own is not yet a legible readout. §4.2 is what supplies the
  microstructure it needs, which makes this enabling work first and a visible
  feature second — the ordering was right for the wrong reason.

A trap found on the way, now documented at `MaterialLayer::depth`: layer depths
are *boundaries*, not thicknesses, and anything below the deepest one falls
through to a built-in default ladder. Equal depths silently make every layer
after the first unreachable, and a gap above the bedrock puts a ring of the
wrong material in the middle of a crater.

### 4.4 Slope zoning

Blend between materials by the normal's vertical component: growth on flats,
bare substrate on steep faces.

**Slope modulates finish, it does not choose substance.** The voxel material
stays the authority on what a thing is, because it is what destruction spends
energy against — a grass voxel rendered as rock on a steep face would be a
surface lying about its own physics, which is the exact failure §2 exists to
prevent. Slope says "nothing settles here, so you see the bare material",
which is both true and consistent with a fresh crater wall.

Likely to go wrong, and this one is specific: **the normal being thresholded is
least accurate exactly where the threshold matters.** Gradient normals are exact
on smooth ground and smeared at rims and creases (see the crease-normal
reference), so a tight slope threshold puts a band of the wrong finish around
every cliff edge and crater rim, tracking the smear rather than the geometry. The
blend has to be wide enough to hide that, which limits how crisp the zoning can
be.

Second, hand-authored levels mottle. `Voxel::solid` writes density ±1 with no
sub-voxel offset, the quantised regime that already mottles baked AO; the same
quantisation mottles the normal, so slope blending will mottle on hand-built
terrain and be clean on generated terrain.

### 4.5 Grass sheen

Not a roughness item at all, and separable from everything above. Fur reads as
fur through **sheen and backscatter**: grass lights up when the sun is behind it,
with a bright soft rim instead of a highlight. That is a diffuse-side BRDF term,
cheap, and it is most of what makes grass look furry rather than look like green
paint.

Worth keeping distinct precisely because it is the one item here that no amount
of normal detail or roughness tuning will produce.

### 4.6 Fresh destruction reads as fresh

Terrain carved by an explosion exposes a different surface to the one outside the
crater. Kept from the original draft, but not as weathering: the interior is
simply a **different face of the same material** — unbroken grain, sharper
detail, cleaner colour — rather than an aged exterior versus a fresh interior.
No notion of age, no weathering over time, permanent and binary.

Close to free, since the destruction system already knows what it carved. Large
storytelling return for one material parameter, it makes every grenade
permanently legible in the level afterwards, and it is squarely on the
physics-showcase theme.

Likely to go wrong: it must survive the chunk re-mesh that destruction already
triggers, which means the interior flag lives in the voxel data rather than in
the mesh.

### Cut: curvature-driven cavity darkening and ridge wear

Dropped from the original §4.1 rather than deferred, for three independent
reasons. It is the naturalism item, and terrain's register is material rather
than history. Baked AO already does cavity darkening, and does it properly
instead of by proxy. And it is the expensive one: curvature is not available in
the fragment shader, so it would need baking at remesh time, and remesh time is
the constraint that shaped the entire AO design — a grenade is already at
roughly 23 ms with AO enabled.

The only part with independent value is the *convex* half — wear on ridge crests,
which AO cannot express because it saturates at "fully open" and cannot tell flat
ground from a crest. If that turns out to be missed, it comes back on its own
merits and with its own perf budget, not attached to §4.1.

### Ordering within §4

1. ~~**4.1 triplanar**~~ — landed. Prerequisite, and a defect fix, but nearly
   invisible alone, exactly as predicted.
2. ~~**4.3 per-fragment roughness from toughness**~~ — landed. Carries the
   per-fragment surface data 4.2 also needs. It was expected to deliver
   chalky-versus-slick on its own and does not; see the measurement above.
3. ~~**4.2 detail normals with variance-coupled roughness**~~ — landed, and it
   was the core: by far the largest visible change in this section, and the
   thing that finally made 4.3's roughness legible.
4. **4.5 grass sheen** — next by default: small, independent, and now the most
   obvious remaining gap, since grass currently reads as textured ground rather
   than as anything furry.
5. **4.4 slope zoning** — on top of a surface system that already works.
6. **4.6 fresh destruction** — last, because it is a modulation of everything
   above.

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

1. ~~**Environment lighting** (§1)~~ — done but for clearcoat (§1.3). Was first
   because everything downstream is tuned against the lighting environment, and
   doing it late means re-tuning all of it.
2. ~~**Sun shadows** (§3 / lighting plan stages 5–6)~~ — done, uncascaded.
3. ~~**Material identity from physics** (§2)~~ — done but for per-instance
   variation, fracture interiors and the trampoline.
4. **Ambient occlusion** (§3) — terrain bake done; **dynamic-body occlusion is
   what remains**, and it is the larger half.
5. **Terrain** (§4).
6. **Sky, sun, clouds, aerial perspective** (§5).
7. **Grade and reactive post** (§6).
