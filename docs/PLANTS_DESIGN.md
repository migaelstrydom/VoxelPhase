# Plants: one architecture for shrubs, trees and other greenery (design)

Status: agreed, 2026-11-05; building breadth-first (see Migration):
plants first, with skins that do not bend; bending, blades and trees
breaking come later. It reshapes `src/foliage/` (branch
`foliage-prototype`) so that one architecture grows many kinds of plant,
replacing today's single kind: round lobed topiary. Levels place a few set
pieces each, so a plant may be rich, but there must be many kinds.
`docs/FOLIAGE.md` describes what exists today.

## What it costs today (render_perf, Test Arena, 2400×1600)

The same camera on the foliage corner and the same grenade among the
shrubs, with the level's 9 clumps and without them:

| | quiet, with | quiet, without | blast, with | blast, without |
|---|---|---|---|---|
| triangles | 1,085k | 928k | 1,084k | 927k |
| GPU total, ms | 5.51 | 5.08 | 11.8 | 10.3 |
| GPU scene pass, ms | 3.83 | 3.28 | 3.57 | 3.49 |
| simulate (CPU), ms | 1.02 | 0.96 | 4.76 | 1.57 |
| uploaded MB/frame | 2.2 | 2.1 | 8.4 | 3.6 |
| particles | 47 | 45 | 586 | 145 |

- **Still clumps** cost the GPU about 0.45 ms, for 157k triangles (about
  17k per clump). The GPU's big spender during a blast is water (5.5 ms
  without foliage), not foliage.
- **Moving clumps** are the real cost. All 9 torn loose cost the CPU about
  3.2 ms per frame:
  - `foliage_skin` bends and rebuilds about 2.1 ms of it, re-uploading
    about 4.8 MB a frame;
  - the loose cores' physics, about 0.5 ms;
  - the rest is their leaf particles.
- **Leaf particles** (about 440 more) add about 1.3 ms to the GPU's
  transparent pass.
- **Triangles are spent badly.** Every lobe is the same 2,562-vertex sphere
  whatever its size, and much of each is buried inside its neighbours. A
  big lobe's vertices are about as far apart as its bumps are wide, so the
  bumps blur into lumps.

## Target kinds

The reference set (`scratch/aesthetic-direction/`) has four kinds, chosen
to stretch the design:

| Kind | Woody part | Leaves |
|---|---|---|
| Topiary shrub (today) | none | one round, bumpy mass |
| Lollipop tree | slender trunk | one or a few round masses on top |
| Cypress | short trunk | one tall, narrow mass |
| Palm or dragon tree | trunk, maybe forked | rosettes of blades at the tips |

If the layers below hold all four on paper, they should hold most others,
such as fruit trees or reeds.

## The layers

```text
  PlantSpec (RON: kind parameters + seed)
     │ grow
     ▼
  Plant ──▶ Skeleton: limbs (capsules in a tree graph) ── rooted: fixed colliders
     │                                                    (a trunk is solid)
     └────▶ LeafMass × n, each on a limb (or on the ground, for a shrub)
               ├─ Volume: ellipsoid lobes ──▶ drag, stride depth, core, bulk, rest pose
               └─ Skin (by SkinKind)       ──▶ mesh, built once, with give and sprig per vertex

  later (parked), per frame, per touched mass:
     Bend: up to 8 primitives (nearby colliders, ground planes) and the squash
       ──▶ a per-frame bend table ──▶ the vertex shader (scene and shadow)
           bends the still mesh: one rule, any skin
```

### Skeleton

- **Shape:** limbs are capsules in a parent-child graph, grown per kind
  (a straight trunk, a fork, a gentle lean).
- **Physics:** a rooted plant's limbs are fixed colliders, so a trunk is
  solid.
- **Later:** breaking a tree (a compound body with fracture joints) is its
  own step, out of scope here.
- **Shrubs** have no skeleton.

### Rooting

`Rooting` gains a third state:

- **Rooted:** on the ground, no body (today's shrub).
- **On a limb:** a leaf mass held by its plant's skeleton. A blast does not
  tear it loose; it stays until limbs can break (decided: later).
  `FoliageUprootSystem` today tears loose every rooted clump in range, so
  this state must be added before the first tree (step 4).
- **Loose:** a dynamic core (today's).

### LeafMass: a volume and a skin

- **The volume stays concrete, not a trait.** A lobe gains an orientation
  and three radii: an ellipsoid. That one change covers:
  - a round topiary mass (equal radii);
  - a cypress (a tall lobe);
  - a palm's crown, for drag and wading (a flat, wide lobe under the
    blades).

  Every place that reads a lobe as a sphere changes with it:
  - **Immersion** (`drag.rs`): a sphere's immersion in an ellipsoid is
    taken in the lobe's own scaled frame (approximate, fine for drag),
    including the cap on how deep a small lobe can be immersed.
  - **Core** (`leaf_core.rs`): chosen by the lobe's proportions:
    - a sphere when it is round;
    - a capsule along its axis when it is prolate (a cypress);
    - a few spheres across it when it is oblate (a palm crown), since an
      oblate lobe has two long axes.
  - **Resting pose:** it measures the core colliders, not the lobes, so it
    keeps its least-over-down-directions search over each collider's
    support (a capsule's is `c·d + h·|a·d| + r`).
  - **Bulk:** `physics::Volume::at` takes a translation only, and there is
    no ellipsoid collider shape. A lobe's bulk is its core collider's shape
    scaled up, which needs `Volume` to take a rotation: a small engine API
    change, made on the engine branch.
  - **The rest:** volume and centre of mass (`r³`), `bounding_radius` and
    the skin's lobe occlusion.

  A trait would only pay for itself once a volume came along that
  ellipsoids cannot approximate, and no target kind needs one.
- **The skin varies, so it is a closed enum of generators.** Each one builds
  a `Skin` once. It carries two values per vertex in the vertex's texture
  coordinates, which foliage leaves unused (its material is untextured), so
  no other mesh changes format:
  - **give**, 0..1: how freely that leaf bends;
  - **sprig**: how far it stands out of the valleys, which pressing reads
    today.

  The generators:
  - `Nubbed`, today's bumpy mass, with two fixes. Subdivision is chosen per
    lobe for a fixed vertex spacing (about 3 cm, so 2–3 vertices across a
    bump). Vertices buried inside another lobe are dropped, along with
    their triangles. Bumps are sized in metres, not as a share of the lobe,
    so a cypress's are not stretched.
  - `Blades`: rosettes of strap-shaped strips (palm, dragon tree, reeds),
    stiff at the base and free at the tip.
    - **Both sides drawn:** an open strip vanishes from behind and shadows
      from one side only. Either doubled faces or a cull-none pipeline
      variant; to decide in step 6.
    - **Shading:** the leaf shader's wrap and edge transmission assume a
      closed mass with outward normals. A thin blade needs its own terms
      (light through the blade by its facing, not by `1 − n·v`), as a
      variant of the same shader.
    - **Bending a blade** vertex by vertex stretches it when a body pushes
      through the crown. Acceptable for small pushes; check it in step 6.
- **Shading** is shared by every kind, with the blade variant above.
- **Leaf particles** are shared too, under one world-wide cap on live
  leaves (300), so a tree's masses shed as a shrub's do without a budget
  of their own.

### Bend: one rule, on the GPU (parked)

Parked on 2026-11-05: skins are drawn as grown until the plants are in.
The CPU bending was removed then (`docs/FOLIAGE.md`), and with it the
sprig heights; restoring them is part of this step. The design stands:

The skin is uploaded once and never rebuilt. Each frame the CPU hands each
touched mass a small list of primitives in its own frame, at most 8.

What is gathered:
- **Bodies:** colliders meeting the mass's bounds (`colliders_meeting`
  sees sleeping bodies too), as they are: a sphere, capsule or box. A hull
  is passed as its box, its one remaining approximation.
  - **The plant's own** skeleton and core are excluded, or a crown would
    part round its own trunk.
  - **Ranked:** past the cap, the deepest into the leaves are kept. A body
    of many colliders (a neighbour clump's 6 core spheres) is merged into
    its bounding sphere or capsule first, so it costs one slot.
- **Ground:** planes.
  - A rooted mass gets them from ray probes at load, and again when a
    terrain edit meets its bounds (a blast can carve nearby without tearing
    it loose).
  - A loose one gets them from its contacts, as today.
- **Squash:** today's spring, kicked by impacts.

Each primitive keeps today's 0.2 s ease-back, held per (body, collider), so
a primitive the cap drops fades out rather than popping. The vertex shader
moves each leaf out of each primitive by today's fold-free map, written in
terms of the primitive's distance:
- a sphere's centre point, a capsule's segment and a box's thin core are
  its core, as `Pusher` has them now;
- a plane uses the knee.

Each displacement is scaled by the vertex's give. The order (squash,
parting, then pressing) reads no neighbouring vertices, so it moves to the
shader exactly. The leaf noise keeps reading the bent position, as it does
today, so the look does not change.

**The rule lives once in GLSL,** mirrored by a Rust function. Two tests hold
them:
- the Rust mirror against today's CPU `deform` on fixed inputs, so moving
  the bend changes nothing;
- the GLSL against the mirror: a headless compute pass runs the shader's
  bend on test points and compares. The `swell` test only checks shared
  constants, and would not catch the logic drifting.

**How the data reaches the shader.** Push constants cannot carry it: the
geometry push block is already 112 of Vulkan's guaranteed 128 bytes. It
follows the surface table's pattern (`surface_buffer.rs`):
- **A bend table:** a fixed-capacity storage buffer, one per frame slot, at
  a new set 0 binding visible to the vertex stage. It is bound once and
  never resized, so the in-flight rules hold.
- **Per draw:** the draw pushes an index into it (in the push block's
  padding), with "none" for every other model.

It touches:
- `descriptors.rs` (binding and layout) and the in-flight frame;
- the geometry push and draw (`GeometryPush`, `submit_draw`, `draw_model`);
- `RenderSystem`, which reads a new bend component;
- the main vertex shader;
- the shadow path: a caster carries the index, and `shadow.vert` bends.

Reflection probes draw the leaves unbent; acceptable.

What this removes:
- the per-frame CPU bend (about 0.23 ms per moving clump);
- the per-frame upload (about 0.5 MB per moving clump);
- the `ProceduralMesh` / `ModelInstance` swapping, which today also builds
  a new model on every swap.

What it costs the GPU: tens of microseconds of vertex work, run in the
scene pass and the shadow pass. The main view and shadow casters are not
culled by bounds, and probe bounds missing a bent edge is harmless, so
leaves bent past their rest bounds need nothing.

## What carries over unchanged

- **Engine:** air drag by a body's bulk (`dragging_as`), collision layers,
  blasts that strike by area.
- **Foliage:** `FoliageDrag`, the stride rule, uprooting of rooted masses,
  the leaf shader, and leaf particles.
- **Parting's map:** the fold-free map, to move into the shader when
  bending returns (it is in git history before the commit that removed
  it).

## Migration

Breadth-first: every kind playable before any looks its best.

Done:
- **Leaf drag:** a fluid by real shape, on both sides.
- **Bending removed:** skins drawn as grown, the core at 80% of each lobe.
- **Blasts and throws:** a porous bulk catches a share of a blast
  (`BulkShape::porous`); a throw is limited by the hand.
- **Ellipsoid lobes:** drag and wading read them as ellipsoids; the core,
  bulk and resting pose read each as a sphere when round and a capsule
  along each of its long axes when stretched, so a column lies down and a
  mound sits flat. Round shrubs are unchanged vertex for vertex.
  `SkinKind` waits for the second skin kind (blades) and `LeafMass` for the
  first plant with several masses (step 3): neither has anything to do
  until then.
- **Skin in metres:** each round lobe's geodesic sphere is sized for about
  2.5 cm between vertices, and a stretched lobe is meshed in rings round
  its long axis at that spacing (a geodesic sphere sized for a column's
  length had five times the triangles); buried vertices are dropped, and bumps are sized
  in metres and spread evenly by area. Foliage triangles fell from about
  157k to 117k. A shrub can be grown to a height, as a column or a mound.
- **Leaf particles:** besides landings and uprooting, whatever moves
  through a clump knocks leaves off (`rustle.rs`), and a clump flying faster
  than 4 m/s trails them (`trail.rs`). At most 300 are alive at once
  (`ParticleSpec::most_live`); past that a new leaf replaces the oldest.
  The arena's blast among the shrubs peaked at 896 particles without the
  cap and 477 with it, for about 0.2 ms of GPU.

Next:
3. **Skeleton and the lollipop tree:** the `plants` module, a trunk with one
   or more masses, and the "on a limb" rooting state.
   - A skin hands over its mesh as a shared `Arc<Model>`: today
     `Foliage::model` copies the vertices, so each clump's mesh is held
     twice in CPU memory.
4. **Cypress:** a tall lobe on a short trunk.

Later:
- **Bending on the GPU,** as designed above: 2a moves the old behaviour
  exactly, 2b adds the new.
- **Blades and the palm:** the second skin kind, drawn from both sides,
  with its own shading terms and an oblate crown's core if it can come
  loose.
- **Trees breaking.**
- **Rewrite `docs/FOLIAGE.md`** as the plants doc.


## Decided (2026-11-05)

1. **Module names.** `foliage` keeps the leaf masses, and a new `plants`
   module above it holds the skeleton and `PlantSpec`.
2. **Ellipsoid lobes, not a volume trait.** Revisit only when a kind needs
   a volume ellipsoids cannot approximate.
3. **Trees breaking** (limbs with fracture joints) is a later step.
