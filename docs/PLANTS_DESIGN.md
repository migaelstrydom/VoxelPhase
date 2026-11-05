# Plants: one architecture for shrubs, trees and other greenery (design)

Status: agreed, 2026-11-05; not yet built. It reshapes `src/foliage/` (branch
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
               └─ Skin (by SkinKind)       ──▶ mesh, built once, with a give weight per vertex

  per frame, per touched mass:
     Bend: nearby colliders as they are, ground planes, squash ──▶ the GPU's
     vertex shader bends the still mesh (one rule, any skin)
```

### Skeleton

- **Shape:** limbs are capsules in a parent-child graph, grown per kind
  (a straight trunk, a fork, a gentle lean).
- **Physics:** a rooted plant's limbs are fixed colliders, so a trunk is
  solid.
- **Later:** breaking a tree (a compound body with fracture joints) is its
  own step, out of scope here.
- **Shrubs** have no skeleton.

### LeafMass: a volume and a skin

- **The volume stays concrete, not a trait.** A lobe gains an orientation
  and three radii: an ellipsoid. That one change covers:
  - a round topiary mass (equal radii);
  - a cypress (a tall lobe);
  - a palm's crown, for drag and wading (a flat, wide lobe under the
    blades).

  Drag, stride depth, the core and the bulk all keep reading lobes:
  - **Immersion:** a sphere's immersion in an ellipsoid is taken in the
    lobe's own scaled frame (approximate, fine for drag).
  - **Core:** a lobe's collider is a sphere when the lobe is round and a
    capsule along its long axis when it is not.
  - **Resting pose:** keeps its least-over-down-directions search, with
    lobe extents in place of radii.

  A trait would only pay for itself once a volume came along that
  ellipsoids cannot approximate, and no target kind needs one.
- **The skin varies, so it is a closed enum of generators.** Each one builds
  a `Skin` once (vertices plus a per-vertex **give**, 0..1: how freely that
  leaf bends):
  - `Nubbed`, today's bumpy mass, with two fixes. Subdivision is chosen per
    lobe for a fixed vertex spacing (about 3 cm, so 2–3 vertices across a
    bump). Vertices buried inside another lobe are dropped, along with
    their triangles.
  - `Blades`: rosettes of strap-shaped strips (palm, dragon tree, reeds),
    stiff at the base and free at the tip, shaded by the same leaf shader.
- **Shading and leaf particles** are shared by every kind. Particles get a
  budget per blast: today 9 clumps shed about 440.

### Bend: one rule, on the GPU

The skin is uploaded once and never rebuilt. Each frame the CPU hands each
touched mass a small list of primitives in its own frame, at most 8:

- **Bodies:** every collider meeting the mass's bounds (`colliders_meeting`
  sees sleeping bodies too), as it is: a sphere, capsule or box. A hull is
  passed as its box, its one remaining approximation.
- **Ground:** planes. A rooted mass gets them once, from ray probes at
  load. A loose one gets them from its contacts, as today.
- **Squash:** today's spring, kicked by impacts.

Each primitive keeps today's 0.2 s ease-back hold. The vertex shader moves
each leaf out of each primitive by today's fold-free map, written in terms
of the primitive's distance:
- a sphere's centre point, a capsule's segment and a box's thin core are
  its core, as `Pusher` has them now;
- a plane uses the knee.

Each displacement is scaled by the vertex's give. The rule lives once in
GLSL, mirrored by a Rust function that the tests exercise, with a test
holding the two together, as `swell.glsl` and `swell.rs` are held.

What this removes:
- the per-frame CPU bend (about 0.23 ms per moving clump);
- the per-frame upload (about 0.5 MB per moving clump);
- the `ProceduralMesh` / `ModelInstance` swapping.

What it costs the GPU: tens of microseconds of vertex work, run in the
scene pass and the shadow pass.

The renderer needs one new thing: per-draw bend data for a model (a small
uniform slot, or push constants). This is the only new coupling.

## What carries over unchanged

- **Engine:** air drag by a body's bulk (`dragging_as`), collision layers,
  blasts that strike by area.
- **Foliage:** `FoliageDrag`, the stride rule, uprooting, the leaf shader,
  and leaf particles.
- **Parting's map:** the fold-free map, moved into the shader.

## Migration

1. **Reshape, no change in look.**
   - `Lobe` becomes an ellipsoid;
   - `Skin` gains the per-vertex give;
   - `SkinKind::Nubbed` takes today's generator;
   - `LeafMass` gathers volume, skin, core and give.

   Gates: every test passes, and the arena renders the same.
2. **Bend on the GPU.** The shader, the Rust mirror and the tests that hold
   them together, then delete `Press` / `Parting` CPU deformation and
   `ProceduralMesh` for foliage.
   - Gate: `render_perf` with a blast among the shrubs. `foliage_skin` time
     and uploads should drop to near zero, with no visible change. Check
     against character_viewer `shrub_*` and an F3 snapshot by the arch.
3. **Fix the bumpy skin's triangles:** a fixed vertex spacing per lobe,
   with buried vertices dropped.
   - Gate: fewer triangles and an equal or sharper look on views of a small
     and a big lobe.
4. **Skeleton and the lollipop tree:** a trunk with one or more masses.
5. **Cypress:** a tall lobe.
6. **Blades and the palm:** the second skin kind.
7. **Review and squash** the branch, and rewrite `docs/FOLIAGE.md` as the
   plants doc.

Steps 1–3 are the shrub's debt; 4–6 each prove the design on a new kind.

## Decided (2026-11-05)

1. **Module names.** `foliage` keeps the leaf masses, and a new `plants`
   module above it holds the skeleton and `PlantSpec`.
2. **Ellipsoid lobes, not a volume trait.** Revisit only when a kind needs
   a volume ellipsoids cannot approximate.
3. **Trees breaking** (limbs with fracture joints) is a later step.
