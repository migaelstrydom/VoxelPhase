# Level Segments Plan

Design document for restructuring the level system around **segments**: independently
placed, independently resolved chunks of voxel terrain joined by an anchor graph.

Supersedes the single-cube world model described in `LEVEL_FILE_FORMAT.md`. That
document remains the reference for object/spawnable syntax, which this plan does not
change.

---

## Problem statement

The level system was built around one cubic SVO spanning `[-world_size, +world_size]` on
every axis, filled by a heightfield pass plus volumetric features, all addressed in
absolute world coordinates. Three distinct problems follow from that.

### 1. The cube cannot express platformer geometry

A platformer level is a *sequence of areas* — a long thin corridor, a spiral tower, a
wide arena — not a cube. Fitting a 400 × 40 × 80 unit level into a cube requires
`world_size: 256`, a 512³ box of which the level occupies under 4%.

This is not merely wasteful. `Level::octree_depth()` computes `log2(world_size /
voxel_size)` while the bounds span `2 * world_size`, so `min_voxel_size` lands at
**twice** the declared `voxel_size` — every shipped level declaring `voxel_size: 1.0` is
running 2.0m voxels. Combined with the depth cap of 10 in `loader.rs`, sub-metre voxels
over a large extent are arithmetically unreachable. The factor-of-two is fixed by this
plan's removal of the `world_size`/depth calculation, not by a separate patch.

### 2. Generation cost scales with the bounding box, not the content

`generate_terrain` pass 1 walks every `(x, z)` column in the full footprint regardless of
whether any feature touches it. A long thin level in a cube pays for the whole cube.

### 3. Absolute coordinates do not compose

Every feature and object is positioned in one flat namespace. Nothing is reusable,
nothing is relocatable, and moving one area means renumbering everything downstream. This
is the dominant practical blocker: neither a human nor an agent can hold a
forty-landmark coordinate space in working memory, which is why hand-authored levels in
this format have been mediocre regardless of who wrote them.

---

## Solution overview

```text
  Level
    │
    ├── Segment "start_plaza"      frame: (0,0,0) yaw 0     voxel 1.0m
    │     ├── chunk grid (local coords, own resolution)
    │     ├── terrain features (local coords)
    │     ├── objects (local coords)
    │     └── anchors: entry, exit_north
    │
    ├── Segment "spiral_tower"     placed: exit_north → entry
    │     └── ...                                            voxel 0.5m
    │
    └── Segment "summit_arena"     placed: exit_top → entry
          └── ...                                            voxel 0.25m
```

A **segment** owns a coordinate frame, its own sparse grid of voxel chunks, its own voxel
resolution, its terrain features, and its objects — all addressed in segment-local
coordinates. Segments are placed either absolutely or by snapping one of their anchors
onto another segment's anchor.

Segments are a **runtime concept**, not an authoring convenience flattened away at load.
Segment identity survives into the running game, which is what later enables streaming,
per-segment reset, and checkpoints. Flattening to absolute coordinates at load would be
less work now and would have to be undone later; retrofitting identity onto flattened
geometry is materially harder than carrying it from the start.

### Why per-segment resolution

Detail budget is not uniform across a level. A boss arena or a set-piece wants 0.25m
voxels; a long traversal corridor is fine at 1.0m and would be wasteful finer. Because
each segment owns its own chunk grid rather than sharing a global lattice, resolution
falls out as a per-segment parameter at no structural cost.

---

## Design rules

Three constraints make the above tractable. Each trades a small amount of authoring
freedom for a large implementation saving, and each is enforced by `level_check`.

### Rule 1 — Terrain may only be continuous across a boundary at equal resolution

Marching cubes across a face where one side has 0.5m voxels and the other 1.0m produces
cracks: the shared face's edge interpolation does not agree between resolutions. Solving
this properly requires Transvoxel-class boundary machinery, which is out of scope.

The normal case is that segments connect at **anchors** — a doorway, a ledge, a bridge
mouth — with terrain not continuous across the join, which makes mismatched resolution
harmless.

But some structures genuinely do span segments: the central column of a spiral tower runs
through every quarter-turn segment. That is permitted **provided the segments share a voxel
resolution**, because equal-resolution grids stay aligned and mesh without a seam. Where a
continuous structure would cross a resolution change, either equalise the two segments or
express the structure as a spawnable instead.

`level_check` warns when two segments abut over a face with differing voxel size, since
that is the configuration where a visible crack is possible.

### Rule 2 — Placement rotation is restricted to 90° yaw multiples

Arbitrary yaw rotates a segment's voxel grid against the world, which turns AABB queries
into OBB queries and forces per-triangle transformation of physics patches. Restricting to
90° multiples makes local↔world an axis swap plus a translation: exact, no interpolation,
no rotated bounds, and grids of power-of-two-related resolutions stay mutually aligned.

For a platformer, 90° turns are the natural layout vocabulary, so the authoring cost is
near zero. Pitch and roll are not supported; slanted geometry is expressed *within* a
segment via terrain features.

### Rule 3 — Segments have static frames; anything that moves is a body

A segment's frame is fixed at load. Moving geometry — floating platforms, lifts, rotating
bridges — is **not** voxels. See "Mobile geometry" below for why this is forced rather than
chosen.

(Translation-only segment motion would not violate Rule 2, since translation preserves axis
alignment. It is left open as a possible future for large set-pieces like a moving ship, but
nothing in this plan depends on it.)

### Rule 4 — Segments may not contend for chunk space

Two segments overlapping at different resolutions has no well-defined meaning: neither
grid is authoritative.

The check is at **chunk granularity, not bounding box**: no two segments may own allocated
chunks whose world extents overlap. Comparing whole-segment AABBs is both too strict — two
interlocking L-shaped segments have overlapping AABBs while contending for nothing — and
too coarse to describe the actual resource. `level_check` rejects chunk contention as an
error, not a warning.

Segment bounds are **derived from allocated chunks and therefore tight**, not declared
columns of infinite height. This matters more than it sounds: a floating island at y=30
does not contend with the plain beneath it at y∈[-8,2], so vertical stacking of independent
segments works normally. Do not let a segment reserve space it has not filled.

---

## Architecture

```text
              ┌─────────────────────────────────────┐
              │           TerrainWorld              │
              │  impl StaticGeometry                │
              │  segment broadphase + dispatch      │
              └──────────────┬──────────────────────┘
                             │  world AABB → per-segment local AABBs
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
        ┌──────────┐   ┌──────────┐   ┌──────────┐
        │ Segment  │   │ Segment  │   │ Segment  │
        │ frame    │   │ frame    │   │ frame    │
        │ voxel sz │   │ voxel sz │   │ voxel sz │
        │ ChunkGrid│   │ ChunkGrid│   │ ChunkGrid│
        └────┬─────┘   └──────────┘   └──────────┘
             │
      ┌──────┴───────┐
      ▼              ▼
  ┌────────┐    ┌────────┐      sparse: only chunks a feature
  │ Chunk  │    │ Chunk  │      actually touches are allocated
  │ SVO    │    │ SVO    │
  │ Mesh   │    │ Mesh   │
  └────────┘    └────────┘
```

### TerrainWorld

Replaces `TerrainManager` as the engine-facing terrain type and carries the
`StaticGeometry` impl. `query_region(&AABB) -> MeshPatch` (`src/physics/static_geometry.rs:21`)
becomes: find segments whose world bounds intersect the query, transform the AABB into
each one's local frame, union the returned patches back in world space. Under Rule 2 that
transform is exact.

Raycasts, probe queries (`ProbeTarget`), and render-data collection dispatch the same way.
The segment broadphase is a linear scan — segment counts are in the dozens, and a
spatial index would be premature.

### ChunkGrid

A sparse `FxHashMap<ChunkCoord, Chunk>` in segment-local space. Chunk extent is fixed per
segment (a chunk is a cube of `chunk_voxels³` voxels at that segment's resolution), so
`ChunkCoord` is a plain integer lattice coordinate. Chunks are allocated on demand:
generation computes each feature's local AABB and touches only intersecting chunks, which
makes generation cost proportional to content rather than bounding box, resolving problem 2.

### Chunk

One `SparseVoxelOctree` plus its meshed output. The existing SVO is already sparse and
collapses uniform regions, so it works unmodified at chunk scope — with the important
difference that its depth is now fixed by the chunk definition rather than derived from a
level-wide `world_size`, which is what removes the factor-of-two defect.

Per-chunk mesh storage replaces the level-wide `MeshOctree`. The existing dirty-region
machinery in `TerrainManager` maps onto per-chunk dirty flags.

---

## Gameplay entities

The current format describes terrain and objects plus a single `player_spawn`. That is a
*terrain* format, not a level format: there is no goal, no checkpoint, no hazard, no
collectible, no trigger. A level nobody can finish cannot be evaluated, which makes this a
prerequisite for judging stages 2 onward rather than a nice-to-have.

The shape that fits segments is a **segment-local volume or marker** primitive — a named
region or point, in segment-local coordinates, carrying a role:

- `Checkpoint` — respawn target, activated on entry.
- `Goal` — level exit.
- `Hazard` — kill volume; the floor of a void, lava, a crusher.
- `Pickup` — collectible.
- `Trigger` — fires a named event on entry, for anything else.

One primitive covers most of it. Note that a per-segment checkpoint generalises
`player_spawn`, which becomes "the checkpoint in the root segment".

---

## Determinism: generation is segment-local

**Noise must be sampled in segment-local coordinates, from a segment-local seed.**

If `TerrainRoughness` and the other noise-driven features sample at *world* position, then
moving a segment changes its terrain. That silently destroys both relocatability and
reuse — the two properties segments exist to provide — and the failure is invisible until
someone moves something and the level quietly changes.

With local sampling, a segment generates identically wherever it is placed, and the same
definition instanced twice produces identical geometry. Stage 1 must not bake world-position
sampling into the chunk generation path.

---

## Player reach

Gap distances need to be authored against real numbers rather than guessed. Derived from
`PlayerConfig` defaults (`src/player/config.rs`) and `gravity: -9.81`
(`src/physics/world.rs:80`):

| Move | Apex | Airtime | Flat range |
|------|------|---------|------------|
| Standing jump (walk 5.0) | 2.50 m | 1.43 s | ~7.1 m |
| Sprint jump (8.0) | 2.50 m | 1.43 s | ~11.4 m |
| Long jump (12.5 horiz, 4.2 vert) | 0.90 m | 0.86 s | ~10.7 m |

Two consequences worth internalising: a sprint jump *out-ranges* a long jump, so the long
jump's purpose is its flat arc under obstacles rather than distance; and the 2.50 m apex
sets the vertical step budget for ledges.

`level_check` must **derive** these from `PlayerConfig` and `PhysicsConfig` at runtime, not
hardcode them, so that retuning the player automatically revalidates every level.

They are also optimistic — point mass, flat-to-flat, perfect input, ignoring
`jump_cutoff_factor` and air steering. Validation should apply a safety margin, and authored
gaps should sit well inside these figures.

---

## The anchor graph

### Placement is a tree; connectivity is a graph

Anchor snapping derives a segment's transform from its parent, which makes **placement**
inherently a spanning tree. Level topology is not a tree: a shortcut looping back to an
earlier area creates a cycle, and then two different paths around that cycle each claim to
determine the same segment's transform. They will disagree.

These must therefore be separated:

- **Placement edges** form a spanning tree and are the only edges that determine transforms.
  Every segment has exactly one placement parent (or an absolute transform, for the root).
- **Connection edges** are everything else — they express that two anchors meet, but derive
  nothing. They are *assertions*, and `level_check` verifies the two anchors actually
  coincide within tolerance.

Without this split, any non-linear level is ill-defined. With it, cycles, hubs with
radiating branches, and shortcuts all work.

### Connections carry a gap

Snapping two anchors coincident is wrong for the common platformer case, where the join
between areas is a deliberate jump. A connection takes a separation:

```ron
Connect(from: "start_isle.exit_east", to: "mid_isle.entry", gap: 6.0)
```

`gap` is the quantity `level_check` validates against the player's jump reach, so it is
load-bearing rather than cosmetic.

### Segments are areas, not objects

A segment is the unit you would want to relocate, reuse, or reason about on its own — a
room, a traversal stretch, a set-piece. A cluster of six stepping-stone islands crossed in
one continuous move is *one* segment, not six. Sizing segments per object multiplies grids,
meshes and broadphase entries for no authoring benefit.

Conversely, a single 400m corridor is storage-efficient as one segment but poor to author
in: it reinstates a large absolute-coordinate namespace, which is the original problem in
miniature. Split by authoring ergonomics, not by storage cost — they are independent axes.

---

## Mobile geometry — deferred, out of scope

**Kinematic bodies are out of scope for this project.** Moving platforms will be needed
eventually; nothing in stages 1–4 should implement them. This section is retained because
the *conclusion* — that mobile geometry can never be voxels — constrains the anchor design
that stage 2 does build, and re-deriving it later would be wasted effort.

The world holds three kinds of thing today: voxel terrain, spawnables (rigid bodies,
from a beach ball to a Greek temple), and the player. Floating platforms and other mobile
terrain are a fourth, and they cannot be voxels.

### Why not voxels

`StaticGeometry::query_region(&AABB) -> MeshPatch` (`src/physics/static_geometry.rs:21`)
returns triangles carrying no velocity; the solver treats them as infinite-mass and
stationary. A voxel segment whose transform changed per frame would hand the solver
stale-frame geometry — a body resting on it would not be carried, and at speed would
tunnel. Moving geometry must enter physics as a body that *has* a velocity. This is an
interface boundary, not a stylistic preference.

### Representation

A mobile piece is a `BodyType::Kinematic` body with a `ColliderShape::ConvexHull` or `Box`
collider, plus a motion driver (path, schedule, easing) evaluated in **segment-local**
coordinates.

Note that `BodyType::Kinematic` (`src/physics/body.rs:14`) is currently unused by the game
— the only references are its definition and the normal-impulse mass override in
`src/physics/solver/normal.rs`. The primitive exists; nothing exercises it. Treat mobile
geometry as new functionality rather than as wiring.

### Authoring vs. engine

In the level file a mobile piece is a spawnable-like entry, so authoring stays a single
uniform vocabulary. In the engine it is a distinct concept: today's spawnables are dynamic
bodies that are spawned and forgotten, whereas a platform needs a motion driver and a
segment-local frame. Component plus system, not a spawn function.

### Consequence for anchors

Anchors are therefore **named local frames within a segment**, not merely segment-to-segment
join points. They serve three jobs: joining segments, mounting mobile geometry, and
supplying waypoints for a motion path. Stage 2 must define them this way from the start;
narrowing them to joins and widening later is a painful retrofit.

### Known risk — carrying riders

A kinematic platform pushes dynamic bodies correctly through ordinary contacts, but a
player *standing* on one must inherit its velocity or it will slide out from under them.
This interacts with the player controller resetting velocity every frame, which is the
same pattern behind the warm-start/restitution defect recorded in the physics notes. This
needs a bench scenario, not just an implementation.

---

## Staging

Stage briefs live in `docs/level_segments/` and are written **just in time** — one at a
time, immediately before that stage runs, once the preceding stage has landed. Briefs
written far ahead encode guesses about APIs that earlier stages will actually decide, and
an agent will follow a stale brief over the real code. This document holds the durable
design; the briefs hold the perishable detail.

| Stage | Deliverable | Why here |
|-------|-------------|----------|
| 1 | Chunked terrain, one implicit whole-world segment | Behaviourally identical to today, so it is independently testable. Establishes Chunk/ChunkGrid/TerrainWorld. |
| 1.5 | `level_check` + schematic export (SVG, no Vulkan) | Pulled ahead of segments so every later stage has automated acceptance instead of visual guesswork. |
| 2 | N segments, local frames, anchor graph, placement | The authoring payoff. Generalises stage 1's single segment. Anchors defined as named local frames per "Mobile geometry" above. |
| 3 | Traversal primitives — swept `Path` first, then `Platform`, `Staircase`, `Shaft` — plus gameplay entities (checkpoint, goal, hazard, pickup, trigger) | Needs segments to be worth authoring against. Gameplay entities land here because without them no level can be finished, and stage 4 has nothing meaningful to look at. |
| 4 | `level_viewer` binary with offscreen render-to-PNG | Last because it is the most expensive and `level_check` covers correctness. |
| — | Mobile geometry (kinematic platforms) | **Out of scope.** Deferred indefinitely; see "Mobile geometry" above for the constraint it places on anchors. |

Stage 1 deliberately introduces the chunk grid with a **single implicit segment** covering
the world. That keeps stages 1 and 2 separable despite segments owning chunk grids: stage 1
proves the chunk machinery against unchanged behaviour, stage 2 generalises the count from
one to N.

### Process

- Each stage is its own branch off `main`, merged before the next begins. Stage 1 touches
  `TerrainManager`, which both the water system and physics static-geometry read; two
  stages in flight will conflict badly.
- Each brief names an explicit **out-of-scope** list and ends in a **command that must
  pass**. Without a mechanical acceptance check, sequential handoff degrades quietly.
- Each agent appends to `docs/level_segments/PROGRESS.md` on completion: what landed, where
  it deviated from the brief and why, and gotchas for the next stage. That file is the
  handoff channel between agents and the cheapest way to re-enter context.

---

## Existing levels

Of the ten level files, two are kept and ported:

- `levels/test_empty_terrain.level.ron` — minimal fixture.
- `levels/test_arena.level.ron` — holds one copy of every spawnable, which makes it the
  natural fixture for `level_check` and the viewer.

The remaining eight are deleted. They were tuned against the effective 2.0m voxel size and
would need rework regardless; their value was as format examples, which this plan replaces.

---

## Seams to leave open

Not built by this plan, but cheap to accommodate now and painful to retrofit:

- **Segment lifecycle.** Give segments a state (`loaded` / `meshed` / `active`) even though
  everything is always active today. Streaming then becomes a scheduling change rather than
  a redesign.
- **Per-segment hot reload.** Flagged as "Tier 2" in `LEVEL_FILE_FORMAT.md` and far more
  tractable with segments — reload one segment, not the world. The largest available
  iteration-speed win. Keep the reload boundary segment-shaped.
- **Per-segment voxel diffs.** Terrain is destructible, so checkpoint restore means undoing
  damage. If edits are recorded as segment-local diffs over generated output, restore is
  "drop the diff and remesh". Nearly free if the edit path is built that way from the start.

## Deliberate non-goals

Stated so that a level idea can be checked against them at a glance. These are boundaries,
not omissions:

- **Rotation other than 90° yaw multiples.** Pitch and roll are never supported; slanted
  geometry is expressed within a segment.
- **Moving voxel terrain.** Forced by the `StaticGeometry` interface, see "Mobile geometry".
- **Continuous terrain across a resolution change.** Accepted limitation; equalise the
  resolution or use a spawnable.
- **Water spanning segments.** Water bodies live in a single segment. The water system needs
  its own design pass; other configurations are deferred to a future plan.
- **Parameterised segment templates.** Plain instancing — placing the same segment definition
  at several transforms — delivers most of the value without the format becoming an
  expression language. Add parameters only when a concrete level demands them.
- **Per-chunk draw calls, LOD, format versioning.** Deferrable and non-structural.

## Open questions

- **Water.** `create_level_water` derives grid extents from `world_size` and assumes a
  single origin. Per-segment water grids, or one world-space grid spanning the segment
  union? Deferred to stage 2, where the answer becomes forced.
- **Cross-segment physics.** A body traversing an anchor join is briefly near two
  segments' geometry. `TerrainWorld::query_region` unions patches, so this should work,
  but it wants a bench scenario to confirm no duplicate contacts arise at the seam —
  compare the "duplicate contact resolution causes phantom forces" lesson in the physics
  notes.
- **Destruction across segments.** Voxel edits are segment-local. An explosion straddling
  a join must be dispatched to both. Cheap to handle, easy to forget.
- **Vertical connections through terrain.** A cave network beneath a surface level stacks
  cleanly, but the entrance shaft joining them is continuous terrain crossing a boundary —
  and unlike a spiral tower's column, surface and caves plausibly want different
  resolutions, so Rule 1 forbids it. Either the shaft becomes its own segment joining both,
  or surface and caves share one segment. No clean answer yet; know this before designing a
  level around it.
- **Reachability validation with mobile geometry.** `level_check`'s jump-reach analysis
  (stage 1.5) assumes static geometry: a gap is either crossable or not. Once platforms
  move, reachability becomes time-dependent and the analysis no longer decides it. Expect
  to scope `level_check` to static reachability and report gaps bridged only by mobile
  geometry as unverified, rather than attempting a timing solver.
