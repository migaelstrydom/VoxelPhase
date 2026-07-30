# Stage 2 — segments, anchors, placement

**Read `docs/LEVEL_SEGMENTS_PLAN.md` first**, then `docs/level_segments/PROGRESS.md` in full.
Stages 1 and 1.5 are merged to `main` and left you a substantial head start; the "For stage 2"
section of `PROGRESS.md` is not optional reading.

Branch: `level-segments-stage-2` off `main`. Merge before stage 3.

---

## Goal

Turn the level from one terrain grid into **N segments**, each with its own coordinate frame,
its own voxel resolution, and named anchors. Segments are positioned by snapping an anchor to
an anchor on an already-placed segment, so an author writes *"the tower joins the plaza at
the north gate, 6 m gap"* instead of absolute coordinates.

This is the stage that makes the whole project worth doing. Everything before it was
plumbing.

## What you inherit

- **`ChunkGrid` already has a local frame.** It stores an `origin` and offers
  `to_local` / `to_world` / `aabb_to_local` / `aabb_to_world`. `TerrainManager` converts at
  every query boundary and generation is entirely grid-local. Today the one grid sits at the
  origin so every conversion is identity — real code on real paths, but **translation is
  exercised only by unit tests**. The first non-zero origin is the moment to re-run the seam
  tests, and you should expect to find something.
- **Noise is sampled in grid-local coordinates.** Moving a grid will not change its terrain.
  Do not regress this; it is what makes a segment relocatable and reusable.
- **`level_check` exists** and is your acceptance harness. Extend it rather than eyeballing.

---

## Deliverable 1 — `TerrainWorld`

Rename `TerrainManager` to `TerrainWorld` and reshape it to own `Vec<Segment>`, where a
`Segment` is a local frame + a `ChunkGrid` + its own voxel size + a name + its anchors.

**The public query surface must not change shape.** `TerrainWorld` is the ECS resource that
`systems/render.rs`, `systems/water.rs`, `systems/terrain_anchor.rs`,
`systems/terrain_update.rs`, `systems/physics_sync.rs` and five terrain-anchored spawnables
all read. Every one of them asks a **world-space** question — `is_solid_at`,
`surface_height_at`, `query_region`, `damage_sphere`, `render_vertices` — and every one of
them should keep asking exactly that, unchanged, while `TerrainWorld` fans the query out
across segments internally. If a consumer outside `src/terrain/` has to learn what a segment
is, the abstraction has leaked. The two exceptions are `level_check` and `level/spawner.rs`,
which legitimately construct and inspect segments.

Concretely:

- `query_region(&AABB)` transforms the world AABB into each candidate segment's local frame,
  queries, and transforms the triangles **back to world space** before returning. Because
  rotation is restricted to 90° yaw multiples, an axis-aligned box stays axis-aligned under
  the transform — that restriction exists precisely so this is exact and cheap, not
  conservative.
- **Bake the segment transform into render vertices** at mesh time. Downstream rendering
  stays world-space and untouched.
- `damage_sphere` routes to every segment whose extent the sphere touches.
- Point queries pick the segment(s) containing the point. Overlap is forbidden by Rule 4, so
  at most one segment can answer — but do not rely on that for correctness of the *search*,
  only for the uniqueness of the answer.

Keep the render buffers concatenated across all segments, as stage 1 kept them concatenated
across chunks. Per-segment draw calls are a stage-4-or-later performance question and
`PROGRESS.md` records the measurement that will decide it. **Do not fix performance here.**

## Deliverable 2 — anchors and the placement tree

An **anchor** is a named local frame within a segment: a position and a yaw, both in segment
local coordinates. Anchors serve three eventual purposes (segment joins, mounting mobile
geometry, motion-path waypoints); this stage implements only the first, but name and model
them so the other two do not require a redesign.

**Placement is a spanning tree.** Exactly one segment is the root, placed at an explicit
world transform. Every other segment declares exactly one placement parent: *my anchor A
snaps to segment S's anchor B, with gap G*. That derives its world transform. A segment with
two placement parents is over-determined and is an **error**; a segment with none and no root
declaration is an orphan and is an **error**; a cycle is an **error**.

**Connectivity is a graph.** Any other relationship between anchors — the shortcut that loops
back to an earlier area, the second bridge across a chasm — is declared separately as an
*assertion*, not a placement. `level_check` verifies the two anchors actually end up where
the assertion claims, within the declared gap. This split is what makes non-linear levels
well-defined at all; without it, a level with a loop has no consistent placement.

Pick and **document a facing convention** for anchors — the obvious one is that an anchor's
local +X points outward, out of the segment, so mating two anchors is a 180° relative yaw
with `gap` separating them along that axis. Whatever you choose, write it in a doc comment
with a small ASCII diagram, because it is the thing every future level author will get wrong
first.

Rotation is restricted to **90° yaw multiples**. Reject anything else at load with a clear
error naming the offending segment. Pitch and roll are not supported.

## Deliverable 3 — RON format

Extend the level format to express segments, and port `test_arena` and `test_empty_terrain`
to it as a single segment named `main` each. Their rendered result must be byte-for-byte the
terrain they produce today — that equivalence is your best regression test that the frame
maths is identity-correct before you trust it at non-zero origins.

Then **add a genuinely multi-segment level**, `levels/test_segments.level.ron`: at minimum
three segments, at least one at a non-zero yaw, at least one at a different voxel resolution
from its neighbour, joined by gaps the player can actually jump. This is the level that
proves the stage and the one stage 3 will build on. Make it something a person could plausibly
play — not three cubes in a row.

Update `docs/LEVEL_FILE_FORMAT.md`. It is the authoring reference and it has drifted before;
a stale format doc is worse than none.

## Deliverable 4 — `level_check` extensions

- **Placement tree validation**: root count, over-determined segments, orphans, cycles.
- **Rule 4 contention**: no two segments may own chunks **containing solid voxels** whose
  world extents overlap. The qualifier is load-bearing. Stage 1.5 measured that the
  seam-neighbour shell is **72% of `test_arena`'s chunks** and bulges a full chunk — 32 m at
  1 m voxels — past real content in −X, −Y and −Z. A bounds-based or allocation-based check
  would reject almost every legitimately adjacent pair. Check solid chunks, and test that two
  segments 6 m apart pass.
- **Gap vs reach**: every connection's gap is measured against the derived jump envelope from
  `src/level_check/reach.rs`. Over the standing-jump range is a warning; over the sprint-jump
  range is an error. Report which jump each gap requires — that is the number an author
  actually wants.
- **Per-segment terrain statistics**, and the whole-level totals as now.

## Deliverable 5 — schematic

The SVG must draw multiple segments: outline each segment's solid extent, label it with its
name, and mark anchors and connections. A reader should be able to see the level's shape and
the order areas connect in without reading the RON.

Two flaws in the stage 1.5 output to fix while you are in there:

1. **Object number labels collide** where objects cluster. Positions stay readable and the
   legend carries exact coordinates, so nothing is lost — but the densest region of the
   picture is the least readable, which is backwards. De-conflict, or drop labels below a
   spacing threshold and rely on the legend.
2. **The elevation view spans the full derived y range** (−64..32 for `test_arena`) while the
   terrain occupies about 10 m of it, so a flat level squashes into a strip. Fit the vertical
   range to content, with a floor so a genuinely flat level does not get absurd magnification.

---

## Out of scope

- **Welded / continuous terrain across a segment boundary.** Ship **gap-joined segments
  only**. Marching cubes samples a one-voxel halo and an unallocated region reads as air, so a
  boundary chunk currently emits a **cap surface** sealing a join that should be open — real
  geometry reaching physics. The fix is cross-segment halo sampling and it is its own piece of
  work. Gap-joined segments are unaffected, because the shell chunk sits in empty space and is
  pruned. If an author declares a welded join, **reject it at load with an error saying it is
  not implemented yet** — do not silently produce a wall.
- **Mobile geometry, kinematic bodies, moving platforms.** Out of scope for the whole project
  by explicit decision. Model anchors so they *could* mount such a thing later; implement
  nothing.
- **Water across segments.** Single-segment water bodies only. The current one world-space
  grid may stay one grid; just make sure it is derived from the segment union rather than from
  a field that no longer exists. Per-segment water grids are a future plan.
- **Streaming, per-segment load/unload, checkpoints.** Runtime-aware segments exist so these
  become *possible*, not so they happen now.
- **Any terrain performance fix.** Measure-only still applies.
- Traversal primitives and gameplay entities — stage 3. The viewer — stage 4.

## Acceptance

```bash
cargo build
cargo test
cargo test --release --features bench_harness
cargo run --bin level_check -- levels/test_arena.level.ron
cargo run --bin level_check -- levels/test_empty_terrain.level.ron
cargo run --bin level_check -- levels/test_segments.level.ron
cargo run --bin level_check -- levels/test_segments.level.ron --svg /tmp/segments.svg
```

All three levels exit 0.

Required tests:

- A segment at a non-zero origin produces the same terrain as the same segment at the origin,
  translated. This is the relocatability guarantee and it must be a test, not an assumption.
- The seam and watertightness tests from stage 1, re-run against a segment at a non-zero
  origin and a non-zero yaw.
- `query_region` returns world-space triangles for a translated and rotated segment.
- Placement tree validation: over-determined, orphan and cyclic levels are each an error.
- Two segments 6 m apart pass the Rule 4 contention check; two genuinely overlapping segments
  fail it.
- A gap beyond sprint-jump range is an error; a gap within standing-jump range is clean.
- A non-90°-multiple yaw is rejected at load.

Attach or describe the generated `segments.svg` so the reviewer can judge legibility without
running it. Legibility is a deliverable, not a nicety — it is how the author checks their own
work from here on.

**The game window cannot be launched from the agent shell.** Say so in `PROGRESS.md` and list
exactly what a human should look at when they run `cargo run` — that request will be honoured,
so make it specific and short.

## On completion

Append to `docs/level_segments/PROGRESS.md`: what landed, deviations and why, the anchor
facing convention you chose, anything that surprised you about non-identity frames, and what
stage 3 should know — particularly how a traversal primitive should express itself in segment
local coordinates.
