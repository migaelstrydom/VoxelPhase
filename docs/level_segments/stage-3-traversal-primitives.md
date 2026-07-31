# Stage 3 — traversal primitives

**Read `docs/LEVEL_SEGMENTS_PLAN.md` first**, then `docs/level_segments/PROGRESS.md` — the
"For stage 3" section at the end, and stage 2's "Deviations from the brief", both bear on this.

Branch: `level-segments-stage-3` off `main`. Merge before stage 4.

---

## Goal

Stages 1–2 built where an area *is*. This stage builds what an area is *made of*.

Terrain can currently be shaped only by heightfield features (hills, plateaus, cliffs, noise)
and blobby volumes (islands, arches, pillars). That is landscape, not a route — which is why
the levels in this repo read as terrain to wander rather than a path to traverse. This stage
adds the geometry a platformer route is actually built from.

**Gameplay entities — checkpoints, goals, hazards, pickups, triggers — are explicitly NOT in
this stage.** They were in an earlier draft of this brief and were cut: the game's respawn and
progression model is an open design question the project owner has not settled, and encoding a
guess at it in the level format would be expensive to undo. Do not add them, and do not add
anything that presupposes them.

## The framing decision, made for you

**Traversal primitives are voxel terrain, not a new object category.** They become new
`VolumeFeature` variants alongside the existing `Island`, `Arch`, `Pillar`, `Tunnel`,
`Overhang`.

The reasoning, so you do not relitigate it: voxel volumes are already generated
segment-locally, so a rotated segment rotates them **for free** with no per-primitive
orientation to get wrong. They are already destructible, already served to physics through
`StaticGeometry` with no new collider work, and already drawn by the existing pipeline. A new
object category would need all four of those built, and would inherit the orientation gap
recorded in stage 2's deviation 2. `VolumeFeature` is the cheap, consistent home.

---

## Deliverable 1 — the primitives

New `VolumeFeature` variants, authored in segment-local coordinates. In priority order:

1. **`Path`** — the highest-leverage one by a distance, and the one to get right even if the
   others end up thin. A polyline swept with a width and a profile: catwalks, bridges, ledges
   cut into a cliff, spiral ramps around a tower. Support at least a flat profile and a rounded
   one, and allow the polyline to change height between points, so a spiral ramp is a single
   `Path` rather than a construction.
2. **`Platform`** — a free-standing slab at a height. The atom of a jump sequence, trivial next
   to `Path`, and the thing an author reaches for most often.
3. **`Staircase`** — discrete steps between two heights. Distinct from a ramp because the step
   rise is what the player has to clear, which makes it checkable.
4. **`Shaft`** — a vertical bore with an optional spiral ledge, for towers and descents.

Each takes a `material`, so a route can read differently from the ground it crosses. Add
**`Sand`** to `VoxelMaterial` and `VoxelMaterialId` while you are there — one enum variant and
one colour, since materials are currently vertex colours only. Do not go further into visual
identity than that single variant; it is its own stage and the most tempting scope creep here.

These must produce **deliberate, readable geometry**. A two-metre `Path` at 0.5 m voxels is
four voxels across and will look like a staircase in plan if it is rasterised naively. Use the
sub-voxel SDF density encoding the existing features use — the thing that lands a surface at
its exact authored height regardless of voxel size. If a primitive cannot read cleanly at a
segment's resolution, have `level_check` say so rather than emitting a mess.

## Deliverable 2 — object orientation under segment yaw

Stage 2's deviation 2, now due. Objects are authored segment-locally but only their *placement*
is transformed — half-extents, row directions and column layouts stay in the spawnable's own
axes, so an object in a yaw-90 segment keeps its world orientation. Rotate a segment containing
a wall and the wall silently faces the wrong way. `test_segments` avoids this by putting only
yaw-invariant objects in rotated segments, which is containment, not a fix.

Give `LevelObject::place_in` an orientation as well as a position and thread it through the
spawnables that have a meaningful axis. `PlankBridge` already carries an explicit `yaw` and
shows the shape of it. Not all 34 need work — a sphere does not care — so **audit which ones
do** and record the list in `PROGRESS.md`. Where a spawnable genuinely cannot be oriented, have
`level_check` warn when it is placed in a rotated segment, so the trap is visible rather than
silent.

## Deliverable 3 — `level_check` extensions

- **A gap bridged by a traversal primitive stops being reported as a jump.** `Crossing::for_gap`
  in `src/level_check/segments.rs` already answers "which jump does this gap need"; a `Path` or
  `Platform` spanning it changes the answer to "walk". Without this, adding a bridge makes the
  report *worse*, which teaches authors to ignore it.
- **Staircase step rise vs jump apex.**
- **`Path` width vs the player's collider** — a 0.5 m catwalk the player cannot stand on is a
  level bug, and the collider radius is available to derive the minimum from rather than
  hardcode, in the spirit of `reach.rs`.
- **A primitive too fine for its segment's voxel size** — warn, with the resolution that would
  render it cleanly. This is the check that stops resolution-dependent routes shipping.

## Deliverable 4 — schematic and a demonstration level

The SVG must draw traversal primitives as the route they are: a reader should be able to trace
the intended path through a level without opening the RON.

Fix the one collision stage 2 left: **`east_ledge` and `west_landing` overprint** as
"east_ledgelanding" across the join in `segments.svg`. The de-confliction handles clustered
objects but not two anchors facing each other across a gap — the one arrangement that recurs at
*every* join, so it is the case most worth handling. Also "1 segments".

Then extend `levels/test_segments.level.ron`, or add a sibling, so that **every primitive this
stage adds appears in a level and is exercised** — including at least one `Path` in a rotated
segment and at least one primitive at each of the two voxel resolutions already in that level.
Keep it a geometry fixture; it does not need to be beautiful, and visual identity is a later
stage you should not spend budget on.

---

## Out of scope

- **Gameplay entities of any kind** — checkpoints, goals, hazards, pickups, triggers, death,
  respawn, progression. See the note above. This is a deliberate cut, not an oversight.
- **Mobile geometry, moving platforms, kinematic bodies.** Out of scope project-wide. A `Path`
  is static geometry; if a primitive tempts you toward a moving version, stop.
- **Welded joins / cross-segment halo sampling.** Still deferred, still rejected at load.
- **Visual identity beyond adding `Sand`** — biomes, textures, props, foliage, lighting.
- **Any terrain performance fix.** Measure-only still stands, and the demonstration level
  should stay well under ~1.5 M vertices so the concatenation tripwire stays untripped.
- The viewer — stage 4.

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

All levels exit 0.

Required tests:

- A `Path` swept along a polyline produces continuous, watertight geometry — including where it
  changes height and where it turns.
- Each new primitive lands its surface at its authored height at two different voxel
  resolutions. This is the sub-voxel-SDF guarantee, and it is what stops a route being
  resolution-dependent.
- Every new primitive is correctly positioned and oriented in a yaw-90 segment.
- An object with a meaningful axis is correctly oriented in a yaw-90 segment.
- A gap spanned by a `Path` is reported as walkable, not as a jump.
- A `Path` narrower than the player's collider is a finding.

**The game window cannot be launched from the agent shell.** As in stage 2, list in
`PROGRESS.md` exactly what a human should check by playing — short and specific. For this stage
the question is whether the primitives are pleasant to walk and jump on, which no headless
check can answer.

## On completion

Append to `docs/level_segments/PROGRESS.md`: what landed, deviations and why, the audit of
which spawnables needed orientation work, anything that surprised you about sweeping geometry
into voxels, and what stage 4's viewer most needs to show that the SVG cannot.
