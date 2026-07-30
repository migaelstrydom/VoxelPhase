# Stage 3 — traversal primitives and gameplay entities

**Read `docs/LEVEL_SEGMENTS_PLAN.md` first**, then `docs/level_segments/PROGRESS.md` — the
"For stage 3" section at the end, and stage 2's "Deviations from the brief", both matter here.

Branch: `level-segments-stage-3` off `main`. Merge before stage 4.

---

## Goal

Stages 1–2 built where terrain *goes*. This stage builds what the player *does on it*: the
geometry a platformer is actually made of, and the entities that let a level be started,
survived and finished.

Right now a level can be walked around but not played. There is no goal, no checkpoint, no
hazard — nothing that makes an area a challenge rather than a shape. And the only ways to
shape terrain are heightfield features and blobby volumes, which is why the current levels
read as landscape rather than as a route.

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

**Gameplay entities are not voxels.** They are markers and volumes with ECS behaviour, so they
need their own list on `SegmentDef` and their own placement path.

---

## Deliverable 1 — traversal primitives

New `VolumeFeature` variants, authored in segment-local coordinates. In priority order:

1. **`Path`** — the highest-leverage one by a distance. A polyline swept with a width and a
   profile: catwalks, bridges, ledges cut into a cliff, spiral ramps around a tower. One
   primitive covering most of what a platformer route is made of. Support at least a flat
   profile and a rounded one, and allow the polyline to change height between points so a
   spiral ramp is expressible as a single `Path`.
2. **`Platform`** — a free-standing slab at a height, the atom of a jump sequence. Trivial
   next to `Path`, and the one an author reaches for most.
3. **`Staircase`** — discrete steps between two heights. Distinct from a ramp because the
   step rise is what the player's jump has to clear, and `level_check` can check it.
4. **`Shaft`** — a vertical bore with an optional spiral ledge, for towers and descents.

Each takes a `material` so an author can make a route read differently from the ground it
crosses. Add **`Sand`** to `VoxelMaterial` and `VoxelMaterialId` while you are there — it is
one enum variant and one colour, materials are currently vertex colours only, and the absence
of a beach-coloured material is a silly reason for a beach to look like a lawn. Do not go
further into visual identity than that one variant; it is its own stage.

These must produce **deliberate, readable geometry**. A `Path` two metres wide at 0.5 m voxels
is four voxels across and will look like a staircase in plan if it is rasterised naively — use
the sub-voxel SDF density encoding the existing features use, which is what lands a surface at
its exact authored height regardless of voxel size. If a primitive cannot be made to read
cleanly at a segment's resolution, say so in `level_check` rather than emitting a mess.

## Deliverable 2 — gameplay entities

One segment-local primitive family, listed on `SegmentDef` alongside `objects` and `anchors`.
A level with none of these cannot be finished, which is the gap that matters most here.

- **`Checkpoint`** — a volume; entering it sets respawn. `player_spawn` should become *the
  checkpoint in the root segment*; stage 2 already authors the spawn in that frame, so this
  is a rename plus a lookup, not a migration.
- **`Goal`** — a volume that ends the level. Exactly one per level; zero or two is an error.
- **`Hazard`** — a volume that kills or damages on entry. A kill plane under a level of
  floating islands is the single most common one, so make sure an unbounded-below hazard is
  expressible without authoring a 400 m box.
- **`Pickup`** — a marker that can be collected. Needs a visual, but keep it to one simple
  existing spawnable rather than new art.
- **`Trigger`** — a volume that fires a named event on entry. Leave the event mechanism
  minimal: what matters this stage is that the *shape* exists so stage 4 and later have
  something to hang behaviour on. Do not build a scripting system.

Model these so a volume and a marker are the same primitive with different extents, rather
than five unrelated types. Give them a real orientation from the start —
`SegmentFrame::rotation()` returns a `UnitQuaternion` that agrees exactly with the integer
yaw, so an entity storing a quaternion composes correctly under a rotated segment.

**Death and respawn must actually work**, or `Hazard` and `Checkpoint` are decoration. Keep it
as simple as it can be: on hazard entry, move the player to the last checkpoint and reset
velocity. No death animation, no lives, no UI beyond what `DebugLines` already gives you.

## Deliverable 3 — object orientation under segment yaw

Stage 2's deviation 2, now due. Objects are authored segment-locally but only their
*placement* is transformed — half-extents, row directions and column layouts stay in the
spawnable's own axes, so an object in a yaw-90 segment keeps its world orientation. Rotate a
segment containing a wall and the wall silently faces the wrong way. `test_segments` avoids
this by putting only yaw-invariant objects in rotated segments, which is containment, not a
fix.

Fix it: give `LevelObject::place_in` an orientation as well as a position, and thread it
through the spawnables that have a meaningful axis. `PlankBridge` already carries an explicit
`yaw` and shows the shape of it. Not every one of the 34 needs work — a sphere does not care —
so **audit which ones do** and record the list in `PROGRESS.md`. Where a spawnable genuinely
cannot be oriented, make `level_check` warn when it is placed in a rotated segment, so the
trap is visible rather than silent.

## Deliverable 4 — `level_check` extensions

- **Exactly one goal per level.** Zero or two is an error.
- **Every checkpoint and the goal must be reachable** — at minimum, each sits on or above
  solid ground within the jump envelope of some other reachable point. A full reachability
  solve is not expected; catch the goal floating 60 m above anything.
- **A gap bridged by a traversal primitive stops being reported as a jump.** `Crossing::for_gap`
  in `src/level_check/segments.rs` already answers "which jump does this gap need"; a `Path` or
  `Platform` spanning it changes the answer to "walk". Without this, adding a bridge makes the
  report worse, which would teach authors to ignore it.
- **Staircase step rise vs jump apex**, and **`Path` width vs the player's collider** — a
  0.5 m catwalk the player cannot stand on is a level bug the check should catch.
- **Hazard volumes overlapping a checkpoint or the spawn** is an error. It is an easy authoring
  mistake and produces an unplayable respawn loop.

## Deliverable 5 — schematic and a real level

The SVG must draw traversal primitives and gameplay entities: the goal and checkpoints
distinctly marked, hazard volumes outlined, paths and platforms visible as the route they are.
A reader should be able to trace the intended path through a level.

Fix the one collision stage 2 left: **`east_ledge` and `west_landing` overprint** as
"east_ledgelanding" across the join in `segments.svg`. The de-confliction handles clustered
objects but not two anchors facing each other across a gap — which is the one arrangement that
recurs at *every* join, so it is the case most worth handling. Also "1 segments".

Then author `levels/test_platformer.level.ron`: a level that can actually be **completed**.
Spawn, a route with real jumps, at least one hazard that can kill you, at least one checkpoint
that saves you, and a goal. Use the traversal primitives for the route rather than shaping it
out of hills. Keep `test_segments` as the geometry fixture it is; this is the playability one.

It does not need to be beautiful — visual identity is a later stage and you should not spend
your budget there. It needs to be *playable*, and it needs to demonstrate every primitive this
stage adds.

---

## Out of scope

- **Mobile geometry, moving platforms, kinematic bodies.** Still out of scope project-wide.
  A `Path` is static geometry. If a primitive tempts you toward a moving version, stop.
- **Welded joins / cross-segment halo sampling.** Still deferred, still rejected at load.
- **Visual identity beyond adding `Sand`** — biomes, textures, props, foliage, lighting. This
  is a real stage and it is not this one. Resist it; it is the most tempting scope creep here.
- **A scripting or event system.** `Trigger` fires a named event and that is all.
- **Lives, score, UI, menus, level transitions.** Reaching the goal can print to `DebugLines`.
- **Any terrain performance fix.** Measure-only still stands, and `test_platformer` should
  stay under ~1.5 M vertices so the concatenation tripwire stays untripped.
- The viewer — stage 4.

## Acceptance

```bash
cargo build
cargo test
cargo test --release --features bench_harness
cargo run --bin level_check -- levels/test_arena.level.ron
cargo run --bin level_check -- levels/test_empty_terrain.level.ron
cargo run --bin level_check -- levels/test_segments.level.ron
cargo run --bin level_check -- levels/test_platformer.level.ron
cargo run --bin level_check -- levels/test_platformer.level.ron --svg /tmp/platformer.svg
```

All four levels exit 0.

Required tests:

- A `Path` swept along a polyline produces continuous, watertight geometry, including where it
  changes height and where it turns.
- Each new primitive lands its surface at its authored height at two different voxel
  resolutions — the sub-voxel-SDF guarantee, which is what stops a route being resolution-
  dependent.
- Every new primitive and entity is correctly positioned **and oriented** in a yaw-90 segment.
- A level with no goal, and a level with two, are each an error.
- A gap spanned by a `Path` is reported as walkable, not as a jump.
- A hazard overlapping a checkpoint is an error.
- Entering a hazard respawns the player at the last checkpoint, not at the level spawn.

**The game window cannot be launched from the agent shell.** As in stage 2, list in
`PROGRESS.md` exactly what a human should check by playing — and this stage that list is the
real acceptance, because "can the level be completed" is not something a headless check can
answer. Keep the list short and specific.

## On completion

Append to `docs/level_segments/PROGRESS.md`: what landed, deviations and why, the audit of
which spawnables needed orientation work, anything that surprised you about sweeping geometry
into voxels, and what stage 4's viewer most needs to show that the SVG cannot.
