# Terrain representation — landscape, structure, clutter

**Status: design agreed, not started.** Written 2026-07-31, after the level-segments project
(stages 1–3) completed and its output was played.

This document exists because playing stage 3 changed the plan. `docs/LEVEL_SEGMENTS_PLAN.md`
describes how areas are *placed*; this one describes what they are *made of*, and why the
answer turned out not to be "voxels" for all of it.

---

## What the play-test found

Stage 3 added four voxel traversal primitives — `Path`, `Platform`, `Staircase`, `Shaft` —
and `test_segments` was rebuilt to use them as its route. Played through, verbatim:

1. A 3 m deck width feels fine.
2. The player walks up a sloped deck fine, **but it looks lumpy.**
3. **The staircase cannot be walked up without jumping**, and the steps are not vertical —
   marching cubes tilts the risers.
4. The two stepping stones sit right at the limit of a standing jump. Reachable once
   ledge-grab exists. You can only climb back on from beside the bowl, not from below.
5. **The spiral ledge looks bumpy.** Walkable downward.
6. The quarter-turned segment looks jagged. It is exactly where the schematic says.
7. A deck survives a grenade that craters the ground beneath it, so it is left floating.

## The diagnosis: two different kinds of blockiness

**Resolution** — features smaller than a voxel cannot exist. Cured by smaller voxels, at
cubic cost.

**Sharpness** — **marching cubes structurally cannot represent a sharp edge or corner.**
Each vertex is placed by linear interpolation along a cell edge, so a vertical riser falling
between two sample planes emerges as a slope. At double resolution it emerges as a *smaller*
slope. It never emerges vertical.

Observations 2, 3, 5 and 6 are all the second kind. **This is the load-bearing conclusion of
the whole document:** authoring routes at finer voxel resolution does not fix them, and no
improvement to the SDF encoding fixes them either. Stage 3's sub-voxel work got surfaces to
land at the right *height*; it cannot make them land at the right *angle*, because that
information does not survive the algorithm.

Note this is entirely distinct from the sample-on-the-surface degeneracy fixed on 2026-07-31
(see `docs/level_segments/PROGRESS.md`, "the heightfield had the same degeneracy"). That one
was a bug and is closed. This one is a property of marching cubes and is not.

## The split, by role

| Role | Representation | Why |
|------|---------------|-----|
| **Landscape** | voxels + marching cubes | Blobby is *correct* for ground, hills, cliffs, caves. Destructibility is per-voxel and matters most here. MC's weaknesses do not show. |
| **Structure** | static compound rigid bodies | Everything the player stands on deliberately: steps, walkways, platforms, ledges, spiral ramps. Wants exact edges. Destructibility is per-piece. |
| **Clutter** | dynamic rigid bodies | Everything you knock about. What most of the 34 spawnables already are. |

**Structure is the category that is currently homeless.** It has been built out of landscape
because landscape was the only thing that could be walked on, and that is precisely what made
the risers tilt.

## Structure is not a new object category

It is a **spawnable**. This was checked rather than assumed:

- The `Spawnable` trait is a pure factory — `material_count`, `create_materials`,
  `spawn(world, materials) -> Vec<Entity>`. It says nothing about physics or body type.
- `BodyType::Static` exists in `src/physics/body.rs` and is honoured by the solver.
- `grep BodyType src/app/spawnables/` returns **zero hits across all 34**. They are dynamic
  only because `RigidBodyDesc` defaults that way.
- **Concave static geometry is already solved** by compound bodies: `attach_collider` puts
  several colliders on one body, and `Table` (top slab + four legs) is a working concave
  example with `compound_cuboid_model` handling the rendering side. Convex decomposition is
  authoring several colliders, not a missing engine feature.

So a static, walkable, concave structure needs **no engine work** — it is a `RigidBodyDesc`
with `body_type: Static` plus the collider pattern `Table` already demonstrates.

A separate top-level category was considered and rejected: spawnables acquired segment-local
placement, orientation under segment yaw, the exhaustive `orientability()` classification, RON
representation and schematic drawing during stages 2–3. A parallel type re-earns all of it and
leaves two things to keep in sync forever.

## What *is* new: a walkable-surface capability

The line that matters is **"contributes walkable surface" vs "doesn't"**, and it cuts across
every category. Terrain has it. A walkway has it. A `Menhir` does not.

Today `level_check` sees terrain as surface and every spawnable as a point
(`ObjectInfo { kind, placement }`). A bridge built from spawnables would be invisible to the
reach checks, and a gap it spans would still be reported as a jump.

**The move:** stage 3 built `route_plan(&VolumeFeature) -> Option<RoutePlan>` in
`src/terrain/traversal/feature.rs` specifically so generation and `level_check` could not
disagree about where a deck is. Generalise it — an optional `route_plan()` on `Spawnable` —
so `RouteMap` stops caring what a route is *made of*. Then `RouteMap`, the gap-bridging check,
the width check and the schematic overlay all keep working while a route migrates from voxels
to structure, one staircase at a time.

Worth confirming early that `RoutePlan` as stage 3 defined it is general enough, rather than
shaped around `VolumeFeature`.

## Dual contouring — the medium-term voxel option

Not chosen, not rejected. A drop-in replacement for marching cubes at the meshing step: store
the intersection point *and* the surface normal per edge (Hermite data), place one vertex per
cell by solving a small least-squares problem. It reproduces sharp edges and corners exactly,
from the same voxel data, at the same resolution.

It would fix the tilted risers, bumpy ledges and rounded platform edges **while changing
nothing else** — still voxels, still destructible, still `query_region`, still the chunk
ownership and seam rules. Normals are free for authored SDF primitives (analytic gradient) and
already computed as central differences for noise terrain.

Costs: naive DC produces non-manifold vertices, so it wants Manifold DC or Cubical Marching
Squares; storage becomes per-edge Hermite data rather than a scalar per voxel; every open-edge
baseline and seam test re-bases. It is a stage, not an afternoon.

It does not replace the role split above — it raises the floor on what "landscape" can express.

---

## Next action: spike before briefing a stage

**Build one `Steps` spawnable** — a static compound body, one box collider per tread — and drop
it into `test_segments` beside the voxel staircase it replaces. Walk both.

Deliberately not a full stage. It answers, by feel rather than by reasoning, the questions that
would otherwise be assumptions in a spec:

- Does a static compound body behave as walkable ground, or does the solver treat it
  differently from `StaticGeometry` in a way that matters underfoot? Observation 2 above says
  surface quality is perceptible.
- Does a hand-authored step feel right where a 1 m voxel riser did not?
- Does terrain-anchored placement land it flush, or leave a gap or an overlap at the base?
- What does a grenade hitting it look like? Indestructible-for-now may be fine, or may read
  worse than the current floating deck.

If it feels good the stage writes itself. If not, an afternoon was spent instead of a stage.

## Design questions to settle before that stage

Ordered by how much they would hurt to get wrong.

1. **Anchoring.** A staircase spans from ground to ledge — defined by its two ends, not by a
   position. Segment anchors are already named local frames *with orientation*, which is
   suspiciously the right shape. Whether structure references anchors or is placed absolutely
   in segment-local coordinates is the decision that most affects authoring ergonomics.
2. **Where the walkable-surface capability lives.** See the `route_plan()` move above.
3. **How much is structure vs landscape.** `Steps`, `Walkway`, `Platform`, `SpiralRamp` are
   clear. `Ladder` needs climbing mechanics the player does not have. Keep the set small and
   name it explicitly.
4. **What happens to the stage 3 voxel primitives.** Keep all four; document the split by role
   rather than deleting anything. `level_check` could reasonably warn when a voxel
   `Staircase`'s rise suggests someone means to walk up it.

## Deferred, with reasons

- **Static-to-dynamic on destruction.** Fracture exists, but a static body becoming dynamic
  mid-frame is a transition the solver and sleep tracker need to handle deliberately. Agreed
  to defer; structure is indestructible in the interim.
- **Automated convex decomposition.** Hand-assembly is adequate for authored architecture.
  Revisit if imported meshes ever matter.
- **Ledge-grab** is a player capability, but it has a knock-on here: `src/level_check/reach.rs`
  is pure ballistics, and a grab changes the reachable set *qualitatively* — "landed short but
  at the right height" starts succeeding. **When ledge-grab lands, `reach.rs` must learn about
  it, or every gap validated before it is validated wrongly after.**
- **"A platform you can fall off should be re-enterable"** — observation 4 is a level-design
  lesson now, and a plausible `level_check` rule later.
- **Deck durability** (observation 7) is just material durability and is easy. It also argues
  for the split: "should this walkway survive the grenade that cratered the hill" is a natural
  per-prop question and an awkward per-voxel one.
