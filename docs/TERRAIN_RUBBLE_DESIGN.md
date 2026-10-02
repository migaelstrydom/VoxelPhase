# Terrain Rubble Design

Terrain that a blast cuts loose becomes rubble: real rigid bodies for pieces
worth simulating, falling scree for slivers, dust for crumbs. Rubble that comes
to rest is then deposited back into the voxel field as new terrain.

**Status:** design only. Nothing here is built yet.

---

## The problem

A grenade carves a sphere out of the density field (`Chunk::carve_sphere`), and
nothing checks what is left. Two kinds of leftover show up after a few blasts:

- **Floating slivers.** Where crater rims meet, the solid between them is a cusp
  thinner than a voxel. The lattice samples that land inside it keep a small
  positive density (`0.01`–`0.1`). Marching cubes draws a closed surface a few
  hundredths of a voxel either side of each one, so the result is a strip or
  sheet that looks paper-thin. Where the cusp's link to the ground fell between
  samples, the strip floats.
- **Zero-thickness shelves.** A blast centred below the surface removes the
  layer under a top-surface sample but just misses the sample itself. What is
  left is a lid one sample thick, cantilevered over the crater.

The mesh is correct. The field holds geometry that cannot stand up, and nothing
lets it fall.

## Goals

1. Terrain disconnected by a blast becomes **real rigid bodies**, collision and
   all.
2. Pieces that are **too thin or too small** to be worth a body fall out of the
   world as debris.
3. **Arbitrary topology is never handed to the physics engine.** A piece
   collides as a compound of convex hulls that the engine already supports.
4. The cost is bounded per blast and visible in `terrain_perf`.

And one goal that makes it more than cleanup:

5. **Rubble returns to terrain.** A piece that comes to rest is written back
   into the voxel field. It can then be dug, walked on as ground, and it
   reshapes the water network.

---

## Architecture

```text
 ExplosionSystem
   │ TerrainWorld::detonate(center, blast)
   ▼
 Segment::detonate ──▶ carve_sphere (unchanged)
   │
   ▼
 FragmentFinder                             terrain/, segment-local
   │  1. read a VoxelBlock around the crater
   │  2. mark which samples bear load
   │  3. flood fill from the block's boundary and bedrock: grounded
   │  4. label what is left: fragments
   │  5. lift each fragment out of the grid (samples → air)
   ▼
 Vec<Fragment>   world-space pose + its own voxel block    (segments invisible)
   │
   │ RubbleQueue (ECS resource)
   ▼
 RubbleSpawnSystem                          rubble/  (new top-level module)
   │  Grade::of(fragment)
   ├── Dust     ──▶ particle burst, gone
   ├── Scree    ──▶ FallingScree: own ballistic integrator, no physics body,
   │                drawn with its MC mesh, crumbles into dust when it hits
   │                ground or leaves the view
   └── Boulder  ──▶ BrickShaper: carved-brick convex compound
                    + RigidBody + ModelInstance(MC mesh) + Debris
                         │
                         │ physics: thrown by the same blast impulse,
                         │ tumbles, lands, sleeps
                         ▼
                    SettleSystem ──▶ TerrainWorld::deposit(fragment, pose)
                                     voxels stamped back into the field,
                                     body removed, water re-laid
```

The terrain side stays inside `src/terrain/`, and segments do not leak out:
`TerrainWorld` returns fragments with world poses and accepts deposits in
world space. The rubble side is an ordinary gameplay module that uses
physics, rendering and the existing debris budget.

---

## Part 1: Finding what came loose (`terrain/fragment.rs`)

### The search region

Connectivity only changes where samples changed, so the search is local. After
the carve, `FragmentFinder` reads the samples of a box around the crater into a
`VoxelBlock`:

```text
half extent = radius + margin,   margin = clamp(1.5 · radius, 4 voxels, 32 voxels)
```

A `VoxelBlock` is not a unit of terrain storage. It is the dense, flat sample
buffer that meshing already reads chunks into, sized to whatever box the caller
asks for. Terrain is stored in 32³-voxel **chunks**, and a crater usually
overlaps several of them. `ChunkGrid::fill_block` already fills one block from
every chunk the box overlaps (it is how a chunk's remesh reads its neighbours'
border samples), so the finder works on one seamless array and never sees a
chunk boundary. The writes back go through the same chunks the carve just
dirtied, plus any further chunk a fragment reaches into.

**Simplifying assumption A: anything that reaches the edge of the search region
is held up.** The flood fill seeds from:

- every bearing sample on the region's boundary;
- every indestructible sample;
- every bearing sample on the **segment's bounds**. Terrain cut off at a
  segment's edge is the edge of the authored world, not a break.

A severed bridge longer than the region stays standing. That is acceptable,
since a pillar that tall would want structural analysis anyway (see Part 6), and
it caps the cost at one flood fill over a region of known size.

If no seed is found, the **largest component is held up**. That is the case of a
small sky-island segment with the whole island inside the region: the island
holds itself up, and only what the blast cut off it falls.

At 1 m voxels and a 3 m crater the region is about 20³ samples. At 0.125 m
voxels it is capped by the 32-voxel margin at about 112³ (1.4 M samples, one
byte of label each). The flood fill is a plain queue over a bitset. The margin
cap is tuned against `terrain_perf`.

### Which samples bear load

A sample's density says how far the surface is from it. Marching cubes still
draws something around a sample of `+0.02`, but nothing that thin could hold
anything up. So the flood fill only travels through **bearing** samples:

| Rule | Why |
|---|---|
| `density ≥ BEARING_DENSITY` (≈ 0.25) | A sample closer than that to the surface is a rind. The slivers' samples all fail this. |
| Not a **sheet sample**: on no axis are both neighbours air. Only applies within `radius + 1` voxel of the blast. | This is what drops the zero-thickness shelves. Limiting it to the blast keeps authored thin decks elsewhere in the block from collapsing. |
| 6-connectivity | Conservative: a link that marching cubes would draw through a cell diagonal does not count. The worst case is that something falls that might have hung by a corner, and that reads correctly. |

Non-bearing solid samples are assigned after the flood fill:

- A non-bearing sample 6-adjacent to a grounded bearing sample stays in the
  ground. This is the **lip**: a shelf breaks off one voxel out from the cliff,
  not flush with it, which looks like a break rather than a cut.
- Every other non-bearing sample joins whichever fragment it touches. Connected
  non-bearing samples that touch no bearing sample form a fragment of their own.
  This is how the floating strips are handled.

#### 6-connectivity

Two samples are neighbours when they differ by one step along **one** axis, so
each sample has six: ±x, ±y, ±z. Samples that only share an edge (12 of them,
two axes differ) or a corner (8, all three differ) are not neighbours. Those
rules are called 18- and 26-connectivity.

One slice of the lattice. Around a sample `X`, `n` marks its neighbours in this
slice (the other two are directly above and below `X`), and `e` marks samples
that only share an edge with it:

```text
        e n e
        n X n
        e n e
```

So with `#` bearing and `.` not:

```text
      one piece                          two pieces

        . . . . .                        . . . . .
        . # # # .                        . # . . .
        . . . # .                        . . # . .   touches the one above
        . . . # #                        . . . . .   only along an edge
        . . . . .                        . . . . .
```

Why the strict rule: what marching cubes draws across a diagonal is a case
ambiguity. A face with two solid corners on one diagonal can be meshed joined
or split, and either way the joined version is a neck far thinner than a voxel.
Under 6-connectivity such a link carries no load, so the worst case is that
something falls that could have hung by a corner. That reads correctly in play.


### Lifting a fragment out

Each fragment becomes a `Fragment`:

```rust
pub struct Fragment {
    /// The fragment's samples and only those; every other sample is air.
    /// Padded by two samples of air: one so its own marching cubes
    /// closes, and one more for the central differences its normals take.
    pub voxels: OwnedVoxelBlock,
    /// Where the block's lattice sits in the world at the moment of the blast.
    pub pose: Isometry3<f32>,
    /// Lattice spacing of the segment it came from.
    pub voxel_size: f32,
}
```

In the grid, the fragment's samples are set to `Voxel::air()` (`-1`), and the
chunks are marked dirty in the same remesh as the carve. In the fragment's own
block, everything outside it is `-1`.

The fracture surfaces cannot overlap. Wherever ground sample *g* and fragment
sample *r* were neighbours, at least one of them was non-bearing, so the
ground's new surface sits at `d_g/(d_g+1) < ½` of the edge from *g*, and the
fragment's surface at `d_r/(d_r+1) < ½` from *r*. That always leaves a hairline
gap between body and socket. `reference_flush_shards_freeze_in_ccd` says a freed
piece needs exactly that gap.

Fragments never cross a segment boundary. Segments do not touch: joins carry a
gap, welded joins are not implemented, and each segment caps the terrain at its
own bounds (`LEVEL_SEGMENTS_PLAN.md`). The finder works on one segment's grid,
and its seeds at the segment's bounds hold up anything that reaches them.

---

## Part 2: Grading (`rubble/grade.rs`)

Three numbers, all cheap from the block:

- `volume`: solid sample count × voxel³.
- `core`: the number of **interior** samples (bearing, and all six neighbours
  solid).
- `extent`: the longest side of the solid samples' bounding box.

| Grade | Rule (initial values, tunable) | What happens |
|---|---|---|
| **Dust** | `volume < 0.02 m³` | A particle puff in the material's colour at the centroid. |
| **Scree** | no `core`, or `volume < 0.25 m³` | `FallingScree` (Part 4). |
| **Boulder** | otherwise, up to `MAX_BOULDER_VOXELS` | A rigid body (Part 3). |
| *too big* | `> MAX_BOULDER_VOXELS` (≈ 4 000) | Treated as grounded and left in the field. Becomes Part 6's question. |

"No core" is the thinness test: a fragment with no fully enclosed sample is a
shell, a sheet or a strip at most two samples thick. Those are exactly the
screenshot cases, and they should fall away rather than land and slide around
as zero-thickness bodies.

---

## Part 3: Boulders as convex compounds (`rubble/brick_shaper.rs`)

### Two shapes for one body

**Simplifying assumption C: a boulder is drawn with its true marching-cubes
mesh, and collides as a coarser compound of convex hulls.** The eye sees the
real fractured surface. The solver sees up to a dozen convex bricks. A boulder
the size of a car does not need sub-voxel collision.

The render mesh comes from running the existing marching cubes on the
fragment's own block, in body-local coordinates (origin at the centre of mass).

Away from the break, the re-run reproduces the old mesh exactly. A marching
cubes cell's triangles depend only on its eight corner samples, and its
normals on the central differences one sample further out. The fragment's block
keeps the global lattice indices (`SampleLattice::base`), so its sample
positions are bit-identical to the chunk's. Every cell whose corners and their
gradient neighbours all lie in the fragment therefore gives the same
triangles. Only the cells within two samples of the break differ, and they have
to, because the break surface did not exist before. Where the fragment was
joined to the ground there was solid, and now there are two new faces.

Taking the existing triangles would be harder and give a worse result:

- They are scattered over several chunks' `MeshOctree`s, interleaved with the
  ground's. Picking out the fragment's means classifying each triangle against
  the labelled samples, which is the same cell lookup marching cubes does
  anyway.
- They form an open mesh. The break faces would still need generating, and
  stitching them to the cut edges of the old triangles is the hard part of
  mesh fracture that the re-run avoids.
- Their baked AO was computed with the ground around them. A rock flying
  through the air is not shaded by the socket it came out of, so AO has to be
  rebaked on the fragment alone (`terrain::ao`) in any case.

The cost is small: a fragment block is tens of samples on a side, against the
32³-plus-border blocks every remesh already runs.

### Carved bricks

The physics engine has no quickhull from a point cloud, and does not need one.
It already has `cube_hull` and `split_hull`, which cuts a convex hull with a
plane. A brick is built from those:

```text
   octree cell of the fragment          its box ∩ a few half-spaces
   ┌──────────────┐                     ┌──────────────┐
   │      ▄▄▄▄▄   │                     │      ╱‾‾‾╲   │
   │   ▄██████▄   │   normal clusters   │    ╱      ╲  │
   │  ██████████  │  ───────────────▶   │   │        │ │
   │  ██████████  │   one plane each    │   │        │ │
   └──────────────┘                     └──────────────┘
```

1. **Partition.** Start from the box around the fragment's solid samples. Split
   a cell at the midpoint of its longest axis while it is *not convex enough*,
   it is wider than two voxels, and the brick budget (`MAX_BRICKS`, ≈ 12)
   allows. Leaves are disjoint, so the bricks are disjoint, and the compound's
   mass and inertia come out right with no override.
2. **Carve.** For each leaf, start from `cube_hull` of the cell clipped to its
   solid samples (plus half a voxel). Cluster the normals of the mesh triangles
   inside the cell into at most four directions. For each cluster, cut with
   the plane `n · x = max(n · v)` over all mesh vertices in the cell. That plane
   contains the cell's surface, so the brick is conservative.
3. **Judge.** A cell is convex enough when its brick's volume is no more than
   1.3 × the solid volume it holds. Past the brick budget, the coarse brick is
   kept and the overcover accepted.
4. **Inset.** Shrink each brick by a few hundredths, the way
   `CLEAVE_INSET` does, so no brick starts in contact with the socket it came
   out of.

`split_hull` already refuses wafers and slivers, so a degenerate cut is
skipped, never panicked on.

### Spawning

The frame order already does what spawning needs:

```text
 frame N:    physics_sync ─ … ─ explosion ─▶ rubble_spawn ─▶ terrain_update (remesh)
                                    │              │
                                    │ impulse      │ body added to PhysicsWorld
                                    ▼ queued       ▼
 frame N+1:  physics_sync: drains the impulse queue, steps against the remeshed terrain
```

`RubbleSpawnSystem` depends on `explosion` and adds the body to the physics
world directly, not through a lazy update. When the blast's `PhysicsImpulse`
is drained at the start of the next frame, the boulder exists and is thrown
like any other body near the blast, so no separate rubble kick is needed.
Physics first steps it after `terrain_update` has remeshed the socket, so the
body never shares a frame with the terrain it was cut from.

Bodies start asleep. A fragment beyond the impulse's reach (a shelf whose
support was carved away from a blast below it) would hang where it was.
Spawning wakes every fragment body.

The body gets:

- the compound of bricks, with the material's density and surface (from
  `VoxelMaterial::surface()`, so grass, rock and sand already have their own
  friction and bounce);
- `Debris { origin, size, age }`, so the existing `DebrisBudget` caps how many
  boulders exist;
- `bulk(...)` left unset, so the bricks are what water sees, and rock sinks.

### Texture that rides with the rock

Terrain is textured by triplanar projection of **world** position
(`shader/triplanar.glsl`). On a moving body the texture would swim across the
surface. Props already solve this for grain: `triangle.vert` passes the
model-space position (`outModelPos`) and normal, and grain is projected from
those.

Rubble uses the same route for its triplanar texture, with one addition: a
per-draw **projection origin**, set to the fragment's world centre of mass at
the moment of the blast. The shader projects `model_pos + origin` (and the
model-space normal) instead of the world position. At spawn the body's frame
has no rotation and sits at that centre, so the sum equals the world position:
the texture is identical to the terrain it came out of at the moment it breaks
off, and from then on it is fixed to the rock. A segment's yaw goes into the
fragment's starting pose, and so does nothing to the texture.

Cost: one flag and a `vec3` in the push constants, and one branch in
`triangle.frag` that picks the position the projection reads. When the boulder
is deposited (Part 5) it is textured by world position again, so the noise
under it jumps to a different patch. The noise is isotropic, so that is a
change in pattern, not in look.

---

## Part 4: Scree (`rubble/scree.rs`)

Scree is never a physics body. `FallingScree` holds a pose, a velocity and a
spin, and a small system integrates them under gravity. It draws its own MC
mesh exactly like a boulder.

- **Initial motion.** The blast's radial falloff at the fragment's centroid,
  plus a random spin. This is the same rule the knockback in `ExplosionSystem`
  already uses.
- **Landing.** Each frame, a terrain ray from the previous centroid to the
  current one is checked (`TerrainWorld` raycast). On a hit, the piece crumbles:
  a dust puff in its material's colour, and the entity is deleted. Nothing is
  seen sliding through the ground.
- **Leaving.** It is deleted once it is below the level's kill height or after
  `SCREE_LIFETIME` (≈ 4 s), whichever comes first.

This puts no load on the narrowphase and the solver, and none of the thin-body
contact trouble that a sheet would bring. It still reads as a sliver of the
cliff falling away.

---

## Part 5: Settling back into terrain (`rubble/settle.rs`)

This is the part that makes rubble more than an effect.

When a boulder has been asleep for `SETTLE_DELAY` (≈ 1 s) and every contact it
has is with static terrain (not resting on a barrel or the player),
`SettleSystem` calls:

```rust
TerrainWorld::deposit(&fragment, pose)
```

The terrain then stamps the fragment back into the field in its new pose:

```text
 for each terrain sample p in the body's world AABB, in the segment under it:
     q = pose⁻¹ · p                        fragment-local
     d = trilinear(fragment.voxels, q)     density there
     union: keep max(existing, d), taking the fragment's material where d wins
```

This is `csg::union_solid` with the fragment's resampled density standing in
for a distance function. Then the body and its entity are removed, and the
chunks remesh like any other edit.

Consequences:

- **The physics cost of rubble is temporary.** It lasts from the blast until
  things settle, typically a few seconds. A cliff blown into a pile leaves a
  pile of terrain, not two hundred sleeping bodies.
- **The pile is ground.** It can be walked on and dug into again. Blasting it
  can make new rubble.
- **Water notices.** A deposit is a terrain edit, so the hydrology network is
  re-laid (`docs/WATER_HYDROLOGY_DESIGN.md`). Bring a cliff down into a river
  and the river is dammed. Blast the dam and it flows again.

### Why the resampled field is valid terrain

Nothing has to be reconstructed, and the deposit does not need to find a shape
that marching cubes could have generated. Every density field is valid terrain.
Marching cubes is only how the field is drawn, and the next grenade carves the
field (`min` with the cut), not the mesh. The fragment's own block is already a
density field: the original terrain's samples. Turning it to a new angle and
resampling it at the target lattice's sample points gives another density
field. Within the ±1-voxel band that marching cubes reads it is close to a
distance, because a trilinear blend of a clamped distance is still close to
one there. From then on it is indistinguishable from generated terrain.

The only question is how closely the new field's surface matches the old one.

### How big the pop is: measured

**Simplifying assumption D: deposition is lossy below a voxel.** Resampling at
a new angle loses detail the lattice cannot hold. Here is how much.


`scratch/deposit_pop.py` (gitignored; `deposit_pop_corrected.py` adds the
bias correction below) runs the real procedure on three fragment shapes, 20
random landing poses each, with marching cubes on both sides:

1. sample a clamped distance on the lattice, as the terrain does;
2. mesh it (the in-flight mesh);
3. rotate and translate to a random pose, trilinearly resample onto the lattice,
   and mesh again (the deposited terrain);
4. measure the distance between the two surfaces, both ways, over dense surface
   samples.

Distances are in voxels (multiply by 0.5 m for most levels).

| Fragment | Volume | Mean | 99th pct | Worst point | Volume change |
|---|---|---|---|---|---|
| rounded rock, r ≈ 3 | 103 vox³ | 0.12 | 0.25 | 0.40 | −6.0% |
| sharp convex shard, 7 faces | 197 vox³ | 0.15 | 0.83 | 1.97 | −6.8% |
| broken shelf, 2.4 thick | 140 vox³ | 0.14 | 0.38 | 0.58 | −8.7% |

The volume loss is a systematic bias: trilinear interpolation of a clamped
distance cuts into convex features. It is almost the same for every shape and
pose. Adding a constant **+0.055 to the resampled density** (±0.003 across all
60 runs) removes it entirely and improves every other figure:

| Fragment | Mean | 99th pct | Worst point | Volume change |
|---|---|---|---|---|
| rock | 0.10 | 0.23 | 0.36 | 0.0% |
| shard | 0.13 | 0.56 | 1.65 | 0.0% |
| shelf | 0.12 | 0.30 | 0.51 | 0.0% |

So at 0.5 m voxels the surface moves by **5–7 cm on average**, and 99% of it by
less than 12–28 cm. The large figure is the tip of a sharp corner, which is
blunted by up to 0.8 m. That is one vertex: a lattice cannot hold a tip
sharper than its spacing at an arbitrary angle. The in-flight mesh only had it
because it was still aligned to the lattice it was cut on. The deposit offset
is a constant (`DEPOSIT_BIAS`), checked by a unit test that deposits a
fragment at random poses and compares volumes.

### Keeping it out of the bug tracker

The deposit itself is a pure function: (fragment, pose, grid) → density
writes. It is unit-testable offline, deterministic, and its errors are the
figures above. Almost all of the risk is in **when** to deposit, so each
condition that could go wrong just keeps the boulder as a body:

| Condition | Why |
|---|---|
| Asleep for `SETTLE_DELAY`, touching only static terrain | Never stamp a rock that is still moving or is resting on a body. |
| No dynamic body or character within its AABB + 1 voxel | The stamped surface can be up to half a voxel proud of the rock. Nothing may end up inside it. |
| Its AABB lies inside one segment's bounds | No deposit into a cap or the gap between segments. |
| Out of view, or further than `DEPOSIT_VIEW_DISTANCE` | Nobody sees the blunted tip or the texture change. |

A boulder that never meets these conditions stays a sleeping body until the
`DebrisBudget` culls it, which is how fractured props behave today. **The worst
case of a deposit bug is today's behaviour**, so Part 5 can be switched off
with one flag, and a regression can't do worse than that.

**Proof of concept:** the geometry is answered by the numbers above. What is
left is engine plumbing, so the PoC is Phase 4's first commit: `deposit()`
with its tests, plus a `level_viewer` before/after picture of a fragment
deposited at an angle onto real terrain. If that picture is wrong, Part 5 is
dropped and Parts 1–4 stand without it.

### Is this new?

Minecraft's falling sand turns back into blocks, but it falls axis-aligned and
lands on the grid. Teardown's voxel debris stays as bodies. I don't know of a
game where arbitrarily rotated rigid chunks of smooth SDF terrain are deposited
back into the field, and where that rebuilds a water network. I can't rule it
out, but blast-a-cliff-to-dam-a-river would be a fair headline.

---

## Part 6 (later): Hanging by a neck

Connectivity alone keeps a huge overhang up as long as one bearing sample still
joins it to the ground. A cheap structural rule can reuse the same flood fill:

- Fill once through samples above `BEARING_DENSITY` (strong), and once through
  any solid sample (weak).
- A component that is grounded in the weak fill but not the strong one is
  **hanging by a neck**. The neck is the set of non-bearing samples between it
  and the ground.
- It breaks if its mass × its lever arm from the neck exceeds the neck's
  capacity (sample count × material strength).

This also sets a policy for fragments over the size cap: a large piece hanging
by a neck can come down, or stay as terrain until the neck goes.

---

## Module layout

```text
src/terrain/
  fragment.rs        FragmentFinder, Fragment, the bearing rules
  segment.rs         detonate() returns fragments; deposit()
  world.rs           TerrainWorld::detonate → Vec<Fragment> (world pose); deposit()

src/rubble/          //! Terrain cut loose by a blast: graded, simulated, and settled back into the ground.
  mod.rs
  grade.rs           Grade::of(&Fragment)
  brick_shaper.rs    Fragment → Vec<ConvexHull> (carved bricks)
  mesh.rs            Fragment → body-local render mesh (terrain marching cubes)
  scree.rs           FallingScree + its system
  spawn.rs           RubbleQueue, RubbleSpawnSystem
  settle.rs          SettleSystem
  config.rs          RubbleConfig (thresholds, budgets, delays)

src/rubble_viewer/   //! Offline harness for rubble: scripted blasts on synthetic segments, judged by invariants.
src/bin/rubble_viewer.rs
```

`TerrainWorld::detonate` changes from `()` to returning fragments. Callers that
don't care (tools, benches) drop them, and the fragments' samples are still
removed from the field. Every tool then sees the same terrain the game does.

---

## Testing

Three layers. None of them needs a GPU, so all of them run in `cargo test`,
except the filmstrip, which is for looking.

### Unit tests: next to the code

| Module | Tests |
|---|---|
| `terrain/fragment.rs` | Hand-built fields, one rule each: the two-carve cusp from the screenshot detaches; a shelf over a crater breaks at a one-voxel lip; a link through an edge only (diagonal) detaches; a pillar reaching the region's edge stays; a sky island with no seed keeps its largest piece; indestructible and segment-bound samples seed. **Conservation:** solid samples before = after + Σ fragments, nothing lost or duplicated. **Gap:** the ground's and the fragment's new surfaces never meet. |
| `rubble/grade.rs` | Table-driven: dust, scree, boulder, too-big at each boundary. |
| `rubble/brick_shaper.rs` | Bricks are disjoint; they cover ≥ 95% of the solid samples; total volume within 30% of the voxel volume; at most `MAX_BRICKS`. A seeded property test over 500 random fragments: no refused hull panics, every brick is a valid `ConvexHull`. |
| `rubble/mesh.rs` | Away from the break, the fragment mesh's triangles are bit-identical to the chunk mesh it was lifted from. |
| `terrain` deposit | The Python measurement ported: random poses, volume within 1%, surface error within the table in Part 5. Union never lowers a density. A deposit outside every segment writes nothing. |
| `rubble/settle.rs` | Each deposit condition blocks a deposit on its own. |

### End to end: a new `rubble_viewer`, not the bench harness

The physics bench harness is the wrong home. It tests the physics engine
alone, with no ECS, no terrain and no explosions, and CLAUDE.md only asks for
it when the physics engine changes. Rubble crosses five systems: explosion,
terrain, rubble, physics and water.

The model is `water_viewer`: a library module `src/rubble_viewer/` with a thin
`src/bin/rubble_viewer.rs`, scripted scenarios on small synthetic segments, a
report over time, and tests that run the catalogue.

```text
 Scenario { terrain recipe, script: [Beat { at, Blast | Drop | Wait }] }
     │
     ▼
 driver: the game's simulation systems, minus RenderSystem
     │      explosion → rubble_spawn → terrain_update → physics → settle → water
     ▼
 per frame: invariants (fail fast, with the frame and the fragment)
 at the end: the scenario's own expectations
     │
     ▼
 report (text, CSV)   ·   --film: filmstrip through level_viewer's renderer (GPU)
```

**Invariants**, checked every frame of every scenario:

- **No floating terrain.** A flood fill over the whole segment (not just the
  search region) reaches every bearing solid sample from a seed.
- **Volume ledger.** Terrain volume + boulders + scree + dust = the initial
  volume − what blasts carved, within the deposit tolerance. The same idea as
  water's ledger.
- **No spawn ejection.** A boulder's kinetic energy after its first step is
  no more than the blast impulse gave it, plus a small margin. A violation
  means the body started inside the terrain.
- **Bounded.** Fragment and body counts stay under their caps. Everything is
  finite.

**Scenarios**, each with expectations of its own:

| Scenario | Expects |
|---|---|
| `shelf` | A blast under a slab: one fragment, a lip ≤ 1 voxel left on the cliff, it lands. |
| `rim_cusps` | The screenshot case reproduced: 30 seeded grenades on one patch of flat ground. After every blast, no fragment of grade scree or dust is left in the terrain. |
| `arch_both_legs` / `arch_one_leg` | Cut both legs: the span falls as one boulder. Cut one: it stays. |
| `sky_island` | A blast at the edge of a small island segment: the island stays, the piece cut off falls. |
| `long_bridge` | A bridge cut at both ends but longer than the search region stays: assumption A, recorded so a change to it is a decision. |
| `settle` | A boulder lands, sleeps, is deposited; the ledger closes. |
| `settle_blocked` | The same with a crate resting on the boulder: no deposit. |
| `river_dam` | The Phase 4 headline: a cliff dropped into a channel deposits and raises the water upstream. |

`cargo test --release --lib rubble_viewer` runs the catalogue. The slow ones
are `#[ignore]`d, like `level_check::rest`.

**Fuzz:** `rubble_viewer --fuzz <seeds>` sets off seeded random blasts over a
real level's terrain (`perf::Ground`) with only the invariants as judge, in
the manner of `water_fuzz` and `physics_fuzz`. A failing seed replays exactly,
and once understood becomes a scenario.

### Performance: the existing benches, one new stage each

| Bench | Addition | Initial budget |
|---|---|---|
| `terrain_perf` | `TerrainStage::FindFragments` and `LiftFragments`; fragments per blast in the record. The mesh fingerprint changes once, on purpose, when Phase 1 lands. | Finder ≤ 0.5 ms per blast at 0.5 m voxels, ≤ 2 ms at 0.125 m. |
| `physics_perf` | A `cliff_collapse` scenario on real terrain: physics stages while the rubble tumbles, after it sleeps, and after it is deposited. | Spawn (mesh + AO + bricks) ≤ 1 ms per boulder. After the deposit, physics cost back to the pre-blast figure. |
| `render_perf` | `RenderCounters` for rubble draws and mesh uploads, during the existing blast scenario. | Within the frame budget with the per-blast fragment cap reached. |

The budgets are starting figures to measure against, not promises.

---

## Phases

Each phase is shippable on its own and checked with the existing tools.

| Phase | What | How it is checked |
|---|---|---|
| **1. Finder + dust** | `FragmentFinder`; every fragment just becomes a dust puff. This alone fixes the screenshots. | Unit tests: two overlapping carves leave a cusp, and the finder removes it; a shelf over a crater breaks off at a lip; a pillar reaching the block edge stays. `terrain_perf`: finder cost per blast, fingerprint changes only near rims. `level_check --mesh-edges`: no open edges. |
| **2. Scree** | `FallingScree`, render mesh from the fragment's own marching cubes. | `render_perf` during a blast: draw counts, cost. Play-test. |
| **3. Boulders** | Brick shaper, compound bodies, debris budget. | Unit tests on the shaper (disjoint bricks, total volume within 30% of the voxel volume, never a refused hull panic). `physics_fuzz`-style seeded blasts: energy, finiteness. `physics_perf`: cost of a cliff collapse. |
| **4. Deposition** | `TerrainWorld::deposit` with `DEPOSIT_BIAS` and its tests first, then `SettleSystem` and its conditions. | Volume conserved to within 1% at random poses; surface error within the measured table. `level_viewer` before/after. Water re-lays: a `water_viewer` scenario where a deposited boulder dams a channel. |
| **5. Necks** | Part 6. | A test overhang that drops when its neck is cut. |

## Risks and open questions

Open, each with the phase that has to close it:

- **A world without a renderer (Phase 1).** `rubble_viewer` needs the game's
  simulation systems in the game's order without `RenderSystem`, and today
  `GameWorld::assemble` takes a `Renderer`. The preferred fix is to split the
  dispatcher builder so the simulation part can be built alone, which the
  game and the tool then share. A hand-assembled subset of systems would
  drift from the real frame order, so it is the fallback, not the plan.
- **Calibrating `BEARING_DENSITY` (Phase 1).** 0.25 is reasoned, not
  measured. `rim_cusps` is the calibration: the lowest value that leaves no
  scree-grade sliver after 30 blasts, then checked against `shelf` for
  over-eager collapse.
- **A per-blast fragment cap (Phase 3).** One blast through a honeycomb can
  free dozens of pieces. Past `MAX_BOULDERS_PER_BLAST`, the smallest are
  downgraded to scree. The cap is set from `physics_perf`'s `cliff_collapse`.

- **Terrain shading on a moving mesh.** Both draw paths use the same `Vertex`
  (colour, normal, AO, surface character) and `triangle.frag`, so a
  `ModelInstance` can carry a fragment's mesh. What is new is the projection
  origin from Part 3. Phase 2 has to confirm that the terrain draw's other
  per-draw state (triplanar scale, material texture) is reachable from a
  `ModelInstance` draw.
- **Thin authored geometry.** The sheet-sample rule only applies near the
  blast, but a grenade on a one-voxel deck now drops a piece of it. That is
  probably wanted. A per-material or per-segment opt-out is cheap if it isn't.
- **Search cost at fine voxels.** The 0.125 m segment in `test_arena` is where
  the margin cap matters. Measure before choosing the cap.
- **The slivers that this does not remove.** A cusp still joined to bearing
  ground through bearing samples stays. If thin fins still show up, the
  sheet-sample rule can be extended to any axis-thin bearing sample in the
  carve shell.
