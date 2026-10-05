# Terrain Rubble Design

Terrain that a blast cuts loose becomes rubble: real rigid bodies for pieces
worth simulating, falling scree for slivers, dust for crumbs. Rubble that comes
to rest is then deposited back into the voxel field as new terrain.

**Status:** Phase 1 is on `main` since 2026-10-04 (`src/terrain/fragment.rs`, `src/terrain/split_race.rs`, `src/rubble/`,
`src/rubble_viewer/`, the play-test level `levels/rubble_garden.level.ron`):
every fragment crumbles into dust. Phases 2 (scree) and 3 (boulders) are on
the `rubble-phase2` branch: fragments are graded, dust crumbles, scree falls
drawn with its own marching-cubes mesh and the terrain's texture, and boulders
are rigid bodies of carved bricks that tumble, land and sleep. Phase 2 was
play-tested 2026-10-04; a boulder crumbling where it landed read as vanishing,
so the branch goes to `main` only with Phase 3, play-tested 2026-10-05. Phases 4–5 are design
only. The play-test
level turned up problems that blocked the design: pieces longer than about
3 m never fell (R1), and box edges read as loose strips (R3, R4). Those are
fixed, and so is the cut-loose cost on `skyway` (R17). A grenade now cuts the
same crater at any voxel size (R2). What is still open is minor or older than
rubble. Issues are tracked in [ISSUES.md](ISSUES.md), and
what was measured to find them is in [NOTEBOOK.md](NOTEBOOK.md).

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

Connectivity only changes where samples changed, so the search is local.
`Crater::survey` reads the samples of a box around the crater into a
`VoxelBlock` **before** the carve, and `Search::run` reads the same box again
after it:

```text
changed = box around the solid samples within radius + 1 voxel
depth   = radius − distance to the nearest of them
region  = changed grown by margin,   margin = clamp(3 · depth, 6 voxels, 32 voxels)
```

The box is sized by what the carve changes, not by its sphere. A grenade in
the open spends its budget on the nearest rock, so its radius can be 11 m to
take a thin cap off a wall 11 m away. Sized from that radius, the survey read
89³ samples of mostly air and cost 25 ms (ISSUES.md R17). For a grenade on a
surface, depth is about the radius and the region is what it always was.

The margin was the reach of the search before the race (below) grew the
region; it now bounds only how far a weak piece is followed. Measured on
`test_arena` at 1 m voxels, per grenade: 1.5 radii cost 0.08 ms, 4 radii
0.25 ms, the 32-voxel ceiling everywhere 5.8 ms. At 3 radii the whole search,
both reads and up to four passes (below), costs 0.31 ms a grenade at 1 m voxels
and 0.32 ms at 0.125 m.

A `VoxelBlock` is not a unit of terrain storage. It is the dense, flat sample
buffer that meshing already reads chunks into, sized to whatever box the caller
asks for. Terrain is stored in 32³-voxel **chunks**, and a crater usually
overlaps several of them. `ChunkGrid::fill_block` already fills one block from
every chunk the box overlaps (it is how a chunk's remesh reads its neighbours'
border samples), so the finder works on one seamless array and never sees a
chunk boundary. The writes back go through the same chunks the carve just
dirtied, plus any further chunk a fragment reaches into.

**Simplifying assumption A: anything that reaches the edge of the search region
is held up.** The flood fill seeds from every bearing sample on the region's
boundary and every indestructible sample, and a piece that reaches the boundary
through weak samples is held up too. With a fixed margin, that held up anything
longer than about 3 m at 0.5 m voxels (ISSUES.md R1), so the region now grows.

#### Growing the region: racing searches (`terrain/split_race.rs`)

After the carve, one breadth-first search starts from every bearing sample
within the undercut reach (+1 voxel) of the crater. They run over bearing
samples only (the sheet rule applies near the crater, as in `classify`),
round-robin, one sample each per round. This is Even and Shiloach's
decremental-connectivity trick:

- searches that touch merge (union-find over search ids);
- a search that reaches an indestructible sample is anchored, held, and stops;
- a search whose frontier empties has **closed off** a whole bearing piece,
  having walked that piece and nothing more;
- the race stops when no search is running, or when one is running and none
  is anchored. That last one is the largest piece, and is never walked.

The region becomes the survey box merged with a box around every closed piece,
padded by 3 samples so the piece's weak rind and lips are inside it. The
search then runs on that region exactly as before: the field before the blast
is the grid read over the region, with the survey's pre-carve samples and every
sample an earlier pass lifted written back over it. So the race decides only
**how far to read**. Whether a piece falls is still decided by the rules below:
before and after, bearing, lip, paper.

Assumption A becomes: **a piece is held up at the edge of the region only if it
is the largest piece racing, or the race ran out of budget (`RACE_BUDGET`,
2¹⁸ samples) before it closed.** Beside an anchored search, even the largest
piece is raced to the end, because an anchor outweighs any size. A blast on a
large authored island walks the whole island when it is not the largest piece
racing, which is what the budget bounds.

There is no seed at the segment's bounds. Terrain cut off there ends in a
visible cap, so a piece hanging only from that cap is floating, and falls.

#### Before and after: authored floating terrain

An island over the ground stands on nothing the search can see. Judged on the
field after the carve alone, any blast near it would bring the whole island
down. So each piece standing free after the carve is judged by what it was
part of before:

- **Cut off something held up:** some of its samples were held up before the
  carve. It falls.
- **Cut out of something that already stood free** (an authored island, or a
  small island segment read whole): the largest such piece that has a bearing
  sample goes on standing as the island did. The rest fall. A piece with no
  bearing sample is no island if the blast reached it.

A carve only removes solid, so every sample of a piece after the carve was solid
before it, and the piece lay within one thing.

### Which samples bear load

A sample's density says how far the surface is from it. Marching cubes still
draws something around a sample of `+0.02`, but nothing that thin could hold
anything up. So the flood fill only travels through **bearing** samples:

| Rule | Why |
|---|---|
| `density ≥ BEARING_DENSITY` (≈ 0.25) | A sample closer than that to the surface is a rind. The slivers' samples all fail this. |
| Not a **sheet sample**: on no axis are both neighbours air. Only applies within `radius + 2` voxels of the blast. | This is what drops the zero-thickness shelves. Limiting it to the blast keeps authored thin decks elsewhere in the block from collapsing. The carve changes samples a voxel past its radius, so a sample one voxel further out can have lost the neighbours either side of it. |
| 6-connectivity | Conservative: a link that marching cubes would draw through a cell diagonal does not count. The worst case is that something falls that might have hung by a corner, and that reads correctly. |

Non-bearing solid samples are assigned after the flood fill:

- A non-bearing sample 6-adjacent to a grounded bearing sample stays in the
  ground. This is the **lip**: a shelf breaks off one voxel out from the cliff,
  not flush with it, which looks like a break rather than a cut.
- A **rind** sample is a lip across an edge or a corner too: one of its 26
  neighbours bearing and grounded is enough. A rind is non-bearing only because
  the surface runs right past it, and marching cubes draws it at least half a
  voxel thick across every axis. The edges and corners of a box authored on the
  lattice are rind: their samples store `SURFACE_BAND`, and their only bearing
  neighbour is diagonal (ISSUES.md R3). A non-bearing sample drawn thinner than
  that reaches its bearer through a face only, or a flap touching ground at a
  corner would stay.
- Every other non-bearing sample joins whichever fragment it touches. Connected
  non-bearing samples that touch no bearing sample form a fragment of their own.
  This is how the floating strips are handled.
- Near the blast, a non-bearing sample that marching cubes would draw
  **paper-thin** (two surfaces less than half a voxel apart across some axis)
  is never a lip, and joins only other paper-thin samples. A lip that thin is
  the flap the screenshots show, and a flap on an island's side is its own
  piece, not part of the island.

Lifting a piece can strip the last neighbour from a sample beside it and leave
that paper-thin in turn, so `cut_loose` searches the same crater again until
nothing more comes away (at most eight passes; no scenario lifts on a sixth).
A piece can reach far past the crater, so beyond the undercut the paper rule
applies to any sample the blast **made** paper-thin: thick in the field before
it, thin after. A strip that was paper as authored stays, or each pass would
unzip it a sample further. Each pass's region takes in everything earlier
passes lifted (ISSUES.md R5).

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
own bounds (`LEVEL_SEGMENTS_PLAN.md`). The finder works on one segment's grid.

---

## Part 2: Grading (`rubble/grade.rs`)

Three numbers, all cheap from the block (as built; `Measure`):

- `volume`: solid sample count × voxel³.
- `samples`: the solid sample count.
- `bearing`: how many samples bear load (`BEARING_DENSITY` or more).

| Grade | Rule (initial values, tunable) | What happens |
|---|---|---|
| **Dust** | `volume < 0.02 m³`, or a skin (no bearing sample) of fewer than 8 samples | A burst of flecks in its materials' colours at the centroid. |
| **Scree** | `volume < 0.25 m³`, fewer than 4 samples, or a skin of 8 or more | `FallingScree` (Part 4). |
| **Boulder** | otherwise | A rigid body (Part 3). |

The design graded a fragment with no **core** sample (bearing, all six
neighbours solid) as scree: a shell, a sheet or a strip, which should not land
and slide around as a body with no thickness. Once bricks were fitted to the
drawn surface a thin piece had a thickness to collide with, and the rule only
turned 8 m lengths of column into scree that crumbled where it landed (R29). A
piece of 1–3 samples is scree whatever its volume: its every face is sub-voxel
detail, and as a body it started inside the crater wall (E23). A skin, with
no sample deep enough to bear load, is drawn a few hundredths to a few tenths
of a voxel thick; the hill's skin came off in pieces of up to 57 samples and
vanished as dust where it broke (R36). A large skin now falls as scree and
shatters where it lands; as bodies, skins kept a collapse from settling
(E26). The too-big cut-off (`MAX_BOULDER_VOXELS`, Part 6) is not built.

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

As built (Phase 3), simpler and in two ways different (`rubble/brick_shaper.rs`,
E22):

- No normal clustering. Each brick is the 26-sided hull (box, twelve edge and
  eight corner bevels) of what its cell holds: its samples, the mesh vertices
  on their edges, and the midpoints to the samples of the cells it meets. It
  is fitted to the **mesh**, not to the samples taken as cubes: those held a
  resting boulder half a voxel off the ground (ISSUES.md R25). Every face
  stands in by the inset, 0.1 voxels.
- Bricks hold no air in a hole, gap or hollow (`Occupancy::encloses`: the
  fragment on both sides along a lattice line). A brick holding some is
  split, however many cells that takes, so no brick spans a hole or the gap
  between stalactites (R30). Cells are cut where the halves' boxes shrink
  most.
- A shape that needs more than `max_bricks` (16) cells is not one body
  (R31). `rubble::Cracker` cracks it into as many pieces as its cells fill
  bodies: seeds spread by farthest-point sampling, pieces grown from them
  through face neighbours with jittered costs, so each is connected and the
  seams wander like cracks (R34, R35). `Fragment::split` cuts each out as a
  fragment of its own, its siblings its obstacles; each is graded and planned
  in turn. A hollow dome comes down like a cracked egg, not as a lid resting
  on its hollow. Breaking a chunk into pieces was the user's suggestion two
  play-tests before it was built.
- The bricks know the ground. A fragment records the samples of its block
  that were solid but not its own (`Sample::Obstacle` in `terrain::Occupancy`).
  A cell whose brick holds one, or the midpoint to a neighbour, is split
  first, whatever its overcover; past the budget it is cut back with a plane.
  Without this a convex brick reached into the ground a column or slab broke
  from, and the solver threw the body out (R26).

The body (`rubble/boulder.rs`) is made at the mesh's origin with one hull
collider per brick, its density set so it weighs what its samples do, and is
recentred on its colliders; the mesh follows the move. It starts still but
spinning, so it is awake, and the shove queued for the next physics step
throws it as the design below says. A long piece's spin is held so its
furthest point turns no faster than 1.5 m/s: at a pebble's rate a boulder's
end swung into the ground in its first frame (R27).

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
  boulders exist. As built: a budget of their own (`rubble::BoulderCullSystem`,
  `ResidentBoulder`), 120 boulders, 4 s grace, the smallest crumbling to dust
  where they lie; the props' 40-piece budget took a collapse's boulders 1.5 s
  after they landed (R37);
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
off, and from then on it is fixed to the rock. A segment's yaw is baked into
the mesh (`Fragment::mesh` returns vertices in world axes), so the starting
orientation is the identity and the yaw does nothing to the texture.

As built (Phase 2): the anchor travels in the surface table, not the push
constants, which hold only a surface index. `SurfaceParams::anchored_at` sets
the source flag `ALBEDO_MODEL_SPACE` and writes the anchor into `detail.yzw`,
and `triangle.frag` projects `inModelPos + anchor` with the model-space normal,
then turns the detail normal into the world. The terrain's texture belongs to
`TerrainWorld`, and a `TextureHandle` cannot be cloned safely, so a fragment is
not a `ModelInstance`: it is a `TerrainMeshInstance`, which `RenderSystem`
draws with `Renderer::draw_model_as`, the terrain's texture and the terrain's
surface, anchored. `probe.frag` takes the same branch, so a piece's texture
holds still in a reflection too. When the boulder
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
- **Landing.** Each frame, a terrain ray along the frame's motion from the
  centroid, as long as the motion plus how far the piece reaches that way (its
  mesh's extreme points over 26 directions, turned with it). On ground (a
  surface facing up) the piece crumbles: a dust puff, and the entity is
  deleted. Nothing is seen sliding through the ground. A wall or a ceiling
  takes the velocity going into it and the piece falls on: slivers in a notch
  were otherwise thrown up into the rock over them and crumbled on their first
  frame (ISSUES.md R18). A piece glancing off walls for more than six frames
  running is wedged, and crumbles there.
- **Leaving.** It is deleted once it is more than 4 m below the terrain's
  lowest point (no level has a kill height) or after 4 s, whichever comes
  first.
- **Speed.** The blast's impulse divided by the piece's mass
  (`VoxelMaterial::mass_density`), capped at 12 m/s.

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
| `terrain/fragment.rs` | Hand-built fields, one rule each: the two-carve cusp from the screenshot detaches; a shelf over a crater breaks at a one-voxel lip; a link through an edge only (diagonal) detaches; a box on the lattice keeps its edges (rind) while a thin flap touching ground at an edge falls; a pillar reaching the region's edge stays; a sky island with no seed keeps its largest piece; indestructible and segment-bound samples seed. **Conservation:** solid samples before = after + Σ fragments, nothing lost or duplicated. **Gap:** the ground's and the fragment's new surfaces never meet. |
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

**Invariants**, checked after every blast of every scenario (Phase 1 has the
first and a second; the rest arrive with bodies):

- **No floating terrain.** A flood fill over the whole segment (not just the
  search region) finds no more solid standing free than before the blast.
- **Nothing paper-thin.** No more samples drawn thinner than half a voxel
  across some axis than before the blast.
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
| `rim_cusps` | The screenshot case reproduced: 60 seeded grenades across a field at 1 m voxels. With nothing lifted they leave 4 samples floating and 7 drawn paper-thin; with Phase 1, none after any blast. |
| `arch_both_legs` / `arch_one_leg` | Cut both legs: the span falls as one boulder. Cut one: it stays. |
| `sky_island` | A blast at the edge of a small island segment: the island stays, the piece cut off falls. |
| `long_bridge` | A bridge cut at both ends, longer than any fixed margin around either crater, falls whole: the race closes it off. |
| `settle` | A boulder lands, sleeps, is deposited; the ledger closes. |
| `settle_blocked` | The same with a crate resting on the boulder: no deposit. |
| `river_dam` | The Phase 4 headline: a cliff dropped into a channel deposits and raises the water upstream. |

`cargo test --release --lib rubble_viewer` runs the catalogue, in a few
seconds. A scenario can record a **known gap**: it is expected to break an
invariant, and fails when it stops doing so (`garden_hill` was one, until R5
was fixed).

**Fuzz:** `rubble_viewer --fuzz <seeds>` sets off seeded random blasts over a
real level's terrain (`perf::Ground`) with only the invariants as judge, in
the manner of `water_fuzz` and `physics_fuzz`. A failing seed replays exactly,
and once understood becomes a scenario.

### Performance: the existing benches, one new stage each

| Bench | Addition | Initial budget |
|---|---|---|
| `terrain_perf` | ✓ `TerrainStage::CutLoose` (survey, search and lift together) and fragments per blast in the table. | Finder ≤ 0.5 ms per blast at 0.5 m voxels, ≤ 2 ms at 0.125 m. Measured: 0.4–0.5 ms mean at 1 m (`test_arena`, `skyway`); 0.9–1.7 ms at 0.5 m (`island_sea`, the Rubble Garden), where the craters are now as big as at 1 m; 41 ms at 0.125 m (ISSUES.md R8). |
| `physics_perf` | A `cliff_collapse` scenario on real terrain: physics stages while the rubble tumbles, after it sleeps, and after it is deposited. | Spawn (mesh + AO + bricks) ≤ 1 ms per boulder. After the deposit, physics cost back to the pre-blast figure. |
| `render_perf` | `RenderCounters` for rubble draws and mesh uploads, during the existing blast scenario. | Within the frame budget with the per-blast fragment cap reached. |

The budgets are starting figures to measure against, not promises.

---

## Phases

Each phase is shippable on its own and checked with the existing tools.

| Phase | What | How it is checked |
|---|---|---|
| **1. Finder + dust** ✓ | `Crater`/`Search`/`cut_loose`, the region grown by `split_race`; every fragment crumbles into a burst of the blast's debris effect, scaled down (a material-coloured puff waits for Phase 2). | 21 unit tests in `terrain::fragment`, one per rule, each shown to fail with its rule removed. `rubble_viewer` (`cargo test --lib rubble_viewer`): every scenario, with "nothing more standing free, nothing more paper-thin after any blast" as the invariant. `terrain_perf`: one `cut loose` stage, fingerprints unchanged on sweeps that cut nothing loose. |
| **2. Scree** (branch) | `Grade`, `FallingScree`, render mesh from the fragment's own marching cubes, the anchored projection. | Unit tests: grading, flight, the mesh bit-identical to the ground's. `rubble_viewer`: every scree lands, none on its first frame. `render_perf` during a blast: 3 scree draws, `rubble_spawn` 0.7 ms, no validation errors. Play-test owed. |
| **3. Boulders** (branch) | Brick shaper, compound bodies, debris budget, at most 8 boulders a blast. | Unit tests on the shaper (every sample in one brick, the surface within the inset, disjoint bricks, volume no more than 30% over, 300 seeded lumps without a refused hull). `rubble_viewer`: every boulder comes to rest, none is moved off its free flight in its first frame (E22). Spawn cost over budget (R28, R33). Play-tested 2026-10-05: tall columns, the table, the pavilion, the hill's shell (cracked into patches, R31–R35, R39). |
| **4. Deposition** | `TerrainWorld::deposit` with `DEPOSIT_BIAS` and its tests first, then `SettleSystem` and its conditions. | Volume conserved to within 1% at random poses; surface error within the measured table. `level_viewer` before/after. Water re-lays: a `water_viewer` scenario where a deposited boulder dams a channel. |
| **5. Necks** | Part 6. | A test overhang that drops when its neck is cut. |

## Risks and open questions

Design risks, each with the phase that has to close it. Problems found by
building and running it are in [ISSUES.md](ISSUES.md).


- **A world without a renderer (Phase 3).** Settled for now: `rubble_viewer`
  steps a `PhysicsWorld` of each blast's boulders directly against the
  remeshed terrain, with the same planner and body builder as the game, and
  needs no ECS. Phase 4's settling may need the game's systems in order.
- **The volume ledger (Phase 3).** With dust as the only outcome there is
  nothing for a ledger to follow: the unit tests check that the samples lifted
  are exactly the samples the fragments carry. The ledger arrives with bodies.
- **`BEARING_DENSITY` (calibrated in Phase 1).** Swept over every scenario:
  from 0 to 0.25 they all pass and `rim_cusps` cuts 11–12 one-sample
  fragments; at 0.4, `rim_cusps` cuts 57 fragments of up to 8 samples out of
  ordinary crater rims and `arch_both_legs` no longer brings the span down.
  The sheet and paper-thin rules do most of the work, so 0.25 is the highest
  value that does not erode sound terrain, and the unit tests' weak bars are
  what it still catches alone.
- **A per-blast fragment cap (Phase 3).** One blast through a honeycomb can
  free dozens of pieces. Past `RubblePlanner::max_boulders` (8, a starting
  value), the smallest are downgraded to scree. No scenario makes more than
  one boulder a blast yet; the cap still wants setting from a measured
  collapse.

- **Terrain shading on a moving mesh.** Both draw paths use the same `Vertex`
  (colour, normal, AO, surface character) and `triangle.frag`, so a
  `ModelInstance` can carry a fragment's mesh. What is new is the projection
  origin from Part 3. Phase 2 has to confirm that the terrain draw's other
  per-draw state (triplanar scale, material texture) is reachable from a
  `ModelInstance` draw.
- **Thin authored geometry.** The sheet-sample rule only applies near the
  blast, but a grenade on a one-voxel deck now drops a piece of it. That is
  probably wanted. A per-material or per-segment opt-out is cheap if it isn't.
  The rule's reach grows with the crater: since R2's fix, the garden's 0.7 m
  lip (one or two samples thick at 0.5 m voxels) comes down in pieces of up to
  22 samples, not whole.
- **Carve cost at fine voxels** and **thin curved authored geometry**: now
  issues R8 and R9 in [ISSUES.md](ISSUES.md).
- **The slivers that this does not remove.** A cusp still joined to bearing
  ground through bearing samples stays. If thin fins still show up, the
  sheet-sample rule can be extended to any axis-thin bearing sample in the
  carve shell.
