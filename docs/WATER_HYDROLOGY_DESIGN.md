# Water: a Hydrology Design

Status: signed off; being implemented stage by stage (§21). Replaces the design in
`WATER_SYSTEM_PLAN.md`.

## 1. The idea in one paragraph

Today's water is a *field*. One 2D heightfield equalises every cell against its neighbours,
a wave grid covers all of it, and a new surface mesh is built every frame. This design
models water as a *network* instead: **stores** that hold volume, joined by **links** that
move it.

- **Still water** (ponds, lakes, the ocean) is a basin store with one level. Nothing is
  simulated per cell.
- **Moving water** is a chain of reach stores along a path found by walking downhill over
  the terrain.
- **Falling water** is not a store at all. It is the geometry of a link whose water leaves
  a lip and lands somewhere below.
- **Sources and sinks** are stores of infinite capacity.

Each frame the solver integrates a few numbers per store. At rest that costs almost nothing:
work is spent only where something changes, such as a front advancing, a lake draining or
terrain being rebuilt.

The meshes stay static between topology changes, and levels reach the GPU as uniforms.
Waves are a separate layer. It is mostly cosmetic, though buoyancy reads it too. It is
analytic everywhere and simulated only in small tiles near whatever disturbed the water.

## 2. Why rewrite

### 2.1 The case is features, not speed

The rewrite is justified by what the current system cannot represent at all:

- pools on floating islands above other water,
- rivers that flow instead of sitting still,
- waterfalls, cliff springs and sky sources,
- water that responds when terrain is destroyed,
- flow without glitches.

Performance is a secondary benefit. §2.3 shows why it could not carry the argument alone.

### 2.2 Structural limits

| Symptom | Root cause |
|---|---|
| No pool on a floating island above other water | `WaterGrid` holds one `(floor, volume)` per XZ column. |
| No waterfalls or springs | Water has no state other than "resting on a floor in this column". |
| Glitchy flow | Flow is explicit diffusion under a stability clamp (`MAX_FLOW_ALPHA`). Shorelines are held together by snapping and hysteresis thresholds (`MIN_VOLUME`, `REWET_VOLUME`) and a min-of-9-samples floor rule. |
| Rivers are expensive | A river is a region that never settles, so it never stops costing. |
| Renderer cost | The water mesh is rebuilt and uploaded every frame, whether or not anything moved (§2.3). |

**What we keep:**

- the buoyancy maths (`buoyancy.rs`),
- the body–wave coupling and its splash and wake events (`coupling.rs`),
- the sleep tracker,
- the water shading: volumetric depth, Fresnel and specular.

### 2.3 Measured baseline

Setup: `render_perf`, release build, 2400×1600, default camera, nothing disturbing the
water. Each level ran twice for 4 s, excluding 0.2 s of warm-up. A range means the two runs
disagreed.

| Level | `water` system mean / p99 | `cpu/water` (mesh build + upload) | Total water CPU | First frame |
|---|---|---|---|---|
| test_arena | 0.20 / 0.25 ms | 0.25 ms | 0.45 ms | 0.3 ms |
| skyway | 0.48–0.63 / 0.64–0.83 ms | 1.5 ms | 2.0–2.2 ms | 7.3 ms |
| subsidence | 0.13 / 0.18 ms | 0.20 ms | 0.33 ms | 1.8 ms |
| thin_ice | 0.25 / 0.30 ms | **4.2 ms** | 4.5 ms | 8.5 ms |
| wrecking_yard | 0.16 / 0.22 ms | 0.97 ms | 1.1 ms | 4.6 ms |

- **Settled water is already cheap to simulate.** The avoidable cost is the renderer
  rebuilding the mesh every frame: up to 4.2 ms on a quiet frame. A profile of thin_ice
  (`sample`) puts about 95% of that stage in `generate_mesh`, which is pure computation.
  - It recomputes each corner's normal from smoothed levels once per quad, and
    neighbouring quads share corners, so each corner's normal is computed up to four times.
  - It allocates about 7.8 MB of `Vec`s every frame.

  The upload itself is about 0.2 ms.
- **No stopgap in the old renderer.** A dirty flag would skip rebuilds on quiet frames, but
  thin_ice's floating ice keeps its waves busy, and thin_ice is the level that would
  motivate one.
- **Measurement note.** One earlier build of `render_perf` reported `cpu/water` on
  thin_ice as 10.6 ms whenever `--worst 300` and `--csv` were both passed. It did so over
  six interleaved runs, with every other stage unchanged.
  - It did not reproduce on a later build, with any flag combination or path length. The
    numbers above match the later build.
  - The cause is unknown. It was not the upload, which is too small (see the profile
    above).
  - Repeat any surprising figure before trusting it.
- **Blast and transient (stage 0, `water_perf`).** A grenade on the shore of each level's
  water, then 10 s of frames after it. Water CPU per frame is flow + wave + mesh build; the
  terrain's own rebuild is excluded.

  | Subject | Quiet mean / p99 | Blast frame | Transient mean / p99 / max |
  |---|---|---|---|
  | test_arena | 0.48 / 0.58 ms | 3.3 ms | 0.50 / 0.63 / 0.71 ms |
  | skyway | 2.6 / 2.8 ms | 4.5 ms | 2.5 / 3.0 / 3.4 ms |
  | subsidence | 0.37 / 0.48 ms | 2.5 ms | 0.37 / 0.47 / 0.59 ms |
  | thin_ice | 4.5 / 5.0 ms | 5.7 ms | 4.3 / 4.8 / 5.3 ms |
  | wrecking_yard | 1.0 / 1.1 ms | 2.3 ms | 1.0 / 1.06 / 1.2 ms |
  | `water_viewer` breach | 0.22 / 0.24 ms | 3.7 ms | 0.25 / 0.28 / 0.35 ms |

  - The blast frame's extra 1.2–3.4 ms is the flow grid re-querying floors by ray for
    every cell under the changed region.
  - A transient costs no more than a quiet frame. The old flow is too slow to be busy: the
    breached pond loses only 29 m³ in its first minute, and most of that is water snapped
    dry on the hillside, not water arriving anywhere.
- **GPU.** Water is drawn inside the composite pass, which `render_perf` cannot split. At
  the default camera `gpu/composite` (water, fire and overlay) is 0.01–0.27 ms.

## 3. Decisions and invariants

### 3.1 Decisions

1. **Finite lakes, endless sources.** Lakes hold real volume and drain when breached.
   Springs, sky sources and the ocean never run dry.
2. **Hydrology plus local ripples, not shallow-water dynamics.** Levels and discharges
   evolve continuously and fronts advance. There are no surges and no sloshing.
3. **Only terrain shapes water.** Bodies float, sink and are carried by currents. They
   never dam or hold water.
4. **Drama comes from conductance, not time.** A per-level `drain_gain` multiplies the weir
   and orifice coefficients, so a breach releases more water, faster. Hydrology always runs
   at real time.
5. **Minor water can be lost.** Shallow basins and small reaches lose water in proportion
   to the area they wet. This ends trickles and dries puddles without thinning rivers or
   lakes. The loss term is built into every store law from the start, and is off by
   default (§7.7).

### 3.2 Invariant: terrain edits only remove material

At runtime the terrain changes only through `TerrainWorld::detonate`, which carves;
`set_voxel` is used only during generation. The design relies on this:

- Floors only drop, and saddles only drop. A span never loses air.
- The point `(column centre, floor_c + ε)` of every old span is still air after an edit, so
  the span remap (§6.5) is total.
- Fill levels (§7.4) only decrease, so drainage repair can propagate decreases only.

`TerrainChangeHandler` asserts the invariant at column centres. A future "place terrain"
feature will fail loudly there instead of quietly corrupting water.

**Measured (spike 0.5a): marching cubes does not keep it exactly.** Carving only lowers
densities, but when a cell near a crater's rim changes case its triangles connect different
edge vertices, and the surface over a fixed point can rise. Over 240 grenades on the five
water levels, `floor_c` rose by up to 0.3 of a voxel. Separately, a pocket blown under a
ledge takes the floor pieces under its ceiling into its own band, which lifts the upper
span's `floor_min`. The rasteriser therefore *holds* the invariant on the data: a re-paired
span's `floor_c` and `floor_min` never rise above those of the old span it rests on. A
`floor_c` rise beyond half a voxel is still a violation, logged and debug-asserted. None
occurred.

### 3.3 Non-goals

- Pressurised flow and trapped air. Water fills a cave to its ceiling and no further.
- Distributaries and braided rivers. Flow picks one downhill direction per cell.
- Water in dynamic containers (decision 3). Static spawnables don't shape water either;
  `level_check` warns when one crosses a waterline (§18).
- Player-placed water. It could be added later as a transient source.

## 4. Alternatives considered

**Minecraft's cellular automaton.** It is cheap and robust, and its core idea is kept: water
advances a bounded amount per tick instead of flood-filling in one frame. It fits our needs
badly:

- It does not conserve volume, which fails decision 1.
- It quantises levels to eighths of a block.
- It assumes cubic voxels of one size, while our terrain is a smooth marching-cubes surface
  built from segments with different voxel sizes.

**Global shallow-water equations (virtual pipes, GPU).** Good surges, but they allow one
layer per column, their cost scales with area, and they conflict with decision 2.

**Particles (SPH/FLIP).** Far too expensive at landscape scale.

**Hydrological routing: the chosen family.**

- **Priority-Flood** (Barnes, Lehman & Mulla, 2014) finds depressions and flow directions.
- **Flat drainage assignment** (Barnes et al., 2014) resolves ties on flat ground.
- **Fill–Spill–Merge** (Barnes, Callaghan & Wickert, 2020) routes finite water through the
  depression hierarchy.
- **Linear-reservoir channel routing** carries water along rivers.

This design adds three things to them:

- *time*: fronts, weirs and implicit integration,
- *multi-layer terrain*: spans, so islands and overhangs work,
- *incremental repair* after edits, made cheap by §3.2.

## 5. Architecture

### 5.1 Layers

```text
 ┌──────────────────────────────────────────────────────────────────────────┐
 │  Presentation    BasinMesher · ReachMesher · FallMesher · OceanMesher     │
 │                  static meshes; levels, Q and fronts arrive as uniforms   │
 └──────────▲────────────────────────────────────────────▲──────────────────┘
            │                                            │
 ┌──────────┴────────────┐                  ┌────────────┴──────────────────┐
 │  Surface detail       │   height         │  Interaction                  │
 │  Swell (analytic)     │─────────────────▶│  WaterQuery::sample(point)    │
 │  RippleTiles (sparse) │                  │  → buoyancy, currents, player │
 └──────────▲────────────┘                  └────────────▲──────────────────┘
            │ level, region, mask                        │ level, Q, ownership
 ┌──────────┴────────────────────────────────────────────┴──────────────────┐
 │  Network   enum Store: Basin · Reach · Ocean · Sink · Reservoir           │
 │            trait Link: Weir · Orifice · ReachOutflow · FixedRate          │
 │                        (any link may carry a FallPath)                    │
 │                                                                          │
 │   HydrologySolver ── integrates volumes; never changes topology          │
 │   TopologyBuilder ── the only writer of topology; applies TopologyEdits  │
 │   VolumeLedger    ── every volume movement goes through one function     │
 └──────────▲───────────────────────────────────────────────────────────────┘
            │ floors, saddles, fill levels, drainage tree, remaps
 ┌──────────┴───────────────────────────────────────────────────────────────┐
 │  Geometry   SpanGraph · SpanRasteriser · DrainageField                    │
 │             built eagerly at load; columns re-paired on terrain edits     │
 └──────────▲───────────────────────────────────────────────────────────────┘
            │ chunk triangles · changed_regions
 ┌──────────┴──────────┐
 │  TerrainWorld       │
 └─────────────────────┘
```

Each layer talks only to its neighbours:

- The network never touches terrain.
- The renderer never touches spans.
- Physics and gameplay see only `WaterQuery`.

`WaterWorld` is the façade, and the single ECS resource.

### 5.2 One event, end to end: a pond breach

This walkthrough follows a grenade that blows a notch in the rim of a 40 m × 40 m pond.

1. **Terrain.** `TerrainWorld::update` rebuilds the cratered terrain chunks and reports
   their `changed_regions`.
2. **Geometry.** `TerrainChangeHandler`:
   - recomputes the crossing cache for the changed terrain chunk,
   - re-pairs the affected columns into spans,
   - asserts the removal-only invariant at the column centres,
   - returns a `SpanRemap`.

   `DrainageField` then repairs itself: fill levels in and behind the notch drop, and the
   drainage directions there are recomputed.
3. **Topology.** The pond's region and crests were touched, so the `TopologyBuilder` issues
   `Reregion`:
   - The flood runs from the pond's wet spans at its current level and finds a new outlet
     crest at the notch, below the water line.
   - A `Weir` is added from the pond to whatever store the drainage path below the notch
     reaches: a depression downhill, which becomes an empty basin, or the sea.
   - `Reroute` lays reaches along that path. Each starts empty.
4. **Solver** (next tick, and every tick after).
   - The weir's discharge is `drain_gain · C_w · cell · Σ (L − h_i)^1.5` over the notch's
     crest cells: 14 m³/s at first, for a 2 m-deep parabolic notch.
   - The pond's volume and the first reach's storage are solved implicitly together. The
     ledger balances.
   - The reach's front advances at the flow velocity.
5. **Interaction.**
   - Near the notch, `WaterQuery` adds the outlet's potential-flow current, so floating
     bodies drift towards the breach.
   - In the reach, bodies are carried at the reach velocity.
6. **Presentation.**
   - The pond's mesh doesn't change; its falling level arrives as a uniform.
   - `ReachMesher` builds the channel mesh once, and a uniform moves its front.
   - A body splashing into the new stream wakes nothing: reaches spawn foam particles
     instead of ripple tiles.
7. **Afterwards.**
   - At `drain_gain = 1` the head halves in about 4 minutes and falls to 10 cm in about an
     hour.
   - When the weir's flow falls below `Q_retire`, the link closes (§7.3). The reach then
     recedes from its top and retires.
   - The water now downstream stays in its new basin. If the level enables loss, the
     trickle's last reach and any shallow puddles are minor and dry up. Deep basins keep
     their water (§7.7).

### 5.3 Glossary

| Term | Meaning |
|---|---|
| **Column** | A 0.5 m × 0.5 m XZ square of a global water lattice, whose structure is sampled at its centre `(0.25 + 0.5i, 0.25 + 0.5k)`. |
| **Span** | A vertical interval of air in a column, from a floor up to a ceiling (`+∞` under open sky). A column over a floating island above the sea has two spans. |
| **`floor_c`** | The span's floor at the column centre. The structure, the remap and the invariant are all defined here. |
| **`floor_min` / `floor_max`** | The lowest and highest point of the span's floor anywhere in the column's square. Saddles use `floor_min`, so water errs low and never stands in the air. Partial wetting (§7.2) and drowning hysteresis (§8.2) use `floor_max`. |
| **Saddle** | The lowest level at which water passes between two adjacent spans: `max(floor_min_a, floor_min_b)`, provided `min(ceiling_a, ceiling_b)` exceeds it. |
| **Descent** | A step from a span in a region to a neighbour whose `floor_min` lies below the saddle between them. |
| **Crest** | A descent at a basin's boundary, with its saddle height. An *outlet* crest drains away below the saddle; a *child* crest leads into a depression of its own. |
| **Fill level** | For a span, the level still water would reach there at steady state: the minimax path height to the nearest outlet (§7.4). |
| **Store** / **Link** | Anything that holds a volume / anything that moves volume between two stores at a rate. |
| **Basin** | A store of still water with one level. |
| **Reach** | A store holding one 8–16 m stretch of a channel. |
| **Front** / **tail** | The downstream and upstream ends of a reach's wetted length. |
| **Fall path** | The ballistic arc a link's water follows when it leaves a lip. |

## 6. Geometry: the span graph

### 6.1 Why spans

Water rests on floors, and its vertical extent is continuous. Spans store exactly that, and
they can represent islands, caves, overhangs and bridges. They are independent of segment
voxel sizes and frames. They are derived from the mesh the player sees, which avoids the
mismatch in `reference_world_voxel_size_wrong_scale`.

### 6.2 Data

```text
SpanChunk (16 × 16 columns = 8 m × 8 m)
  column_offsets: [u16; 257]            // CSR into spans
  spans:          Vec<Span>             // bottom-to-top within a column
  owner:          Vec<SpanOwner>        // parallel to spans; network state, §6.6
  generation:     u32                   // debug builds: bumped on every rebuild
Span      { floor_c: f32, floor_min: f32, floor_max: f32, ceiling: f32 }   // 16 bytes
SpanRef   { column: IVec2 (global), ordinal: u8, generation: u32 (debug only) }
```

**Neighbours.**

- **Orthogonal neighbours** are implicit: spans in adjacent columns whose intervals overlap
  above the higher floor.
- **Diagonal neighbours** exist only for routing (D8, §7.4). A diagonal step is open only if
  one of the two orthogonal columns between them has a span overlapping both. Water never
  leaks through a corner that 4-connectivity would close.

In debug builds, dereferencing a `SpanRef` whose generation doesn't match its chunk panics.

### 6.3 Building: the rasteriser

`SpanRasteriser` turns terrain triangles into spans.

- **Crossings.** For each terrain chunk, triangles are binned into the columns they cover.
  At each column centre, a `Crossing { y, facing }` is recorded, where `facing` is the sign
  of `normal.y`. Crossings are cached per *(terrain chunk, column)*.
- **Rebuild.** When a terrain chunk changes, only its crossing lists are recomputed. Each
  affected column then re-pairs the cached crossings of *every* terrain chunk and segment
  stacked in it. Pairing needs the whole vertical stack; recomputation only needs the
  changed chunk.
- **Fill rule.** A column centre on an edge shared by two triangles belongs to exactly one
  of them. Every projected triangle is oriented counter-clockwise, and the GPU top-left
  convention is applied to its edge functions.
  - This is the lesson of `reference_mc_lattice_coincidence`, applied to projected edges,
    which cross the MC lattice diagonally.
  - A centre on a *silhouette* edge, shared by an upward- and a downward-facing triangle,
    can produce a zero-height floor/ceiling pair. Such pairs are dropped.
- **Parity check.** Sorted crossings must alternate floor, ceiling, floor, … starting with a
  floor.
  - A column that violates this is repaired with `is_mesh_solid_at` at each ambiguous
    height. This is not `density_at`, which is a voxel lookup and disagrees with the mesh
    exactly where parity breaks.
  - The repair is counted under `Water/Geometry/ParityRepairs`.
  - One open edge anywhere in a segment must never silently invert a column of water.
- **Floor bounds.** Each triangle is clipped to the column square, and the upward-facing
  pieces are split into *floor bands*. A span's band runs up to its own ceiling. It runs
  down to the ceiling of the span below (−∞ for the lowest), or to the top of the highest
  downward-facing piece in the square under the span's floor, whichever is higher. The `y`
  range of the clipped polygon inside a band folds into that span's `floor_min` and
  `floor_max`. At an island rim, floor that lies below the ledge goes to the lower span.
  Floor under a roof that the column's centre never sees belongs to no span. Such floor
  might be a cave reaching into the square's corner, or the undercut rim of a blast. Folded
  in, it would give solid ground a `floor_min` down at the cave's floor, and water would
  flood through the rock. A cave's end wall faces up and down by turns all the way to its
  roof, so a floor piece that does not rise above that roof is dropped.

Ray casting is not used. `mesh_surface_heights_at` drops downward-facing hits, so it cannot
see ceilings. Every ray also allocates an `FxHashSet` in `MeshOctree::ray_cast_all`, the
pattern that cost 88% of `query_region` (`project_narrowphase_query_cost`). Spike 0.5a
still measures both approaches (§21).

### 6.4 Eager build

At load, every span chunk over every active segment is built in parallel. A 128 m × 128 m
level is 65k columns × ~2 spans × 16 bytes, about 2 MB, plus the crossing cache.

With no lazy path:

- floods never suspend,
- fronts never wait for a chunk,
- there is no build queue.

Lazy building would return only with segment streaming.

### 6.5 Rebuilds and the span remap

`changed_regions()` marks affected columns stale, and they are re-paired before the
network's next tick. A blast touches 1–4 span chunks.

A rebuild returns a `SpanRemap`. Each old span maps to the new span containing
`(column centre, old.floor_c + ε)`. That point is air on both sides of the edit (§3.2), so
the remap is total.

Several old spans can map to one new span when the floor between them is blown through.
§8.3 decides what happens to their owners.

### 6.6 Ownership lives with the spans

```rust
pub struct SpanOwner {
    /// Water body whose region contains this span (wet or not; see §7.2 for wetness).
    pub body: Option<WaterBodyId>,
    /// Reach cell whose channel runs over this span.
    pub reach: Option<(ReachId, u16)>,
}
```

Ownership is an attribute of the span, and the remap rewrites it on rebuild.

Network objects hold `SpanRef`s only where the `TopologyBuilder` re-derives them: reach
paths, crests and flood seeds. A holder with a ref into a rebuilt chunk is re-derived
before the next tick, with the bounded exception of §9.2.

Where two cross-sections overlap at a bend, a span belongs to the reach cell with the
nearest centreline point.

`WaterQuery` resolves a point with a fixed precedence:

1. the body, if that body is wet at the span,
2. otherwise a reach cell, if the reach is wetted there,
3. otherwise `None`.

## 7. The network: stores and links

### 7.1 Stores are a closed set; links are the extension point

```rust
/// Where a link attaches to a store. Basins ignore it; a reach has two ends.
pub enum Port { Upstream, Downstream }

pub enum Store {
    Basin(Basin),
    Reach(Reach),
    Ocean(Ocean),
    Sink,
    Reservoir,
}

impl Store {
    /// Water held, in m³. f64: see §10.3.
    pub fn volume(&self) -> f64;
    /// Surface level at `port` implied by `volume`.
    pub fn level_at(&self, volume: f64, port: Port) -> f32;
    /// Water lost to the ground and air at `volume`, in m³/s; zero unless minor (§7.7).
    pub fn loss(&self, volume: f64) -> f64;
}

pub trait Link {
    /// Discharge in m³/s from `up` to `down`. Negative means reversed flow
    /// (only a weir between two basins does this).
    fn discharge(&self, up: StoreView, down: StoreView) -> f64;
    /// d(discharge)/d(v_up) and d(discharge)/d(v_down), for the implicit solve.
    fn jacobian(&self, up: StoreView, down: StoreView) -> (f64, f64);
    /// The arc the water follows if it leaves a lip, for FallMesher and re-tracing.
    fn fall_path(&self) -> Option<&FallPath>;
}

/// A store's volume and level function, seen from one port.
pub struct StoreView<'a> { store: &'a Store, volume: f64, port: Port }
```

**Why this shape.**

- **Links own their parameters.** A `Weir` holds its crest saddles. An `Orifice` holds its
  hole area. A `ReachOutflow` holds its reach's `ReachId`, and rating tables live in an
  arena indexed by `ReachId`, not borrowed from a store. A store answers only for its
  volume, its level at a port and its loss, so no link ever downcasts a store.
- **A fall is not a law.** It is the downstream end of another link (a crest's weir, a
  reach's outflow, a spring's fixed rate) whose `down` store is wherever `FallTracer`
  lands. The link just carries a `FallPath`.
- **`Store` is an enum** because its kinds are closed and stored contiguously.
- **`Link` is a trait** because new physical laws are where extension happens. Each law has
  its own unit tests.

| Store | Volume | Level at a port | Ledger term |
|---|---|---|---|
| `Basin` | finite | `Hypsometry::level(V)` | – |
| `Reach` | finite | Rating-curve depth at that end, plus the bed | – |
| `Ocean` | ∞ | fixed | `ocean_in` / `ocean_out` |
| `Sink` | ∞ | −∞ | `sunk` |
| `Reservoir` | ∞ | n/a | `emitted` |

Finite stores also contribute to `lost` (§7.7). A junction is a store with more than one
inbound link, not a separate kind.

| Link | Law | From → to |
|---|---|---|
| `Weir` | Per-cell sum over the crest saddles, free or submerged, scaled by `drain_gain` (§7.3) | Basin → any store; basin ↔ basin |
| `Orifice` | `drain_gain · C_d · a · √(2gH)`, smooth-min'd with the weir along the hole's perimeter at low head | Basin → lower store, through a floor hole |
| `ReachOutflow` | Inverse rating curve (§7.5) | Reach → next store |
| `FixedRate` | Authored `Q` | Reservoir → first store |

### 7.2 Basins: the flood, the hierarchy and the hypsometry

**The invariant.** A basin is lumped: one level for the whole region. That is exact only if,
for every level `L`, the region's spans with `floor_min < L` form one connected set.
Otherwise the hypsometry puts water behind a ridge it cannot cross.

The flood enforces this with one rule: **every descent ends the region and becomes a
crest**, which is Priority-Flood's own semantics.

```text
flood(seeds, level):
  region ← seeds; crests ← []; frontier ← open neighbours keyed by (saddle, SpanRef)
  cap ← +∞                                                 # set at the first outlet
  while let Some((h, n)) = frontier.pop_min():
      if h ≥ cap: break
      if n.floor_min < h:                                  # a descent
          if fill(n) < h:                                  # drains away: an outlet
              crests.push(Outlet(n, h)); cap ← max(level, h)
          else if pit_behind(n, h) is a pothole (§7.4):
              absorb it: its spans join the region with floor raised to h, and its
              volume below h becomes dead storage that fills at h
          else:                                            # a depression of its own
              crests.push(Child(n, h))
          continue
      region.insert(n); push n's open neighbours           # ceiling test inside `open`
  return region, crests
```

The outlet test comes before the pothole test, so a small drop just past a notch that
really drains away down the hill is an outlet, not a pothole.

**`cap`** is `max(current level, lowest outlet saddle)`:

- A newly born basin, whose level is its floor, floods up to its spill.
- A basin standing above its spill after a breach floods up to its current level.

**Child crests** lead into depressions of their own. One rule covers every case: **a child
crest links to the store that owns the far span, through `SpanOwner`.**

- **Owned, at the same level** (authored pools, re-floods, a crater in a lake floor): the
  two merge at once, and the ridge is recorded as a **merge saddle**.
- **Owned, at a different level** (a lower pond beyond a ridge, or a basin met during a
  `Reregion`): a `Weir` links them, and they merge only once their levels meet.
- **Unowned and dry:** a new basin is flooded there, behind a `Weir`. This basin spills into
  it until it fills to the ridge, and then they merge. This is Fill–Spill–Merge.

**Crest grouping.** A flood can reach one pit's rim from several directions, and every
such crest must link to the same child. Crest cells are grouped by the store they reach.
For an unowned pit, the first crest floods the child, and any later crest whose far span
falls in that child's region joins the same group. Grouping by drainage direction would
not work: inside a depression, directions point *out* through its spill, not down to its
bottom.

Creating an authored pool therefore builds its hierarchy recursively, with every internal
ridge recorded as a merge saddle. When the lake later drains below a ridge, it splits there
and each part keeps its water.

**Hypsometry, with partial wetting.** Floor heights are assumed evenly spread between
`floor_min` and `floor_max`, so each span wets gradually as `L` rises through that range:

```text
area_i(L) = cell_area · clamp((L − floor_min_i) / (floor_max_i − floor_min_i), 0, 1)
            (0 once L > ceiling_i)
area(L)   = Σ area_i(L)
volume(L) = ∫ area + Σ_potholes V_dead_p · smoothstep(h_p − δ, h_p + δ, L)
```

- **Volume.** `volume(L)` is piecewise quadratic, with breakpoints at every span's
  `floor_min`, `floor_max` and `ceiling`. It is stored as prefix sums of counts and slopes.
- **Level.** `level(V)` is a binary search plus one quadratic solve.
- **Smoothness.** The law is C¹, so Newton sees no kinks.
- **Absorbed potholes.** Each adds its dead volume as a smoothed step at its own saddle
  `h_p` (δ = 2 cm), which is when it actually fills. So a filling staircase pool never
  pauses before it rises.
- **Capacity.** `area(L)` reaches zero only when every column of the region is capped by a
  ceiling, i.e. the pocket is sealed. A hole in a cave roof is not a cap, because the shaft
  is one span: a filling cave rises up the hole and floods the ground above. A sealed store
  at capacity refuses inflow, and its inbound links clamp to zero. `level_check` warns
  about any source inside a sealed pocket (§18).

### 7.3 The weir: saddles, gain and closure

```text
Q_free(L) = drain_gain · C_w · cell_size · Σ_crest max(L − h_i, 0)^1.5     C_w ≈ 1.7 m^0.5/s
```

`h_i` is each crest cell's **saddle**, the lip itself. Summing over the crest's real profile
makes a V-shaped notch behave like a V-notch weir (`∝ H^2.5`) without any special case.

Crest cells are 4-connected, so a crest running at 45° has √2 × as many cells as its length
implies. That overstates discharge by up to 41%, which is inside `C_w`'s own uncertainty.
If play-testing shows it, weight each cell by its edge projected on the crest tangent.

**Submerged weirs.** Between two basins, with heads `H_up` and `H_down` above each saddle:

```text
Q = Q_free(H_up) · f(H_down / H_up)
f(r) = (1 − r^1.5)^0.385       for r ≤ 0.98
f(r) linear to 0 at r = 1      beyond
```

The linear tail gives Villemonte's correction a finite derivative.

**`drain_gain`** (per level, default 1) scales `C_w` and `C_d`. It is the only dial on how
dramatic a breach is:

- A lake drains `k×` faster by releasing `k×` the discharge, so the river it releases is
  visibly bigger and faster.
- Fronts still travel at their true velocity, and nothing else needs to know about it.
- Every drain time below divides by `k` exactly.

**Closure.** Flow over a lip decays algebraically, never to exactly zero. Without a
threshold, every drained lake feeds a trickle forever. So a weir or orifice whose
discharge falls below `Q_retire` (default 2 × 10⁻³ m³/s) is **closed** by the
`TopologyBuilder`, and reopens only when `Q_free > 2 · Q_retire`.

- A closed link carries exactly zero.
- The water above the lip stays in the basin, so the ledger is untouched.
- The reach below sees `Q_in = 0`, recedes and retires.

A residual head (a weir that returns zero below 1–2 cm of head) would not do this job. It
moves the asymptote without ending the tail: with a 2 cm residual head, the worked lake
below still puts out 0.049 m³/s after an hour, exactly as without one.

**Worked example.** A 40 m × 40 m lake is breached by a 5 m-wide, 2 m-deep, roughly
parabolic grenade notch. Its discharge is close to `Q ≈ 3.55 · drain_gain · H²`.

| `drain_gain` | Initial Q | Half head | 25 cm head | 10 cm head | Link closes |
|---|---|---|---|---|---|
| 1 | 14 m³/s | 3.8 min | 26 min | 71 min | 5.2 h (2.4 cm head) |
| 4 | 57 m³/s | 56 s | 7 min | 18 min | 2.6 h (1.2 cm head) |

The long tail is physics, and it costs little: one flowing reach with a static mesh and a
tiny depth. Loss (§7.7) ends it sooner on any level that enables it.

### 7.4 Routing: the drainage field

`DrainageField` is one data product over the whole span graph. The router and the flood
share it.

**Fill level.** For each span, the lowest achievable maximum `floor_min` along any open path
to an outlet. The outlets are:

- ocean spans, at sea level,
- open world edges, at their floor,
- authored sinks.

It is computed serially at load by Priority-Flood, in `O(n log n)`: about 10–20 ms for 130k
spans.

**Drainage direction.** For each span, the neighbour it was reached through, using D8 over
the filled surface. Flats are resolved by flat drainage assignment, which drains towards a
flat's lower edge and away from its higher one. Ties break on `SpanRef` order, never hash
order.

**Repair.** Under §3.2 an edit can only lower fill levels.

- A decrease-only Dijkstra, seeded from the edited columns, relaxes a neighbour only while
  its fill level actually drops.
- Flat resolution is global over each flat: a lowered edge changes directions across the
  whole flat, including where fill did not change. Repair therefore re-runs it for every
  flat the Dijkstra touched.

**Channels.** A channel follows drainage directions until it enters a real depression. It
ends there in a basin, which is created empty if none exists. That basin's outlet crest
starts the next channel.

**Potholes are decided by geometry, not discharge.** A depression at most `pothole_depth`
deep (default 0.3 m) **and** holding at most `pothole_volume` (default 1 m³) is absorbed.
It becomes dead storage, in a reach (§7.5) or a basin (§7.2), and its water stays in the
ledger. The rule depends only on terrain, so a draining lake upstream cannot flip a pit
between pothole and basin.

**Centreline.** The D8 path is smoothed by two passes of Chaikin's algorithm, and
cross-sections are cut perpendicular to the smoothed tangent. A diagonal river is then
neither blocky nor √2 too wide.

**A channel ends:**

- at a real depression,
- at the first span whose `floor_min` lies below the level of the body that owns it (the
  rest of the channel is drowned, §8.2). Its last link has a lip like any other (§7.9):
  whether a sheet is drawn there follows from the heights, not from the walk,
- at a fall step (§7.6),
- where it joins an existing reach, forming a junction,
- at the ocean or a sink.

### 7.5 Reaches: rating curves, fronts and tails

The path is cut into reaches of 8–16 m, with extra cuts at junctions, falls and depression
entries.

**Rating curve.** At routing time, `CrossSection` solves Manning's equation at each path
cell for 8 log-spaced discharges between `Q_min` and `Q_design`. The scan runs across the
smoothed tangent, and gives a table `Q → (area, top width, depth, velocity)` held in the
rating arena.

- Example: 2 m³/s in a 4 m bed at a 1% slope, `n = 0.035`, runs 0.37 m deep at 1.33 m/s.
- The reach's mean area per unit length, `Ā(Q)`, is tabulated alongside.
- Below `Q_min`, the curve extrapolates as a power law (`depth ∝ Q^0.6`).
- Above `Q_design`, the reach is re-scanned and `ReachMesher` rebuilds (§15).
- Otherwise a change in `Q` is only a lookup.

**State.** A reach holds `S` m³, plus two presentation positions: the front `x_f` and the
tail `x_t`. Its wetted length is `ℓ = x_f − x_t`.

- **Advancing** (`x_f < L_r`). The front sits where storage fills the channel:
  `S = Ā(Q_in)·ℓ + S_dead(x_t, x_f)`. `S_dead` is the cumulative pothole storage between
  tail and front, so the front pauses at each pothole instead of waiting for all of them.
  - The front never retreats: `x_f = max(x_f_prev, solution)`. A rising `Q_in` fattens the
    wet section instead of pulling the front back.
  - Nothing flows out yet: `Q_out = 0`.
- **Flowing** (`x_f = L_r`). `Q_out = Ā⁻¹((S − S_dead) / ℓ)`, continuous with advancing:
  `Q_out = Q_in` at the switch.
- **Receding** (`Q_in = 0`, which a closed link guarantees exactly).
  - The tail moves downstream at `v(Q_out)`, and `Q_out` keeps the flowing law over the
    shrinking `ℓ`.
  - At constant depth this loses `Ā·v = Q_out` per second, so mass balances.
  - The channel dries from the top and retires after about `L_r / v`.
- **Resumed** (`Q_in > 0` again while receding). `x_t` snaps back to 0. The state follows
  from `x_f`: flowing if the front had reached the end, advancing otherwise. The remnant's
  water is already in `S`.

The implicit solve (§10.2) treats `x_f` and `x_t` as fixed within a tick and updates them
after it. `Q_out` is monotone in `S` at fixed `ℓ`, so the step stays unconditionally
stable. `S`, not `x_f`, `x_t` or `ℓ`, is what the ledger sees.

### 7.6 Falls and sources

**Fall steps.** A step in a channel's bed that drops more than `fall_threshold` (0.75 m)
within a metre, at a cliff lip or an island rim, is a fall step: the walk cuts the channel
there, because water that cannot follow the bed is no longer a reach. That is the
threshold's only job. It does not decide where arcs are or whether a sheet is drawn:
every link has a lip, an arc where its jet separates, and heights read each frame (§7.9).

`FallTracer` sweeps a parabola against terrain in short segments. It launches with the
upstream reach's velocity, or a source's authored direction. It lands on **ground**, not
on water, so one arc serves every level the water below may stand at. The outbound link's
`down` store is chosen by geometry alone (§7.9), and the link keeps the arc as its
`FallPath`.

For example, a 10 m drop at 1.4 m/s lands 2 m out after 1.4 s.

A path is re-traced, in place, only when an edit touches its arc above the water it lands in (§7.9).

- **`Spring`**: a `Reservoir` with a `FixedRate` link, launched along a `FallPath` from a
  point with a direction. It is anchored in space, so blasting the rock around it changes
  where the water lands, not where it comes from.
- **`SkySource`**: a spring with zero launch velocity.
- **`Sink`**: an authored column range that absorbs any span whose floor lies within its
  `y` range. Open world edges are implicit sinks.
- **Staircases** are nothing special. Each step is a small basin, each lip an outlet crest,
  and each riser a weir carrying a `FallPath`. A 1.5 m × 1.5 m step carrying 1.5 m³/s has a
  time constant of about 0.7 s, which §10.2 makes harmless.

### 7.7 Loss: minor water only

Loss exists to clear up leftover water:

- trickles that should end,
- puddles on a drained lake bed that should dry,
- sheets on dead-flat ground that should stop spreading.

It must not thin rivers or lower lakes.

A single rate over all wetted area cannot do both. Water loses in proportion to the area it
wets, so a rate that ends a 0.01 m³/s trickle within about 70 m (1000 mm/h) would also take
1.1 m³/s per km from a 4 m-wide river. Loss therefore applies only to **minor** stores:

```text
basin:  Q_loss = loss_rate · area(L)                if minor, else 0
reach:  Q_loss = loss_rate · top_width(Q) · ℓ       if minor, else 0
```

**Minor** is a flag on each finite store:

- A **basin** is minor while its deepest point, `L − min floor_min`, is below `minor_depth`
  (default 0.1 m).
- A **reach** is minor while its `Q_in` is below `minor_discharge` (default 0.02 m³/s).
- A store leaves minor at 1.5 × the threshold, so the flag has hysteresis.

The `TopologyBuilder` sets the flag between ticks (`SetMinor`), in the same way it closes
links. Within a tick the flag is fixed, and loss is `rate × wetted area`, which increases
with the store's own volume. The implicit solve therefore stays unconditionally stable
(§10.2). A gate evaluated per cell, by depth, inside the law would break that: loss would
fall as volume rises.

**Configuration.**

- `loss_rate` is authored per level in mm/h, and defaults to 0.
- `minor_depth` and `minor_discharge` are per-level overrides.
- Loss enters the ledger's `lost` term. The guard `(Q_out + Q_loss) · dt ≤ V` covers it.

**What it does, when enabled:**

- **Trickles end.** An advancing minor reach stalls where its loss equals its inflow, the
  way Minecraft's water stops after seven blocks.
- **Puddles dry.** A basin left on a drained lake bed, or a pond drained to a few
  centimetres, is minor. It loses volume until it reaches the `Dried` transition.
- **Sheets stay bounded.** A spring on dead-flat ground makes a shallow, minor basin. That
  basin stops growing when its loss equals the inflow.
- **Rivers and lakes are untouched.** Anything deeper or larger than the thresholds never
  loses, so `loss_rate` is a pure gameplay dial, not a soil model.

| `loss_rate` | 5 cm puddle dries in | 0.01 m³/s trickle (0.5 m wide) ends after | Sheet from a 0.1 m³/s spring stops at | Rivers, full lakes |
|---|---|---|---|---|
| 100 mm/h | 30 min | 720 m | 3600 m² | untouched |
| 500 mm/h | 6 min | 144 m | 720 m² | untouched |
| 1000 mm/h | 3 min | 72 m | 360 m² | untouched |

**The one blur is a wide, shallow stream.** 0.2 m³/s in a 4 m bed runs about 9 cm deep,
but it carries ten times `minor_discharge`, so it keeps flowing. Minor status for reaches is
decided by discharge, not depth, for exactly this reason.

### 7.8 Currents in still water

A basin's bulk velocity is zero, but a crate should still drift towards the outlet. Each
crest or inlet adds an analytic potential-flow term within `R = 3 × crest width`:

```text
v = Q / (π · r · depth) · r̂ · fade(r / R)
```

- `r̂` points towards an outlet crest, and away from an inlet.
- `fade` is 1 out to `2R/3`, then falls linearly to 0 at `R`.
- The term is stateless. `WaterQuery` adds it to any reach velocity, capped at the reach
  velocity at the crest.

### 7.9 Interfaces: where stores meet

Every link joins two stores, and where they meet the water surface has a height on each
side. Getting that height step right (a fall where the water leaves the ground, surfaces
that meet where it does not) is one rule for every link, not a case per link type or per
way the topology was built.

```text
      upper ─────────┐                         a step: upper − max(lower, lip) > ε
                     │╲  sheet, drawn from       the sheet is drawn from upper at
      lip  ▓▓▓▓▓▓▓▓▓▓│ ╲ upper down to lower     the lip down to lower
                ▓▓▓▓▓│  ╲
                ▓▓▓▓▓│~~~~~ lower               free (lower < lip): the nappe is
                ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓                  aerated, white as it falls

      upper ──────╮                             submerged (lower ≥ lip): the sheet
      lip ▓▓▓▓▓▓▓▓╰───── lower                  is a short, clear drop from upper
                ▓▓▓▓▓▓▓▓▓▓▓▓▓                   to lower, gone once they meet
```

**Direction is the flow's, not the link's.** A reversible weir between two basins is laid
once, `up → down` by whichever basin found it first; the sea pouring into a lowland runs a
lowland → ocean weir backwards. Every rule below reads the two sides by the sign of `Q`:
`upper` is the side water leaves, `lower` the side it enters. A reversible link has a lip
on each side of its crest (the same crest cell, facing each way), each with its own arc,
both traced when the link is made.

**The lip.** The point on the floor where water leaves the upper store, and the horizontal
direction it leaves in. It is fixed when the link is made; its position and direction are
kept only so its arc can be traced and re-traced.

| Link | Lip | Direction |
|---|---|---|
| `Weir` | The crest's lowest cell, at its saddle, on the column edge | Inside column centre to outside |
| `Orifice` | The middle of the hole's cells, at the hole's lip | Straight down |
| `ReachOutflow` | The end of the reach's last column, at its bed | The centreline's direction there |
| `FixedRate` | The source's position | Its authored direction |

```rust
/// Where water leaves the upper store of a link.
pub struct Lip {
    /// On the floor where the water leaves.
    pub at: Point3<f32>,
    /// Unit horizontal direction it leaves in; zero for straight down.
    pub direction: Vector2<f32>,
}
```

`LinkEntry` holds `lip: Lip` (two for a reversible link) beside `fall: Option<FallPath>`.
A reach's drawn surface runs to its outflow's lip, not to the middle of its last column.

**An arc where the jet separates.** A link gets an arc when the ground within `FALL_RUN`
past its lip lies more than the jet's thickness below the lip: the critical depth
`d_c = (q²/g)^⅓` at its design discharge per metre of lip. Water thinner than the drop
cannot follow the ground; water thicker than it runs down it as a steep reach. This is
physical, has no tuning constant, and keeps sliver arcs off steep channels. A hole
(straight down) and a source always have one.

**Three heights.** For any link, arc or not, as a pure function of the network
(`fn interface(&LinkEntry, &Network) -> Interface`, computed where needed, not stored):

| Height | From |
|---|---|
| `upper` | The side water leaves, `Store::level_at(port)`: a basin's level; a reach's surface at its end before easing; a source's position |
| `lip` | `lip.at.y` on that side |
| `lower` | `max(level, landing floor)` of the side it enters: a basin's level, a reach's un-eased surface at the landing's distance along it, sea level; for a sink or the void, the arc's end |

Neither end reads the other's easing (below), so no port's height is an input to its own.

- **The sheet** is drawn wherever the link has an arc and `upper − max(lower, lip) > ε`
  (1 cm): from `upper` at the lip, along the arc, cut off at `lower`. The arc was traced from
  the surface at the discharge the link was laid for, so it is shifted by the change in
  `upper` since, and its top always meets the water it leaves. A drop of any size draws.
- **The regime**, free while `lower < lip`, submerged once `lower ≥ lip`, sets only the
  sheet's look: aerated and white when free, a clear drop when submerged. At submergence the
  sheet's length is `upper − lower`, which shrinks to zero as the two meet, so nothing pops.
- A submerged weir between two basins, still passing flow, draws its step the same way.

The weir's own law already has the free/submerged split (§7.3); this is its geometric twin.
None of it is topology: nothing is re-laid when a height moves.

**A reach's surface meets its ports.** A reach's normal surface is its bed plus its rating
depth. `Reach::surface_at(distance, ends)` eases it, smoothstep over `EASE_LENGTH`
(`max(2 m, 4 × depth)`) of channel, to a height at each end:

- **Downstream**, towards `max(lower, lip + d_brink)`, with `d_brink ≈ 0.715 d_c` the depth
  at a free brink. A lake above the lip draws the end down or backs it up to the lake; a lake
  below it leaves the end at the brink, where the sheet starts. The target is continuous
  through the regime change, so the surface never jumps.
- **Upstream**, from a basin, towards the basin's level, so a river leaves its lake without a
  step.

Two reaches that meet without a sheet between both ease to the same height at their
joint, the one the upper leaves at, so the channel is continuous there however short a
cut leaves either. Each end's ease is clamped to half its reach, so the two never
overlap. `surface_at` is the one definition: `WaterQuery` samples it for
buoyancy, the fall's launch and landing read it, and `river.vert` mirrors it from the
reach's port targets pushed per draw (`pc.tile.yzw` is free). A test holds the shader to it
as it does for swell. The mesh does not change.

**Where an arc lands.** `FallTracer` sweeps to **ground**, not to water, so one arc serves
every level the water below may stand at, and the renderer cuts it at the water. The
link's `down` store is the first whose water stands in the arc's way now; if none does, a
channel is laid on from where it meets the ground. A weir's target is still chosen from
its crest's far side, which is what its law is about.

The first design chose the store by `cap` instead, so that it would not change with
levels. That made the whole valley below a closed pit's brim the pit's: a spring landing
anywhere in it was the pit's water at once, and no river was laid down to the lake. Water
landing on a basin's dry bed is not yet the basin's; it runs down as a channel, and a
rising lake drowns that channel (§8.2). A lake that falls away from under an arc it
catches leaves the arc landing on its dry bed; the water is still the lake's, which its
region holds, and no channel is drawn down that bed. That is deferred.

**Edits and arcs.** An edit touches an arc only where it re-pairs a column the arc crosses
**above the surface of the water it enters**. Below that the arc runs through water, and a blast in a
plunge pool is not a blast in the air. A touched arc is re-traced in place from its lip; the
link is remade only if its landing store changed, and the channels above and below keep
their reaches and storage.

**What this replaces.** Each of these decided a fall once, when the topology was built,
with its own rule:

- a spill straight into water well below a crest (`spill_fall`),
- a hole's fall kept only if it drops more than `fall_threshold` (`hole_fall`),
- a channel's walk ending in a basin more than `fall_threshold` below its last cell,
- a fall on a reversible weir that kept drawing once the lower basin filled, and none at all
  when the flow ran backwards.

All become a lip, an arc where the jet separates, and the heights each frame.

## 8. Topology and its state tables

The `TopologyBuilder` is the only component that changes the network. It runs between
solver ticks and applies `TopologyEdit`s:

```rust
pub enum TopologyEdit {
    AddStore(StoreSpec),
    RemoveStore { store: StoreId, residual_to: StoreId },
    AddLink(LinkSpec),
    RemoveLink(LinkId),
    SetLinkOpen { link: LinkId, open: bool },
    SetMinor { store: StoreId, minor: bool },           // loss gate, §7.7
    Transfer { from: StoreId, to: StoreId, volume: f64 },
    Reregion { basin: BasinId, seeds: Vec<SpanRef> },   // re-flood, keep volume
    Reroute  { from: SpanRef },
}
```

Every volume movement at a topology change is a `Transfer` through
`VolumeLedger::transfer(from, to, v)`. That is the one place where a topology change can
break conservation.

### 8.1 Basin

| State | Meaning | Solver behaviour |
|---|---|---|
| **Filling** | `L <` lowest outlet saddle | Weir returns 0; an orifice may still drain it |
| **Spilling** | `L ≥` lowest outlet saddle, link open | Weir active |
| **At capacity** | Sealed and full | Inbound links clamp to 0 |

| Transition | Trigger | Edits | Ledger |
|---|---|---|---|
| Created | An authored pool, or a channel entering a dry real depression | Recursive flood (§7.2); owned children at the same level merge at once and record their saddles | — |
| Linked to a child | A child crest's far span is owned by another store, or `L` reaches a dry child's crest | `AddLink(Weir)`, plus `AddStore(child, V = 0)` if it was unowned | — |
| Filling ↔ Spilling | `L` crosses the lowest outlet saddle | None: the weir is always linked | — |
| Link closed / reopened | `Q < Q_retire` / `Q_free > 2 · Q_retire` (§7.3) | `SetLinkOpen` | — |
| Becomes / stops being minor | Deepest point below `minor_depth` / above 1.5 × `minor_depth` | `SetMinor` | — |
| Merge | Two basins share a crest, both stand above it, and `\|L_a − L_b\| < 5 mm` | `AddStore(union)`, `Transfer a→new`, `Transfer b→new`, record the merge saddle | 2 transfers |
| Split | `L < merge saddle − 1 cm` | Multi-seed flood from each side's wet spans; each child gets `child.volume(L)`, and the f64 remainder goes to the larger child | 2 transfers |
| Re-region | An edit touches the region, its crests, or the ring around them | `Reregion`: seeds are the surviving wet spans, flooded at the current level | — (volume kept) |
| Floor holed | The remap merges region spans into a lower store's span (§8.3) | The spans leave the region; an `Orifice` to the landing store is added | — |
| Crest newly drains elsewhere | After an edit, a crest reaches a store it was not linked to | `AddLink(Weir)`. §9.1 step 5 is this event seen from the ocean. | — |
| Dried | `V < 1e-3 m³` and no inflow | `RemoveStore { residual_to: outlet target, or the ledger's lost }` | 1 transfer |

**Merge and split.**

- **Hysteresis.** The 1 cm gap between merging and splitting stops a lake that rests
  exactly at its merge saddle, as a lake at steady state does, from flickering between one
  body and two.
- **Merging is about identity, not stability.** One body, one uniform, one ripple key. The
  implicit solve keeps a pair of basins stable without it.
- **Split cascades.** If spike 0.5b finds that drained lake beds shed many shallow children,
  children shallower than a threshold are dissolved at split time: their water is
  `Transfer`red to the sibling. Loss (§7.7) dries whatever remains.

**The ocean never re-regions.** Re-flooding it would touch 50–100k spans. It grows only
through §9.1 step 5, plus a mask update.

Only merge saddles are stored as hierarchy. A split recomputes its children's regions, so
an edit inside a merged basin cannot leave the hierarchy stale.

### 8.2 Reach

| State | Meaning |
|---|---|
| **Advancing** | `x_f < L_r`, `Q_out = 0` |
| **Flowing** | Fully wetted, `Q_out` from the rating curve |
| **Receding** | `Q_in = 0`, tail moving downstream |

| Transition | Trigger | Edits | Ledger |
|---|---|---|---|
| Created | Route or re-route | `AddStore(S = 0)`, `AddLink` | — |
| Advancing → Flowing → Receding | The law of §7.5 | None | — |
| Resumed | `Q_in > 0` while Receding | None: `x_t ← 0`, and the state follows from `x_f` (§7.5) | — |
| Becomes / stops being minor | `Q_in` below `minor_discharge` / above 1.5 × `minor_discharge` | `SetMinor` | — |
| Retired | Receding and (`ℓ ≤ 0`, or mean depth < 5 mm, or `Q_out < Q_retire`) | `RemoveStore { residual_to: downstream }`; pothole dead storage goes with the residual | 1 transfer |
| Drowned | The receiving basin's `L > floor_max + 0.05 m` over the reach's last cells | Truncate the reach, or remove it; `Transfer` the drowned storage to the basin | 1 transfer |
| Exposed | The receiving basin's `L < floor_min` at its shoreline cell | `Reroute` from the old end; the new reach starts empty, because its water is already in the basin's hypsometry | — |
| Re-routed | An edit crosses the path | Storage in blasted cells is `Transfer`red to the store the remap assigns; intact old cells keep receding; the new path starts empty | 1 per orphaned group |

Drowning is judged on `floor_max` and exposure on `floor_min`, so a level hovering at a
shoreline cannot make a reach flicker in and out of existence.

**Drowned and Exposed move one boundary, not a channel.**

- **Drowned** is judged per reach cell against whichever body owns its span, not only the
  basin the channel feeds: a lake that rises over the middle of a channel running past it,
  or a basin an edit opens under one, drowns those cells too. The reach is cut at the
  drowned run. The part above ends in that body (its outflow gets a new lip and, if the jet
  separates there, an arc); the part below, if any, recedes as a channel with no inflow.
  - The water moved is what the drowned length holds, not a share by length: the rating
    area integrated over `[max(x_t, cut), x_f]` at the reach's `Q_out`, plus the pothole
    storage in that range. Mouth sections are the widest, so a share by length moves too
    little. `Q_out` stays continuous across the cut, and `x_f` is clamped to the new length.
  - The kept part is rebuilt from its cells (`build_reaches`): its `Ā`, length, claimed
    spans and mesh.
  - A reach drowned whole is removed with a `Transfer` of its storage, and every reach feeding
    it (each branch at a junction) is tested next.
- **Exposed** walks on from the channel's last cell, or from a fall's landing, over the cells
  the basin has let go, as a channel from there, and appends what it lays. The new reaches
  start empty: their water is in the basin's hypsometry. The walk may end over a fall, into
  the basin again further out, or anywhere else a channel ends.
- Reaches upstream of the boundary keep their state.

**Why this cannot loop.** Drowned only ever moves water into the body that drowned the cells,
so it only raises that body's level, which can only drown more: a cascade, but monotone and
bounded by the channel's length. Exposed moves no water, so it cannot raise or lower anything.
Neither direction feeds the other. The `floor_max + 5 cm` / `floor_min` band stops a level
hovering at a shoreline from flickering; it is not what stops the loop. On a small tread
(about 2 m²) one cut can raise the level by decimetres and drown the next cells too; the
steady settle test bounds the sweeps.

A rising lake meets a fall's lip first through the heights (§7.9): the sheet shortens to
nothing. Only once the lake covers the channel's last cells does Drowned cut it.

### 8.3 Span-remap conflicts

When several old spans map into one new span, the new span goes to the owner of the
**lowest** old span, the one it still rests on. The upper store loses those spans and gets
a `Floor holed` edit.

In the island-pool scenario, this means the pool keeps its water, and an orifice pours it
through the hole onto the sea.

## 9. Terrain destruction

### 9.1 The handler

`TerrainChangeHandler` runs after `TerrainWorld::update` and before the solver.

1. **Re-pair** the stale columns from the crossing cache, and collect their remaps (§6.3,
   §6.5).
2. **Assert** the removal-only invariant at column centres (§3.2).
3. **Repair the drainage field:** decrease-only fill, plus flat re-resolution (§7.4).
4. **Emit edits:**
   - `Reregion` for touched basins, except the ocean (§8.1),
   - `Reroute` for touched reaches,
   - `Floor holed`, and weirs to newly reached stores,
   - a re-trace of every `FallPath` whose arc was touched.
5. **Handle newly connected below-sea-level spans.** They become a new basin at their
   current water, or empty if dry, joined to the ocean by a `Weir`. The lowland floods at
   the weir's rate and merges into the ocean under the normal rule. The ocean never claims
   spans instantly. The new basin is laid over the pit found by walking down through spans
   nobody owns. The steepest way down from a breach's mouth leads out over the lip into
   the sea, not into the lowland behind it.

### 9.2 Worst case, and the deferral valve

Most blasts fit the 2 ms water budget (§19). The worst cases may not:

- a breach whose drainage repair spans a whole lowland,
- a re-region of the largest lake, including its mesh rebuild.

`water_perf` measures both first, on the largest basin in any level (stage 0).

Only if they are over budget, a `Reregion` or `Reroute` may lag by up to N frames:

- The holder is marked **frozen**. No links run through it, its volume stays still, and
  the ledger is untouched.
- A frozen holder is the one permitted exception to "stale refs never survive a frame".
  The debug generation check (§6.2) skips it.

### 9.3 Acceptance scenarios

These are the stage 6 tests. Each asserts the ledger and the stated outcome.

1. **A pond's wall is breached.** The pond drains towards the notch saddle at the per-cell
   weir rate, and a channel advances from the breach. When the flow falls below
   `Q_retire`, the link closes and the channel retires.
2. **An island pool's floor is blown out.** The pool drains through an orifice, its water
   following a `FallPath` onto the sea. The sea's region is unchanged.
3. **A river is diverted into a crater.** The crater fills as a new basin and spills, and
   the river rejoins downstream. The abandoned reach recedes from its tail and retires.
4. **A sea wall is breached.** The lowland floods through a weir, merges with the ocean at
   sea level, and joins the ocean mask.
5. **The rock around a spring is destroyed.** The spring keeps flowing, and its
   `FallPath` is re-traced.
6. **A crater is blown in a lake floor.** The crater becomes an owned child, merges at once
   and records its rim. The level drops by `ΔV / area`, and nothing flows. The lake is then
   drained below the crater's rim: the crater splits off and keeps its water.

## 10. The solver

### 10.1 Tick

The solver runs every frame at a fixed `dt = 1/60 s`, from an accumulator. Integration
costs microseconds, so a slower tick would save nothing. It would also put 2–3 cm level
steps into small pools, and into the bodies floating in them.

```text
HydrologySolver::tick():
  for group in stores in topological order:      # upstream → downstream
      solve group implicitly for V' (§10.2)
      commit link transfers and losses through the ledger
  advance reach fronts and tails (§7.5)
  VolumeLedger::check()
```

- **No time scaling.** Hydrology always runs at real time (decision 4).
- **Fast-forward in the harness.** `water_viewer --fast-forward N` runs N ticks per frame at
  the true `dt`, so a scenario plays out quickly without changing any law.
- **Topology.** The solver never changes it. At steady state, its cost is O(stores + links).

### 10.2 Implicit integration

Each store solves:

```text
V' = V + dt · (Σ Q_in(V'_up) − Σ Q_out(V') − Q_loss(V'))
```

with 2–3 Newton steps, using `Link::jacobian`. Stores upstream are solved first, so their
outflow at `V'` is already known.

- **Coupled groups.** Bidirectional weirs between basins are the only cycles. Each strongly
  connected group is solved jointly: in practice, one 2×2 Newton step.
- **Stability.** Every outflow and loss law increases monotonically with its own store's
  volume, so backward Euler is unconditionally stable. Large `drain_gain` values degrade
  accuracy, as lag, never stability.
- **Guards.** `(Q_out + Q_loss) · dt ≤ V` for every store. Each link's transfer is computed
  once, and debited and credited with the same f64.

### 10.3 Numerics and determinism

- **Precision.** Volumes, storages and the ledger are f64. Levels passed to rendering are
  f32.
- **Ordering.** Stores and links live in `Vec`s indexed by id. Iteration follows id or
  topological order, never `HashMap` order. Priority queues break ties on `SpanRef`.
- **Ledger.**

  ```text
  Σ V_basin + Σ S_reach = V_0 + emitted − sunk − lost + ocean_in − ocean_out − discarded
  ```

  It is checked every tick to 1e-6 relative in debug builds, and printed under
  `Water/Ledger/`. `discarded` is non-zero only in stage 2 (§21), and is asserted zero from
  stage 4a on.

## 11. Surface detail

### 11.1 Swell

Swell is a vertical sum of 4–6 sines per body, with normals from its analytic derivative.

It is not Gerstner. Gerstner waves displace horizontally, so the CPU would have to invert
them to find the height at `(x, z)`. At 0.2–0.5 m amplitude the two look the same, and the
sine sum is identical on CPU and GPU, so buoyancy and visuals share one function.

- **Ocean:** authored amplitude.
- **Basins:** amplitude scales with fetch (√area).
- **Shores:** attenuated by `smoothstep(0, 1 m, depth)`. Depth comes from the per-vertex
  floor in the basin mesh, or from a per-tile floor texture in the fine ripple mesh.
- **Reaches:** no geometric swell. They get flow-mapped normals from per-vertex velocity,
  and foam at high slope or speed.

### 11.2 Ripple tiles

- **Key.** Tiles are world-anchored, 8 m square at 64 × 64 cells (0.125 m), and keyed by
  **`(tile, WaterBodyId)`**. An island pool and the sea beneath it get separate tiles, and
  each tile is masked to its body's columns.
- **Waking.** A tile wakes on coupler impulses, or when energy crosses in from a neighbour.
  It sleeps after 2 s below ε. Unwoken neighbours are absorbing boundaries.
- **Budget.** At most 32 active tiles (about 0.4 ms), ranked by energy and camera distance.
  A fall's landing splash holds its tile awake, so it counts against the budget, and is the
  first evicted by distance.
- **Upload.** Only active tile heights are uploaded: at most 0.5 MB per frame.
- **Merge and split.** Either one changes a tile's key, so its tiles restart calm.
- **Scope.** Basins and the ocean only. Disturbances in reaches spawn foam particles.

## 12. The ocean and open edges

```ron
ocean: Some((level: 0.0, open_edges: All)),          // an island
ocean: Some((level: 0.0, open_edges: [South])),      // a coast
```

**Region.** At load, the ocean floods from its open edges through spans below sea level. It
never re-regions; it grows only through weir-and-merge (§9.1 step 5).

**Drawing.** The ocean is drawn only by `OceanMesher`, never by `BasinMesher`:

- Beyond the level bounds, a camera-centred ring mesh covers the open edges, snapped to its
  own spacing.
- Inside the bounds, a 2D ocean mask decides where it draws. The mask is exact, because a
  column has at most one span that contains sea level.

**Closed edges** need scenery. They act as sinks, and `level_check` warns if steady-state
routing sends water to one.

## 13. Level format

```ron
water: Some((
    ocean: None,
    bodies: [
        // Unchanged. The span is the one containing surface_level in the seed column.
        Pool(seed: (48.0, 32.0), surface_level: 3.0),
        Spring(position: (10.0, 22.0, 5.0), direction: (1.0, 0.0, 0.0), discharge: 1.5),
        SkySource(position: (30.0, 60.0, 30.0), discharge: 0.8),
        Sink(min: (-2.0, -30.0, 62.0), max: (2.0, -20.0, 66.0)),
    ],
    drain_gain: 1.0,     // scales weir and orifice discharge (§7.3)
    loss_rate: 0.0,      // mm/h over the wetted area of minor stores only (§7.7)
    settle: Steady,      // Steady | AsAuthored
)),
```

- **Migration.** `ocean_level: Option<f32>` becomes `ocean: Option<OceanConfig>`. All five
  current water levels author only `Pool`s, and they parse unchanged.
- **Rivers.** A river is a spring plus a carved bed. Routing finds its course; it is not
  authored.
- **`settle: Steady`** opens the level at rest:
  - Channels are routed, and every reach is at steady storage.
  - Each fed basin is at the level where its inflow equals its outflow plus its loss.
  - Basins are solved one at a time in topological order. With no loss, this is
    Fill–Spill–Merge run to completion.
  - Unfed pools keep their authored volume.

## 14. Interaction

```rust
pub struct WaterSample {
    /// Surface height here, including swell and ripples.
    pub surface: f32,
    /// Floor the water rests on (floor_min of the span).
    pub floor: f32,
    /// Bulk velocity: the reach's velocity, or the basin's potential-flow current (§7.8).
    pub velocity: Vector3<f32>,
    /// The body this point belongs to.
    pub body: WaterBodyId,
}

impl WaterQuery<'_> {
    /// Water at a 3D point, or `None` if the point's span holds no water.
    pub fn sample(&self, point: Point3<f32>) -> Option<WaterSample>;
}
```

A lookup goes to the chunk, then the column, then scans its 1–3 spans, then reads
`SpanOwner` with the precedence of §6.6. It is O(1), and cheap enough for every buoyancy
substep.

- `BuoyancyForceProvider` samples through `WaterQuery`, with drag on
  `v_body − sample.velocity`.
- `WaterSleepTracker` keys on `(body, WaterBodyId, surface)`.
- `WaveBodyCoupler` injects into `RippleTiles`. Its events are unchanged.
- Swimming and the player read the same sample.

## 15. Presentation

Water at rest generates no mesh per frame.

| Mesher | Geometry | Rebuilt on |
|---|---|---|
| `BasinMesher` | One quad per region column (ocean excluded), in 8 m tiles, extended one column under terrain. Each vertex carries its floor, for depth tint and swell attenuation. Level, swell and colour come from a per-basin uniform. The depth test makes the shorelines. | Re-region, merge, split. Never on a level change. |
| Fine tile mesh | A static 64 × 64 grid, displaced by the tile's ripple texture and masked to its body. It replaces the coarse tile while the ripple tile is active. | Never |
| `ReachMesher` | One quad per cross-section cell, out to the `Q_design` top width. Each vertex carries its distance along the reach, a reference depth and a velocity. Per-reach uniforms set the depth to `d_ref · (Q/Q_design)^0.6` and discard outside `[x_t, x_f]`, and carry the port levels its ends ease to (§7.9). The depth test trims the width at lower `Q`. | Route, re-route, or `Q_design` growth |
| `FallMesher` | A ribbon along each link's `FallPath`, with width from its `Q`, drawn last in the water pass (the sorted transparency pass is deferred with the water/smoke pass order). Per draw, the link's heights (§7.9) shift it to the upper surface, cut it at the lower one, and set its look by the regime. Mist comes from the particle system. | Re-trace in place |
| `OceanMesher` | Ring mesh plus mask (§12) | Mask on ocean growth |

The shading carries over: volumetric depth, Fresnel, specular, and the depth-read subpass.

## 16. Requirements traced

| # | Requirement | Mechanism |
|---|---|---|
| 1 | Ponds | Basin (§7.2); `Pool` unchanged |
| 2 | Rivers through the landscape | Spring, drainage field, reaches with rating curves, flow-mapped mesh, currents |
| 3 | Ocean, one direction or all | Ocean store, open edges, ring mesh plus mask (§12) |
| 4 | Waterfalls | `FallPath` on a link (§7.6) |
| 5 | Water staircases | Basins, outlet crests and fall paths in a chain (§7.6) |
| 6 | Streams from a cliff face | `Spring` |
| 7 | Streams from a source in the sky | `SkySource` |
| 8 | Responds to terrain destruction | §9, with its six scenarios |
| – | Island pool above other water | Spans, a 3D `WaterQuery`, ripple tiles keyed by body |

## 17. Debug visualisation

`WaterNetworkDebug` writes to `DebugOverlays`, and `level_viewer` draws the same shapes:

- basin ids and levels,
- crests, coloured by kind (outlet or child) and by head,
- merge saddles and closed links,
- reach centrelines coloured by state, with fronts, tails and junctions,
- fall paths and their landing points,
- optionally, drainage directions near the camera.

F3 prints `Water/Ledger/*`, `Water/Stores`, `Water/Links`, `Water/Geometry/ParityRepairs`
and `Water/Time/*`. The ledger says *that* water went missing; the overlays say *where*.

## 18. Validation (`level_check`)

- A `Pool` authored above its lowest outlet saddle: reported, with the crest's location.
- A static spawnable whose AABB crosses a basin's surface: a warning (§3.3).
- A source inside a sealed pocket: reported.
- Steady-state routing that reaches a closed-edge sink: reported.
- Any parity repair (§6.3): reported, with its column.
- The steady-state ledger must balance.
- `--svg` draws spans, basins, crests and routed channels.

## 19. Budget

Targets are for a release build on an M-series machine. `water_perf` fails a run that
misses one. Frame statistics exclude warm-up, and report mean, p99 and max.

| Situation | Target | Baseline (§2.3) |
|---|---|---|
| **Steady state, total water CPU per frame** | **≤ 0.8 ms** | **0.33–4.5 ms** |
| … of which network | ≤ 0.1 ms | 0.13–0.63 ms (flow + wave step) |
| … of which ripple tiles | ≤ 0.4 ms at 32 active tiles | (included in the row above) |
| … of which render CPU | ≤ 0.3 ms | 0.20–4.2 ms |
| Ripple texture upload | ≤ 0.5 MB per frame | whole mesh every frame |
| Blast frame, water share (re-pair, drainage repair, re-region, re-route) | ≤ 2 ms at p99 | 2.3–5.7 ms (§2.3) |
| Worst-case blast (largest basin re-region, whole-lowland repair) | reported; the §9.2 valve only if over | no equivalent; the shore blast above |
| Transient (breach draining), CPU per frame | ≤ 0.5 ms | 0.25–4.3 ms, same as quiet (§2.3) |
| Load with `settle: Steady` | ≤ 200 ms | 1.8–8.5 ms first-frame spike |
| GPU water pass at 2400×1600 | ≤ today's | composite pass 0.01–0.27 ms (§2.3) |

**Where the cost will sit:**

- The solver at steady state: microseconds.
- Re-pairing four chunks' columns from the crossing cache: ≤ 1 ms. This is the risk item,
  measured in spike 0.5a.
- Drainage repair: proportional to the fill levels that drop, usually one depression.
- Re-flooding a 40 m basin (6.4k spans): 0.3–1 ms.
- A change in `Q`: a rating-curve lookup plus a uniform write.

## 20. Module layout

```text
src/water/
  mod.rs                     uses only
  world.rs                   WaterWorld façade, WaterBodyId
  query.rs                   WaterQuery, WaterSample
  geometry/
    span.rs                  Span, SpanRef, SpanChunk, SpanOwner
    span_graph.rs            SpanGraph, neighbours (4 + open diagonals)
    crossings.rs             crossing cache per (terrain chunk, column), fill rule
    rasteriser.rs            SpanRasteriser: pairing, parity repair, floor bands, SpanRemap
    drainage.rs              DrainageField: fill, D8, flat resolution, repair
  network/
    store.rs                 enum Store, Port, StoreView; Basin, Reach, Ocean
    link.rs                  Link trait, FallPath
    links/                   weir.rs, orifice.rs, reach_outflow.rs, fixed_rate.rs
    hypsometry.rs            Hypsometry (partial wetting, pothole steps)
    depression.rs            DepressionFinder (the flood of §7.2, crest grouping)
    rating.rs                CrossSection, RatingCurve, rating arena
    centreline.rs            Chaikin smoothing, tangents
    fall_tracer.rs           FallTracer
    loss.rs                  loss law (§7.7)
    currents.rs              potential-flow terms
  topology/
    builder.rs               TopologyBuilder
    edit.rs                  TopologyEdit
    terrain_change.rs        TerrainChangeHandler
  solver/
    solver.rs                HydrologySolver::tick
    implicit.rs              per-store and per-group Newton
    ledger.rs                VolumeLedger
  surface/
    swell.rs                 Swell (shared with the shader)
    ripple_tiles.rs          RippleTiles (from wave.rs)
  interaction/
    buoyancy.rs · coupling.rs · sleep_tracker.rs     kept, re-plumbed
  debug.rs                   WaterNetworkDebug
src/rendering/water/         BasinMesher, ReachMesher, FallMesher, OceanMesher, pipeline
src/water_viewer/ + src/bin/water_viewer.rs      offline harness
src/bin/water_perf.rs                            budget bench
```

Deleted by the end: `grid.rs`, `placer.rs`, the whole-area `WaveGrid`, and the per-frame mesh
build in `rendering/water/renderer.rs`.

## 21. Delivery plan

Each stage ships a working game.

### Stage 0: harness and baseline

- `water_viewer`, in the mould of `anim_viewer`:
  - scripted scenarios, with `--fast-forward`,
  - `--no-render` reports of levels, discharges, fronts, tails and the ledger,
  - filmstrips through the real meshers.
- `water_perf`, first run against the **old** system:
  - a scripted pond breach (the transient row of §19),
  - a blast on each level's largest basin (the worst-case row).

```bash
cargo run --bin water_viewer -- --list
cargo run --bin water_viewer -- breach --no-render --fast-forward 60
cargo run --bin water_viewer -- island_pool --tiles 8 --from 0 --to 30
cargo run --release --bin water_perf
```

### Stage 0.5: spikes with kill criteria

Nothing is deleted until both spikes pass.

**(a) Span sampling.** The rasteriser against the ray path, on all five water levels.
Ground truth is 8 × 8 sub-samples per column, assigned to spans by the floor-band rule of
§6.3. To pass:

- Re-pairing four chunks' columns takes ≤ 1 ms in parallel.
- `floor_min` is never above the true band minimum, and is within 0.1 m of it in 99% of
  columns.
- The five levels need zero parity repairs. Any repair is investigated, not tolerated.

**(b) Routing and hierarchy.** A spring routed down skyway's carved river bed and down a
synthetic staircase, plus a full drain of each current level's pool. To pass:

- The centreline deviates ≤ 0.5 m laterally from the bed's lowest line.
- Top width varies ≤ 20% from cell to cell.
- There are at most 2 real depressions per 100 m of river.
- Children per lake bed, and splits per full drain, are reported. If they are large, the
  dissolve rule of §8.1 is enabled.

If the routing criteria fail, the fallback is an optional authored centreline hint,
decided before stage 2.

**Results** (`water_spike spans` and `water_spike routing`):

- **(a) Spans.**
  - Zero parity repairs on all five levels. Zero spans with `floor_min` above the sampled
    minimum.
  - `floor_min` within 0.1 m of the 8 × 8 interior samples in only 97.5–99.5% of spans.
    With the square's rim sampled too, 99.86–100%: the misses are cliffs, where the lowest
    floor lies on the rim that interior samples never reach. The rasteriser's `floor_min` is
    the exact minimum of the clipped triangles; the interior grid is the less accurate side.
  - Re-pairing after a grenade: mean 0.46–0.64 ms, max 0.95 ms over 48 grenades per level,
    once each blast re-rasterises only the 8 m tiles its changed region touches. Rasterising
    whole terrain chunks took 3.5 ms.
  - The incremental rebuild matches a full rebuild of the blasted terrain in every column.
  - The centre ray disagrees with the spans in 2–34 columns per level, all at silhouette
    edges, where the ray reports a zero-height pair the fill rule drops.
- **Drainage field.** Load 19–83 ms (test_arena's 135k spans are the slow case). Repair after
  a grenade: mean 0.08–0.65 ms. The worst was 13 ms: one breach on wrecking_yard lowered
  the fill of 30k spans, the whole-lowland case of §9.2. Flats are not resolved on the blast
  frame; the router resolves one when it first follows a drain on it.
- **(b) Routing.** The staircase passes all three criteria: worst lateral deviation 0.38 m,
  worst width step 3%, no real depressions. Skyway's bed fails: p95 deviation 3.4 m, worst
  width step 153%. Its bed is level along its length, with 0.8 m of roughness, so neither
  the drains nor a "lowest line" have a direction to follow.
- **(b) Hierarchy.** Each current pool has at most two children and one split per full drain.
  The dissolve rule is not needed.
- **Skyway's pool stands above its outlet.** Its bed runs out of both ends of the level, so the
  fill level at the seed is 1.01 against an authored surface of 3.0.

### Stage 1: span graph

- the rasteriser, crossing cache and fill rule,
- parity repair,
- eager build, remap, ownership and the generation check,
- `DrainageField` with repair,
- the invariant assert,
- `level_check --svg` and the first pieces of `WaterNetworkDebug`.

### Stage 2: basins on the real skeleton

- **The skeleton:** `enum Store` (only `Basin` in use), the `Link` trait (no links yet),
  `HydrologySolver`, `VolumeLedger`, `TopologyBuilder`, and the loss term.
- **Basins:** the §7.2 flood with crest grouping, children, merge saddles and hysteresis.
  `Hypsometry` with partial wetting and pothole steps.
- **Consumers:** `WaterQuery`, with buoyancy, the sleep tracker and coupling re-plumbed.
- **Output:** `BasinMesher`, re-region after edits, and `level_check` validation.
- **Ledger:** on from day one. Water above a new outlet crest has nowhere to go until
  stage 4a, so it is booked to `discarded`.
- The old grid is deleted.

**As built.**

- **The flood** runs in two phases. Under the water, everything connected below
  the level is one body already; the wet flood takes it all and stops only at
  outlets. Above the water, the climb follows the design's rule: every descent
  is a crest. Merge saddles come from merging the wet spans in saddle order
  (Kruskal), recording each merge of two real depressions. Seed placement then
  does not matter.
- **The climb stops** at `max(level, lowest crest of any kind) + 5 cm`, and never
  more than 0.5 m above the level. A child crest bounds it as well as an outlet,
  since water spills there. The 0.5 m bound stops a pond drained into a cave
  system from claiming the flat arena its crests lie on: that re-flood once
  climbed 132k spans in 42 ms. A basin rising near the top of its region is
  re-flooded.
- **Open world edges.** A region span that opens onto the void is an outlet
  crest at its floor.
- **Children below the level.** Until weirs exist (stage 4a), the wet flood
  absorbs an unowned depression under the water at once, so its volume comes
  out of the level.
- **Waves** were deleted here, not in stage 3. The old renderer was their only
  visible output, so buoyancy and the surface stayed flat until ripple tiles
  arrived in stage 3. Splash and wake particles still fired. The coupler
  disturbs any `RippleField`.
- **Decisions at the gate.** Skyway's pool is left as authored: it drains off the
  world at load, and `level_check` reports it. Routing proceeds without
  centreline hints. Stage 4b re-runs the three routing criteria on a sloped
  carved-channel scenario, and hints return only if that fails.
- **Cost** (`water_perf`): a quiet frame is 0.001–0.002 ms of water CPU on every
  level (baseline 0.37–4.5 ms). Load is 30–110 ms. A blast frame is 0.7–2.5 ms
  on four levels. thin_ice is 6.7 ms: re-flooding its 17.7k-span lake takes 4.0 ms
  and rebuilding its mesh 1.4 ms. This is the worst case of §9.2, over budget.
  Stage 6 takes it on.

### Stage 3: surface detail

- `Swell`, `RippleTiles` keyed by body, and the fine tile mesh.
- `WaveGrid` is deleted.

**As built.**

- **Swell.** Five sines shared by `swell.rs` and `swell.glsl`, and a test holds
  the two spectra together. Amplitude is 1.5 mm per metre of fetch, capped at
  15 cm: thin_ice's lake gets 10 cm, test_arena's pond 2 cm. Each body has its
  own phase.
- **Ripple tiles.** A padded, branch-free stencil. Dry cells are held at zero.
  An edge facing a sleeping tile gets a 6-cell sponge. Energy in an edge strip
  wakes the neighbour it faces, unless the budget is full.
- **Cost** (`water_perf`): 0.23 ms per frame with 32 tiles awake on thin_ice,
  0.03–0.19 ms elsewhere. The upload is 544 KB per frame at 32 tiles: 512 KB of
  heights and 32 KB of column masks, a little over the 0.5 MB target.
- **Rendering.** The fine tile is a static 65 × 65 grid. It reads heights from a
  per-slot storage buffer rather than a texture, so it needs no image layout
  transitions, and the fragment shader discards it outside the body's
  columns.
- **Queries** add swell and ripples to the surface. `WaterQuery::level_at` gives
  the still level for harnesses and anything else that wants hydrology alone.

### Stage 4a: the correctness core

- `Weir` (per cell, free and submerged, with closure), `Orifice`, `drain_gain`, and the
  implicit solver with coupled groups.
- Merge and split with hysteresis.
- **Instant routing:** each outlet crest's `Weir` has as its `down` the store its drainage
  path reaches. Breaching a pond already drains it into the next depression.
- The §8.1 state table is the acceptance checklist.
- `discarded` reaches zero and is asserted to stay there.

**As built.**

- **Outflows.** A basin groups its crest cells into outflows: cells of one kind
  whose inside columns touch, and every cell into a hole as one orifice.
- **Links are made lazily.** An outflow is linked once its basin is within
  2 cm of its lip, which matches the design's "always linked, carries zero
  while filling". Its target is resolved then:
  - for a child crest, the store owning the far side, or a new empty basin
    at the pit's bottom;
  - for an outlet, the store its drainage path reaches (instant routing),
    with an empty basin made at the first real depression on the way, or
    the void sink at an open edge.

  A link whose store is removed by a merge, split or drying is dropped, and
  its outflow relinks by the same rule. Re-floods drop a basin's links too,
  so they churn during big edits. That is harmless, but noisy in the
  topology log.
- **One weir per pair.** Two basins across one ridge share one reversible weir.
  A second weir the other way would carry the same water twice.
- **Re-flooding after an edit** keeps the basin's claim through the flood, so
  its own spans are its own. A span re-paired by the edit seeds the flood only
  if water stood over it before. Without that rule, a crater blown into a dry
  bank filled from the lake beside it. A dry depression the basin can now
  reach under its level becomes a child behind a weir, not part of the lake.
- **Holes.** Where the remap lands an upper basin's span in a lower store's
  (§8.3), the basin records a hole. The crest cells into the hole's columns
  form one orifice.
- **Scenarios** (`water_viewer`, all tests):
  - `breach`: 58 of 62 m³ leave over the weir in about 10 s, and the weir
    closes at 31 s.
  - `spill_merge`: two halves of a trench merge at 104 s at one level, with
    no water lost.
  - `drain_split`: a lake split in two by its divider.
  - `island_hole`: an island pool poured onto the pond below.
- **Cost:** 0.02 ms per frame through the breach's transient. Skyway's pool,
  draining off the world over its weir, costs 0.36 ms per frame. Inverting its
  4.9k-span hypsometry dominates.

### Stage 4b: rivers

- Reaches, rating curves, fronts and tails, and the §8.2 state table.
- Currents.
- `ReachMesher` with flow mapping.

This is the stage most likely to need iteration on looks. 4a stands without it.

**As built.**

- **Routing** (`topology/router.rs`). An outlet whose drainage path runs at
  least 3 m before it reaches a store is laid as a channel: the path is cut
  into reaches of about 12 m (none shorter than 8 m), linked in series by
  `ReachOutflow` into the store at the bottom. A shorter path keeps 4a's
  instant weir.
- **Rating curves** (`network/rating.rs`). Manning with n = 0.035, sampled on
  cross-sections 1 m apart and ±8 m wide over the span floors. The slope is
  taken over a 3 m window and floored at 1e-3. Each reach tabulates eight
  points from 0.01 m³/s to its design discharge, the outlet's flow at 0.3 m of
  head; below the first point, flow goes as depth^0.6.
- **Reaches** (`network/reach.rs`). Kinematic storage with a front and a tail.
  The front advances at the rated velocity and the reach fills behind it. A
  reach retires once nothing feeds it and it has drained below 5 mm. It never
  retires mid-channel while its upstream still runs.
- **Re-routing.** When a receiving basin's level moves more than 0.25 m from
  the level its channel was routed against, the channel is laid again.
  Relinking follows 4a's lazy rule: an outflow that already had a link is not
  relinked while its free flow is at or below 2·`Q_RETIRE`, so a retiring
  channel does not churn.
- **Currents.** `WaterSurface::sample` returns the water's velocity.
  Buoyancy drag acts relative to it, so floating bodies ride the current. On a
  reach, `level_at` falls back to the reach's sample, so probes see water.
- **Meshing.** `ReachMesher` builds a strip along each reach's centreline, and
  `river.vert` raises it to the reach's depth per vertex. Flow maps scroll the
  normals along the channel. Shore foam is for still water only; white water
  appears above 3 m/s and stays faint.
- **Scenario** `river` (test): a 44 m³ lake is breached at the head of a
  zig-zag carved channel. Five reaches of 12–14 m run into the void sink. The
  front wets the upper probe by 2 s and the lower one by 6 s. The sink
  receives its first water at 10 s and 38 m³ by 90 s. The ledger stays at
  zero.
- **Routing criteria, re-tested on the carved channel.**
  - 88% of centreline points are within 0.5 m (p95 0.62 m).
  - Every miss is the 0.62 m diagonal of a single column. The 3 m outliers are
    inside the lake crater at the channel head, where the path has not yet
    entered the channel.
  - The width-step p95 is 22%, which is column quantisation.
  - No false depressions.

  The misses are all at column resolution, so **centreline hints are not
  added** (§22).
- **Cost.** In `breach`, routing runs on the blast frame: the settle stage takes
  1.9 ms of a 3.4 ms water frame. The transient that follows is 0.07 ms mean.

### Stage 5: falls and sources

- `FallTracer`, `FallPath` on links, `Spring`, `SkySource`, `Sink` and `FallMesher`.
- The staircase scenario passes.

**As built.**

- **`FallTracer`** (`network/fall_tracer.rs`). It sweeps the parabola in
  0.25 m steps through the span graph, not the mesh. A point is in air when a
  span holds it above that span's floor. Where the arc lands:
  - it stops at ground under it, a wall across it (at the wall's foot), or
    water standing in its way;
  - off the map, it lands in the void;
  - a source placed inside rock is moved up to 2 m along its direction to
    find air, and one that finds none is *sealed*.
- **Fall steps are measured over a run.** A drop of more than 0.75 m within
  1 m horizontally is a fall. Marching cubes rounds every riser through one
  intermediate column (10.0 → 9.5 → 8.75 on the staircase), so a
  column-by-column test would never fire. The lip is the highest cell of the
  drop. A crest counts as a lip at its saddle height, so a riser right at a
  pool's edge falls from the crest.
- **A fall is geometry on the link.** `FallPath` lives on `LinkEntry`, not
  behind `Link::fall_path()`: a weir, a reach's outflow and a spring's
  `FixedRate` all carry one the same way, and no law duplicates it.
- **Routing through falls.** A channel that reaches a fall step ends there.
  Its last link carries the arc, launched from the surface the reach runs at
  as it is laid (not its design discharge, which has headroom) at that
  velocity, or at the critical velocity over a 0.3 m design head for a bare
  weir. The reach's drawn surface runs on to the arc's first point. From
  where the arc lands, a new channel is laid. When a fall lands on a ledge
  too short to be a channel, its arc is joined to the next fall's, so a chain
  of falls is one path.
- **A channel's outlet is kept as a point** (`ChannelOutlet`), with its fall.
  A `SpanRef` would not survive the rebuild of its chunk. If the store a
  channel fed is replaced (merged, split), its last reach relinks to
  whatever stands there now.
- **Channel removal is topological, not by id.** Laying a channel over a fall
  lays the one below first, so ids no longer run downstream.
- **Sources.** A `Spring` is a `Reservoir` plus a `FixedRate` link launched
  along its traced arc; its `direction` is the launch velocity in m/s. A
  `SkySource` has zero launch velocity. A source whose link goes relinks on
  the next settle.
- **Re-tracing.** An edit that re-pairs a column under any fall's arc drops
  that link, and the channels it joins, and they relink by the usual rule.
- **Sinks** are boxes in `Outlets`, tested against the span in hand, so they
  hold through rebuilds. They drain into the same void sink as the open
  edges.
- **`settle: Steady`** (`topology/steady.rs`). Gauss–Seidel sweeps run
  Fill–Spill–Merge to completion:
  - Each sweep settles topology, then brings every store a source feeds to
    steady, nearest the source first.
  - A reach gets its storage at its inflow, running its whole length.
  - A basin gets the volume at which its inflow equals its outflow plus loss.
  - A fed basin that nothing yet drains fast enough is held two `CAP_MARGIN`s
    under its cap. There its lips link, but a re-flood does not drop the
    links. It goes to the cap only if it already stands there and still
    cannot pass its inflow.

    Setting it straight to the cap made every sweep re-flood it, lose its
    links, and climb 5 cm, without end.
  - Water brought in is booked as `emitted`. Unfed pools keep their volume.
- **`FallMesher`** and `fall.vert`/`fall.frag`. Each fall is one static
  ribbon, drawn last in the water pass with a fragment shader of its own:
  - Its width comes from √Q and is pushed per draw, so the mesh never
    changes with the flow. A trickle thins.
  - Streaks are keyed to seconds from the lip, so they fall at the water's
    pace.
  - The sheet whitens and breaks up as it drops.

  It is drawn in the water pass rather than the sorted transparency pass.
  That is a shortcut: it refracts the opaque scene behind it, so glass behind
  a fall shows unrefracted.
- **Scenarios** (`water_viewer`, all tests):
  - `staircase`: a 1 m³/s spring runs down eight treads, over eight falls.
    As authored, the front reaches the bottom in 80 s. Opened steady, it
    passes 1 m³/s from the first frame, holding the same 82.9 m³ as the
    dynamic run.
  - `spring_pools`: a sky source over a divided trench. It opens already
    merged, 641 m³ filled, passing 0.5 m³/s over its notch, with the volume
    constant to 1e-6.
- **Cost.** Opening steady costs 14 ms (staircase, 3 sweeps) and 17.5 ms
  (spring_pools, 37 sweeps), against the 200 ms budget.
- **`level_check`** lists each source and where it lands, and reports a
  sealed one. It does not report a source-fed basin standing over its outlet,
  and warns if the steady settle did not converge. The SVG draws falls as
  dashed arcs with their landing points.
- **Also fixed:**
  - The river pipeline was never destroyed.
  - A level with rivers or falls but no basin drew nothing.

### Stage 6: destruction

- The six scenarios of §9.3 pass in `water_viewer`.
- Play-tested in a level.

**As built.**

- **Scenarios** (`water_viewer`, all tests):

  | §9.3 | Scenario | What passes |
  |---|---|---|
  | 1 | `breach` | A channel advances from the notch; once the weir closes, it recedes and retires, and no link is left |
  | 2 | `island_hole` | The orifice carries a straight-down `FallPath` from the hole onto the pond; no water is lost |
  | 3 | `river_diversion` | A crater blown in a running bed at 12 s fills as a new basin and spills, and the river reaches the lower probe again |
  | 4 | – | The sea wall needs the ocean; stage 7 |
  | 5 | `spring_rock` | The rock around the spring is blown away at 5 s. The spring keeps its point, its fall is traced again into the crater the blast left, which fills and spills, and 1 m³/s leaves the bottom again |
  | 6 | `crater_drain` | A crater under a lake merges at once (one body, one level). The dam is blown, the lake drains away, and the crater splits off holding its water |

- **Cutting a channel, not removing it.** An edit across a reach, or across a
  fall's arc, used to remove the whole channel, and every reach's water was
  dumped downstream in one frame: 36 m³ into the void at once in
  `river_diversion`, 83 m³ in `spring_rock`. Now only the touched reach and
  everything above it go, upstream first, each into the reach below it. The
  reaches below keep their water and recede, as §8.2 says, unless the channel
  laid again joins them.
- **A basin whose bed is blown away** runs its water off along the drainage
  from where it stood, to whatever store that reaches. It used to go to the
  void.
- **An empty basin a channel is still advancing towards is fed.** It used to
  dry up and be removed before the front arrived.
- **The §9.2 valve.** A re-flood of a region over 8000 spans (about 1.8 ms)
  waits one frame, off the frame that already carries the terrain rebuild.
  - The basin is **frozen**: no link runs through it, its volume holds still,
    it neither settles nor merges, and nothing reads its region's spans.
  - The deferred re-flood runs at the start of the next step, or before the
    geometry takes another edit, against the remap of the edit it followed.
  - thin_ice's shore blast: the blast frame's water work falls from 6.1 ms to
    0.39 ms, and the next frame carries the re-flood and mesh rebuild,
    5.8 ms. The peak frame falls from about 9 ms (terrain plus water) to
    5.8 ms. **The 17.7k-span lake's own re-flood is still 4.4 ms**, over the
    2 ms budget: meeting it needs an incremental re-flood, not a delay.
  - skyway's blast frame is 2.5 ms, just over budget, below the valve's size.
- **Not done: play-testing.** The game window cannot be launched from the
  agent's shell, so no level has been played with the new water.

### Stage 7: ocean

- Open edges, the ocean flood, weir-based lowland flooding, the mask and the ring mesh.
- Swell at sea.
- One demo level with an island pool over the ocean.

**As built.**

- **Level format.** `ocean: Some((level: 0.0))` is an island; list the edges
  for a coast: `open_edges: [South]` (`West` −x, `East` +x, `South` −z,
  `North` +z).
  - Leaving `open_edges` out opens all four edges. RON has no clean way to
    write `All` beside a list, so that spelling from §12 was dropped.
  - `swell` defaults to 0.25 m.
  - `ocean_level` is gone. Every level had it as `None`.
- **`Store::Ocean`** has a fixed level and endless volume, and books against
  the ledger's `Ocean` account. Its region is held only as span ownership,
  which the remap already carries through rebuilds, so the ocean keeps no
  span list of its own.
- **At load**, the sea claims every span below sea level joined to a column on
  an open edge. It is made before any pool, so a pool seeded in it *is* the
  sea.
- **Edges.**
  - A span on an open edge drains at `max(floor, sea level)`.
  - Water leaving the map there goes to the sea: off a walk, a crest or a fall
    arc. Everything else goes to the void.
  - Sea-edge columns are found by scanning, not taken from the graph's bounds:
    the graph's chunk-aligned bounds overhang the terrain by up to a chunk.
- **Lowlands (§9.1, step 5).**
  - After an edit, a below-sea span newly joined to the sea seeds an empty
    basin.
  - A crest whose far side stands over its lip links at once, so the sea
    pours in over a reversible weir.
  - Once the basin is within 5 mm of sea level and both stand over the lip,
    it is absorbed: its spans below sea level become the sea's, and its water
    the ocean's.
  - A weir into the sea is reversible only if the sea stands above the lip.
- **`fed` counts water coming back up a reversible link.** Without that, an
  empty lowland dried and was removed before the sea reached it.
- **The steady settle's search stops at endless stores.** A spring into the sea
  used to lead the search back up through a floating pool's weir, and the
  pool was "steadied" to empty.
- **`OceanMesher`**:
  - One quad per sea-owned column, plus the shore ring, which leaves out any
    lowland the sea could stand in but does not own.
  - Strips beyond each open edge out to 1 km, at 8 m near the edge and
    coarser further out.
  - Drawn through the basin pipeline at sea level, with the authored swell.
  - It is a mesh of its own, rebuilt only when the sea grows. Folded into the
    basin mesh, every lake's re-flood rebuilt the sea too: 3.6 ms of the
    island_sea blast frame.

  The design called for a 2D mask texture. Per-column quads from ownership
  are that mask in geometry, and share the basin path.
- **Scenario `sea_wall`** (§9.3, scenario 4; test). A dry lowland 2 m below
  the sea is breached at 2 s. It floods over the weir for about 17 s, then
  joins the sea: 494 m³ booked to `ocean_in` and back out as it is absorbed,
  with the ledger balanced.
- **`levels/island_sea.level.ron`** is the demo:
  - an island with a crater lake;
  - a spring falling off its east cliff into the sea;
  - a floating rock whose pool stands 11.5 m above the sea in the same
    columns.

  `water_perf` includes it: quiet 0.004 ms, blast frame 1.2 ms. A test holds
  the two bodies per column.
- **`level_check`** names the sea, and warns if water runs off a closed edge
  of a level that has one.

**Revised after the first play-test** (island_sea: seams, a hard edge to the
sea, and spikes after a grenade).

- **Ripple stability.** The step is explicit, and a grenade frame is long
  (the frame clock caps at 100 ms, against a limit of 44 ms at 0.125 m
  cells). It now takes at most half a cell's crossing time (31 ms): a long
  frame runs the ripples slow. Damping is implicit, and heights are capped
  at 0.4 m or half the depth. A test runs 100 ms frames.
- **Waking.** A tile wakes its neighbour when energy enters the sponge, not
  once it reaches the edge: the sponge had absorbed it first, and the
  ripples stopped at a square.
- **Seams between awake tiles.** Each tile is uploaded with a one-cell apron
  holding its neighbours' cells, so both interpolate the shared edge alike.
- **The edge beside coarse water** is sealed. The apron there mirrors the
  edge with its sign flipped, so ripples meet it at zero. The swell runs
  straight between column corners, as the coarse quads' edges do.
- **Corner floors.** Every surface takes a corner's floor as the lowest wet
  column touching it (`corner_floor`), so swell fades the same on either side
  of a tile edge. Ripple tiles upload their 17 × 17 corner floors. Upload at
  32 tiles is 612 KB.
- **The ring.** The first 8 m past the map's edge are column-wide quads
  whose swell height fades out, so they meet the sea vertex for vertex and
  the flat coarse rows beyond at no height. The ring's 8 m quads had drawn a
  crack at the edge and a flat, pale sea beyond it.
- **Swell normals are per pixel** for every surface, each wave fading as it
  grows too short for the pixel, so the flat ring still reads as swell.
- **Ripple slope** is capped at 1 for the normal.
- `level_viewer` takes `--blast`, `--splash` and `--run` to look at ripples.

**Second play-test** (test_arena: a crash, the pool vanishing, a floor
raised). `water_fuzz` blows a level's water about at random and checks it
after every frame; it found each of these.

- **Reverse flow crashed the current.** A weir running backwards gave its
  current term a negative speed cap. The cap is `|Q|`.
- **A flooded pocket is merged.** A cave under a lake, with a hole blown
  between them, filled to its cap and could rise no further, so it never
  met the lake's level to merge; it sloshed several m³ a frame. Two basins
  over a shared lip now merge also when one is full to its cap and the
  other stands above that cap.
- **Slivers.** An air span thinner than a voxel is below what marching cubes
  resolves; a nearby blast can close it with no material added. Its remap
  entry is marked as not resting: it no longer claims the new span's owner,
  counts as a blown-through floor (which gave the lake a phantom hole to
  drain through), sets the drain, or holds the floor.
- **Re-triangulation tolerance** is one voxel: blasts into floors and caves
  raised a floor 0.75 of a voxel, and the surface stays within its cell.
- **Wakes and bobbing** stir the surface only while a body breaks it.
- **Ripple cells are 0.25 m** (32 × 32 per tile), not 0.125 m, and waves run
  at 1.25 m/s, not 2. The wave equation runs every wavelength at one speed;
  real ripples disperse, `c = √(gλ/2π + 2πσ/ρλ)`, and 2 m/s was water's at
  λ ≈ 2.5 m while the 0.125 m grid mostly showed λ of 0.25–0.5 m (0.6–0.9 m/s):
  too fine and too fast. 1.25 m/s is water's at λ = 1 m, four cells. The
  stable step is now 100 ms, the frame clock's cap. At 32 tiles: step
  0.09 ms, upload 212 KB.
- **The ring has no T-junctions.** It is nested rectangles around the map,
  0.5 m apart with a vertex every 0.5 m out to 8 m, then twice as far out
  each time, and neighbouring rectangles are stitched with triangles that
  share every vertex. Where the 0.5 m band met 8 m quads, a pixel crack
  showed along the join.

### Stage 8: interfaces

§7.9, and §8.2's Drowned and Exposed as designed. Until now a channel ending in a basin
kept the basin's level when laid (`routed_to_level`) and was re-laid whole when the level
moved 0.25 m (`REROUTE_SHIFT`); a channel ending in a fall was never re-laid; and falls
were decided at build time in four places. Re-laying a whole river poured its storage into
a small receiving lake, which moved the lake past 0.25 m and re-laid it again: water_park's
steady settle never converged until its catch lake was authored at its settled level.

- `Lip` on every `LinkEntry`, one per side for a reversible link; `spill_fall`, `hole_fall`
  and the walk's store-end fall give way to one `link_arc(lip, speed, d_c)`, which traces
  only where the jet separates.
- `FallTracer` lands on ground; the landing store is the first water standing in its way.
- `interface(link, network)`, read by the fall renderer and `surface_at`.
- `Reach::surface_at` with its port targets; `river.vert` mirrors it; `WaterQuery` and the
  fall's launch use it.
- `fall.vert` shifts, cuts and styles each sheet from its heights.
- Edits touch an arc only above the water it enters, and re-trace it in place.
- Drowned and Exposed as incremental edits; `routed_to_level` and `REROUTE_SHIFT` go.
- Check first: whether a mouth section's rating counts the lake's samples (`wet_run` across
  a shore with no bank), so `Ā` holds water the basin's hypsometry also holds. If so, drop
  samples whose span another body owns.

**Tests.**

- Unit: `interface` for a basin's crest, a reach's end, a hole, a source, and a reversible
  weir run each way; the sheet's length through submergence goes to zero without a jump.
- Unit: `link_arc` gives none on a steep channel whose ground stays within `d_c` of the lip,
  one at a riser, and always one for a hole or a source.
- Unit: `surface_at` eases to a lower lake, backs up to a higher one, stays continuous as
  `lower` crosses the lip, starts at the basin's level, eases across a reach boundary, and
  clamps overlapping eases on a short reach; the shader's copy matches.
- Unit: Drowned conserves volume, moves the integrated area not a length share, and keeps
  `Q_out` continuous.
- Unit: a rating across a shore with no bank (§ check above).
- Scenario: a river mouth 0.3 m over a lake draws a sheet that reaches the lake's surface.
- Scenario: the sea wall breached: the sea pours into the lowland over the lowland → ocean
  weir, and its sheet is drawn.
- Scenario: a reversible weir between two basins draws a sheet only while there is a step.
- Scenario: a lake rising under a river's fall (outflow blocked): the sheet shortens to
  nothing, then the channel's end drowns and is cut, and no reach above the cut changes its
  storage in that frame; lowered again, the end is exposed and the sheet returns. Volume
  balanced throughout.
- Scenario: a lake rising over the middle of a channel that runs past it.
- Scenario: a fall landing mid-reach reads `lower` at the landing's distance.
- Scenario: an underwater blast near a fall's landing leaves the channel above it unchanged.
- water_park and staircase: the arc count does not grow; water_park opens steady with the
  catch lake authored at 1.0 as well as 3.3, within a sweep bound, and its fall test finds
  the catch lake's link by `down`, with the sheet's top within 1 cm of `surface_at`.
- `water_fuzz` on every level, and the island_sea, water_park and staircase views compared
  in `level_viewer` before and after.

**As built.** Four commits, one per part: lips and arcs with sheets drawn from the heights
(`network/lip.rs`, `network/interface.rs`), reach surfaces eased to their ports
(`Reach::surface_at`, `reach_ends`), Drowned and Exposed (`topology/builder/shoreline.rs`),
and ratings stopped at a shore.

- **Arcs.** Where the jet parts is judged mid-step over `FALL_RUN`, so no sample falls on a
  column edge. Skyway's pool, standing over its own 35 outlets, gets an arc at each where
  before it had none: each drops 1.3 m into the channel below. Arc counts elsewhere are
  unchanged; arcs are taller, since they now run on to the ground.
- **A channel too short for a reach** hands its weir the lip it actually falls from, the
  far side of its one cell, not the crest.
- **Heights** are computed where needed. Reach ends are worked out once a tick, with the
  levels, and cached by store slot: `WaterQuery` samples them in every physics substep.
- **Sheets** are lifted by `upper` minus the arc's first point, cut at `lower`, and whiten
  by their share of the drop that shows rather than of the time to the ground. Their
  aeration eases from clear to white over the 5 cm above submergence.
- **Drowned** passes over the lakes a channel drains from. A lake standing over its own
  outlet (skyway) covers the first cells of every channel leaving it; without this, each
  was drowned and laid again every tick, and skyway's steady settle ran to 400 sweeps.
- **Exposed** lengthens the channel's last reach in place when the walk from its dry shore
  reaches the same water within 24 m (the front stays, and advances over the new length),
  and otherwise lays a channel on from the shore. Laying a new channel every time left
  nothing to lay when the water had moved one cell, and churned.
- **Ratings** skip samples where another body's water stands now, the lakes the channel
  drains from excepted. At three water_park rivers 15–22% of the samples under a reach's
  surface stood in a lake.
- **Found on the way, older than this stage:**
  - A channel ending in a dry pit claims the pit's first span as its last cell. Relinking
    it once the pit's empty basin had dried found the channel itself, standing there and
    at the end of a walk from there, and dumped it into the void: a spring into a dry pit
    lost every drop. The walk and the standing-water test now pass over the channel's
    own claim, and a channel into a dry pit gets the pit's basin again.
  - A reach removed with nothing below it booked its water to the void's store account,
    not to `sunk`: the ledger lost it.
- **Cost.** Quiet frames are unchanged on every level (skyway 0.82 ms, the rest under
  0.01 ms); skyway loads in 38 ms.
- **Tests.** `shoreline` (a lake climbs a river's channel, cutting it back cell by cell
  with every drop going to the lake and its level never jumping, then drains down it and
  the channel is lengthened); the sea pouring back over a lowland's weir draws its sheet;
  water_park's river leaves its head lake at the lake's level and falls from its own end
  into the catch lake; water_park opens at rest with the catch lake authored at 1.0 or 3.3
  (24 and 6 sweeps); every level's water comes to rest when it opens; plus unit tests for
  the lip, the heights, `surface_at` and its shader mirror, and a rating at a shore.
- **Open.** A lake's re-flood when it rises to its region's cap still drops and relays its
  outflow channel (the churn noted at stage 5); nothing is lost, since that water was
  going where the channel took it.

## 22. Decided and deferred

| Question | Status |
|---|---|
| Stopgap dirty flag in the old renderer | **No.** thin_ice's waves don't go quiet (§2.3). |
| Drain speed | **`drain_gain`** per level (§7.3). No time scaling. |
| Loss to ground and air | **Built in, off by default, minor stores only** (§7.7). Ends trickles and dries puddles; rivers and lakes never lose. |
| Crates grounding on outlet lips | **Play-testing decides.** Steady head over a 5 m lip is about 8 cm at 0.2 m³/s and about 31 cm at 1.5 m³/s, so river-sized outlets will carry most crates over. |
| Authored centreline hints | **Spike 0.5b decides.** |
| Segment streaming | **Not planned.** If it arrives, lazy span building, and fronts waiting at segment borders, come back. |
| Skyway's pool above its outlet | **Left as authored** (stage 2 gate). It drains off the world at load; `level_check` reports it. |
| Centreline hints after spike 0.5b | **Not added.** Re-tested at stage 4b on a sloped carved channel: every miss is one column quantum (0.62 m). Hints come back only if play-testing shows a visible wander. |
