# Water: a Hydrology Design

Status: awaiting sign-off. Replaces the design in `WATER_SYSTEM_PLAN.md`.

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
  rebuilding and uploading the mesh every frame: up to 4.2 ms on a quiet frame.
- **No stopgap in the old renderer.** A dirty flag would skip rebuilds on quiet frames, but
  thin_ice's floating ice keeps its waves busy, and thin_ice is the level that would
  motivate one.
- **Measurement caveat.** `render_perf`'s `cpu/water` is 10.6 ms on thin_ice whenever both
  `--worst 300` and `--csv` are passed, and 4.2 ms otherwise, deterministically, with every
  other stage unchanged. Those flags are read only after the run, so the likely cause is
  memory layout interacting with the per-frame upload into mapped memory. The table uses
  runs without `--csv`. The anomaly belongs to `render_perf` and is tracked separately.
- **Not yet measured:** water under active flow, and water after a blast. Stage 0 measures
  both for the old system (§21).

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
- **Floor bounds.** Each upward-facing triangle is clipped to the column square and split
  into *floor bands*. A span's band runs from the ceiling of the span below (−∞ for the
  lowest) up to its own ceiling. The `y` range of the clipped polygon inside a band folds
  into that span's `floor_min` and `floor_max`. At an island rim, floor that lies below the
  ledge goes to the lower span.

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
- at the first span whose `floor_min` lies below the receiving basin's current level (the
  rest of the channel is drowned, §8.2),
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

**Fall steps.** A step that drops more than `fall_threshold` (0.75 m), at a cliff lip or an
island rim, is a fall step. `FallTracer` sweeps a parabola against terrain in short
segments. It launches with the upstream reach's velocity, or a source's authored direction.
The outbound link's `down` store becomes whichever store owns the span the arc hits, and
the link keeps the arc as its `FallPath`.

For example, a 10 m drop at 1.4 m/s lands 2 m out after 1.4 s.

A path is re-traced only when an edit touches its arc.

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
   spans instantly.

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
| `ReachMesher` | One quad per cross-section cell, out to the `Q_design` top width. Each vertex carries its distance along the reach, a reference depth and a velocity. Per-reach uniforms set the depth to `d_ref · (Q/Q_design)^0.6` and discard outside `[x_t, x_f]`. The depth test trims the width at lower `Q`. | Route, re-route, or `Q_design` growth |
| `FallMesher` | A ribbon along each link's `FallPath`, with width from its `Q`, drawn in the sorted transparency pass. Mist comes from the particle system. | Re-trace |
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
| Blast frame, water share (re-pair, drainage repair, re-region, re-route) | ≤ 2 ms at p99 | unmeasured (stage 0) |
| Worst-case blast (largest basin re-region, whole-lowland repair) | reported; the §9.2 valve only if over | unmeasured (stage 0) |
| Transient (breach draining), CPU per frame | ≤ 0.5 ms | unmeasured (stage 0) |
| Load with `settle: Steady` | ≤ 200 ms | 1.8–8.5 ms first-frame spike |
| GPU water pass at 2400×1600 | ≤ today's | measured in stage 0 |

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

### Stage 3: surface detail

- `Swell`, `RippleTiles` keyed by body, and the fine tile mesh.
- `WaveGrid` is deleted.

### Stage 4a: the correctness core

- `Weir` (per cell, free and submerged, with closure), `Orifice`, `drain_gain`, and the
  implicit solver with coupled groups.
- Merge and split with hysteresis.
- **Instant routing:** each outlet crest's `Weir` has as its `down` the store its drainage
  path reaches. Breaching a pond already drains it into the next depression.
- The §8.1 state table is the acceptance checklist.
- `discarded` reaches zero and is asserted to stay there.

### Stage 4b: rivers

- Reaches, rating curves, fronts and tails, and the §8.2 state table.
- Currents.
- `ReachMesher` with flow mapping.

This is the stage most likely to need iteration on looks. 4a stands without it.

### Stage 5: falls and sources

- `FallTracer`, `FallPath` on links, `Spring`, `SkySource`, `Sink` and `FallMesher`.
- The staircase scenario passes.

### Stage 6: destruction

- The six scenarios of §9.3 pass in `water_viewer`.
- Play-tested in a level.

### Stage 7: ocean

- Open edges, the ocean flood, weir-based lowland flooding, the mask and the ring mesh.
- Swell at sea.
- One demo level with an island pool over the ocean.

## 22. Decided and deferred

| Question | Status |
|---|---|
| Stopgap dirty flag in the old renderer | **No.** thin_ice's waves don't go quiet (§2.3). |
| Drain speed | **`drain_gain`** per level (§7.3). No time scaling. |
| Loss to ground and air | **Built in, off by default, minor stores only** (§7.7). Ends trickles and dries puddles; rivers and lakes never lose. |
| Crates grounding on outlet lips | **Play-testing decides.** Steady head over a 5 m lip is about 8 cm at 0.2 m³/s and about 31 cm at 1.5 m³/s, so river-sized outlets will carry most crates over. |
| Authored centreline hints | **Spike 0.5b decides.** |
| Segment streaming | **Not planned.** If it arrives, lazy span building, and fronts waiting at segment borders, come back. |
