# Water as a Shallow-Water Field: Spike

Status: on branch `water-field`, the game runs the field (§7). Play-tested
and adopted; the hydrology network (`WATER_HYDROLOGY_DESIGN.md`) it replaced
has been removed, and the water tools run the field.

## 1. Why

Rivers in the network misbehave where stores meet. A grenade in a riverbed shows it
(`river_blast`, network):

| t (s) | above the crater | below the crater | what the network did |
|---|---|---|---|
| 0 | 8.738 | 7.324 | steady river |
| 10 | **9.454** | 7.271 | blast; the crater is a basin, the river above is cut to end in it |
| 14–32 | 9.454 | **dry** | the reaches below have no inflow and drain; the network is laid again 4 times as the crater's basin climbs the channel |
| 60 | 9.454 | 7.322 | the river above never comes back to its level |

Both causes are structural. A reach does not feel the level of the water below it,
and every span belongs to one store. A boundary that moves therefore changes the
topology, and the topology is rebuilt each time. Patching that took easing,
drowned/exposed relays, hysteresis bands, and exceptions to the "laid again"
invariant.

Decision 2 of the hydrology design ("no shallow-water dynamics") is dropped
(2026-10-08).

## 2. The idea

One cell per span, holding a depth. Faces join spans in orthogonally adjacent columns,
by the span graph's own rule, and carry a discharge. Nothing else is state: there are
no stores, links, routes or topology.

A cell's depth stands on its span's *bed*: the median height of the span's floor over
the column's 0.5 m square, by area seen from above. The span itself is found, and
remapped across edits, at the column's centre, but a centre sample on a terrace can
land on a riser and make a shelf the column mostly lacks, which held water up to
0.8 m above the drawn terrain and poured it as a sheet. An edit holds the bed at or
below its old height.

```text
  TerrainWorld ──▶ SpanRasteriser ──▶ SpanGraph ──▶ FieldCells (+ faces)
       │ edit           │ re-pair         │ remap        │
       └────────────────┘                 └── carry ─────┤
  Sources ── pour ──▶                                    ▼
  Sinks   ◀─ drain ──                         FieldSolver (awake chunks)
                                                         │
                                       FieldScene ──▶ river pipeline
```

- **Flow law** (`water/field/solver.rs`). The local inertial scheme (Bates et al.
  2010) with de Almeida et al.'s θ (2012). Momentum lives on faces, volume in
  cells. Each face moves the same volume out of one cell and into the other, so
  the ledger balances to rounding (1e-13 m³).
- **Limits.** A cell never gives more than it holds, nor takes more than fits
  under its ceiling, so a cave fills to its roof and no further. A face never runs
  faster than Froude 1, which stands in for critical flow over an edge.
- **Edges.** The void takes a free overfall. The sea is a radiating (Flather)
  edge, so waves leave the map. A closed edge of a map with a sea is a wall below
  sea level and void above it.
- **Edits** (`remap.rs`). The rasteriser re-pairs the columns a blast touched.
  Each old span's water moves into the new span that holds its old floor. Faces
  are laid again only in the rebuilt chunks and the chunks owning faces into them.
  An edit costs 1.7–2.2 ms: about 1 ms re-pairing, the rest carrying water.
- **Sleep** (`activity.rs`). An 8 m chunk sleeps after 2 s calm. Any flow into
  it, an edit or a source wakes it. Still water costs nothing.
- **Load** (`fill.rs`). Every span's *spill level* is found first: the lowest
  surface it could keep, by Priority-Flood inward from every way off the map.
  A pool fills everything joined to its seed under `min(authored, spill)`, so
  a pool authored above its rim opens at its rim. The sea fills everything
  joined to its open edges below sea level. A level with springs then fills
  each hollow below each spring to its crest plus a weir head over its lip,
  wets the channel between them at normal depth, and runs (at most 30 s in
  the game) to settle what those guesses got wrong.

## 3. Results

`river_blast` on the field, against the table above:

| t (s) | upper | above | crater | below | lower |
|---|---|---|---|---|---|
| 0 | 9.544 | 8.832 | 8.261 | 7.401 | 6.687 |
| 10–40 | 9.544 | 8.832 | filling | 7.11 → 7.40 | 6.40 → 6.67 |
| 60+ | 9.544 | 8.832 | 7.895 | 7.404 | 6.687 |

The river above the crater does not move. The river below runs thin while the
crater takes the inflow, which is real: a fresh hole takes the flow until it is
full. It then comes back to its own level. The crater settles 0.37 m below the
old surface there because the blast lowered the bed, and its downstream rim is the
control now. The network gets that part right too.

On `river`, field and network agree within 4 cm everywhere: lake 3.299 vs 3.297,
upper 0.687 vs 0.679.

`cargo test --release --lib water_viewer::field_tests -- --ignored` checks:

- the river above a crater stays within 3 cm, and the river below returns within 2 cm;
- on every scenario, the ledger balances, no cell goes negative, and everything stays finite;
- still water does not move and sleeps;
- disturbed water (`sea_wall`, `spill_merge`) comes to rest and sleeps;
- a cave fills to its roof and no further;
- a pool fills its own hollow.

**Cost** (release, one thread, M-series): stepping takes 0.08–0.7 ms mean and
0.8 ms p99 at worst (`dam_break`, 64 chunks awake). Still water sleeps and costs
nothing.

## 4. What the bug hunt found

`water_viewer`'s catalogue now ends with eleven scenarios aimed at where a field
breaks (`water_viewer/hunt.rs`): a cliff fall, a dam break over dry land, a 30 %
chute, a diagonal channel, a one-column slot, a cave pool, eight blasts in fifteen
seconds, a sheet on a flat plain, a spring on a ridge, an island waterfall, and a
seiche.

| Found | Cause | Fix |
|---|---|---|
| The sea drained to −2 m | Closed edges drained the sea into the void | Below sea level a closed edge is a wall |
| The sea sloshed ±0.5 m for minutes | A fixed-level edge reflects every wave | Flather radiating edge |
| A lake authored above its rim filled the valley (19,000 m³) | The fill followed every span below the level | Priority-Flood that stops at the crest |
| The island's pond was never filled | A seed on a slope read as beyond a crest | Start the flood from the bottom of the seed's hollow |
| The tunnel's water stood at 4.35 m under a 3 m roof | Inflow was not limited by the ceiling | Each cell's inflow is limited to the room under its ceiling |
| Unlimited outflow from some cells | The limiter skipped cells in chunks just past the stepped ones | The limiter covers every cell a stepped face touches |
| Lakes jittered ±1.5 cm and shattered when drawn | The local inertial scheme oscillates over deep, slow water | θ = 0.9 |
| A centimetre's seiche for minutes, never asleep | Nothing in the scheme stops a pond sloshing | A linear drag on slow, deep water that fades out with speed and depth |
| Films crept down slopes for ever | Friction grows without bound as a film thins | Water under 5 mm (never drawn) seeps away at 0.5 mm/s, booked as lost |
| A 30 cm mound under a source | θ cancels the outflow at a point source | A source pours over its landing cell and the cells around it |

## 5. Rendering

`FieldScene` (`rendering/water/field_mesher.rs`) draws the field through the river
pipeline, with no new shader:

- one quad per wet cell;
- each corner is the mean surface of its group among the cells meeting there,
  split where one falls to the next: a step over 1 m, or over 0.5 m where the
  upper water's bed stands above the lower water, which a slope would cut
  through;
- a sheet down any edge where one falls to the next, as deep as the water pours
  over it, hung out at the lip where the lower column's own floor carries the
  ledge on (each span keeps a bed per quarter of its square for this);
- a sheet off the map's edge where a cell's water leaves through it: down to
  the sea's level past an open edge, or 6 m into the void where there is no
  sea (the same `falls` rule, so water level with the sea draws none);
- a shore patch half a column into each dry column beside a wet cell, and a
  quarter into each diagonal one between two dry sides.

A shore patch over rock (the dry column's bed stands above the water) is never
cut: the terrain rising through it draws the shoreline, at walls, steps and inside
corners alike. Over a hollow below the water its far
corners carry a negative depth, and the draw's wet range, which discards below
5 mm, cuts it midway as a contour.

Looked at and fixed:

- shards on a sloshing lake (corner heights disagreed between neighbours);
- stripes on a film on a slope;
- steps along a sloping bank;
- a staircase at every thin front;
- white "shore foam" over draining riverbeds (the cells read as still).

Open:

- **Falls are curtains.** A cliff's curtain is ragged where neighbouring edges
  hold different heights. A real fall needs a sheet built from the lip's flow,
  as the network's `FallMesher` builds.
- **The mesh is rebuilt on every draw.** Fine offline, not in the game. The
  ripple tiles already draw a displaced grid from a storage buffer, and the field
  wants the same: per-chunk heights uploaded when awake.
- **Shore foam covers wide shallow sheets.** It is the shader's rule for basins,
  where shallow means near a shore.
- **A sheet spreads as a diamond at first.** That is 4-connectivity showing; it
  rounds out within a minute.

## 6. Known gaps in the physics

- **Supercritical flow is capped at Froude 1.** A 5 m dam break's front runs
  4.5 m/s instead of about 14 m/s. The scheme is unstable above 1 without a
  better limiter or a Riemann-solver scheme.
- **The still drag is a gameplay term, not physics.** It is tuned so a
  centimetre's slosh dies in tens of seconds while rivers and sheets keep their
  speed.
- **Seep loss is always on**, against the hydrology design's decision 5 (loss off
  by default). It takes only water too thin to draw.
- **Sources pour into a 3 × 3 footprint** at the arc's end. There is no momentum
  from the jet.

## 7. In the game

`WaterWorld` holds the field; `WaterQuery` samples it (surface, bed, the cell's
velocity); the renderer draws it through `FieldScene` and the sea past the
map's open edges as the ocean ring. `water_perf` measures it on every water
level (release, one thread):

| level | opening | quiet, mean / p99 | blast frame | after the blast, mean / p99 |
|---|---|---|---|---|
| island_sea | 0.29 s | 1.6 / 2.0 ms | 5.5 ms | 3.8 / 4.4 ms |
| water_park | 2.7 s | 3.8 / 4.6 ms | 4.3 ms | 4.0 / 4.8 ms |
| thin_ice | 36 ms | 0.6 / 1.4 ms | 6.7 ms | 1.4 / 2.3 ms |
| swim_test | 15 ms | 0.2 / 0.6 ms | 3.2 ms | 1.8 / 2.7 ms |
| test_arena, subsidence, wrecking_yard | ≤ 0.12 s | ≤ 0.8 / 1.4 ms | ≤ 2.1 ms | ≤ 0.7 / 1.0 ms |

Each figure is stepping, carrying the water over the edit, and rebuilding the
drawn surface. A chunk's surface is rebuilt only when its depths have moved by
more than a millimetre (`SurfaceVersions`), along with the neighbours whose
edges meet the cells that moved. The renderer keeps each chunk's mesh apart
(`RiverParts`). A moved surface whose shape held (no cell wet or dry, no
corner's group split or joined, no relaid cells) is read again in place from
the cells each vertex records; only a reshaped one is built again, at most
eight a frame in the game, stalest first. Rebuilding alone, with the budget,
left water_park's lakes (a hundred chunks moving) drawn at three or four
updates a second, which play showed as jumps; read in place, no surface lags
while quiet and at most two lag for three frames after a blast. The offline
viewers rebuild everything.

Found getting there:

| Found | Fix |
|---|---|
| Rebuilding the whole surface took 86 ms a frame | A per-cell velocity table read once a step (it was quadratic per chunk); a one-way neighbour lookup in the mesher; per-chunk versions and rebuilds, capped per frame |
| `water_park` took 23 s to open (250 s of settling) | Fed hollows filled to a weir head and channels wet at normal depth before the run; a 0.1 s settle step |
| Pools authored above their rim drained at load; water behind a shoal was left empty | Spill levels from the map's edges in place of the crest rule |
| A spring's 3 × 3 footprint wet the bank above the channel it fell into | A jet's footprint spreads level and down, not up |
| A river between high walls was not drawn | A dry cell counts at most 10 cm below the water in a corner's depth |

Waves over the field (`water/waves.rs`): the network's ripple tiles and the
sea's swell, laid over the field's surface and carrying no water. A ripple
tile wakes only where its 8 m of water is flat to 3 cm (lakes, the sea; not
running water) and is drawn on the fine grid in place of that chunk's
surface. Water takes the sea's swell by how near sea level it stands, all of
it there and none 0.5 m above or below, so a lake above the sea stays still
and a river takes the swell up over its last stretch. The query adds both to
the surface, so floats ride what is drawn. A sample's surface is otherwise
its cell's, flat across the 0.5 m cell.

`skyway`'s pool was authored at 3.0 m in a channel that ran off both ends of
the map at its 1.0 m bed (the network found the same outlet and drained it
slowly; the field opened it empty). A sill across each end, level with the
canyon floor, now holds it: 1756 m³.

## 8. Not done

- ripples on running water, and a swell on lakes (the network gave large ones one);
- grenades do not ripple the water, only bodies moving in it;
- the GPU heightfield path for drawing;
- parallel stepping (chunks are independent within a substep's phases);
- fall sheets;
- blended surfaces divided by the field's water (only the sea's ring is in
  `WaterDivide`, so what stands in a lake counts as this side of it).
