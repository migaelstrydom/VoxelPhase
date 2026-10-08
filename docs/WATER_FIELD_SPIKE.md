# Water as a Shallow-Water Field: Spike

Status: spike on branch `water-field`. The game still runs the hydrology network
(`WATER_HYDROLOGY_DESIGN.md`); the field runs beside it in `water_viewer --field`.

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
- **Load.** A pool fills its own hollow by Priority-Flood from the bottom of the
  hollow its seed lies in, and stops at the crest. A level with springs runs
  until its volume holds steady.

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
  split where two surfaces differ by more than 1 m;
- a curtain down any edge where they differ by more;
- a skirt over a dry bank the water reaches into.

Each vertex carries a signed depth, negative over a bank. The draw's wet range
discards below 5 mm, so a shoreline is a contour and not a staircase of columns.

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

## 7. Not done

- the game (`WaterWorld`, `WaterQuery`, buoyancy, splash coupling, the sleep
  tracker), which still sees only the network;
- the GPU heightfield path for drawing;
- parallel stepping (chunks are independent within a substep's phases);
- fall sheets;
- the network's removal.
