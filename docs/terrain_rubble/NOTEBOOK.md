# Terrain Rubble: Lab Notebook

Experiments run on the rubble proof of concept ([DESIGN.md](DESIGN.md)), in the
order they were run. Problems they turned up are in [ISSUES.md](ISSUES.md).

Rules for this file:

- **Append only.** A result is never edited after the fact. If a later
  experiment overturns it, the later entry says so and links back.
- Each entry gives the **question**, the **setup** (commit, command, level or
  scenario, and anything temporary that was not committed), the **result** as
  numbers, the **conclusion**, and the **issues** it opened or closed.
- Commands are run from the repository root, in `--release`.

Counts are in samples, the lattice points of the voxel field: one sample is
0.125 m³ at 0.5 m voxels and 1 m³ at 1 m voxels. *Free* means standing free
(nothing holds it up) and *paper-thin* means drawn thinner than half a voxel,
both counted over the whole terrain by `TerrainWorld::loose_samples` and
`paper_thin_samples`.

---

## E1. Reproducing the floating strips and shelves (2026-10-02)

**Question.** Do random grenades really leave terrain floating and paper-thin,
and does Phase 1 remove it?

**Setup.** `rubble_viewer rim_cusps`: a flat field at 1 m voxels, 60 seeded
grenades (seed 1) at x, z in ±12 and y in −3..0.5, so their crater rims cross.
Run with lifting switched off, then on, at `477795b`.

**Result.**

| | Free after the run | Paper-thin after the run |
|---|---|---|
| Lifting off | 4 | 7 |
| Lifting on | 0 | 0 |

With lifting on, the run cuts 11–12 fragments of one sample each.

**Conclusion.** Reproduces the screenshots, and Phase 1 removes it on open
ground.

## E2. How far deposition moves the surface (2026-10-02)

**Question.** If a settled boulder is resampled back into the field at an
arbitrary pose, how much does its surface move?

**Setup.** `scratch/deposit_pop.py` and `deposit_pop_corrected.py`
(gitignored, `.venv` with scipy and scikit-image): three fragment shapes, 20
random poses each, marching cubes before and after.

**Result.** The table is in DESIGN.md Part 5, "How big the pop is: measured".
In short, trilinear resampling loses 6–9% of the volume, a constant bias. A
`+0.055` density offset removes the bias exactly (±0.003 across 60 runs). After
that, the mean surface error is 0.10–0.13 voxels, and only sharp tips move by
more than a voxel.

**Conclusion.** Deposition is viable, with `DEPOSIT_BIAS = 0.055`. Not built.

## E3. Search-region margin against cost (2026-10-02)

**Question.** How big a margin around the crater can the search afford?

**Setup.** `terrain_perf`, the `cut loose` stage, one grenade at 1 m voxels, at
different margins.

**Result.**

| Margin | Cut-loose cost per grenade |
|---|---|
| 1.5 crater radii | 0.08 ms |
| 4 crater radii | 0.25 ms |
| 32 voxels (the cap) | 5.8 ms |

**Conclusion.** Chose `clamp(3·radius, 6 voxels, 32 voxels)`. E12 and E13 later
showed that at 0.5 m voxels this is about 3 m, too small (R1).

## E4. What cut loose costs (2026-10-02)

**Setup.** `terrain_perf` at `477795b`, grenade sweeps at 1 m and 0.125 m
voxels.

**Result.** 0.31 ms a grenade at 1 m voxels, 0.32 ms at 0.125 m. The mesh
fingerprint is unchanged on sweeps that cut nothing loose.

## E5. What the carve costs at fine voxels (2026-10-02)

**Setup.** `terrain_perf --level scratch/fine.level.ron`: a flat 0.125 m
field, one grenade, before and after Phase 1.

**Result.** `blast::effective_radius` takes 343 ms a grenade, the same both
times.

**Conclusion.** An existing cost, not rubble's. Opened R8.

## E6. Sweeping `BEARING_DENSITY` (2026-10-02)

**Question.** How dense must a sample be to bear load?

**Setup.** Every `rubble_viewer` scenario at thresholds from 0 to 0.4.

**Result.** From 0 to 0.25, every scenario passes, and `rim_cusps` cuts 11–12
one-sample fragments. At 0.4, `rim_cusps` cuts 57 fragments of up to 8 samples
out of ordinary crater rims, and `arch_both_legs` no longer brings the span
down.

**Conclusion.** 0.25, the highest value that does not erode sound terrain.

## E7. Can the unit tests fail? (2026-10-02)

**Setup.** Disable each rule in `terrain/fragment.rs` in turn (holding at the
region edge, the bearing density, the sheet rule's reach, the island rules)
and run its unit tests.

**Result.** At first, several rules could be removed without any test
noticing. The fixtures were moved so that each depends on its rule: the deck
inside the region, a weak bar two samples thick rooted on a wall, islands
inside the region, and a new paper-flap island. Afterwards, every rule's
removal fails at least one of the 21 tests.

## E8. The water weir test after Phase 1 (2026-10-02)

**Setup.** `water_viewer`'s
`the_sea_pouring_back_over_a_lowland_s_weir_falls_along_its_far_side`, before
and after `477795b`.

**Result.** After Phase 1 the breach also clears paper-thin rim flaps. The
lowland fills in about 12 s instead of 24 s, and the backflow sheet forms at
2.5–3 s.

**Conclusion.** Moved the test's captures to `[2.5, 3.0, 4.0, 8.0, 16.0]`.
Opened R11.

## E9. The Rubble Garden as authored (2026-10-03)

**Question.** Does the play-test level start clean, with nothing free but the
authored island?

**Setup.** `rubble_viewer garden_hoodoo_stem`, reading the "as authored" line,
with the first version of `levels/rubble_garden.level.ron`. To see where the
free samples were, a temporary probe was added and then removed, never
committed: with `LOOSE_DUMP` set, `Search::audit` printed each free piece's
size and world bounds.

**Result.** 2,673 free and 737 paper-thin before any blast.

| Piece | Samples | What it was |
|---|---|---|
| floating island | 976 | authored, expected |
| big arch | 368 | the whole arch (R6) |
| pavilion roof | 150 | the roof's edges (R3) |
| cave pockets under the floor | 146, 56, 31, 17, … | free rock left by `Caves` (R10) |
| 2 m and 1 m slab walls | 140, 136 | the walls' edges (R3) |
| middle arch | 124 | the whole arch (R6) |
| table slab | 80 | the slab's edges (R3) |
| cliff-lip tips | 45, 20, 12 | lips tapering to nothing |
| thin arch | 40 | (R6, R9) |
| stalactites | 10, 8, 8, 6 | thinner than a voxel |

Densities across the 1 m wall: on its faces 0.01 (`SURFACE_BAND`), one bearing
row of 1.0 down its middle, and the faces' samples along every convex edge
touch the bearing row only diagonally.

**Conclusion.** Opened R3, R6, R10.

## E10. The arch with a true distance (2026-10-03)

**Setup.** As E9, after the `distance_to_arc` fix to `apply_arch`.

**Result.** Free 2,673 → 2,175; paper-thin 737 → 720. No arch is free as
authored. The synthetic `arch_both_legs` broke: its arch is now 6 m wide
instead of 4. Its 1 m blasts no longer sever the true legs, and the span then
reaches past the search region. With 1.5 m blasts, the span falls as a
92-sample piece and `arch_one_leg` still stands.

**Conclusion.** Fixed R6.

## E11. The cave hill (2026-10-03)

**Question.** Can `Caves` make a hill of struts and bridges to blast at?

**Setup.** `level_viewer levels/rubble_garden.level.ron --eye 50,12,44 --look
62,5,58`, with the cave thresholds varied.

**Result.**

| Depth curve (depth: threshold) | What it looks like |
|---|---|
| 0: 0.9, 1.5: 0.4, 6: 0.45 | a few holes at the surface |
| 0: 0.55, 1.5: 0.3, 6: 0.35 | the grass layer left as paper-thin strips over open caves |
| 0: 1.0, 0.8: 1.0, 1.6: 0.3, 7: 0.35 | a solid shell; three grenades open skylights |

Along the way `--blast` turned out to do nothing on a level without water.

**Conclusion.** Kept the shell. Opened R7 and R10.

## E12. What a grenade severs at 0.5 m voxels (2026-10-03)

**Question.** The garden's stations were sized by eye. What does a grenade
actually cut through?

**Setup.** `rubble_viewer` garden scenarios with real grenades
(`Blast::grenade`, the game's `BlastConfig::default()`), each placed on the
surface of its target.

**Result.**

| Target | Grenades | Severed? |
|---|---|---|
| columns 4, 3 and 1.5 m across | 1 each | no |
| columns 1 m and 0.7 m across | 1 each | yes; both stay up (R1) |
| stem 1 m across, under a 4 m cap | 1 | yes; the cap falls (233 samples) |
| 1.5 m-wide deck, 1 m thick | 2 at each end, on top | no |
| 1 m-wide deck, 1 m thick | 1 at each end | no |
| 1 m-wide deck, 1 m thick | 2 at each end | yes; the 4 m middle stays up (R1) |
| same deck, `Blast::sized` radius 1 m | 1 at each end | yes; the middle falls (47 samples) |
| cliff lip 1.2 m thick, 5 m long | 3 along its root | no |
| cliff lip 0.7 m thick, 4 m long | 3 along its root | comes away about 15 samples a grenade |

**Conclusion.** A grenade's crater here is about 0.6 m in radius. It spends its
budget per sample, so it shrinks with the voxel size (R2). That leaves the
search margin at its 3 m floor, so anything longer than about 3 m that is cut
free stays up (R1). The search itself is sound: given a 1 m blast, the deck
falls. Stations were resized to the grenade (1 m stem, 1 m short deck, 4 m-deep
lips, a short front row of columns), and those that cannot fall in the current
design were kept, so it is visible in play.

## E13. Every scenario, Rubble Garden included (2026-10-03)

**Setup.** `rubble_viewer all` at `f9007f4`. The garden starts with 2,133 free
and 718 paper-thin; the garden's 6 m bridge deck is 1 m wide, the other two
1.5 m.

**Result.**

| Scenario | Largest fragment | Outcome |
|---|---|---|
| `garden_tall_columns` | 3 | the 1 m and 0.7 m columns cut and left standing: free +98 (R1) |
| `garden_short_columns` | 21 | the 1 m and 0.7 m columns fall; the 1.5 m one is not severed |
| `garden_boundary_column` | 11 | falls |
| `garden_table_one_leg` | 4 | stands |
| `garden_table_every_leg` | 4 | slab cut free and left standing: free +451 (R1) |
| `garden_hoodoo_stem` | 233 | the cap falls |
| `garden_island_chip` | 1 | chipped, stays up |
| `garden_stalactite_root` | 8 | falls; paper-thin +11 (R4) |
| `garden_short_bridge` | 1 | deck cut free and left standing: free +33 (R1) |
| `garden_lip_root` | 15 | the 0.7 m lip comes away piece by piece |
| `garden_lip_underside` | 3 | invariants hold |
| `garden_hill` | 31 | 44 fragments over 30 grenades; blast 18 free +1 (R5) |
| `garden_fins_and_walls` | 9 | invariants hold; one grenade doesn't cut a wall through |

All 19 scenarios pass, with R1, R4 and R5 pinned as `known_gap`. The full
suite, `cargo test --release`, passes: 1,684 tests.

**Conclusion.** Opened R1, R4, R5. R1 is the one the design has to answer
before anything else.

## E14. Racing searches instead of a fixed margin (2026-10-03)

**Question.** If the search region grows to take in every bearing piece that
a race out of the crater closes off, do the pieces R1 left standing fall, and
does anything else change?

**Setup.** `f3563f3`: `terrain/split_race.rs`, called from `Crater::before_in`
(DESIGN.md "Growing the region"). `rubble_viewer all`, then `cargo test
--release`. The race's own unit tests check it on hand-built graphs, including
that it never walks the larger side of a cut.

**Result.** The four R1 known gaps stopped showing, as the suite reported by
failing them:

| Scenario | Before (largest fragment, change in free) | With the race |
|---|---|---|
| `long_bridge` | 20, free +298 | 465, free falls to 0 |
| `garden_tall_columns` | 3, free +98 | 86 (the 1 m column), free −4 |
| `garden_table_every_leg` | 4, free +451 | 547 (the slab, edges and all), free −96 |
| `garden_short_bridge` | 1, free +33 | 65, free −36 |

The unit test that pinned assumption A, a column 58 samples tall cut at its
foot, now drops the column as one 522-sample piece; it became
`a_column_cut_at_its_foot_falls_whole`. The garden's decks became 1 m wide (a
grenade cannot sever 1.5 m, E12), and a new `garden_long_bridge` drops the 28 m
deck's middle as a 461-sample piece. Every other scenario's result is
unchanged, including R4 and R5. All 20 scenarios pass, and so do the 1,690
tests.

**Conclusion.** Fixed R1. The rules that judge a piece are untouched; only
the region they read grew.

## E15. What the race costs (2026-10-03)

**Setup.** `terrain_perf --level <level> --blasts 40 --repeats 3`, at
`f3563f3` and again with `closed_off` returning `None` (the race off), a
temporary edit that was reverted.

**Result.** The `cut loose` stage, ms per grenade:

| Level | Race | Mean | p95 | Max | Fingerprint |
|---|---|---|---|---|---|
| `test_arena` | off | 0.339 | 0.445 | 0.958 | `3ee6e924…` |
| `test_arena` | on | 0.443 | 0.518 | 1.135 | `3ee6e924…` |
| `rubble_garden` | off | 0.300 | 0.334 | 0.373 | `e90c0b86…` |
| `rubble_garden` | on | 0.445 | 0.496 | 1.194 | `e90c0b86…` |
| `skyway` | off | 2.479 | 17.702 | 23.987 | `eab0fd0c…` |
| `skyway` | on | 2.660 | 18.676 | 24.568 | `eab0fd0c…` |

The fingerprints are the same with the race on and off: on these sweeps, which
cut nothing long free, it changes no terrain.

**Conclusion.** The race adds 0.1–0.2 ms a grenade on average, and up to
0.8 ms in the worst blast measured. `skyway`'s cost is there without the race:
opened R17. Not yet measured: a blast on a large authored island that is not
the largest piece racing, which the race walks whole, up to its budget.

## E16. Where R4's paper-thin samples are, and the thin arch (2026-10-03)

**Question.** The play-test showed one stalactite that already looks
paper-thin before any blast. Is that R4? And does R9 still show in the garden
now that `Arch` is fixed?

**Setup.** At `1f58fba`, two temporary probes that were then reverted:
- `PAPER_DUMP` made `Search::paper_thin_samples` print every paper-thin
  sample. `rubble_viewer garden_stalactite_root`, with the lists before and
  after the blast diffed.
- `probe_thin_arch` put two grenades at each foot of the garden's 0.8 m arch,
  at (81, 2.4, 84) and (88, 2.4, 84).

**Result.**
- R4: 11 samples are new, and none went away. All have density 0.01 (skin),
  at x 23.0–23.5, y 5.0–7.5, z 60.5–61.0: the 0.9 m stalactite, 2.3 m from the
  blast at (22.2, 7.4, 59.0), not the one the grenade hit. 22 stalactite
  samples were paper-thin as authored, including the 0.5 m stalactite, which
  is one voxel across and can only be drawn as a sheet. That is the one the
  play-test saw.
- Thin arch: the fragments were 10, 1, 37 and 0 samples across the four
  blasts, and the free count went 2,493 → 2,484: the arch falls, and nothing is
  left free. Blast 0 raised the paper-thin count from 718 to 720.

**Conclusion.** R4 follows from R3: lifting the free skin exposes skin behind
it as paper-thin, outside the paper rule's reach. R9 does not show in the
garden. The paper-thin strip the play-test saw is the sub-voxel stalactite,
there by design.

## E17. A rind is a lip across an edge or a corner (2026-10-04)

**Question.** If a weak sample that marching cubes draws whole may be a lip
through any of its 26 neighbours, do the R3 edge strips go, and does anything
that should fall stop falling or any flap come back?

**Setup.** At `75cdbd0` plus `Role::Rind` in `terrain/fragment.rs` (ISSUES.md
R3, Fix). `rubble_viewer all`, before and after. Two temporary probes, reverted:
`LIFT_DUMP` made `Search::lift` print every lifted sample, and `LOOSE_DUMP` made
`Search::loose_samples` print every free sample at each audit. `terrain_perf
--level levels/rubble_garden.level.ron`, before and after.

**Result.**

| Scenario | Free as authored, before → after | Fragments, before → after |
|---|---|---|
| `rim_cusps` | 0 → 0 | 12 → 11; still nothing left floating or paper-thin |
| `arch_both_legs` | 9 → 0 | 7 → 6, largest 92 → 95 |
| `arch_one_leg` | 9 → 0 | 4 → 3 |
| `long_bridge` | 194 → 0 | 17 → 11, largest 465 both |
| Rubble Garden (every `garden_*`) | 2,493 → 1,186 | as below |
| `garden_short_bridge` | | 19 → 15, largest 65 → 63 |
| `garden_long_bridge` | | 19 → 15, largest 461 → 459 |
| `garden_hill` | | 44 → 33, largest 31 → 2 |

- Paper-thin as authored is 718 in the garden and 24 on `long_bridge`, before
  and after: the edge strips were never drawn paper-thin.
- `garden_stalactite_root`, grenade at (22.2, 7.4, 59.0): before, 2 fragments,
  9 samples. `LIFT_DUMP` showed them all at density 0.01: 8 down the corner of
  the 0.9 m stalactite at x 23.0–23.5, z 60.5–61.0, and 1 at (21.5, 7.5, 58.0)
  on the skin of the 1.2 m one. After, nothing falls and paper-thin stays at
  718. Grenades at the roots: (21.5, 8.0, 59.0) drops nothing; (21.5, 7.6, 59.0)
  drops the 1.2 m stalactite, 3 fragments, largest 44; (25.0, 8.0, 59.0) and
  (23.5, 8.0, 61.0) drop 23 samples each. No invariant breaks at any of them.
- `garden_hill`: blast 18 left one more sample free before; after, blast 10
  leaves one more (1,186 → 1,187) and blast 18 two more (1,187 → 1,189).
  `LOOSE_DUMP` puts the new samples at (66.0, 2.5, 63.5), (66.5, 1.5, 64.0) and
  (66.5, 2.0, 63.5), density 0.02–0.04, drawn whole. Each joins a free piece of
  35–37 samples, none denser than 0.09, spanning x 65–67, y −0.5–2.5,
  z 61–64: into the caves, 3.7 m below the blast.
- `terrain_perf` on the garden: cut loose 0.442 → 0.438 ms mean, 1.194 → 1.214 ms
  worst. Mesh fingerprint `32799680603a21ed` both.
- `cargo test --release --lib`: every test passes once the two scenarios are
  re-pinned. Two new `fragment.rs` tests: `a_box_on_the_lattice_keeps_its_edges`
  fails with the rind lip limited to faces, and
  `a_thin_flap_touching_ground_at_an_edge_falls` fails with every weak sample
  allowed all 26 neighbours. `level_check::rest` fails before and after with the same output.

**Conclusion.** The rind lip removes R3 and with it R4, and `rim_cusps` shows
no flap coming back. E13's reading of `garden_stalactite_root` was wrong: its
grenade never dropped a stalactite, and its fragment was R3's strip. The
scenario now uses (21.5, 7.6, 59.0). The `garden_hill` breaks join a weak-only
piece the search cannot see the end of; they are R5, now at two blasts.

**Issues.** Closes R3, R4. Updates R5.

## E18. Where `skyway`'s cut-loose time goes, and what charging per m³ moves (2026-10-04)

**Question.** Which of `skyway`'s blasts pay R17's 24 ms, and in which part of
`cut_loose`? Separately, how big is R2's fix?

**Setup.** At `689c07b`. `terrain_perf --level levels/skyway.level.ron`, then
temporary probes timing the race, `Crater::before_in`, the post-carve read and
`Search::run`, reverted. After the fix: `terrain_perf` on `test_arena`, the
Rubble Garden and `skyway`, and `rubble_viewer all` against `689c07b`. For R2:
cost × step³ in `effective_radius`, `cargo test --release`, `rubble_viewer
all` and `terrain_perf` on the Rubble Garden and `island_sea`, reverted.

**Result.**

- Blasts #9 (92, 1, 29) and #16 (92, 4, 50) cost 25.6 and 19.9 ms in `cut
  loose` and cut nothing loose. Every other blast is 0.26–0.58 ms.
- At both, the race took 0.4 ms (229 samples) and closed nothing, so the
  region was the survey alone: 89 × 89 × 89 samples, against 17³ elsewhere.
  `Search::run` took 23.6 and 18.6 ms over it.
- 89 samples at 1 m voxels is a radius of about 12 m with the 32-voxel margin:
  the charges are in the open, and `effective_radius` pays for the nearest rock
  however far it is.

| Level | `cut loose` mean / worst, before | after | fingerprint |
|---|---|---|---|
| `test_arena` | 0.48 / 0.52 ms | 0.41 / 0.44 ms | `0d5349cdc69f039f`, unchanged |
| Rubble Garden | 0.49 / 1.25 ms | 0.45 / 1.28 ms | `32799680603a21ed`, unchanged |
| `skyway` | 2.86 / 26.1 ms | 0.48 / 1.08 ms | `48713b9006f02ac8`, unchanged |

- Sizing the box by the changed samples took #9 to 1.25 ms; clipping the race's
  seeds to them, 1.02 ms. What is left is a search over a cap that dirtied six
  chunks.
- `rubble_viewer all`: byte-identical report before and after.
- A new unit test, `a_charge_in_the_open_reads_only_around_what_it_cuts`, reads
  23 × 14 × 23 samples; with the old sizing it read 85³ and fails.
- R2 trial: the results are in ISSUES.md R2, "Size of the fix".

## E19. Craters independent of voxel size (2026-10-04)

**Question.** With the yield charged per cubic metre, what else makes a crater
depend on the voxel size, what does each part cost, and which scenarios move?

**Setup.** At `efa0e6e`. A unit test detonating `BlastConfig::default` on flat
sand and rock at 1, 0.5 and 0.25 m, at the surface and 4 m down. `terrain_perf`
on `scratch/fine.level.ron` (0.125 m), the Rubble Garden, `island_sea`,
`test_arena`, `thin_ice`, `wrecking_yard`. `rubble_viewer all`.

**Result.**

| Material, charge | Radius at 1 / 0.5 / 0.25 m |
|---|---|
| sand, surface | 1.42 / 1.66 / 1.77 |
| sand, buried | 1.42 / 1.59 / 1.54 |
| rock, surface | 1.01 / 1.01 / 1.03 |
| rock, buried | 1.01 / 0.87 / 0.90 |

- Per m³ alone, sand at the surface was 1.42 / 0.79 / 0.39 before it.
- The voxel probe at 1.5 voxels instead of 1.5 m changes these by nothing at
  0.5 m and 6–9% at 0.25 m on flat ground. It matters where the geometry is
  thin, where a 0.75 m probe reads a 1 m ledge as more buried than the 1 m
  lattice does.
- Sampled on the grid's own lattice, the 1.5 m probe made `detonate` take
  1,260 ms on the 0.125 m field. At 1 m spacing: 51 ms.
- Enclosure read only for the voxels paid for: `detonate` 1.20 → 0.16 ms
  (`thin_ice`), 1.21 → 0.18 ms (`wrecking_yard`), about 7 → 1.0–1.4 ms at
  0.5 m. 1 m fingerprints unchanged: `0d5349cdc69f039f`, `041cfc605329ca47`,
  `1c339fa57c62480d`.
- `cut loose` at 0.5 m: 0.4–0.5 → 0.9–1.7 ms, a deeper cut getting a wider
  margin. At 0.125 m: 41 ms (R8).
- Scenarios. `garden_short_bridge`: grenades at x 8.8 and 13.2 left 1.4 m of a
  6 m deck, largest piece 35; one grenade at each end (8.2, 13.8) drops 63.
  `garden_boundary_column`: the crater at y 2.6 left the top 1.4 m, largest 6;
  at y 1.6, 12. `garden_lip_root`: grenades at x 83.6 cut nothing on the third
  blast and left nothing free, largest piece 8. On the cliff top at x 84.4: 22,
  and at 84.8, inside the rock: 41. Taken: 84.4. `garden_tall_columns`, which
  failed in E18's trial, passes. `garden_hill`: nothing is left free any more;
  blasts 9 and 16 each leave one more sample paper-thin (R5).


## E20. Where `garden_hill`'s new paper-thin samples come from (2026-10-04)

**Question.** Which samples do blasts 9 and 16 of `garden_hill` leave
paper-thin (R5), and why does the paper rule miss them?

**Setup.** At `f3d13da`. The paper-thin audit printing every sample with its
six neighbours' densities, diffed blast to blast; probes of those samples'
roles in every pass of `cut_loose`.

**Result.**

- Blast 9 lifted something on all four passes (10, 5, 2, 1 samples). The
  fourth lifted the sample above (61.5, 6.0, 53.0), a 0.019 rind; no fifth
  pass ran, so it stayed, drawn 0.04 voxels thick. Raising the cap to 8 fixes
  blast 9 alone.
- Blast 16's two samples, (62.5, 4.0, 50.5) and (62.5, 4.0, 51.0), are 7 m from
  the crater. Pass 1 lifted a 96-sample piece beside them. Pass 2's region is
  the survey and what the race closes off, and the race runs on the grid as it
  is, so it no longer saw that piece: the region shrank by a voxel and the two
  samples sat on its face, held. Out of the paper rule's reach (crater radius +
  2 voxels) they would have been lips anyway.
- A first fix, paper rule on any sample face-adjacent to a lifted one, fixed
  both but unzipped authored paper: a column's lattice-face strip in
  `garden_tall_columns` lost one sample per pass up to the cap, and three
  blasts ran all 8 passes. Those strips were paper-thin as authored.
- Kept: the paper rule applies within the undercut as before, and anywhere
  else to a sample the blast made paper-thin (thick in the field before it,
  thin after). The region always takes in what earlier passes lifted, padded by
  3 samples. Over every scenario, searches per blast: 1 (55), 2 (55), 3 (19),
  4 (3), 6 (1); the cap is 8. `cut loose` and the fingerprints on `test_arena`,
  the Rubble Garden and `island_sea` unchanged.

## E21. Scree flown against the garden (2026-10-04)

**Question.** Once graded and launched by `RubblePlanner`, does every piece of
scree fall clear of where it broke and land, in every scenario?

**Setup.** `rubble_viewer all`, each blast followed by `TerrainWorld::update`
and every scree `Flight` stepped at 60 Hz against the remeshed terrain. Blast
shove: `Explosion::new(centre).physics_impulse()`. `render_perf` on the Rubble
Garden at (84.4, 50.5), the cliff lip, with synchronization validation.

**Result.**

- No piece expired: every one landed, 22–107 frames after it broke off,
  dropping 0.2–9.5 m.
- First version, any hit a landing: `garden_fins_and_walls` had 13 of 18
  pieces land on their first frame, 0.15 m from where they started (R18), and
  the table legs landed 0.3 m *above* where they broke, two frames in, thrown
  up into the slab. With walls and ceilings glanced off: no first-frame
  landing anywhere, every drop positive, the table legs land 3 m down.
- Most fragments are dust: of `garden_hill`'s 160, 18 are scree; the rest have
  no bearing sample, or are under 0.02 m³ (only at fine voxels: one sample at
  0.5 m is 0.125 m³).
- `render_perf`: draws go from 2 to 5 while three pieces fly and back to 2 once
  they land. `rubble_spawn` (grade, mesh, AO) 0.7 ms for the blast. No
  validation or synchronization messages. A snapshot 0.23 s after the blast
  shows the lip's grass-topped pieces in the air with the terrain's texture.
