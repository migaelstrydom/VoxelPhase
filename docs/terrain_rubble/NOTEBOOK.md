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
