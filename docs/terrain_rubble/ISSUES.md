# Terrain Rubble: Issues

Problems found while building and running the rubble proof of concept
([DESIGN.md](DESIGN.md)). One section per issue, never renumbered. Experiments
cited as E*n* are in [NOTEBOOK.md](NOTEBOOK.md).

An issue's **status** is one of:

- **open**: not understood, or understood but not fixed;
- **pinned**: open, with a scenario that reproduces it (`known_gap` in
  `rubble_viewer`, so the suite stays green and fails when the issue goes away);
- **fixed**, with the commit;
- **won't fix**, with the reason.

| # | Issue | Status | Severity |
|---|---|---|---|
| [R1](#r1) | Pieces longer than about 3 m never fall at 0.5 m voxels | fixed `f3563f3` | blocks the design |
| [R2](#r2) | A grenade's crater shrinks with the voxel size | open | high |
| [R3](#r3) | Box edges on the lattice read as loose, paper-thin strips | open | medium |
| [R4](#r4) | A grenade at a stalactite's root leaves 11 more samples paper-thin | pinned | medium |
| [R5](#r5) | One cave-hill blast leaves one more sample standing free | pinned | low |
| [R6](#r6) | The terrain `Arch` drew its legs as slivers | fixed `f9007f4` | high |
| [R7](#r7) | `level_viewer --blast` did nothing on a level without water | fixed `f9007f4` | low |
| [R8](#r8) | The carve costs 343 ms a grenade at 0.125 m voxels | open | medium |
| [R9](#r9) | Thin curved tubes are joined to themselves only diagonally | open | medium |
| [R10](#r10) | `Caves` generates floating rock and paper-thin skins | open | medium |
| [R11](#r11) | Clearing rim flaps changed a water test's timing | open, for review | low |
| [R12](#r12) | A piece was judged held by its first sample alone | fixed `477795b` | high |
| [R13](#r13) | Authored floating islands would have fallen | fixed `477795b` | high |
| [R14](#r14) | Paper-thin flaps survived as "lips" | fixed `477795b` | high |
| [R15](#r15) | Lifting a piece left new flaps behind it | fixed `477795b` | medium |
| [R16](#r16) | The search stopped one voxel short of what the carve changes | fixed `477795b` | medium |
| [R17](#r17) | Cut loose costs up to 24 ms a grenade on `skyway` | open | medium |

---

<a id="r1"></a>
## R1. Pieces longer than about 3 m never fall at 0.5 m voxels

**Status:** fixed in `f3563f3` (E14). Was pinned by `garden_tall_columns`,
`garden_table_every_leg`, `garden_short_bridge` and the older `long_bridge`,
which now expect the pieces to fall. **Found by:** E12, E13.

**Symptom.** In the Rubble Garden, a 10 m column cut through at its foot, the
table slab after its fourth leg, and a 6 m bridge deck cut at both ends all stay
where they are, standing on nothing. The whole-terrain audit counts them as
free after the blast (the 6 m table slab adds 451 samples), but `cut_loose`
never lifts them.

**Cause.** The search region is the crater plus a margin of
`clamp(3·radius, 6 voxels, 32 voxels)`, and anything that reaches the region's
edge is held (assumption A). A grenade's crater at 0.5 m voxels is about 0.6 m
in radius (R2), so the margin is the 6-voxel floor: 3 m, about 3.6 m from the
blast. The design accepted assumption A for long bridges; it did not expect
that a 3 m piece is already "long".

**Options.**
- Grow the margin. E3 measured 5.8 ms a grenade at the 32-voxel cap, and a
  bigger box only moves the limit.
- *Proposed:* replace the box with a budgeted flood. Flood outward from the
  crater's shell over solid samples; a piece that reaches fixed ground, or
  exhausts a sample budget, is held, and one that closes off inside the budget
  falls. Cost then scales with the piece, not the crater, and assumption A
  becomes "a piece bigger than the budget is held".

**Fix.** Racing searches (`terrain/split_race.rs`, DESIGN.md "Growing the
region"), a refinement of the budgeted flood: searches start from every
bearing sample around the crater and advance one sample each per round, so the
pieces that close off are walked whole and the largest piece is never walked.
The region grows to take in every closed piece, and the existing rules judge
it. Assumption A is now: held at the region's edge only as the largest piece
racing, or past a 2¹⁸-sample budget.

<a id="r2"></a>
## R2. A grenade's crater shrinks with the voxel size

**Status:** open. Not introduced by rubble; rubble made it visible. **Found
by:** E12.

**Symptom.** A grenade at 0.5 m voxels cuts about 0.6 m deep. It does not sever
a 1.5 m column, a 1 m × 1 m deck, or a 1.2 m cliff lip.

**Cause.** `blast::effective_radius` spends `charge_yield` (12) per *sample*:
each sample costs `toughness × confinement`, with no factor for the sample's
volume. At 0.5 m voxels a grenade removes an eighth of the volume it removes at
1 m; at 0.125 m voxels, a 512th.

**Options.** Charge per cubic metre (cost × step³). That changes every
crater in every level at any resolution other than 1 m, so it needs a
play-test across the levels and a decision on what a grenade should cut.

<a id="r3"></a>
## R3. Box edges on the lattice read as loose, paper-thin strips

**Status:** open. **Found by:** E9.

**Symptom.** In the Rubble Garden as authored, every slab wall, deck, table and
roof has a strip of "free" samples running along its edges: 140 samples on the
2 m wall, 136 on the 1 m wall, 150 on the pavilion roof. Most of the level's
2,133 free and 718 paper-thin samples before any blast are these.

**Cause.** A face that lies exactly on lattice points writes density
`SURFACE_BAND` (0.01) there: a weak sample. On a flat face it is grounded as a
lip, because its inward neighbour bears. Along a convex edge, its only bearing
neighbour is diagonal, so it is neither bearing nor a lip, and the edge's weak
samples join into one free strip.

**Effect in play.** Small: when a blast reaches such a strip, the lifted samples
chamfer the edge by up to half a voxel and add one-sample dust puffs. It
pollutes every audit count, though, and hides real regressions in the noise.

**Options.**
- Let the lip rule accept an edge-diagonal (18-neighbourhood) bearing
  neighbour. Still one step only, and paper samples are still never lips. Must
  be rerun against `rim_cusps` to show it does not bring flaps back.
- Don't count a sample within `SURFACE_BAND` of the surface as matter at all.

<a id="r4"></a>
## R4. A grenade at a stalactite's root leaves 11 more samples paper-thin

**Status:** pinned (`garden_stalactite_root`). Not investigated. **Found by:**
E13.

**Symptom.** A grenade at (22.2, 7.4, 59) drops the thickest stalactite (an
8-sample fragment), and the paper-thin count rises from 718 to 729. That breaks
the invariant that no blast leaves more terrain paper-thin than before it.

**Suspects.** A stub left on the roof drawn thinner than half a voxel but
outside the paper rule's reach (crater radius + 2 voxels, about 1.6 m here),
or the neighbouring sub-voxel stalactites (R10's kind of authored geometry).

<a id="r5"></a>
## R5. One cave-hill blast leaves one more sample standing free

**Status:** pinned (`garden_hill`, blast 18 at (67.55, 2.17, 64.39)). Not
investigated. **Found by:** E13.

**Symptom.** Thirty seeded grenades into the cave hill. Blast 18 leaves one
more sample standing free than before it (2,131 → 2,132); every other blast
holds the invariant.

<a id="r6"></a>
## R6. The terrain `Arch` drew its legs as slivers

**Status:** fixed in `f9007f4`. **Found by:** E9, E10.

**Symptom.** Both large arches in the Rubble Garden were entirely free as
authored (368 and 124 samples), so the before/after rule treated them as
islands and cutting their legs brought nothing down.

**Cause.** `apply_arch` measured a point's offset from the arc *vertically*.
Where the legs turn steep, a point beside the leg is far from the arc in y, so
the legs came out as weak slivers. It also skipped everything past the span,
which cut off the outer half of each leg.

**Fix.** `distance_to_arc`: true distance to the half-ellipse (Quilez's
first-order ellipse distance, exact for a circle), and to the feet below the
springing line. No shipped level uses the terrain `Arch` (they use the
`VoussoirArch` object), so only the garden and the `visual_bench` terrain scene
changed. The synthetic arch in `arch_both_legs` became 6 m wide instead of 4, so
both arch scenarios now use 1.5 m blasts.

<a id="r7"></a>
## R7. `level_viewer --blast` did nothing on a level without water

**Status:** fixed in `f9007f4`. **Found by:** E11.

`LevelViewer::stir_water` returned before detonating when the level had no
`WaterWorld`. It now blasts any level and only stirs water when there is some.

<a id="r8"></a>
## R8. The carve costs 343 ms a grenade at 0.125 m voxels

**Status:** open. Not introduced by rubble. **Found by:** E5.

`blast::effective_radius` takes 343 ms a grenade on a flat 0.125 m field
(`terrain_perf --level` on a scratch level), before and after Phase 1 alike.
The search costs 0.32 ms at the same resolution (E4). If R2 is fixed by
charging per cubic metre, the crater at fine voxels grows and so does this.

<a id="r9"></a>
## R9. Thin curved tubes are joined to themselves only diagonally

**Status:** open; a limit of 6-connectivity. **Found by:** Phase 1
`rubble_viewer` arch scenarios.

A tube two voxels thick touches itself only diagonally where it curves, so the
audit finds parts of it standing free as authored, and the before/after rule
then treats it as an island: cutting its feet brings nothing down. The
garden's 0.8 m arch is there to show it. Related to R3: both are about weak
samples joined diagonally.

<a id="r10"></a>
## R10. `Caves` generates floating rock and paper-thin skins

**Status:** open; a generation problem the rubble audit exposes. **Found by:**
E9, E11.

- With a carve threshold below about 0.9 at depth 0, `Caves` hollows out the
  ground under the heightfield's top layer and leaves that layer as sheets one
  sample thick. These are the same zero-thickness shelves the project set out
  to remove, made by generation.
- Under the garden's floor it leaves pieces of rock free-floating inside the
  caves: 146, 56, 31 and 17 samples, among others.

The garden works around both with a solid shell (threshold 1.0 down to 0.8 m).
A generator that ran the same finder over its own output would remove them at
load.

<a id="r11"></a>
## R11. Clearing rim flaps changed a water test's timing

**Status:** open, for review. **Found by:** E8.

`the_sea_pouring_back_over_a_lowland_s_weir_falls_along_its_far_side` failed
after Phase 1. The breach also clears the paper-thin rim flaps, so the lowland
fills in about 12 s instead of 24 s. The backflow sheet the test looks for
still forms, but at 2.5–3 s, so its captures moved to `[2.5, 3.0, 4.0, 8.0,
16.0]`. Decide whether the test should instead keep the scenario's original
timing.

<a id="r12"></a>
## R12. A piece was judged held by its first sample alone

**Status:** fixed in `477795b`. `fallen` checked only whether the piece's first
sample had been held before the blast. A piece now falls if any of its samples
was held.

<a id="r13"></a>
## R13. Authored floating islands would have fallen

**Status:** fixed in `477795b`. Judged on the terrain after the blast alone, an
authored island near any blast is free and came down whole. The fix is the
before/after rule: `Crater::survey` reads the region before the carve. A piece
cut from something that was held up falls. Of something that already stood
free, the largest piece that can stand stays.

<a id="r14"></a>
## R14. Paper-thin flaps survived as "lips"

**Status:** fixed in `477795b`. The one-step lip that keeps a shelf's root on
the cliff was itself the paper-thin flap of the original screenshots. Samples
drawn thinner than half a voxel near the blast (`Role::Paper`) are never a lip
and join only each other.

<a id="r15"></a>
## R15. Lifting a piece left new flaps behind it

**Status:** fixed in `477795b`. Removing a piece can strip the last support
from a sample beside it. `cut_loose` now repeats search-and-lift up to
`MAX_PASSES` (4); no scenario has needed more than two.

<a id="r16"></a>
## R16. The search stopped one voxel short of what the carve changes

**Status:** fixed in `477795b`. The carve changes samples one voxel past its
radius, so the sheet and paper rules now reach crater radius + 2 voxels.

<a id="r17"></a>
## R17. Cut loose costs up to 24 ms a grenade on `skyway`

**Status:** open. Not introduced by the race: the same with it switched off.
**Found by:** E15.

`terrain_perf --level levels/skyway.level.ron`: the `cut loose` stage averages
2.5 ms a grenade, with a p95 of 18 ms and a worst blast of 24 ms. On
`test_arena` and the Rubble Garden it is 0.3–0.45 ms. Not yet profiled, and
the per-blast table does not show which of `skyway`'s blasts pay it.
