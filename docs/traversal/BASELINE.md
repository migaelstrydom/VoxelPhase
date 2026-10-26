# Traversal baseline (phase 0)

How the player crosses each station of `levels/traversal_garden.level.ron` before
phase 1 changes how the body is held up. Measured on `d6ddf39` (2026-10-16): the
capsule body, 0.25 m radius and 1 m tall, resting on the terrain and moved by the
traction drive. Walk asks 5 m/s, run 8 m/s.

## Reproduce

```bash
cargo build --release --bin character_viewer
SDK=~/software/VulkanSDK/1.4.309.0/macOS
VK_LAYER_PATH=$SDK/share/vulkan/explicit_layer.d DYLD_LIBRARY_PATH=$SDK/lib \
VK_ICD_FILENAMES=$SDK/share/vulkan/icd.d/MoltenVK_icd.json \
./target/release/character_viewer traversal --tiles 0 --summary
```

Each scenario takes a second or two; the run is deterministic. Drop `--summary`
and name one scenario for its frame-by-frame report (body position in the last
three columns); add `--tiles 8 --from 0.3 --to 0.6` for a filmstrip.

## The stations

Every lane runs along +x: start at x = -5, 12 m of flat run-up, station face at
x = 7 (bridges at x = 3), run-out to x = 35. The level file's header has the map.

| Lane z | Station | Judged across x |
|---|---|---|
| -12, -8, -4, 0, 4 | lips of 3, 8, 15, 25, 40 cm | 7..9 |
| 10 | the Test Arena's plank bridge, plank for plank (gaps 0.22-0.69 m) | 3..13 |
| 18 | a plank bridge with wider gaps (7 planks, gaps 0.35-0.88 m) | 3..13 |
| 26 | rough field: 40 craggy rocks standing 8-30 cm proud | 7..19 |
| 34, 42 | 40° and 50° slopes up to a shelf | 7..15, 7..13 |
| 50 | 35° scree: the slope strewn with rocks | 7..15 |
| 58 | a 0.7 m ledge (no mantle yet) | 7..9 |
| 66 | a cave lip: a roof 0.9 m up, 10 cm below the capsule's top | 7..19 |

The bridges lie on the ground between decks level with their planks (0.56 m), so
the lane is flush; the gap from deck to first plank is about 0.7 m on both. A plank
bridge lays its planks out from a hash of its position, which is why the lane-10
bridge stands at the Test Arena's (8, 0, 10).

Terrain is 0.25 m voxels, so every edge is bevelled across one voxel by marching
cubes: a "40 cm step" is a vertical face with a rounded top, not a sharp corner.
That is why it can be climbed here though the traction-drive bench's box step
stops the walker.

## Metrics (`src/character_viewer/traversal.rs`)

Everything is judged in a **window**: from the first frame the body is within 1 m
of the station's near end to the first frame more than 1 m past its far end.

| Column | Meaning |
|---|---|
| cross | the body ended the run past the station's far end |
| asked | planar speed the intent asked for (`resolve_ground_speed`) |
| min | lowest planar speed the body *travelled* at while pushing (displacement per frame, not solver velocity: a wedged body keeps a velocity it never moves at) |
| time/ideal | seconds from near end to far end, against the distance at the asked speed |
| snags | frames travelling more than 20 % below the asked speed while pushing (separate episodes in brackets) |
| air | frames the Support Set did not hold the body up / frames in the window |
| rise, drop | largest upward and downward body speed, m/s |
| rms_ay | RMS vertical acceleration of the body, m/s²; steady climbing contributes nothing |
| slip | largest horizontal move of a drawn foot in one frame while holding the same plant on a grounded body, m |
| slip_sum | total such slide, m |
| pops | a planted foot's anchor moving more than 5 cm with no step between |

Speeds are horizontal, so on a slope part of every "snag" is projection: running
5 m/s along a 40° face is 3.8 m/s across the ground. Rise on a slope is mostly
the climb rate. On the slopes read time/ideal and rms_ay rather than the snag
count and rise.

## Baseline

```text
scenario           cross asked   min  time/ideal    snags     air  rise  drop rms_ay  slip slip_sum pops
lip3_walk            yes  5.00  4.92  0.40/ 0.40   0( 0)   0/24   0.54  0.00    4.1 0.019    0.019    0
lip3_run             yes  8.00  7.87  0.23/ 0.25   0( 0)   4/15   0.86  0.78   10.6 0.006    0.006    0
lip8_walk            yes  5.00  4.57  0.40/ 0.40   0( 0)   8/24   1.40  1.27   13.3 0.000    0.000    0
lip8_run             yes  8.00  6.97  0.27/ 0.25   0( 0)   9/15   2.23  1.92   22.1 0.006    0.006    0
lip15_walk           yes  5.00  3.75  0.43/ 0.40   4( 1)   9/25   2.16  1.92   19.5 0.013    0.019    0
lip15_run            yes  8.00  4.76  0.30/ 0.25   2( 1)  11/17   3.61  2.58   34.5 0.000    0.000    0
lip25_walk           yes  5.00  1.07  0.47/ 0.40   9( 1)  10/28   2.66  1.92   21.6 0.017    0.038    0
lip25_run            yes  8.00  0.55  0.43/ 0.25  17( 1)  13/23   4.20  2.90   39.0 0.000    0.000    0
lip40_walk           yes  5.00  0.22  0.43/ 0.40  11( 1)  15/32   2.16  1.92   18.5 0.002    0.004    0
lip40_run            yes  8.00  0.14  0.50/ 0.25  20( 1)  14/31   2.89  1.27   23.1 0.016    0.016    1
planks_walk          yes  5.00  2.02  2.20/ 2.00  13( 3)  52/79   2.41  1.92   27.3 0.033    0.033    1
planks_run           yes  8.00  5.93  1.30/ 1.25   5( 2)  27/47   3.08  2.25   30.6 0.001    0.001    2
planks_wide_walk     yes  5.00  2.55  2.20/ 2.00  21( 4)  57/81   2.89  2.04   30.8 0.033    0.033    0
planks_wide_run      yes  8.00  6.02  1.30/ 1.25   1( 1)  32/48   3.32  2.25   31.7 0.000    0.000    0
rough_walk           yes  5.00  1.63  2.63/ 2.40  15( 2)  50/91   2.48  2.58   20.4 0.030    0.081    1
rough_run            yes  8.00  2.83  1.87/ 1.50  37( 4)  45/66   3.98  3.56   35.0 0.000    0.000    1
slope40_walk         yes  5.00  2.79  2.73/ 1.60  86( 1)   7/95   2.46  1.27    9.6 0.014    0.031    0
slope40_run          yes  8.00  4.64  1.70/ 1.00  57( 1)   5/61   3.94  0.61   19.4 0.030    0.106    0
slope50_walk         yes  5.00  1.53  2.93/ 1.20  95( 1)   9/103  2.48  1.59   10.0 0.009    0.061    0
slope50_run          yes  8.00  2.42  1.90/ 0.75  64( 1)  11/68   3.97  1.59   17.9 0.007    0.018    0
scree_walk           yes  5.00  0.82  3.30/ 1.60 103( 1)  38/113  2.49  1.65   16.5 0.021    0.100    0
scree_run            yes  8.00  0.81  2.60/ 1.00  82( 1)  31/87   3.85  1.59   27.9 0.020    0.083    0
ledge_walk            NO  5.00  0.00     -/ 0.40  86( 1)  25/106  1.82  1.65   12.0 0.001    0.002    0
cave_walk             NO  5.00  0.00     -/ 2.40 145( 1)   0/166  0.10  0.02    0.4 0.029    0.060    0
```

## Observations

| Scenario | What happens |
|---|---|
| lip3_walk | Even 3 cm throws a 0.54 m/s hop; no speed lost. |
| lip3_run | 0.86 m/s hop, 4 frames airborne; speed held. |
| lip8_walk | 1.4 m/s hop, 8 frames off the ground; loses 0.4 m/s. |
| lip8_run | 2.2 m/s hop and a 1.9 m/s landing; loses 1 m/s. |
| lip15_walk | First lip that snags: down to 3.75 m/s, 2.2 m/s hop. |
| lip15_run | 3.6 m/s hop (a jump's worth), down to 4.8 m/s. |
| lip25_walk | Nearly stops (1.07 m/s), then hops 2.7 m/s onto the deck. |
| lip25_run | Hits it hardest: 8 → 0.55 m/s, a 4.2 m/s launch; the worst rms_ay of all. |
| lip40_walk | Stops dead (5 → 0.2 m/s in one frame, velocity turned into a 1.6 m/s rise), then crawls up the bevelled edge in 0.4 s. |
| lip40_run | As walking, plus a foot pop as the plant re-anchors onto the deck. |
| planks_walk | Off the Support Set 52 of 79 frames. Three slowdowns: to 2 m/s at the 0.7 m deck-to-plank gap (x 3.5), to 2.8 m/s at the next gap, to 3.7 m/s at the exit gap; a hop of 1-2 m/s at every gap between. |
| planks_run | Holds speed better than walking (5.9 m/s low) but still airborne 27 of 47 frames; two foot pops. |
| planks_wide_walk | Four slowdowns, to 2.5 m/s; 70 % of frames airborne. |
| planks_wide_run | Running skims the wide gaps: one brief dip to 6 m/s. |
| rough_walk | 30 cm rocks stall it to 1.6 m/s twice; airborne half the time; 2.5 m/s hops. |
| rough_run | Four stalls (to 2.8 m/s) and 4 m/s hops; the bounciest crossing on flat ground. |
| slope40_walk | Climbs at 2.8-3.8 m/s horizontal (time 1.7× ideal); steady, little bounce. Rise is the climb rate itself. |
| slope40_run | Climbs at 4.6 m/s horizontal or better (1.7× ideal); rise is the climb rate (4.6 × tan 40°). |
| slope50_walk | Climbs, slowly (1.5 m/s low, 2.4× ideal). |
| slope50_run | Climbs, 2.4 m/s horizontal at worst; 2.5× ideal. |
| scree_walk | Gets up, but stalls to 0.8 m/s on the rocks; 2× ideal and a third of frames airborne. |
| scree_run | As walking: running buys little on scree (0.8 m/s low, 2.6× ideal). |
| ledge_walk | Stops at the wall and bounces against it (1.8 m/s rises, 25 airborne frames) for the rest of the run. |
| cave_walk | Wedged under the roof's edge at x = 7.16, pressed 4 cm into the ground and pitched 4°; the solver reports 1.5 m/s of velocity the body never travels. |

## Plan driving, at 60 Hz, from five starts

The game runs at about 60 Hz; the phase 0 table above was taken at 30. Each
station is crossed five times, from starts staggered back 0.11 m at a time
(`--starts 5`), because one crossing turns on where the frames and the
stride fall at the station. `cross` counts the crossings that got across;
the row is the worst one, and `best` is the best crossing's slowest speed.
Measured on `2e1749c` (2026-10-26), with the plan driving (`--track`):

```bash
./target/release/character_viewer traversal --tiles 0 --summary --track --starts 5
```

```text
scenario           cross asked   min  time/ideal    snags     air  rise  drop rms_ay  slip slip_sum pops  best
lip3_walk            5/5  5.00  4.91  0.40/ 0.40   0( 0)   0/48   0.54  0.00    5.3 0.004    0.006    0  4.92
lip3_run             5/5  8.00  7.84  0.25/ 0.25   0( 0)   8/30   0.86  0.78   13.9 0.005    0.006    0  7.88
lip8_walk            5/5  5.00  4.41  0.42/ 0.40   0( 0)  13/49   1.39  1.27   17.6 0.010    0.030    1  4.53
lip8_run             5/5  8.00  6.98  0.27/ 0.25   0( 0)  17/31   2.26  1.76   36.4 0.000    0.000    0  7.30
lip15_walk           5/5  5.00  3.39  0.47/ 0.40  14( 1)  16/52   2.16  1.59   25.5 0.010    0.033    1  3.65
lip15_run            5/5  8.00  5.02  0.33/ 0.25  19( 2)  20/36   3.56  2.25   51.1 0.000    0.000    0  5.87
lip25_walk           5/5  5.00  4.42  0.40/ 0.40   0( 0)  26/48   1.86  1.59   24.0 0.010    0.031    0  4.99
lip25_run            5/5  8.00  6.93  0.27/ 0.25   0( 0)  29/31   3.27  2.25   50.2 0.000    0.000    1  7.96
lip40_walk           5/5  5.00  4.14  0.42/ 0.40   0( 0)  31/49   2.47  1.76   28.3 0.011    0.031    0  4.74
lip40_run            5/5  8.00  6.80  0.27/ 0.25   0( 0)  29/31   3.45  2.25   53.7 0.000    0.000    1  7.95
planks_walk          5/5  5.00  4.82  2.00/ 2.00   0( 0)  96/144  1.70  1.90   26.5 0.000    0.000    0  4.95
planks_run           5/5  8.00  4.15  1.27/ 1.25  14( 1)  68/97   3.45  2.28   39.7 0.039    0.045    0  7.87
planks_wide_walk     5/5  5.00  4.66  2.00/ 2.00   0( 0) 111/144  1.58  1.69   29.1 0.004    0.004    0  4.95
planks_wide_run      5/5  8.00  0.53  1.40/ 1.25  20( 1)  88/108  2.64  2.95   43.9 0.131    0.189    1  7.81
rough_walk           5/5  5.00  3.16  2.72/ 2.40  40( 1) 119/187  2.75  2.44   28.3 0.046    0.168    1  4.66
rough_run            5/5  8.00  6.53  1.58/ 1.50   0( 0)  86/109  2.79  2.88   36.9 0.013    0.013    1  7.85
slope40_walk         5/5  5.00  1.89  2.77/ 1.60 179( 1)  13/195  2.46  1.27   12.5 0.012    0.140    0  2.60
slope40_run          5/5  8.00  3.07  1.72/ 1.00 115( 1)  13/123  3.99  1.27   26.8 0.013    0.034    0  4.63
slope50_walk         5/5  5.00  0.91  3.03/ 1.20 193( 1)   7/213  2.49  0.32   10.8 0.041    0.176    0  1.50
slope50_run          5/5  8.00  1.38  1.87/ 0.75 126( 1)  16/134  4.03  1.43   27.4 0.006    0.022    0  2.44
scree_walk           0/5  5.00  0.05     -/ 1.60 240( 1)  99/282  2.87  2.59   21.1 0.442    0.714    0  0.37
scree_run            3/5  8.00  0.52     -/ 1.00 175( 1)  82/213  4.18  1.63   26.9 0.079    0.256    1  1.44
ledge_walk           0/5  5.00  0.00     -/ 0.40 171( 1)  76/210  2.78  3.18   18.9 0.286    0.311    0  0.00
cave_walk            0/5  5.00  0.00     -/ 2.40 288( 1)   0/330  0.06  0.02    0.4 0.035    0.068    0  0.00
```

Read the gap between `min` and `best` before any single number: a plank
bridge crossed at a run turns from clean to a near stop on a 0.1 m shift of
the start.


## Milestone E closed: the plan against the bare drive

Measured on `a34d8f1` (2026-10-26), 60 Hz, five starts, with the plan driving
(`--track`) and without it. Time lost is the worst crossing's time less the
ideal, in seconds; `-` is a station not crossed on every start.

| Station | bare | plan | | Station | bare | plan |
|---|---|---|---|---|---|---|
| lip3_walk | 0.00 | 0.00 | | planks_walk | 0.18 | 0.00 |
| lip3_run | 0.00 | 0.00 | | planks_run | - (4/5) | 0.00 |
| lip8_walk | 0.00 | 0.02 | | planks_wide_walk | - (3/5) | 0.02 |
| lip8_run | 0.02 | 0.03 | | planks_wide_run | 0.40 | 0.00 |
| lip15_walk | 0.03 | **0.18** | | rough_walk | 0.22 | **0.48** |
| lip15_run | 0.05 | 0.10 | | rough_run | 0.42 | 0.00 |
| lip25_walk | 0.07 | 0.00 | | slope40_walk / run | 1.13 / 0.72 | 1.13 / 0.72 |
| lip25_run | 0.15 | 0.00 | | slope50_walk / run | 1.77 / 1.12 | 1.78 / 1.15 |
| lip40_walk | 0.10 | 0.00 | | scree_walk | 1.67 | **1.85** |
| lip40_run | 0.13 | 0.00 | | scree_run | 1.08 | **1.47** |

Every station the bare drive crosses, the plan crosses from every start; the
ledge and the cave neither crosses. Known issues, left open when E closed:
rough_walk, scree_walk, scree_run and lip15_walk lose more than the bare
drive (in bold), and the 8 and 15 cm lips at a run a few hundredths more.
Slopes and scree are drive-limited: the plan rides what the drive rides.

```text
scenario           cross asked   min  time/ideal    snags     air  rise  drop rms_ay  slip slip_sum pops  best
lip3_walk            5/5  5.00  4.91  0.40/ 0.40   0( 0)   0/48   0.54  0.00    5.3 0.004    0.006    0  4.92
lip3_run             5/5  8.00  7.85  0.25/ 0.25   0( 0)   8/30   0.86  0.78   13.9 0.005    0.006    0  7.88
lip8_walk            5/5  5.00  4.41  0.42/ 0.40   0( 0)  13/49   1.39  1.27   17.6 0.010    0.031    1  4.53
lip8_run             5/5  8.00  6.69  0.28/ 0.25   0( 0)  17/32   2.23  1.76   36.0 0.000    0.000    0  7.07
lip15_walk           5/5  5.00  2.00  0.58/ 0.40  31( 1)  22/59   2.17  1.84   23.9 0.028    0.066    1  3.64
lip15_run            5/5  8.00  5.03  0.35/ 0.25  26( 1)  20/37   3.56  2.25   50.4 0.000    0.000    0  5.50
lip25_walk           5/5  5.00  4.99  0.40/ 0.40   0( 0)  16/48   1.52  0.94   10.1 0.096    0.118    1  5.00
lip25_run            5/5  8.00  7.99  0.25/ 0.25   0( 0)  16/30   1.76  0.70   10.5 0.008    0.026    0  7.99
lip40_walk           5/5  5.00  4.99  0.40/ 0.40   0( 0)  16/48   1.77  0.52    8.3 0.097    0.119    1  5.00
lip40_run            5/5  8.00  7.86  0.25/ 0.25   0( 0)  23/30   1.12  1.27   23.9 0.000    0.000    0  8.00
planks_walk          5/5  5.00  4.81  2.00/ 2.00   0( 0)  89/145  1.95  1.98   24.4 0.061    0.185    3  4.99
planks_run           5/5  8.00  7.84  1.25/ 1.25   0( 0)  62/91   1.44  1.50   23.6 0.022    0.028    4  7.99
planks_wide_walk     5/5  5.00  4.79  2.02/ 2.00   0( 0)  88/145  1.76  1.84   24.3 0.054    0.152    2  4.98
planks_wide_run      5/5  8.00  7.84  1.25/ 1.25   0( 0)  66/90   1.97  1.96   28.7 0.073    0.079    2  7.99
rough_walk           5/5  5.00  2.51  2.88/ 2.40  76( 3) 138/197  2.72  3.60   31.3 0.036    0.071    2  4.87
rough_run            5/5  8.00  7.84  1.50/ 1.50   0( 0)  85/105  2.65  3.10   32.5 0.074    0.101    2  7.97
slope40_walk         5/5  5.00  0.81  2.73/ 1.60 196( 1)  26/207  2.46  2.09   14.0 0.011    0.131    0  2.63
slope40_run          5/5  8.00  3.06  1.72/ 1.00 114( 1)  12/122  4.06  1.10   28.0 0.013    0.033    0  4.64
slope50_walk         5/5  5.00  0.10  2.98/ 1.20 207( 1)  24/225  2.55  1.71   12.7 0.064    0.202    0  1.22
slope50_run          5/5  8.00  0.12  1.90/ 0.75 157( 1)  36/164  4.09  2.70   28.7 0.009    0.035    0  2.42
scree_walk           5/5  5.00  0.82  3.45/ 1.60 221( 1)  76/236  2.87  1.49   25.0 0.015    0.105    1  0.90
scree_run            5/5  8.00  0.64  2.47/ 1.00 176( 1)  62/183  4.11  1.62   36.7 0.033    0.114    2  1.48
ledge_walk           0/5  5.00  0.00     -/ 0.40 171( 1)  57/210  2.67  1.87   17.8 0.016    0.026    0  0.00
cave_walk            0/5  5.00  0.00     -/ 2.40 288( 1)   0/330  0.06  0.02    0.4 0.035    0.068    0  0.00
```

## Milestone F: the legs on the plan

Measured on the F commit (2026-10-26), 60 Hz, five starts, the plan driving.
Three columns are new: `off`, plants not on the floor under them, over all
plants; `miss` and `miss_max`, the mean and worst distance of a plant from
the plan's footfall for that foot (counted for footfalls due within 0.2 s).
Time lost is as above. "E" is the plan driving when E closed.

| Station | time E | time F | miss E | miss F | off F | | Station | time E | time F | miss E | miss F | off F |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| lip3_walk | 0.00 | 0.00 | 0.60 | 0.04 | 0 | | planks_walk | 0.00 | 0.00 | 1.28 | 0.01 | 0 |
| lip3_run | 0.00 | 0.00 | 0.33 | 0.01 | 0 | | planks_run | 0.00 | 0.00 | 1.20 | 0.01 | 0 |
| lip8_walk | 0.02 | 0.02 | 0.58 | 0.17 | 0 | | planks_wide_walk | 0.02 | 0.02 | 0.73 | 0.01 | 0 |
| lip8_run | 0.03 | 0.03 | 1.00 | 0.04 | 0 | | planks_wide_run | 0.00 | 0.00 | 1.75 | 0.02 | 0 |
| lip15_walk | 0.18 | 0.20 | 0.72 | 0.12 | 0 | | rough_walk | 0.48 | 0.27 | 0.65 | 0.04 | 0 |
| lip15_run | 0.10 | 0.10 | 0.52 | 0.07 | 0 | | rough_run | 0.00 | 0.02 | 1.71 | 0.00 | 0 |
| lip25_walk | 0.00 | 0.00 | 0.51 | 0.12 | 0 | | slope40_walk / run | 1.13 / 0.72 | 1.13 / 0.72 | 0.18 / 0.28 | 0.08 / 0.08 | 0 |
| lip25_run | 0.00 | 0.00 | 0.40 | 0.13 | 0 | | slope50_walk / run | 1.78 / 1.15 | 1.78 / 1.12 | 0.45 / 0.85 | 0.09 / 0.04 | 0 |
| lip40_walk | 0.00 | 0.00 | 0.56 | 0.11 | 0 | | scree_walk | 1.85 | 1.75 | 0.26 | 0.01 | 0 |
| lip40_run | 0.00 | 0.02 | 0.50 | 0.09 | 0 | | scree_run | 1.47 | 1.12 | 0.30 | 0.16 | 0 |

When E closed the plan's footfalls were somewhere the feet were not: the
legs stepped on their own capture point, 0.3 to 1.75 m from the plan, and
its hops came down with a foot replanted under the hips. Now the plan says
where and the legs say when (`PLANNER_DESIGN.md`, *The plan*): a foot lifts
on the gait's clock and lands on the plan's footfall, a planned flight is
the landing foot's swing, and the plan starts from the feet the legs have
down. Every plant is on its footing, the worst miss on the planks is 0.16 m,
and foot slip is the bare drive's. No station lost time against E; rough_walk
and scree gained some. The first attempt, with the plan's clock lifting the
feet, halved the stance at a run and put both feet ahead of the torso; it
also found the planner planning 0.62 m steps against the legs' 0.73, from a
reach rule that judged both feet from a body midway between them.

Open: at 8 m/s the legs plant only about 0.09 m ahead of the hips (the swing
is at its 0.09 s floor), so their stance is asymmetric and the planner's
symmetric stance model is 0.2 m off; a fresh plan from a planted foot
beyond leg reach refuses the push-off, which a kept commitment usually
hides.

```text
scenario           cross asked   min  time/ideal    snags     air  rise  drop rms_ay  slip slip_sum pops  best     off  miss miss_max
lip3_walk            5/5  5.00  4.91  0.40/ 0.40   0( 0)   0/48   0.54  0.00    5.3 0.031    0.103    0  4.92    0/28  0.04     0.47
lip3_run             5/5  8.00  7.84  0.25/ 0.25   0( 0)   8/30   0.86  0.78   13.9 0.011    0.023    0  7.89    0/26  0.01     0.10
lip8_walk            5/5  5.00  4.48  0.42/ 0.40   0( 0)  13/49   1.40  1.27   17.7 0.048    0.192    0  4.53    0/20  0.17     0.60
lip8_run             5/5  8.00  6.40  0.28/ 0.25   0( 0)  17/33   2.23  1.76   35.5 0.008    0.008    0  6.90    0/16  0.04     0.17
lip15_walk           5/5  5.00  1.74  0.60/ 0.40  31( 1)  22/60   2.19  1.84   23.8 0.035    0.143    0  3.74    0/23  0.12     0.48
lip15_run            5/5  8.00  4.85  0.35/ 0.25  26( 1)  19/38   3.47  2.25   48.9 0.049    0.122    0  5.11    0/18  0.07     0.43
lip25_walk           5/5  5.00  4.99  0.40/ 0.40   0( 0)  18/48   1.84  0.94   10.3 0.032    0.107    0  4.99    0/21  0.12     0.48
lip25_run            5/5  8.00  7.99  0.25/ 0.25   0( 0)   9/30   0.75  0.56    8.1 0.090    0.149    0  7.99    0/18  0.13     0.46
lip40_walk           5/5  5.00  4.99  0.40/ 0.40   0( 0)  15/48   1.78  0.51    7.6 0.034    0.167    0  5.00    0/19  0.11     0.46
lip40_run            5/5  8.00  7.52  0.27/ 0.25   0( 0)  25/30   1.90  1.76   36.3 0.000    0.000    0  7.99    0/18  0.09     0.49
planks_walk          5/5  5.00  4.83  2.00/ 2.00   0( 0)  87/144  1.90  2.02   24.6 0.034    0.199    0  4.98    0/47  0.01     0.11
planks_run           5/5  8.00  7.84  1.25/ 1.25   0( 0)  61/91   1.46  1.65   24.0 0.045    0.230    0  7.99    0/43  0.01     0.11
planks_wide_walk     5/5  5.00  4.79  2.02/ 2.00   0( 0)  86/145  1.73  1.87   24.2 0.036    0.200    0  4.98    0/51  0.01     0.10
planks_wide_run      5/5  8.00  7.56  1.25/ 1.25   0( 0)  64/90   2.02  2.09   30.6 0.047    0.127    0  7.98    0/34  0.02     0.16
rough_walk           5/5  5.00  1.27  2.67/ 2.40  38( 1) 116/184  2.82  3.25   30.5 0.023    0.097    0  4.94    0/53  0.04     0.64
rough_run            5/5  8.00  7.67  1.52/ 1.50   0( 0)  89/105  2.65  2.71   31.9 0.000    0.000    0  7.95    0/33  0.00     0.05
slope40_walk         5/5  5.00  2.56  2.73/ 1.60 180( 1)  16/195  2.46  1.76   15.0 0.031    0.249    0  2.72   0/151  0.08     0.31
slope40_run          5/5  8.00  3.06  1.72/ 1.00 115( 1)  13/123  4.06  1.27   27.9 0.013    0.053    0  4.35   0/112  0.08     0.15
slope50_walk         5/5  5.00  0.10  2.98/ 1.20 207( 1)  24/225  2.55  1.71   12.7 0.010    0.109    0  1.50   0/156  0.09     0.24
slope50_run          5/5  8.00  1.38  1.87/ 0.75 131( 1)  16/139  4.03  1.43   26.9 0.019    0.041    0  2.44   0/148  0.04     0.25
scree_walk           5/5  5.00  0.77  3.35/ 1.60 235( 1)  92/248  2.90  2.58   26.7 0.012    0.104    0  0.90   0/129  0.01     0.05
scree_run            5/5  8.00  1.29  2.12/ 1.00 143( 1)  46/151  4.13  1.43   42.4 0.149    0.234    0  1.48   0/108  0.16     0.41
ledge_walk           0/5  5.00  0.00     -/ 0.40 171( 1)  57/210  2.67  1.87   17.8 0.002    0.003    0  0.00    1/96     -        -
cave_walk            0/5  5.00  0.00     -/ 2.40 288( 1)   0/330  0.06  0.02    0.4 0.006    0.066    0  0.00   0/158     -        -
```
