# Stage 1.5 — level_check, schematic export, timing

**Read `docs/LEVEL_SEGMENTS_PLAN.md` first**, then `docs/level_segments/PROGRESS.md` for
what stage 1 actually landed and where it deviated. This brief holds only what stage 1.5
does.

Branch: `level-segments-stage-1-5` off `main` (stage 1 is merged). Merge before stage 2.

---

## Goal

Build the feedback loop. Every later stage — and every level anyone authors — currently
depends on someone launching the game and looking. This stage replaces that with mechanical
checks and a picture.

Three deliverables: a `level_check` binary, an SVG schematic exporter, and timing
instrumentation for terrain updates.

## Why it is worth doing before segments

Levels in this project have historically been mediocre not because the format was
expressive enough or the author careless, but because there was no way to see or verify a
level without playing it. Stage 2 introduces segment placement, where a transposed sign
puts an area 60 m underground and nothing says so. Building the check first means stages 2
onward have mechanical acceptance instead of visual guesswork.

---

## Deliverable 1 — `level_check` binary

`src/bin/level_check.rs`, registered in `Cargo.toml` alongside `bench_viewer`. No Vulkan,
no window, no ECS. Loads a level, generates terrain, reports.

```bash
cargo run --bin level_check -- levels/test_arena.level.ron
```

Exit non-zero if any **error** is found; warnings do not fail. Report:

### Terrain statistics

- Authored `Terrain.bounds` vs derived `TerrainManager::bounds()`.
- Total chunks, chunks containing solid voxels, and chunks allocated only as seam shell.
  Stage 1 found roughly half of `test_arena`'s 114 chunks are shell, so **chunk count is not
  a proxy for content** — report the split rather than a single number.
- Triangle and vertex counts.

### Mesh integrity

Count open edges (edges belonging to exactly one triangle). Stage 1 measured a baseline:
**218 open edges of 191 116 triangles** for `test_arena`, **22 of 98 948** for
`test_empty_terrain`, arising from ambiguous marching-cubes configurations in cave noise and
not from chunk seams. Report the count and compare against a committed baseline; a
significant rise is an error, since it means new cracks.

### Placement checks

- **Player spawn** — error if inside solid terrain; error if no terrain surface below it
  within a fall the player could survive.
- **Objects in rock** — error if an object's centre is inside solid terrain.
- **Objects in void** — warning if an object has no terrain beneath it within a generous
  distance. Deliberately a warning: dropping objects onto terrain is legitimate.

### Player reach

Report the derived jump envelope: standing jump, sprint jump and long jump, each as apex
height, airtime and flat range.

**Derive these from `PlayerConfig` and `PhysicsConfig::gravity` at runtime.** Do not
hardcode. Retuning the player must automatically change what every level is validated
against. The plan quotes ~2.50 m apex / ~7.1 m range for a standing jump at current
defaults — treat that as a sanity check on your derivation, not as a constant to enter.

These are point-mass figures ignoring `jump_cutoff_factor`, air steering and collider size,
so they are optimistic. State that in the output.

Gap-vs-reach validation between areas is **not** in this stage — there are no segments or
anchors to measure gaps between yet. That lands in stage 2.

---

## Deliverable 2 — schematic export

An SVG picture of a level, written without Vulkan so it runs anywhere:

```bash
cargo run --bin level_check -- levels/test_arena.level.ron --svg out/arena.svg
```

Two views in one document:

- **Top-down (xz)** — terrain surface height as a shaded heightmap, objects as labelled
  markers, player spawn marked, chunk boundaries as a faint grid.
- **Elevation (xy)** — a slice or a max-height profile, enough to read vertical structure.

The audience is someone who cannot run the game and needs to know whether a level looks
approximately as intended. Favour legibility over fidelity: axis labels and a scale bar
matter more than shading quality.

---

## Deliverable 3 — terrain update timing

Terrain destruction is perceptibly sluggish and it is unmeasured. Instrument
`TerrainManager::update()` with separate timers for:

1. **Remesh** — per-chunk marching cubes, plus how many chunks were dirtied.
2. **Buffer concatenation** — rebuilding the flat render vertex/index buffers.
3. **Adjacency rebuild.**

Report through `DebugLog` (written every frame, printed on F3) — that mechanism exists for
exactly this. Keep the overhead negligible when nothing is dirty.

Do **not** fix the performance problem. The point is to learn which term dominates: remesh
is O(chunks dirtied) and roughly constant with level size, whereas buffer concatenation is
O(total level triangles) and grows as levels get bigger. They imply different fixes and the
choice should not be guessed. Record the measured split in `PROGRESS.md` — that number
decides the eventual fix.

---

## Out of scope

- Segments, anchors, placement — stage 2.
- Gap-vs-reach validation between areas — needs anchors.
- **Any performance fix**: per-chunk draw calls, sub-chunk dirty regions, changing
  `CHUNK_VOXELS`. Measure only.
- The Vulkan viewer — stage 4.
- New terrain or traversal primitives.
- Hot reload.

## Acceptance

```bash
cargo build
cargo test
cargo test --release --features bench_harness
cargo run --bin level_check -- levels/test_arena.level.ron
cargo run --bin level_check -- levels/test_empty_terrain.level.ron
cargo run --bin level_check -- levels/test_arena.level.ron --svg /tmp/arena.svg
```

Both levels must pass with exit code 0 — if a shipped level reports errors, either the level
or the check is wrong, and say which in `PROGRESS.md`.

New tests required:

- Open-edge counting is correct on a hand-built mesh with a known number of open edges.
- A synthetic level with an object buried in terrain is reported as an error.
- A synthetic level with a valid object placement is reported clean.
- Derived jump reach matches a hand-computed value for a known `PlayerConfig`.

Attach the generated `arena.svg` output, or describe it, so the reviewer can judge
legibility without running it.

## On completion

Append to `docs/level_segments/PROGRESS.md`: what landed, deviations and why, **the measured
timing split** for a representative grenade, the committed open-edge baselines, and anything
stage 2 should know — particularly any assumption about how `level_check` will extend to
multiple segments.
