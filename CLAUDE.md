# Instructions to coding assistants

## Build Commands

```bash
cargo build                      # debug
cargo build --release
cargo build --profile dist       # release + fat LTO (slow to link), output in target/dist/; to share or ship
cargo run                        # the game (default-run in Cargo.toml)

cargo test --release             # unit tests, every area; run them in release, which is far faster than debug
cargo test --release --features physics_scenarios   # + an area's scenario suite: whole scenes simulated for seconds
cargo test --release --all-features --no-run        # after an API change: compile every suite
```

Shaders (GLSL under `shader/`) are compiled to SPIR-V by `build.rs` on every build that touches `shader/`. It finds glslc via `$GLSLC`, `$VULKAN_SDK/bin`, or PATH.

When running in Cursor, prepend `env -u RUSTUP_HOME -u CARGO_HOME` to cargo commands.

A Python venv exists at `.venv/` (gitignored); use `.venv/bin/python3` and `.venv/bin/pip`. `scratch/` (gitignored) is for temporary scripts, data and throwaway files.

### Test suites

Unit tests (one component, milliseconds) always run. Each area's scenario tests, which simulate whole scenes for seconds, sit behind a Cargo feature named for the area: `physics_scenarios` (the bench harness), `water_scenarios` (the water viewer's scenarios and the tests that open water levels), `rubble_scenarios` (the rubble viewer's blasts), `level_scenarios` (checks over every shipped level) and `traversal_scenarios` (the Traversal Garden's stations captured and planned against their authored twins, and Stack Yard places that planned wrongly in play). Narrow any run by module path, e.g. `cargo test --release --features water_scenarios water::`. A new test that simulates a scene for seconds goes in its area's suite, in a module gated `#[cfg(all(test, feature = "<area>_scenarios"))]`.

### Tools: see and measure instead of guessing

The game window cannot be watched from an agent shell, so these tools are how you check work. Each binary's `//!` header in `src/bin/<name>.rs` documents its usage with examples; read it before running the tool. Perf benches are always `--release`.

| Tool | Use it to |
|---|---|
| `level_check <level>` | Validate a level offline (no Vulkan): placement, reach, open mesh edges (`--mesh-edges`), SVG schematic (`--svg`). It also wakes every object and reports what doesn't stay at rest. All levels: `cargo test --release --lib --features level_scenarios level_check`. |
| `level_viewer <level>` | Look at a level as authored, through the real pipeline, to a PNG. Interiors need `--eye x,y,z --look x,y,z`; `--blast/--splash/--run` blast or stir the water and run it first. |
| `visual_bench <scene>` | Render bench scenes (materials, props, terrain forms) headlessly with the real shaders. |
| `anim_viewer <scenario>` | Gait on analytic or moving ground, judged against the foot placer's plan; `--no-render` needs no GPU. |
| `character_viewer <scenario>` | The real game headless with a scripted pilot: anything the physics body itself does (swimming, falling in, coming ashore). `traversal --summary` measures crossing the Traversal Garden's stations (lips, plank bridges, rough ground, slopes, scree, a ledge, a cave lip): speed lost, snags, bounce, foot slip, whether it got across; baseline in `docs/traversal/BASELINE.md`. `contract` measures the traction drive against the traversal planner's `GaitParams` (slope speed, acceleration, takeoff, coyote, ride) and fails on any row that does not hold. `--track` has the plan drive the character (F8 in game), and `--slowing never|avoid|cost` sets when the plan may slow it (F10 in game). |
| `bench_viewer <scenario>` | Physics bench scenarios in a window (the user runs this, not the agent). |
| `physics_fuzz` | Seeded structures knocked down by the real player body; checks energy, floor and finiteness. |
| `physics_perf` | Where a frame's physics time goes (`FrameProfile` stages). |
| `terrain_perf` | What a grenade costs the terrain, stage by stage, plus a mesh fingerprint (a speed-only change must leave it unchanged). |
| `rubble_viewer <scenario>` | Scripted blasts on synthetic terrain and the Rubble Garden: what each cut loose, audited for anything left standing free or paper-thin, every piece of scree flown until it lands, and every boulder stepped as a body until it sleeps. Times each blast's carve, remesh and plan, and a physics frame of its pile at rest. `cargo test --release --lib --features rubble_scenarios rubble_viewer` runs them all. |
| `render_perf` | A frame of the real game during a blast: CPU stages, GPU spans, draw counts, slowest systems. |
| `water_viewer <scenario>` | Scripted water scenarios on synthetic terrain, on the field the game uses (`docs/WATER_FIELD_SPIKE.md`): probe levels, volume, ledger, filmstrip. Its tests are `cargo test --release --lib --features water_scenarios water_viewer`. |
| `water_perf`, `water_fuzz` | The field's CPU cost per frame on every water level, opening included; random blasts checking that the field holds together (depths, ledger, speeds, coming to rest). |
| `section_svg <scene.ron>` | The traversal planner on a scene (`--time` for plan times, `--speed v` to plan it at another speed, `--phases` to list the plan's rides, hops and roll-offs) (`crates/traversal_planner`, `-p traversal_planner`): side view over a top-down strip of the lanes, footholds coloured by footing, the body floor and ceiling, and the plan (body path, flights dashed, footfalls by foot) or, for a refusal, how far it got in red. Prints each scene's outcome against the one it expects. Needs no GPU. Scenes and fixtures in `crates/traversal_planner/fixtures/`; tests are `cargo test --release -p traversal_planner`. |
| `plan_fuzz` | Random traversal scenes planned and checked against every planner property (`crates/traversal_planner`, `-p traversal_planner`); `--write <dir>` shrinks each failing scene to a fixture. A 300-seed sweep runs with the planner's tests. |
| Traversal Planner Bench | The planner in a browser (`crates/traversal_planner_web`), published as a private artifact: drag obstacles, tune gait and costs, scrub time with loose footing giving way, import and export scenes. Rebuild with `cargo build -p traversal_planner_web --target wasm32-unknown-unknown --profile dist` and republish `page/index.html` with that `.wasm` as `planner.wasm` beside it. Its tests: `cargo test --release -p traversal_planner_web`. |
| `traversal_perf <level>` | What traversal planning costs a frame in the game's physics world (no GPU): a body walked along a heading following its plan, each frame's casts, strip sampling and planning timed, the first frame (a path laid) apart; `--blast` keeps rubble moving, `--verify` checks the kept strip against a fresh one. |
| `section_capture <level>` | A settled physics scene captured as a traversal planner fixture: the level spawned headless (no GPU), optionally blasted, left to come to rest, and sampled into the planner's strip from `--at x,z --heading deg` through `PhysicsCaster` (`src/traversal/`). Writes a scene file `section_svg` reads; the command that made it heads the file. |
| `explosion_preview` | The explosion's particle choreography on a CPU rasteriser, to a PNG. |

The same stage timings print in game on F3 under `Physics/Time/`, `Terrain/LastUpdate/`, `Render/` and `Systems/Slowest/`.

**Headless Vulkan from an agent shell.** `vulkan_context.rs` always requests `VK_LAYER_KHRONOS_validation`, so tools that render fail with `ERROR_LAYER_NOT_PRESENT` unless the shell is unsandboxed and the SDK environment is set:

```bash
SDK=~/software/VulkanSDK/1.4.309.0/macOS
VK_LAYER_PATH=$SDK/share/vulkan/explicit_layer.d DYLD_LIBRARY_PATH=$SDK/lib \
VK_ICD_FILENAMES=$SDK/share/vulkan/icd.d/MoltenVK_icd.json ./target/release/render_perf ...
```

To check GPU synchronization, add `RUST_LOG=warn VK_LAYER_ENABLES=VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT`; hazards print to stderr.

## Development Standards

This project will likely grow much bigger than it currently is. For this reason, the emphasis should be on proper abstractions, modularity and maintainable code. The lead maintainer is a former Amazon Senior Development Engineer with 10+ years of experience and high standards, so try not to disappoint him.

Some specific requirements:
- mod.rs is for `mod`/`use` statements and the module's `//!` header only — no structs or other code.
- Always do imports at the top of the file.
- Every top-level module's `mod.rs` opens with a `//!` line saying what the module is. Write one for a new module, and update it when a module's role changes.
- A serde default that repeats a type's own default is inert and misleads the reader; delegate to the type's default or name the constant.
- An unused parameter or a zero-valued field is not necessarily a bug. Check whether it is deliberate before "fixing" it.

### Coding Style Preferences

- Favor small, composable components with clean APIs (e.g., `NormalSmoother`, `NormalClusterer`).
- Move configuration into the component that owns it instead of growing global config structs.
- Keep data flows easy to visualize with a potential ASCII component diagram.
- Use clear, field-level doc comments for new struct fields.
- Prefer engineering-style structure and naming over math-heavy, inline logic.
- Use the SOLID principles when possible.
- When asked for a design, think in terms of high-level architecture first; only then design the components.

## Architecture Overview

Voxel Phase is a Vulkan-based 3D engine and game written in Rust, using `ash` for Vulkan bindings and `specs` for ECS.

The crate is a library (`src/lib.rs`, all engine modules) plus binaries: the game (`src/main.rs`) and the tools in `src/bin/`, each a thin front end over a library module of the same name (`src/level_viewer/`, `src/physics_perf/`, ...).

### Module map

Every top-level module's `mod.rs` opens with a one-line `//!` summary. List them all with:

```bash
grep -m1 -H '^//!' src/*/mod.rs
```

Design docs for the larger subsystems are in `docs/` (e.g. `PHYSICS_ARCHITECTURE.md`, `WATER_FIELD_SPIKE.md`, `LEVEL_SEGMENTS_PLAN.md`, `SWIMMING_PROJECT.md`, `VISUAL_DIRECTION.md`). Check the relevant one before changing a subsystem's design.

How the major pieces connect, which no single module states:

- **Terrain → physics:** physics sees terrain only through the `StaticGeometry` trait (triangle patches by region). Terrain is sparse chunked SVOs grouped into `Segment`s with their own frames; segments are invisible outside `src/terrain/`.
- **Terrain → rubble:** `TerrainWorld::detonate` carves, then lifts out whatever the blast cut loose (`terrain/fragment.rs`) and returns it as `Fragment`s with world poses; `src/rubble/` decides what becomes of them. Callers that ignore the fragments still see them gone from the field. Design and open issues: `docs/terrain_rubble/`.
- **Water → everything else:** `WaterWorld` is the façade and ECS resource, over the shallow-water field (`water/field/`): depths in cells, discharges on faces; an edit re-pairs the columns it touched and carries their water. Physics and gameplay see water only through `WaterQuery`; buoyancy, splash coupling and the sleep tracker read it through the `WaterSurface` trait, which also gives the falling water (`water/falling/`: falls and spring jets) striking a body. The renderer draws falls and jets from the same lip rules and paths.
- **Suspension → force field:** a `ForceFieldVisual` on an entity shows any of its suspensions as a field (`src/force_field/`), read from `PhysicsWorld::suspension` readouts every frame: emissive rings (opaque, so they bloom), columns drawn as light by the `force_field` shader in the sorted blended pass (`BlendedShading::ForceField`), kick-up particles and an underglow light. The hover sled (`src/sled/`, spawnable `HoverSled`) is its first wearer.
- **Intent seam:** the player's input and creature brains both write `CharacterIntent`; locomotion systems (walking, rolling, swimming, legged rigs) read it without knowing who wrote it.
- **Traversal planning:** how the player crosses broken ground (and later grabs ledges and climbs) is being rebuilt as a pure planner over a strip of ground ahead (`crates/traversal_planner`), which the body tracks through its contacts; the game reads its physics world into the planner's path coordinates in `src/traversal/`. Design and milestones: `docs/traversal/PLANNER_DESIGN.md`.
- **Bodies vs their bulk:** a body's colliders are its contact envelope. Where that is not the body (a character's capsule, a sealed barrel), `RigidBodyDesc::bulk(BulkShape)` declares what weighs it and what fluids see.

### Key Patterns

- **RAII with Drop**: all GPU resources clean up automatically when dropped.
- **Arc<Managed*>**: Vulkan handles wrapped for shared ownership across threads.
- **Component-driven**: all behaviour is defined through components and systems; no god objects. Every system is wrapped in `Timed` by `TimedDispatcherBuilder`.
- **Thread-local render**: `RenderSystem` is thread-local and runs last in the dispatcher. It is the only ECS coupling of the renderer.

### Rendering

`Renderer` (`src/rendering/renderer.rs`) is a thin Vulkan orchestration layer with no ECS dependencies, delegating to per-pass renderers under `src/rendering/`. Scene descriptor set 0 holds the scene UBO, lights, shadow map, surface table, grain texture and reflection probes; set 1 holds the material texture (bindings listed in `src/rendering/descriptors.rs`).

Two frames are in flight (`src/rendering/in_flight/`): the CPU records one while the GPU draws the previous one. Anything the CPU writes while recording (mesh and uniform buffers, the surface table, the scene descriptor set, command buffers, the fence, the GPU timer) has one copy per slot (`PerFrame<T>`, `InFlightFrame`), and a slot is rewritten only after its fence is waited on. GPU-only images (depth, HDR target, shadow map) are shared and ordered by the render passes' external dependencies. A descriptor set is never rewritten while a frame may be using it, and a released texture is retired until the frames that might sample it have finished.

Frame profiling (`src/rendering/profile/`): CPU time per `RenderStage`, GPU time per `GpuSpan` from timestamp queries (bounded at render-pass edges only, because a tiled GPU cannot split a pass), and deterministic `RenderCounters`. A frame's profile is published two frames late; `RenderProfile::frame` says which frame it is.

A mesh whose shape changes every frame (a trampoline's bed) is a `ProceduralMesh` component: its owning system rebuilds it, and `RenderSystem` uploads and draws it in its entity's frame with its own material.

Shaders: `build.rs` compiles every `.vert`/`.frag`/`.comp` under `shader/` to `$OUT_DIR/shader/<path>.spv` (with `-I shader`, so shared `.glsl` includes work), and `embedded_spirv!("<path>")` (`src/rendering/shaders.rs`) embeds it. A new shader needs no build change, only the `embedded_spirv!` at its use site. `shader/swell.glsl` mirrors `src/water/surface/swell.rs`, and a test holds them together.

### Physics Engine

Self-contained rigid-body physics in `src/physics/`, handle-based, with no ECS dependency. `docs/PHYSICS_ARCHITECTURE.md` describes the pipeline. Points to know before changing it:

- The narrowphase runs once per frame, but integration substeps up to 12 times (1/240 fixed). CCD and speculative contacts are sized for the whole frame.
- The solver is pluggable (`ConstraintSolver`): PGS + NGS with shock propagation (a `ManifoldConditioner`). Joint constraints live in an arena and expand to `ConstraintRow`s each frame.
- A body's origin is its centre of mass. Re-centre after detaching colliders.
- Every body starts asleep. Apply impulses through `PhysicsWorld` methods, which wake the body.

`src/physics/bench_harness/` holds scenario-based end-to-end tests (`PhysicsBenchScenario`), run headlessly by `run_scenario()` and in a window by `bench_viewer`.

### Debug Output

Three ECS resources in `src/debug.rs`, all cleared every frame:

- `DebugLines::add(key, value)`: on-screen text, sorted by key.
- `DebugOverlays::add_sphere/add_line/add_point`: 3D shapes in the scene, drawn by `src/rendering/debug_render.rs` (reusable by standalone tools).
- `DebugLog::add("Area/Key", value)`: printed to stdout only when **F3** is pressed. Use it for detailed statistics that would clutter the screen.

F3 also writes a snapshot to `debug_snapshots/` (gitignored): the next frame's screenshot and that debug log, `snapshot-<ms>.png` and `.txt`. When the user reports something seen in play, ask for one. F9 steps the game through slow motion (×0.25, ×0.1, normal) by scaling `Time`'s step.

### Other instructions

- Don't insert comments that indicate how you have changed the code. Only have comments that explain the current state.
- Don't enter plan mode unless I explicitly ask for it. If you want to ask questions about my request or discuss ideas, do it as a normal discussion.
- Be careful with tokens - I run out frequently. For example, don't spawn a sub-agent to do a comprehensive search of the code base if you can probably guess where the relevant code is and go there directly.
- Keep this file current: when a change makes something here wrong (a tool, a command, a cross-module rule), fix it in the same change. Keep per-module detail in the module's own docs, not here.
