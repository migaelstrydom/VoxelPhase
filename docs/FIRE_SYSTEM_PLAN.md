# Volumetric Fire System Design

## Overview

A GPU-driven volumetric fire simulation using 3D fluid dynamics on compute shaders, rendered via raymarching. Fire ignites on wooden boxes when grenades explode nearby, consumes fuel, transitions to short-lived smoke, and emits light via a simple uniform array passed to the main fragment shader.

This is the first compute shader pipeline in the engine.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ECS Layer                                │
│                                                                 │
│  ExplosionSystem ──ignites──▶ FireIgnitionSystem                │
│                                    │                            │
│                              writes FireVolume                  │
│                              components + fuel                  │
│                                    │                            │
│  FireSimSystem ◀───────────────────┘                            │
│       │  reads FireVolume, Time                                 │
│       │  dispatches compute shaders                             │
│       ▼                                                         │
│  RenderSystem                                                   │
│       │  reads FireVolume                                       │
│       │  calls fire_renderer.render()                           │
│       ▼                                                         │
│  FireRenderer (raymarches volume → transparent pass)            │
└─────────────────────────────────────────────────────────────────┘
```

```
┌──────────────────────── GPU per frame ─────────────────────────┐
│                                                                 │
│  ┌─────────────┐   ┌─────────────┐   ┌──────────────────────┐  │
│  │  Advection   │──▶│  Buoyancy + │──▶│  Fuel consumption +  │  │
│  │  (compute)   │   │  Turbulence │   │  Temperature decay   │  │
│  │              │   │  (compute)  │   │  (compute)           │  │
│  └─────────────┘   └─────────────┘   └──────────────────────┘  │
│         ▼                                       ▼               │
│  ┌─────────────┐                      ┌──────────────────────┐  │
│  │  Pressure    │                      │  Raymarching         │  │
│  │  projection  │                      │  (fragment shader)   │  │
│  │  (compute)   │                      │  reads 3D textures   │  │
│  └─────────────┘                      │  outputs to screen   │  │
│                                        └──────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## 1. Fluid Simulation (Compute Shaders)

### 3D Volume Representation

Each fire instance owns a 3D texture grid. Default resolution: **32×48×32** (width × height × depth). Height is taller because fire rises.

Two RGBA 3D storage textures (ping-pong):

| Channel | Field         | Notes                              |
|---------|--------------|------------------------------------|
| R       | Temperature  | 0.0 = ambient, 1.0 = max flame    |
| G       | Fuel density | 1.0 = fully fueled, consumed → 0  |
| B       | Smoke density| Produced as fuel burns out         |
| A       | Divergence   | Used by pressure projection        |

One RGBA 3D velocity texture (ping-pong):

| Channel | Field | Notes                |
|---------|-------|----------------------|
| R       | Vx    | Horizontal velocity  |
| G       | Vy    | Vertical velocity    |
| B       | Vz    | Depth velocity       |
| A       | Pressure | Scalar pressure field |

### Simulation Passes (per frame)

All dispatched as compute shaders with workgroup size (4, 4, 4):

**Pass 1 — Advection** (`fire_advect.comp`)
- Semi-Lagrangian advection: trace each voxel backward through velocity field
- Trilinear sample from source texture at traced position
- Advect both the field texture (temp/fuel/smoke) and velocity texture
- MacCormack correction for reduced numerical diffusion

**Pass 2 — Forces** (`fire_forces.comp`)
- **Buoyancy**: `Vy += buoyancy_strength * temperature * dt`
- **Turbulence**: Curl noise displacement, scaled by temperature
  - 3D curl noise computed analytically (no lookup table needed)
  - Turbulence amplitude modulated by temperature (hotter = more turbulent)
- **Cooling**: `temperature -= cooling_rate * dt` (faster near edges)
- **Fuel → heat**: Where fuel > 0 and temperature > ignition threshold:
  - `temperature += combustion_rate * fuel * dt`
  - `fuel -= burn_rate * dt`
  - `smoke += smoke_production * burn_rate * dt`
- **Smoke rise**: Smoke gets gentle upward velocity, slower than fire

**Pass 3 — Pressure projection** (`fire_pressure.comp`)
- Jacobi iteration (20-30 iterations) to solve pressure from divergence
- Makes velocity field divergence-free (incompressible flow)
- Boundary conditions: open top, closed sides/bottom
- Each Jacobi iteration is a separate dispatch (reads previous iteration)

**Pass 4 — Velocity correction** (`fire_correct.comp`)
- Subtract pressure gradient from velocity: `v -= grad(pressure)`

### Simulation Parameters (push constants)

```glsl
layout(push_constant) uniform FireSimParams {
    float dt;                   // Time step
    float buoyancy_strength;    // 3.0 default
    float cooling_rate;         // 0.8
    float combustion_rate;      // 2.0
    float burn_rate;            // 0.3
    float turbulence_amplitude; // 0.5
    float turbulence_frequency; // 2.0
    float smoke_production;     // 0.4
    float time;                 // For animated noise
    int jacobi_iteration;       // Current pressure solve iteration
};
```

### Volume Sizing

The volume is axis-aligned in world space. The world-space size is determined by the burning object's bounding box, expanded upward for rising flames:

- Width/depth: object AABB + 0.5m margin per side
- Height: object AABB height × 2.5 (flames rise above the object)

The volume-to-world transform is stored per fire instance for the raymarcher.

## 2. Raymarching Renderer (Fragment Shader)

### Pipeline

A new graphics pipeline rendering a fullscreen quad (or per-fire bounding box quad) in the **transparent pass**, after water and before particles.

**Vertex shader** (`fire.vert`):
- Takes the fire volume's world-space bounding box
- Projects 8 corners, renders as a screen-space quad covering the volume

**Fragment shader** (`fire.frag`):
- Reconstructs ray from camera through fragment
- Clips ray to volume AABB (ray-box intersection)
- Marches through volume in fixed steps (32-64 steps)

```glsl
// Raymarching loop (simplified)
vec4 accum = vec4(0.0);
for (int i = 0; i < MAX_STEPS; i++) {
    vec3 uvw = world_to_volume(sample_pos);
    if (any(lessThan(uvw, vec3(0))) || any(greaterThan(uvw, vec3(1))))
        break;

    vec4 field = texture(volume_tex, uvw);  // temp, fuel, smoke, _
    float temperature = field.r;
    float smoke = field.b;

    // Fire emission: blackbody-inspired color ramp
    vec3 fire_color = blackbody_ramp(temperature);
    float fire_alpha = smoothstep(0.05, 0.3, temperature) * fire_opacity;

    // Smoke absorption
    float smoke_alpha = smoke * smoke_opacity * step_size;
    vec3 smoke_color = vec3(0.15, 0.12, 0.1); // Dark grey-brown

    // Front-to-back compositing
    // Fire is emissive (adds light), smoke is absorptive (blocks light)
    accum.rgb += (1.0 - accum.a) * (fire_color * fire_alpha + smoke_color * smoke_alpha);
    accum.a += (1.0 - accum.a) * (fire_alpha + smoke_alpha);

    if (accum.a > 0.98) break;  // Early termination
    sample_pos += ray_dir * step_size;
}
```

### Blackbody Color Ramp

Temperature → color mapping inspired by blackbody radiation:
- 0.0–0.2: transparent (below ignition)
- 0.2–0.4: deep red / dark orange
- 0.4–0.6: bright orange
- 0.6–0.8: yellow-orange
- 0.8–1.0: yellow-white (hottest core)

Implemented as a small lookup in the shader (no texture needed).

### Depth Integration

The raymarcher reads the depth buffer from the opaque pass to stop marching when occluded by geometry. This prevents fire from rendering through walls/terrain.

```glsl
float scene_depth = texture(depth_sampler, frag_uv).r;
float scene_z = linearize_depth(scene_depth);

// In march loop:
float sample_z = linearize_depth(project(sample_pos).z);
if (sample_z > scene_z) break;
```

### Descriptor Sets

Fire rendering needs:
- Set 0: Scene UBO (reuse existing — view/proj matrices)
- Set 1: Fire volume texture (3D combined image sampler)
- Set 2: Depth buffer (combined image sampler, from opaque pass)

Or flatten to push constants for camera + per-fire UBO for volume transform, keeping descriptor count lower.

**Proposed layout:**
- Push constants: view matrix, proj matrix, volume_to_world matrix, camera_pos (256 bytes total)
- Set 0: 3D volume sampler (the field texture after simulation)
- Set 1: Depth buffer sampler

This avoids touching the existing scene UBO descriptor set layout.

## 3. Compute Pipeline Infrastructure

This is the engine's first compute pipeline. New infrastructure needed:

### `src/rendering/compute.rs`

```
ComputePipeline {
    device: Arc<ManagedDevice>,
    pipeline: vk::Pipeline,
    pipeline_layout: vk::PipelineLayout,
    descriptor_set_layouts: Vec<vk::DescriptorSetLayout>,
}
```

Responsible for:
- Loading compute SPIR-V via `ShaderManager` (new method: `load_compute()`)
- Creating compute pipeline layout
- Creating `vk::Pipeline` with `vk::ComputePipelineCreateInfo`

### 3D Texture Support

Extend `ManagedTexture` to support 3D images rather than creating a separate type. The differences are minimal:
- `image_type`: `vk::ImageType::TYPE_3D` (instead of `TYPE_2D`)
- `image_view_type`: `vk::ImageViewType::TYPE_3D` (instead of `TYPE_2D`)
- Extent includes a non-1 depth
- `usage` includes `STORAGE` (written by compute, read by fragment)
- `format`: `vk::Format::R16G16B16A16_SFLOAT` (half-float for precision + bandwidth)

All RAII cleanup, memory allocation, and sampler management is reused.

### Pipeline Barriers

Compute → graphics synchronization via pipeline barriers:

```
// After compute dispatch, before fragment read:
vkCmdPipelineBarrier(
    src_stage: COMPUTE_SHADER,
    dst_stage: FRAGMENT_SHADER,
    image_barrier: {
        old_layout: GENERAL,           // compute writes
        new_layout: SHADER_READ_ONLY,  // fragment reads
        src_access: SHADER_WRITE,
        dst_access: SHADER_READ,
    }
)
```

Between compute passes (advect → forces → pressure → correct):
```
vkCmdPipelineBarrier(
    src_stage: COMPUTE_SHADER,
    dst_stage: COMPUTE_SHADER,
    ...
)
```

## 4. ECS Components & Systems

### New Components

```rust
/// Marks an entity as flammable. Required for fire ignition.
pub struct Flammable {
    /// Total fuel available (seconds of burn time).
    pub fuel: f32,
    /// Temperature threshold to catch fire (0.0–1.0).
    pub ignition_threshold: f32,
}

/// Active fire on an entity. Created by FireIgnitionSystem.
pub struct OnFire {
    /// Handle to the GPU fire volume resources.
    pub volume: FireVolumeHandle,
    /// World-space AABB of the fire volume.
    pub world_aabb: Aabb,
    /// Remaining fuel (decremented as fire burns).
    pub fuel_remaining: f32,
    /// Time the fire has been burning.
    pub burn_time: f32,
}

// No FireLight component — fire positions and intensities are collected
// each frame and passed to the main fragment shader as a small uniform array.
```

### New Systems

**FireIgnitionSystem** (runs after ExplosionSystem)
- Reads: `Explosion`, `Flammable`, `Position`, `RigidBodyComponent`
- Writes: creates `OnFire` component, initializes GPU volume
- Logic:
  - For each explosion, find flammable entities within blast radius
  - Distance falloff determines initial fuel injection intensity
  - Create 3D volume textures, inject fuel into voxels overlapping the object
  - Attach `OnFire` component

**FireSimSystem** (runs after physics, before render)
- Reads: `OnFire`, `Time`
- Writes: updates `OnFire` (fuel tracking), dispatches compute
- Logic:
  - For each `OnFire` entity, dispatch the 4 compute passes
  - Track total remaining fuel; when exhausted and temperature negligible, remove `OnFire`
  - Collect fire position + average temperature for the lighting uniform array

**FireCleanupSystem** (runs after FireSimSystem)
- Removes `OnFire` component and frees GPU resources when fire is fully extinguished
- Box entity remains unchanged (no destruction or model swap)

### Dispatcher Order Update

```
... → Explosion → FireIgnition → ...
... → PhysicsSync → FireSim → FireCleanup → ParticleSpawn → ...
... → RenderSystem (calls fire_renderer between water and particles)
```

## 5. Fire Renderer Integration

### `src/fire/renderer.rs` — `FireRenderer`

```
FireRenderer {
    device: Arc<ManagedDevice>,
    sim_pipelines: FireSimPipelines,    // 4 compute pipelines
    render_pipeline: FireRenderPipeline, // graphics raymarching pipeline
    descriptor_pool: vk::DescriptorPool,
    volume_sampler: vk::Sampler,        // trilinear, clamp-to-border
}
```

**Methods:**
- `new(device, render_pass, depth_image_view)` — creates all pipelines, allocates pool
- `create_volume() -> FireVolumeHandle` — allocates 3D textures for a new fire
- `destroy_volume(handle)` — frees 3D textures
- `simulate(cb, volumes, dt)` — records compute dispatches for all active fires
- `render(cb, volumes, view, proj, camera_pos, depth_view)` — records raymarching draws

### Render Pass Integration

In `Renderer::render_fire()` (new method), called from `RenderSystem`:

```
// During transparent pass, after water, before particles:
renderer.render_fire(command_buffer, &fire_volumes, view, proj, camera_pos);
```

The fire raymarching shader uses **additive blending** for the emissive fire component and standard alpha blending for smoke. This can be approximated with:
- Blend mode: `SRC_ALPHA, ONE_MINUS_SRC_ALPHA` (same as particles)
- The shader encodes emission into premultiplied alpha (emission adds energy)

## 6. Shader Files

New shaders in `shader/fire/`:

| File                    | Type    | Purpose                              |
|-------------------------|---------|--------------------------------------|
| `fire/advect.comp`      | Compute | Semi-Lagrangian advection            |
| `fire/forces.comp`      | Compute | Buoyancy, turbulence, combustion     |
| `fire/pressure.comp`    | Compute | Jacobi pressure projection iteration |
| `fire/correct.comp`     | Compute | Velocity correction from pressure    |
| `fire/fire.vert`        | Vertex  | Volume bounding box projection       |
| `fire/fire.frag`        | Fragment| Raymarching + compositing            |

Compilation commands to add to CLAUDE.md:
```bash
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/advect.comp -o shader/fire/advect.comp.spv
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/forces.comp -o shader/fire/forces.comp.spv
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/pressure.comp -o shader/fire/pressure.comp.spv
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/correct.comp -o shader/fire/correct.comp.spv
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/fire.vert -o shader/fire/fire.vert.spv
~/software/VulkanSDK/1.4.309.0/macOS/bin/glslc shader/fire/fire.frag -o shader/fire/fire.frag.spv
```

## 7. Performance Budget

### Per sim slot:
- **3D textures**: 2 field + 2 velocity + 2 pressure (ping-pong) = 6 × 32×48×32 × 8 bytes (RGBA16F) = **~2.25 MB**
- **Compute dispatches**: 4 passes + ~25 pressure iterations = ~29 dispatches per slot per frame
- **Raymarch**: 64 steps per fragment, only fragments covered by the volume's screen-space bounding box

### Shared simulation pool:
- **5 sim slots** are allocated at startup and reused for the lifetime of the renderer
- Multiple fires share slots — each fire references a slot index and renders with its own world transform
- Compute cost is O(pool_size), not O(fire_count) — only active slots are simulated
- Source intensity per slot is the max across all fires sharing it
- Slot assignment uses least-used strategy for even distribution
- Total VRAM: ~11.25 MB (5 slots × 2.25 MB) — fixed regardless of fire count
- Compute cost: ~145 dispatches/frame max (5 slots × 29)
- Fragment cost: O(fire_count) draw calls, but these are cheap compared to simulation

### Simulation time scaling:
- `SIM_TIME_SCALE = 1.5` multiplies dt and total_time passed to compute shaders
- Makes flames evolve faster and feel more energetic without extra dispatches
- Smart alternative to sub-stepping (which would multiply compute cost)

### Optimizations (apply if needed):
- **LOD**: Reduce volume resolution for distant fires (16×24×16)
- **Temporal reprojection**: Run simulation at half rate, interpolate
- **Hybrid particles**: Use particles for small/distant fires, volumes for close ones

## 8. Implementation Order

### Phase 1 — Compute Infrastructure
1. Add `ShaderManager::load_compute()` for compute shader loading
2. Create `src/rendering/compute.rs` with `ComputePipeline` struct
3. Extend `ManagedTexture` to support 3D images
4. Verify compute queue support (likely same queue family as graphics on most GPUs)

### Phase 2 — Fluid Simulation
5. Write compute shaders: advection, forces, pressure, correction
6. Create `FireSimPipelines` — wraps the 4 compute pipelines + descriptors
7. Test with a single hardcoded volume — inject fuel, run simulation, dump to CPU to verify

### Phase 3 — Raymarching Renderer
8. Write fire vertex + fragment shaders
9. Create `FireRenderPipeline` — graphics pipeline for transparent pass
10. Create `FireRenderer` with `simulate()` and `render()` methods
11. Integrate into `Renderer` — new `render_fire()` method
12. Test with hardcoded fire volume — verify visual output

### Phase 4 — ECS Integration
13. Add `Flammable`, `OnFire`, `FireLight` components
14. Wire `Flammable` into box spawner (wooden boxes get it by default)
15. Implement `FireIgnitionSystem` (explosion → fire)
16. Implement `FireSimSystem` (per-frame compute dispatch)
17. Implement `FireCleanupSystem` (extinguish + cleanup)
18. Update dispatcher ordering

### Phase 5 — Polish
19. Pass fire positions + intensities as uniform array to main fragment shader for simple lighting
20. Tune smoke fade rate so volumes are freed quickly after burnout
21. Tune visual parameters (color ramp, turbulence, opacity)

## 9. Module Structure

```
src/
├── fire/
│   ├── mod.rs              // pub use only
│   ├── components.rs       // Flammable, OnFire
│   ├── volume.rs           // FireVolumeHandle, GPU resource management
│   ├── renderer.rs         // FireRenderer (compute sim + raymarching)
│   ├── pipeline.rs         // FireSimPipelines + FireRenderPipeline
│   └── systems.rs          // FireIgnitionSystem, FireSimSystem, FireCleanupSystem
├── rendering/
│   ├── compute.rs          // ComputePipeline (generic compute infra)
│   └── (ManagedTexture extended for 3D — no new file)
```

## 10. Design Decisions

1. **Dynamic lighting**: Simplest approach — pass fire positions + intensities to the main fragment shader as a small uniform array. No point light system needed for now.

2. **Fire spread**: No spread between objects in v1. Each fire is independent.

3. **Destruction**: Box remains as-is after burnout. No destruction or model swap.

4. **Smoke lifetime**: Smoke fades quickly after fire dies. Aggressive cooling rate on smoke density so volumes can be freed promptly.

5. **Shared sim pool**: Rather than each fire owning its own GPU volume, a fixed pool of 5 sim slots is shared. This caps compute cost and VRAM regardless of fire count. Fires sharing a slot render the same fluid state at different positions, which is less noticeable when 5 distinct simulations are running.

6. **Per-fire noise seeds**: Each sim slot gets a unique noise offset (spaced 200 apart) so simultaneous fires don't evolve identically despite sharing the same shader code.

7. **No per-fire GPU allocation/deallocation**: Sim slots are allocated once at startup. Creating/destroying a fire is a lightweight metadata operation — no `device_wait_idle`, no descriptor allocation/freeing.

## 11. Implementation Notes

### Tuned simulation parameters (in `FireSimParams::default()`):
- `buoyancy_strength: 8.0` — tall, fast-rising flames
- `cooling_rate: 0.6` — gradual temperature decay
- `combustion_rate: 6.0` — bright core
- `burn_rate: 0.05` — slow voxel fuel consumption for sustained flames
- `turbulence_amplitude: 6.0` — chaotic, flickery flames
- `turbulence_frequency: 6.0` — fine-detail turbulence
- `smoke_production: 2.5` — visible but not overwhelming smoke

### ECS fuel tracking:
- `Flammable.fuel` defaults to 20.0 (wood preset)
- `FUEL_BURN_RATE` in `FireCleanupSystem` is 0.3/sec — fire lasts ~67 seconds
- `OnFire` is removed when `fuel_remaining <= 0` and `burn_time > 2.0` (grace period for smoke)
- `source_intensity` in the shader is `fuel_remaining / initial_fuel`, so injection fades as fuel depletes

## 12. Future Improvements

- Dynamic point light system driven by fire volumes
- Fire spread between adjacent flammable objects
- Box destruction / charred model swap on burnout
- Lingering smoke with slow dissipation
- Volume merging for adjacent fires
- Heat distortion post-process (screen-space refraction)
- Sound effects
- Reduce fire start rate when many fires already active
