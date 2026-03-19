//! System that steps the water simulation and provides debug visualisation.

use nalgebra::Point3;
use specs::{Join, LendJoin, Read, ReadStorage, System, Write};

use crate::components::{Position, RigidBodyComponent, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugOverlays};
use crate::rendering::Colour;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainManager;
use crate::time::Time;
use crate::water::{BodySnapshot, WaterGrid, WaveBodyCoupler, WaveGrid};

/// Steps the water flow simulation and wave equation each frame.
///
/// Reads terrain dirty_regions to detect floor changes under water, then
/// advances the flow sim by the frame's delta time. Wave-body coupling
/// injects disturbances from rigid body interactions before the wave
/// equation step produces fine-resolution surface ripples.
pub struct WaterSystem;

const WATER_DEBUG_COLOUR: Colour = Colour {
    r: 0.2,
    g: 0.5,
    b: 1.0,
    a: 0.8,
};

struct WaterDebugConfig {
    draw_wet_cells: bool,
    debug_colour: Colour,
    sphere_radius: f32,
}

const WATER_DEBUG_CONFIG: WaterDebugConfig = WaterDebugConfig {
    draw_wet_cells: true,
    debug_colour: WATER_DEBUG_COLOUR,
    sphere_radius: 0.3,
};

fn overlapping_cell_range(
    min: f32,
    max: f32,
    grid_origin: f32,
    cell_size: f32,
    dim: usize,
) -> Option<(usize, usize)> {
    if dim == 0 {
        return None;
    }

    let grid_min = grid_origin;
    let grid_max = grid_origin + dim as f32 * cell_size;
    if max <= grid_min || min >= grid_max {
        return None;
    }

    let start = ((min - grid_origin) / cell_size).floor() as isize;
    let end = ((max - grid_origin) / cell_size).ceil() as isize - 1;

    let clamped_start = start.clamp(0, dim as isize - 1) as usize;
    let clamped_end = end.clamp(0, dim as isize - 1) as usize;
    if clamped_start > clamped_end {
        None
    } else {
        Some((clamped_start, clamped_end))
    }
}

impl<'a> System<'a> for WaterSystem {
    type SystemData = (
        Option<Write<'a, WaterGrid>>,
        Option<Write<'a, WaveGrid>>,
        Option<Write<'a, WaveBodyCoupler>>,
        Option<Read<'a, TerrainManager>>,
        Read<'a, Time>,
        Read<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, VelocityDriven>,
        Write<'a, DebugOverlays>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            water_opt,
            wave_opt,
            coupler_opt,
            terrain_opt,
            time,
            physics,
            bodies,
            positions,
            velocities,
            velocity_driven,
            mut debug_overlays,
            mut _debug_lines,
        ): Self::SystemData,
    ) {
        let Some(mut grid) = water_opt else {
            return;
        };

        let dt = time.delta_seconds();

        // Propagate terrain damage to water floor levels.
        if let Some(ref terrain) = terrain_opt {
            let dirty = terrain.dirty_regions();
            if !dirty.is_empty() {
                let mut dirty_cells = Vec::new();
                let dims = grid.dims();
                let origin = grid.origin();
                let cell_size = grid.cell_size();
                for region in dirty {
                    // Convert AABB to grid cells that overlap, including edge regions.
                    let x_range = overlapping_cell_range(
                        region.min.x,
                        region.max.x,
                        origin.x,
                        cell_size,
                        dims.0,
                    );
                    let z_range = overlapping_cell_range(
                        region.min.z,
                        region.max.z,
                        origin.z,
                        cell_size,
                        dims.1,
                    );
                    if let (Some((i0, i1)), Some((j0, j1))) = (x_range, z_range) {
                        for j in j0..=j1 {
                            for i in i0..=i1 {
                                dirty_cells.push((i, j));
                            }
                        }
                    }
                }
                if !dirty_cells.is_empty() {
                    grid.mark_dirty_floors(&dirty_cells);
                }
            }
        }

        // Step the flow simulation.
        let terrain_ref = terrain_opt.as_deref();
        grid.step(dt, |x, z| {
            terrain_ref.and_then(|t| t.approx_surface_height_at(x, z))
        });

        // Wave-body coupling + wave equation step.
        if let Some(mut wave_grid) = wave_opt {
            // Inject wave disturbances from rigid body interactions.
            if let Some(mut coupler) = coupler_opt {
                let snapshots = build_body_snapshots(
                    &physics,
                    &bodies,
                    &positions,
                    &velocities,
                    &velocity_driven,
                );
                coupler.update(&snapshots, &mut wave_grid, &grid);
            }

            wave_grid.step(dt, &grid);
        }

        // Debug visualisation: spheres at each wet cell's surface.
        if WATER_DEBUG_CONFIG.draw_wet_cells {
            let dims = grid.dims();
            let cell_area = grid.cell_area();

            for j in 0..dims.1 {
                for i in 0..dims.0 {
                    let cell = grid.cell(i, j);
                    if cell.volume <= 0.0 {
                        continue;
                    }
                    let x = grid.cell_center_x(i);
                    let z = grid.cell_center_z(j);
                    let y = cell.surface_level(cell_area);
                    debug_overlays.add_sphere(
                        Point3::new(x, y, z),
                        WATER_DEBUG_CONFIG.sphere_radius,
                        WATER_DEBUG_CONFIG.debug_colour,
                    );
                }
            }
        }
    }
}

/// Collect rigid body snapshots for wave-body coupling.
fn build_body_snapshots(
    physics: &PhysicsResource,
    bodies: &ReadStorage<RigidBodyComponent>,
    positions: &ReadStorage<Position>,
    velocities: &ReadStorage<Velocity>,
    velocity_driven: &ReadStorage<VelocityDriven>,
) -> Vec<BodySnapshot> {
    let mut snapshots = Vec::new();

    for (body_comp, pos, vel, vd) in (bodies, positions, velocities, velocity_driven.maybe()).join()
    {
        let is_velocity_driven: bool = vd.is_some();
        let Some(rb) = physics.world.body(body_comp.0) else {
            continue;
        };
        if !rb.is_dynamic() {
            continue;
        }

        let collider_handle = match rb.colliders().first() {
            Some(h) => *h,
            None => continue,
        };
        let Some(collider) = physics.world.collider(collider_handle) else {
            continue;
        };

        // Use the arena index raw parts to create a stable u64 ID.
        let (slot, gen) = body_comp.0.raw_parts();
        let id = (gen << 32) | (slot as u64);

        let footprint_radius = collider.shape().footprint_radius();

        snapshots.push(BodySnapshot {
            id,
            position: Point3::new(pos.0.x, pos.0.y, pos.0.z),
            velocity: vel.0,
            footprint_radius,
            is_velocity_driven,
        });
    }

    snapshots
}
