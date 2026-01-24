//! Terrain query system for sensor probes.

use nalgebra::{Point3, Vector3};
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{ContactCandidate, ContactCandidates, SensorSet};
use crate::components::{Position, ProbeShape, Rotation};
use crate::debug::DebugOverlays;
use crate::rendering::Colour;
use crate::terrain::TerrainManager;

/// System that runs terrain probes and produces contact candidates.
pub struct TerrainQuerySystem;

impl<'a> System<'a> for TerrainQuerySystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, SensorSet>,
        WriteStorage<'a, ContactCandidates>,
        Write<'a, DebugOverlays>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            entities,
            positions,
            rotations,
            sensors,
            mut candidates,
            mut debug_overlays,
        ): Self::SystemData,
    ) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (entity, position, sensor_set) in (&entities, &positions, &sensors).join() {
            let yaw = rotations.get(entity).map(|r| r.0).unwrap_or(0.0);
            let mut results = ContactCandidates::default();

            for probe in &sensor_set.probes {
                if probe.length <= 0.0 {
                    continue;
                }

                let local_dir = probe.local_direction;
                if local_dir.magnitude_squared() < 1e-6 {
                    continue;
                }

                let world_origin = Point3::new(position.0.x, position.0.y, position.0.z)
                    + rotate_y(probe.local_origin, yaw);
                let world_dir = rotate_y(local_dir.normalize(), yaw);
                let world_end = world_origin + world_dir * probe.length;

                let radius = match probe.shape {
                    ProbeShape::Ray { radius } => radius,
                    ProbeShape::SphereSweep { radius } => radius,
                };

                if let Some(contact) =
                    terrain_manager.query_swept_sphere(world_origin, world_end, radius)
                {
                    let distance = (contact.t * probe.length).max(0.0);
                    results.candidates.push(ContactCandidate {
                        purpose: probe.purpose,
                        point: contact.point,
                        normal: contact.normal,
                        distance,
                    });
                    match probe.purpose {
                        crate::components::ProbePurpose::FootLeft => {
                            debug_overlays.add_sphere(contact.point, 0.08, Colour::BLUE);
                        }
                        crate::components::ProbePurpose::FootRight => {
                            debug_overlays.add_sphere(contact.point, 0.08, Colour::RED);
                        }
                        _ => {}
                    }
                }
            }

            let _ = candidates.insert(entity, results);
        }
    }
}

fn rotate_y(vec: Vector3<f32>, yaw: f32) -> Vector3<f32> {
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    Vector3::new(
        vec.x * cos_yaw + vec.z * sin_yaw,
        vec.y,
        -vec.x * sin_yaw + vec.z * cos_yaw,
    )
}
