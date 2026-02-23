//! Generic terrain probe system.
//!
//! Executes probes against all registered probe targets and produces
//! contact candidates. Has no knowledge of what the probes are used for.

use specs::{Entities, Join, Read, System, Write, WriteStorage};

use super::probe::{ContactCandidate, ContactCandidates, ProbeTarget, SensorSet};
use crate::debug::DebugOverlays;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainManager;

/// System that executes terrain and rigid body probes.
///
/// This system is completely generic — it runs each probe against all
/// available `ProbeTarget` implementations and returns the earliest hit.
pub struct SensorProbeSystem;

impl<'a> System<'a> for SensorProbeSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        Option<Read<'a, PhysicsResource>>,
        Entities<'a>,
        WriteStorage<'a, SensorSet>,
        WriteStorage<'a, ContactCandidates>,
        Write<'a, DebugOverlays>,
    );

    fn run(
        &mut self,
        (terrain_opt, physics_opt, entities, sensors, mut candidates, mut _debug_overlays): Self::SystemData,
    ) {
        let targets: Vec<&dyn ProbeTarget> = [
            terrain_opt.as_ref().map(|t| &**t as &dyn ProbeTarget),
            physics_opt.as_ref().map(|p| &p.0 as &dyn ProbeTarget),
        ]
        .into_iter()
        .flatten()
        .collect();

        if targets.is_empty() {
            return;
        }

        for (entity, sensor_set) in (&entities, &sensors).join() {
            let mut results = ContactCandidates::default();

            for probe in &sensor_set.probes {
                if probe.length <= 0.0 {
                    continue;
                }

                let mut earliest = None;
                for target in &targets {
                    if let Some(hit) = target.swept_probe(
                        probe.origin,
                        probe.direction,
                        probe.length,
                        probe.radius,
                    ) {
                        if earliest
                            .as_ref()
                            .map_or(true, |(t, _): &(f32, _)| hit.t < *t)
                        {
                            earliest = Some((hit.t, hit));
                        }
                    }
                }

                if let Some((_, hit)) = earliest {
                    let distance = (hit.t * probe.length).max(0.0);
                    results.candidates.push(ContactCandidate {
                        tag: probe.tag,
                        point: hit.point,
                        normal: hit.normal,
                        distance,
                    });
                    // _debug_overlays.add_line_with_radius(
                    //     probe.origin,
                    //     hit.point,
                    //     probe.radius,
                    //     Colour::YELLOW,
                    // );
                }
            }

            let _ = candidates.insert(entity, results);
        }
    }
}
