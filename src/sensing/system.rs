//! Generic terrain probe system.
//!
//! Executes probes against all registered probe targets and produces
//! contact candidates. Has no knowledge of what the probes are used for.

use specs::{Entities, Join, Read, System, Write, WriteStorage};

use super::probe::{ContactCandidate, ContactCandidates, ProbeSet, ProbeTarget, SensorSet};
use crate::debug::DebugOverlays;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// System that executes terrain and rigid body probes.
///
/// This system is completely generic — it runs each probe against all
/// available `ProbeTarget` implementations and returns the earliest hit.
pub struct SensorProbeSystem;

impl<'a> System<'a> for SensorProbeSystem {
    type SystemData = (
        Option<Read<'a, TerrainWorld>>,
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
        let targets = ProbeSet::new([
            terrain_opt.as_ref().map(|t| &**t as &dyn ProbeTarget),
            physics_opt.as_ref().map(|p| &p.world as &dyn ProbeTarget),
        ]);

        if targets.is_empty() {
            return;
        }

        for (entity, sensor_set) in (&entities, &sensors).join() {
            let mut results = ContactCandidates::default();

            for probe in &sensor_set.probes {
                if probe.length <= 0.0 {
                    continue;
                }

                if let Some(hit) = targets.raycast(probe.origin, probe.direction, probe.length) {
                    let distance = (hit.t * probe.length).max(0.0);
                    results.candidates.push(ContactCandidate {
                        tag: probe.tag,
                        point: hit.point,
                        normal: hit.normal,
                        distance,
                    });
                    // _debug_overlays.add_line(
                    //     probe.origin,
                    //     hit.point,
                    //     Colour::YELLOW,
                    // );
                }
            }

            let _ = candidates.insert(entity, results);
        }
    }
}
