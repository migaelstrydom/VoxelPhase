//! Generic terrain probe system.
//!
//! Executes probes against terrain and produces contact candidates.
//! Has no knowledge of what the probes are used for.

use specs::{Entities, Join, Read, System, Write, WriteStorage};

use super::probe::{ContactCandidate, ContactCandidates, SensorSet};
use crate::debug::DebugOverlays;
use crate::terrain::TerrainManager;

/// System that executes terrain probes.
///
/// This system is completely generic - it just executes probes and
/// returns results.
pub struct SensorProbeSystem;

impl<'a> System<'a> for SensorProbeSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        Entities<'a>,
        WriteStorage<'a, SensorSet>,
        WriteStorage<'a, ContactCandidates>,
        Write<'a, DebugOverlays>,
    );

    fn run(
        &mut self,
        (terrain_opt, entities, sensors, mut candidates, mut _debug_overlays): Self::SystemData,
    ) {
        let Some(terrain) = terrain_opt.as_ref() else {
            return;
        };

        for (entity, sensor_set) in (&entities, &sensors).join() {
            let mut results = ContactCandidates::default();

            for probe in &sensor_set.probes {
                // Skip invalid probes
                if probe.length <= 0.0 {
                    continue;
                }

                // Compute end point
                let end = probe.origin + probe.direction * probe.length;

                // Query terrain
                if let Some(contact) = terrain.query_swept_sphere(probe.origin, end, probe.radius) {
                    let distance = (contact.t * probe.length).max(0.0);

                    results.candidates.push(ContactCandidate {
                        tag: probe.tag,
                        point: contact.point,
                        normal: contact.normal,
                        distance,
                    });
                }
                // debug_overlays.add_line_with_radius(
                //     probe.origin,
                //     end,
                //     probe.radius,
                //     Colour::YELLOW,
                // );
            }

            let _ = candidates.insert(entity, results);
        }
    }
}
