use serde::{Deserialize, Serialize};

use crate::section::SectionQuery;

use super::body::BodyState;
use super::phase::{Contact, Motion, Phase};

/// The body's phases from now to the planning horizon. A footfall that lands
/// as a flight takes off, or at the very end, shows as a stance zero long.
///
/// Phases `[0, committed)` are settled: the next replan keeps them unless the
/// body strays from them, so the drive and the legs can act on them without
/// the plan flickering under them. The rest is tentative.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    /// In time order, each starting where the one before ends.
    pub phases: Vec<Phase>,
    /// How many leading phases are committed.
    pub committed: usize,
    /// The speed along the path the plan was made for, in m/s: a plan is
    /// kept frame to frame only while the player asks for the same.
    #[serde(default)]
    pub target_speed: f32,
}

impl Plan {
    /// Every footfall with the time it lands: a contact whose limb did not
    /// hold the body in the phase before. The first phase's contacts were
    /// planted before the plan began.
    pub fn footfalls(&self) -> impl Iterator<Item = (f32, &Contact)> {
        self.phases.windows(2).flat_map(|pair| {
            let (before, phase) = (&pair[0], &pair[1]);
            phase
                .contacts
                .iter()
                .filter(|contact| before.contacts.iter().all(|held| held.limb != contact.limb))
                .map(move |contact| (phase.start, contact))
        })
    }

    /// The index of the phase where the first footfall lands.
    fn first_footfall_phase(&self) -> Option<usize> {
        self.phases
            .windows(2)
            .position(|pair| {
                pair[1].contacts.iter().any(|contact| {
                    pair[0]
                        .contacts
                        .iter()
                        .all(|held| held.limb != contact.limb)
                })
            })
            .map(|before| before + 1)
    }

    /// Seconds from now until the plan's last phase ends.
    pub fn duration(&self) -> f32 {
        self.phases
            .last()
            .map_or(0.0, |phase| phase.start + phase.duration)
    }

    /// The phase under way at `t` seconds, the last one past the end.
    fn phase_at(&self, t: f32) -> Option<(usize, &Phase)> {
        let index = self
            .phases
            .iter()
            .position(|phase| t < phase.start + phase.duration)
            .unwrap_or(self.phases.len().checked_sub(1)?);
        Some((index, &self.phases[index]))
    }

    /// The body `t` seconds into the plan, as if it tracked it exactly: on the
    /// ground its height is the floor's.
    pub fn body_at(&self, t: f32, query: &dyn SectionQuery) -> Option<BodyState> {
        let (_, phase) = self.phase_at(t)?;
        let elapsed = (t - phase.start).clamp(0.0, phase.duration);
        let (s, speed) = phase.progress(elapsed);
        let (z, climb) = match phase.motion {
            Motion::Flight { takeoff, gravity } => {
                (phase.flight_height(s)?, takeoff.climb - gravity * elapsed)
            }
            Motion::Grounded { .. } => (query.floor(s)?.z, 0.0),
        };
        Some(BodyState { s, z, speed, climb })
    }

    /// The foot holding the body `t` seconds into the plan; `None` in flight.
    pub fn stance_at(&self, t: f32) -> Option<&Contact> {
        self.phase_at(t)?.1.contacts.first()
    }

    /// The plan from `t` seconds on, its clock starting there: the phases
    /// already over dropped, so the footfalls already landed are too, and the
    /// phase under way starting where the body is. If the committed footfall
    /// has landed, the next one is committed.
    pub fn after(&self, t: f32) -> Plan {
        let Some((first, _)) = self.phase_at(t) else {
            return Plan::default();
        };
        let mut phases: Vec<Phase> = self.phases[first..]
            .iter()
            .map(|phase| Phase {
                start: phase.start - t,
                ..phase.clone()
            })
            .collect();
        if let Some(current) = phases.first_mut() {
            current.clip_start(-current.start);
        }
        let mut later = Plan {
            phases,
            committed: self.committed.saturating_sub(first),
            target_speed: self.target_speed,
        };
        // Once the committed footfall has landed, the commitment rolls on to
        // the next one.
        if later.committed == 0 {
            later.committed = later.first_footfall_phase().unwrap_or(0);
        }
        later
    }
}
