use smallvec::SmallVec;

use crate::plan::{BodyState, Contact, Motion, Phase, Plan};

use super::body_path::{BodyPath, BodySegment};
use super::footfall_search::Footfall;

/// Slack on matching a footfall to a point of the body's path, in metres.
const TOLERANCE: f32 = 1.0e-3;

/// The plan of a body path and the footfalls along it: a phase for each
/// stretch between footfalls on the ground (held by the latest foot to land)
/// and one for each flight. The first footfall, and all before it, is
/// committed.
pub fn build_plan(body: &BodyPath, planted: &Contact, footfalls: &[Footfall]) -> Plan {
    let mut builder = Builder {
        body,
        phases: Vec::new(),
        stance: *planted,
        footfalls,
        next: 0,
    };
    for segment in &body.segments {
        match *segment {
            BodySegment::Ride {
                from,
                to,
                start_speed,
                end_speed,
            } => builder.ride(segment, from, to, start_speed, end_speed),
            BodySegment::Flight {
                takeoff, gravity, ..
            } => builder.flight(segment, takeoff, gravity),
        }
    }
    builder.land_through(f32::MAX);
    if let Some(last) = footfalls.last() {
        let shown = builder
            .phases
            .last()
            .is_some_and(|phase| phase.contacts.contains(&last.contact));
        if !shown {
            builder.ground(last.body_s, last.body_s, 0.0, 0.0);
        }
    }
    let committed = footfalls.first().map_or(0, |first| {
        builder
            .phases
            .iter()
            .position(|phase| phase.contacts.contains(&first.contact))
            .unwrap_or(builder.phases.len())
    });
    Plan {
        phases: builder.phases,
        committed,
    }
}

struct Builder<'a> {
    body: &'a BodyPath,
    phases: Vec<Phase>,
    /// The foot holding the body now.
    stance: Contact,
    footfalls: &'a [Footfall],
    /// The first footfall not yet landed.
    next: usize,
}

impl Builder<'_> {
    /// A ride, split where each footfall along it lands.
    fn ride(&mut self, segment: &BodySegment, from: f32, to: f32, start: f32, end: f32) {
        self.land_through(from);
        let mut at = from;
        while let Some(footfall) = self
            .footfalls
            .get(self.next)
            .filter(|footfall| footfall.body_s < to - TOLERANCE)
        {
            let split = footfall.body_s.max(at);
            if split > at + TOLERANCE {
                self.ground(at, split, segment.speed_at(at), segment.speed_at(split));
            }
            self.stance = footfall.contact;
            self.next += 1;
            at = split;
        }
        let speeds = if at <= from + TOLERANCE {
            (start, end)
        } else {
            (segment.speed_at(at), end)
        };
        self.ground(at, to, speeds.0, speeds.1);
    }

    /// A flight; a foot landing at its takeoff shows as a zero-long stance.
    fn flight(&mut self, segment: &BodySegment, takeoff: BodyState, gravity: f32) {
        if self.land_through(takeoff.s) {
            self.ground(takeoff.s, takeoff.s, takeoff.speed, takeoff.speed);
        }
        self.phases.push(Phase {
            start: self.body.time_at(takeoff.s),
            duration: segment.time_to(segment.to()),
            contacts: SmallVec::new(),
            motion: Motion::Flight { takeoff, gravity },
        });
    }

    /// Lands every footfall up to `s`; whether any landed.
    fn land_through(&mut self, s: f32) -> bool {
        let mut landed = false;
        while let Some(footfall) = self
            .footfalls
            .get(self.next)
            .filter(|footfall| footfall.body_s <= s + TOLERANCE)
        {
            self.stance = footfall.contact;
            self.next += 1;
            landed = true;
        }
        landed
    }

    fn ground(&mut self, from: f32, to: f32, start_speed: f32, end_speed: f32) {
        let start = self.body.time_at(from);
        self.phases.push(Phase {
            start,
            duration: self.body.time_at(to) - start,
            contacts: SmallVec::from_slice(&[self.stance]),
            motion: Motion::Grounded {
                from,
                to,
                start_speed,
                end_speed,
            },
        });
    }
}
