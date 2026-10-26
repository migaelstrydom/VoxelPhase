use smallvec::SmallVec;

use crate::plan::{BodyState, Contact, Motion, Phase, Plan};

use super::body_path::{BodyPath, BodySegment};
use super::footfall_search::Footfall;

/// Slack on matching a footfall to a point of the body's path, in metres.
const TOLERANCE: f32 = 1.0e-3;

/// The plan of a body path and the footfalls along it: a phase for each
/// stretch between footfalls on the ground (held by the latest foot to land)
/// and one for each flight. The first footfall, and all before it, is
/// committed. Every flight of `body` must end with a footfall, but a leap
/// the path ends in; cut a path the feet did not follow with
/// [`BodyPath::until_unlanded`] first.
pub fn build_plan(body: &BodyPath, planted: &Contact, footfalls: &[Footfall]) -> Plan {
    let mut builder = Builder {
        body,
        phases: Vec::new(),
        stance: *planted,
        footfalls,
        next: 0,
    };
    // The planted foot holds the body as the plan starts, if only for an
    // instant before a flight.
    if let Some(BodySegment::Flight { takeoff, .. }) = body.segments.first() {
        builder.ground(takeoff.s, takeoff.s, takeoff.speed, takeoff.speed);
    }
    for segment in &body.segments {
        match *segment {
            BodySegment::Ride {
                from,
                to,
                start_speed,
                end_speed,
            } => builder.ride(segment, from, to, start_speed, end_speed),
            BodySegment::Flight {
                takeoff,
                gravity,
                rise_held,
                pushed,
                ..
            } => builder.flight(segment, takeoff, gravity, rise_held, pushed),
            BodySegment::Leap {
                takeoff, gravity, ..
            } => builder.flight(segment, takeoff, gravity, None, true),
        }
    }
    builder.land_through(f32::MAX);
    let ends_grounded = body.leap().is_none();
    if let (Some(last), Some(end), true) = (footfalls.last(), body.end(), ends_grounded) {
        let shown = builder
            .phases
            .last()
            .is_some_and(|phase| phase.contacts.contains(&last.contact));
        if !shown {
            let speed = body.speed_at(end).unwrap_or_default();
            builder.ground(end, end, speed, speed);
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
        target_speed: 0.0,
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
    /// A hop leaves the ground with both feet; rolling off it, the stance
    /// foot stays planted behind the body.
    fn flight(
        &mut self,
        segment: &BodySegment,
        takeoff: BodyState,
        gravity: f32,
        rise_held: Option<f32>,
        pushed: bool,
    ) {
        if self.land_through(takeoff.s) {
            self.ground(takeoff.s, takeoff.s, takeoff.speed, takeoff.speed);
        }
        let contacts = if pushed {
            SmallVec::new()
        } else {
            SmallVec::from_slice(&[self.stance])
        };
        self.phases.push(Phase {
            start: self.body.time_at(takeoff.s),
            duration: segment.time_to(segment.to()),
            contacts,
            motion: Motion::Flight {
                takeoff,
                gravity,
                rise_held,
            },
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
