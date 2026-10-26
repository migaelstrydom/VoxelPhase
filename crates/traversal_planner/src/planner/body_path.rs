use crate::plan::{flight_at, BodyState};
use crate::section::SectionQuery;

/// Slack on matching a flight's landing to a footfall, in metres.
const LANDED_TOLERANCE: f32 = 1.0e-3;

/// One stretch of the body's path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodySegment {
    /// Riding the body floor from `from` to `to`, the speed changing evenly
    /// in time from `start_speed` to `end_speed`.
    Ride {
        from: f32,
        to: f32,
        start_speed: f32,
        end_speed: f32,
    },
    /// Ballistic from `takeoff` until it meets the floor again at `landing`,
    /// its speed along the path held.
    Flight {
        takeoff: BodyState,
        landing: f32,
        landing_z: f32,
        /// Downward acceleration, in m/s².
        gravity: f32,
        /// How long the body rises before the drive cancels the rise, if it
        /// does ([`Motion::Flight`](crate::plan::Motion::Flight)).
        rise_held: Option<f32>,
        /// Whether a foot pushed it off: a hop, which a foot lands. A body
        /// rolling off ground that falls away is not pushed and needs no
        /// foot at either end, and no foot lands while it flies.
        pushed: bool,
    },
    /// Pushed off a brow into what lies out of sight, ballistic from
    /// `takeoff`: the path ends in it at `until`, where the search stops, its
    /// arc having met no floor it saw. Replanned once the ground it falls to
    /// comes into sight, it is a hop that lands.
    Leap {
        takeoff: BodyState,
        until: f32,
        /// Downward acceleration, in m/s².
        gravity: f32,
    },
}

impl BodySegment {
    /// A ride's acceleration along the path, in m/s²; `None` for a flight.
    pub fn ride_acceleration(&self) -> Option<f32> {
        match *self {
            Self::Ride {
                from,
                to,
                start_speed,
                end_speed,
            } if to > from => {
                Some((end_speed * end_speed - start_speed * start_speed) / (2.0 * (to - from)))
            }
            Self::Ride { .. } => Some(0.0),
            Self::Flight { .. } | Self::Leap { .. } => None,
        }
    }

    pub fn from(&self) -> f32 {
        match *self {
            Self::Ride { from, .. } => from,
            Self::Flight { takeoff, .. } | Self::Leap { takeoff, .. } => takeoff.s,
        }
    }

    pub fn to(&self) -> f32 {
        match *self {
            Self::Ride { to, .. } => to,
            Self::Flight { landing, .. } => landing,
            Self::Leap { until, .. } => until,
        }
    }

    /// Speed along the path at `s` within the segment.
    pub fn speed_at(&self, s: f32) -> f32 {
        match *self {
            Self::Ride {
                from,
                to,
                start_speed,
                end_speed,
            } => {
                // Speed changing evenly in time changes its square evenly in
                // distance.
                let fraction = if to > from {
                    (s - from) / (to - from)
                } else {
                    1.0
                };
                (start_speed * start_speed
                    + (end_speed * end_speed - start_speed * start_speed) * fraction)
                    .max(0.0)
                    .sqrt()
            }
            Self::Flight { takeoff, .. } | Self::Leap { takeoff, .. } => takeoff.speed,
        }
    }

    /// Seconds from the segment's start to `s`.
    pub fn time_to(&self, s: f32) -> f32 {
        let distance = s - self.from();
        match *self {
            Self::Ride { start_speed, .. } => {
                let speed = start_speed + self.speed_at(s);
                if speed > 0.0 {
                    2.0 * distance / speed
                } else {
                    0.0
                }
            }
            Self::Flight { takeoff, .. } | Self::Leap { takeoff, .. } => distance / takeoff.speed,
        }
    }
}

/// Where the body's bottom-sphere centre goes: rides and flights in order,
/// each starting where the one before ends.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyPath {
    pub segments: Vec<BodySegment>,
}

impl BodyPath {
    /// Where the path starts along the path.
    pub fn start(&self) -> Option<f32> {
        self.segments.first().map(BodySegment::from)
    }

    /// Where the path ends along the path.
    pub fn end(&self) -> Option<f32> {
        self.segments.last().map(BodySegment::to)
    }

    /// The flights that land, hops and roll-offs, in order: a foot leaves
    /// near each takeoff and the next lands at its landing.
    pub fn flights(&self) -> impl Iterator<Item = &BodySegment> {
        self.segments
            .iter()
            .filter(|segment| matches!(segment, BodySegment::Flight { .. }))
    }

    /// The leap the path ends in, if it does.
    pub fn leap(&self) -> Option<&BodySegment> {
        self.segments
            .last()
            .filter(|segment| matches!(segment, BodySegment::Leap { .. }))
    }

    /// The segment the body is in at `s`, a flight's landing counting as the
    /// ground after it.
    pub fn segment_at(&self, s: f32) -> Option<&BodySegment> {
        self.segments
            .iter()
            .find(|segment| s >= segment.from() && s < segment.to())
            .or_else(|| self.segments.last().filter(|last| s >= last.to()))
    }

    /// Height of the bottom-sphere centre at `s`: the floor's where the body
    /// rides, the arc's where it flies.
    pub fn height_at(&self, s: f32, query: &dyn SectionQuery) -> Option<f32> {
        match self.segment_at(s)? {
            BodySegment::Flight {
                takeoff,
                landing,
                gravity,
                rise_held,
                ..
            } if s < *landing => {
                let t = (s - takeoff.s) / takeoff.speed;
                Some(flight_at(takeoff, *gravity, *rise_held, t).0)
            }
            BodySegment::Leap {
                takeoff, gravity, ..
            } => {
                let t = (s - takeoff.s) / takeoff.speed;
                Some(flight_at(takeoff, *gravity, None, t).0)
            }
            _ => query.floor(s).map(|floor| floor.z),
        }
    }

    /// Speed along the path at `s`.
    pub fn speed_at(&self, s: f32) -> Option<f32> {
        self.segment_at(s)
            .map(|segment| segment.speed_at(s.min(segment.to())))
    }

    /// Seconds from the path's start until the body is at `s`.
    pub fn time_at(&self, s: f32) -> f32 {
        let mut time = 0.0;
        for segment in &self.segments {
            if s <= segment.to() {
                return time + segment.time_to(s.max(segment.from()));
            }
            time += segment.time_to(segment.to());
        }
        time
    }

    /// The path as far as feet that last landed with the body at `landed`
    /// carry it: up to the takeoff of the first hop landing past there.
    pub fn until_unlanded(&self, landed: f32) -> BodyPath {
        let segments = self
            .segments
            .iter()
            .take_while(|segment| match segment {
                BodySegment::Flight {
                    landing,
                    pushed: true,
                    ..
                } => *landing <= landed + LANDED_TOLERANCE,
                BodySegment::Leap { .. } => false,
                _ => true,
            })
            .copied()
            .collect();
        BodyPath { segments }
    }
}
