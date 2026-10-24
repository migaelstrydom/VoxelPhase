use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{Motion, Phase};

use super::query::{FloorHit, Foothold, FootholdWindow, Ride, SectionQuery};
use super::strip::{Profile, Section};

/// How far from either end of a flight its arc may run along the floor
/// instead of clearing it, in metres: an arc leaves and meets the floor at
/// its ends.
const FLIGHT_END_ZONE: f32 = 0.1;
/// How far an arc may dip under the floor near its ends, in metres:
/// rounding, not a collision.
const FLIGHT_END_TOLERANCE: f32 = 0.005;
/// Slack on a ride's slope test for rounding, in metres per station.
const RIDE_TOLERANCE: f32 = 1.0e-4;

/// One line of the strip indexed by station, so a lookup is not a search.
struct Line {
    start: f32,
    spacing: f32,
    samples: Vec<Option<FloorHit>>,
}

impl Line {
    fn new(profile: &Profile, start: f32, spacing: f32, end: f32) -> Self {
        let count = ((end - start) / spacing).round() as usize + 1;
        let mut samples = vec![None; count];
        for surface in &profile.surfaces {
            for point in &surface.points {
                let station = ((point.s - start) / spacing).round();
                if let Some(sample) = samples.get_mut(station as usize).filter(|_| station >= 0.0) {
                    *sample = Some(FloorHit {
                        z: point.z,
                        normal: point.normal,
                        footing: surface.footing,
                    });
                }
            }
        }
        Self {
            start,
            spacing,
            samples,
        }
    }

    fn station_s(&self, station: usize) -> f32 {
        self.start + station as f32 * self.spacing
    }

    fn sample(&self, station: isize) -> Option<FloorHit> {
        usize::try_from(station)
            .ok()
            .and_then(|station| self.samples.get(station).copied().flatten())
    }

    /// The line at `s`, interpolated between the stations either side; `None`
    /// unless both are present (or `s` is on one).
    fn at(&self, s: f32) -> Option<FloorHit> {
        let position = (s - self.start) / self.spacing;
        let below = position.floor();
        let fraction = position - below;
        let below = below as isize;
        if fraction < 1.0e-4 {
            return self.sample(below);
        }
        if fraction > 1.0 - 1.0e-4 {
            return self.sample(below + 1);
        }
        let (low, high) = (self.sample(below)?, self.sample(below + 1)?);
        let nearer = if fraction < 0.5 { low } else { high };
        Some(FloorHit {
            z: low.z + fraction * (high.z - low.z),
            ..nearer
        })
    }

    /// The stations strictly between `from` and `to`.
    fn stations_between(&self, from: f32, to: f32) -> impl Iterator<Item = usize> + '_ {
        let first = ((from - self.start) / self.spacing).floor() as isize + 1;
        let last = ((to - self.start) / self.spacing).ceil() as isize - 1;
        (first.max(0)..=last.min(self.samples.len() as isize - 1)).map(|station| station as usize)
    }
}

/// Where the body can ride along the floor line, worked out once per station
/// so a ride is two lookups.
struct Rideway {
    /// Whether the body can stand at each station: floor there, not too
    /// steep for the drive, headroom over it.
    standable: Vec<bool>,
    /// The furthest station each one rides to without a break.
    reach: Vec<usize>,
    /// Height gained and lost riding from the first station to each one.
    travel: Vec<f32>,
}

impl Rideway {
    fn new(floor: &Line, ceiling: &Line, params: &GaitParams) -> Self {
        let steepest = params.body.max_ground_slope_degrees.to_radians();
        let (least_normal_z, greatest_rise) = (steepest.cos(), floor.spacing * steepest.tan());
        let count = floor.samples.len();
        let standable: Vec<bool> = (0..count)
            .map(|station| {
                floor.samples[station].is_some_and(|hit| {
                    hit.normal.z >= least_normal_z
                        && has_headroom(ceiling, floor.station_s(station), hit.z, params)
                })
            })
            .collect();
        let mut travel = vec![0.0; count];
        let mut linked = vec![false; count];
        for station in 1..count {
            let rise = match (floor.samples[station - 1], floor.samples[station]) {
                (Some(low), Some(high)) => high.z - low.z,
                _ => 0.0,
            };
            travel[station] = travel[station - 1] + rise.abs();
            linked[station - 1] = standable[station - 1]
                && standable[station]
                && rise.abs() <= greatest_rise + RIDE_TOLERANCE;
        }
        let mut reach: Vec<usize> = (0..count).collect();
        for station in (0..count.saturating_sub(1)).rev() {
            if linked[station] {
                reach[station] = reach[station + 1];
            }
        }
        Self {
            standable,
            reach,
            travel,
        }
    }

    /// Height gained and lost riding from the first station to `s`, with `s`
    /// between stations taken in proportion.
    fn travel_to(&self, floor: &Line, s: f32) -> f32 {
        let position = ((s - floor.start) / floor.spacing).max(0.0);
        let below = (position.floor() as usize).min(self.travel.len() - 1);
        let above = (below + 1).min(self.travel.len() - 1);
        let fraction = position - below as f32;
        self.travel[below] + fraction.min(1.0) * (self.travel[above] - self.travel[below])
    }
}

/// Whether the top sphere, its bottom sphere's centre at `(s, z)`, is under
/// the ceiling.
fn has_headroom(ceiling: &Line, s: f32, z: f32, params: &GaitParams) -> bool {
    ceiling
        .at(s)
        .map_or(true, |ceiling| z + params.body.spine() <= ceiling.z)
}

/// [`SectionQuery`] over a sampled strip for one gait: the body floor and
/// ceiling along the path, the footholds along each lane, and where the body
/// can ride within the gait's limits.
pub struct StripQuery<'a> {
    section: &'a Section,
    params: &'a GaitParams,
    floor: Line,
    ceiling: Line,
    rideway: Rideway,
}

impl<'a> StripQuery<'a> {
    pub fn new(section: &'a Section, params: &'a GaitParams) -> Self {
        let layout = &section.layout;
        let start = section.origin - layout.behind;
        let end = section.origin + layout.ahead;
        let floor = Line::new(&section.body_floor, start, layout.body_spacing, end);
        let ceiling = Line::new(&section.ceiling, start, layout.body_spacing, end);
        let rideway = Rideway::new(&floor, &ceiling, params);
        Self {
            section,
            params,
            floor,
            ceiling,
            rideway,
        }
    }

    fn clears_flight(&self, phase: &Phase) -> bool {
        let (from, to) = match phase.motion {
            Motion::Flight { takeoff, .. } => (takeoff.s, phase.end_s()),
            Motion::Grounded { .. } => unreachable!("a flight"),
        };
        self.floor.stations_between(from, to).all(|station| {
            let s = self.floor.station_s(station);
            let z = phase.flight_height(s).expect("a flight");
            let near_end = s - from < FLIGHT_END_ZONE || to - s < FLIGHT_END_ZONE;
            let margin = if near_end {
                -FLIGHT_END_TOLERANCE
            } else {
                self.params.flight.clearance
            };
            let above_floor = self
                .floor
                .sample(station as isize)
                .map_or(true, |floor| z >= floor.z + margin);
            above_floor && has_headroom(&self.ceiling, s, z, self.params)
        })
    }
}

impl SectionQuery for StripQuery<'_> {
    fn floor(&self, s: f32) -> Option<FloorHit> {
        self.floor.at(s)
    }

    fn ride(&self, from: f32, to: f32) -> Ride {
        let stuck = Ride {
            end: from,
            vertical_travel: 0.0,
        };
        let position = (from - self.floor.start) / self.floor.spacing;
        let Ok(station) = usize::try_from(position.floor() as isize) else {
            return stuck;
        };
        if !self
            .rideway
            .standable
            .get(station)
            .copied()
            .unwrap_or(false)
        {
            return stuck;
        }
        let reach_s = self.floor.station_s(self.rideway.reach[station]);
        let end = to.min(reach_s);
        if end <= from {
            return stuck;
        }
        Ride {
            end,
            vertical_travel: self.rideway.travel_to(&self.floor, end)
                - self.rideway.travel_to(&self.floor, from),
        }
    }

    fn footholds(&self, window: &FootholdWindow, out: &mut Vec<Foothold>) {
        let radius = self.section.foot_radius;
        let contact =
            |s: f32, z: f32, normal: PathVec, d: f32| PathVec::new(s, d, z) - normal * radius;
        for lane in &self.section.foothold_lanes {
            if !window.d.contains(&lane.d) {
                continue;
            }
            let mut last_bucket = None;
            for surface in &lane.profile.surfaces {
                let (Some(first), Some(last)) = (surface.points.first(), surface.points.last())
                else {
                    continue;
                };
                let first_s = contact(first.s, first.z, first.normal, lane.d).s;
                let last_s = contact(last.s, last.z, last.normal, lane.d).s;
                for point in &surface.points {
                    let at = contact(point.s, point.z, point.normal, lane.d);
                    if !window.s.contains(&at.s) {
                        continue;
                    }
                    let bucket = (at.s / window.spacing).floor() as i64;
                    if last_bucket == Some(bucket) {
                        continue;
                    }
                    last_bucket = Some(bucket);
                    out.push(Foothold {
                        at,
                        normal: point.normal,
                        footing: surface.footing,
                        edge_margin: (at.s - first_s).min(last_s - at.s),
                    });
                }
            }
        }
    }

    fn clears(&self, phase: &Phase) -> bool {
        match phase.motion {
            Motion::Grounded { from, to, .. } => {
                from == to || self.ride(from, to).end >= to - RIDE_TOLERANCE
            }
            Motion::Flight { .. } => self.clears_flight(phase),
        }
    }
}
