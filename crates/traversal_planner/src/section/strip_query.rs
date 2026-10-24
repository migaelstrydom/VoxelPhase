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

/// [`SectionQuery`] over a sampled strip: the body floor and ceiling along
/// the path, the footholds along each lane.
pub struct StripQuery<'a> {
    section: &'a Section,
    floor: Line,
    ceiling: Line,
}

impl<'a> StripQuery<'a> {
    pub fn new(section: &'a Section) -> Self {
        let layout = &section.layout;
        let start = section.origin - layout.behind;
        let end = section.origin + layout.ahead;
        Self {
            section,
            floor: Line::new(&section.body_floor, start, layout.body_spacing, end),
            ceiling: Line::new(&section.ceiling, start, layout.body_spacing, end),
        }
    }

    /// Whether the top sphere, its bottom sphere's centre at `(s, z)`, is
    /// under the ceiling.
    fn has_headroom(&self, s: f32, z: f32, params: &GaitParams) -> bool {
        self.ceiling
            .at(s)
            .map_or(true, |ceiling| z + params.body.spine() <= ceiling.z)
    }

    fn clears_flight(&self, phase: &Phase, params: &GaitParams) -> bool {
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
                params.flight.clearance
            };
            let above_floor = self
                .floor
                .sample(station as isize)
                .map_or(true, |floor| z >= floor.z + margin);
            above_floor && self.has_headroom(s, z, params)
        })
    }
}

impl SectionQuery for StripQuery<'_> {
    fn floor(&self, s: f32) -> Option<FloorHit> {
        self.floor.at(s)
    }

    fn ride(&self, from: f32, to: f32, params: &GaitParams) -> Ride {
        let stuck = Ride {
            end: from,
            vertical_travel: 0.0,
        };
        let Some(start) = self.floor.at(from) else {
            return stuck;
        };
        if !self.has_headroom(from, start.z, params) {
            return stuck;
        }
        let steepest = params.body.max_ground_slope_degrees.to_radians();
        let (least_normal_z, greatest_grade) = (steepest.cos(), steepest.tan());
        let mut ride = stuck;
        let mut last = (from, start.z);
        let stations = self
            .floor
            .stations_between(from, to)
            .map(|station| self.floor.station_s(station));
        for s in stations.chain(std::iter::once(to)) {
            let Some(floor) = self.floor.at(s) else {
                return ride;
            };
            let rise = floor.z - last.1;
            let rideable = rise.abs() <= (s - last.0) * greatest_grade + RIDE_TOLERANCE
                && floor.normal.z >= least_normal_z
                && self.has_headroom(s, floor.z, params);
            if !rideable {
                return ride;
            }
            ride.end = s;
            ride.vertical_travel += rise.abs();
            last = (s, floor.z);
        }
        ride
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

    fn clears(&self, phase: &Phase, params: &GaitParams) -> bool {
        match phase.motion {
            Motion::Grounded { from, to, .. } => self.ride(from, to, params).end >= to,
            Motion::Flight { .. } => self.clears_flight(phase, params),
        }
    }
}
