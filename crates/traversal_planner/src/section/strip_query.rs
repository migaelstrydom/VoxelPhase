use std::ops::Range;

use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{Motion, Phase};

use super::query::{FloorHit, Foothold, FootholdWindow, Ride, SectionQuery};
use super::strip::{FootholdLane, Profile, Section};
use super::surface::SurfacePoint;

/// How far from either end of a flight its arc may run along the floor
/// instead of clearing it, in metres: an arc leaves and meets the floor at
/// its ends.
const FLIGHT_END_ZONE: f32 = 0.1;
/// The same in seconds of flight, for a fast, flat arc, which meets a rising
/// lip well before its landing.
const FLIGHT_END_TIME: f32 = 0.04;
/// How far an arc may dip under the floor near its ends, in metres:
/// rounding, not a collision.
const FLIGHT_END_TOLERANCE: f32 = 0.005;
/// How far apart, in degrees, two floor normals may be for both to be the
/// one surface: a mesh facets a surface a degree or two either way.
const SAME_SURFACE_DEGREES: f32 = 5.0;
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
    /// steep for the drive, or falling ahead no steeper than the feet stand
    /// on, and headroom over it.
    standable: Vec<bool>,
    /// The furthest station each one rides to without a break.
    reach: Vec<usize>,
    /// Height gained and lost riding from the first station to each one.
    travel: Vec<f32>,
}

impl Rideway {
    fn new(floor: &Line, ceiling: &Line, params: &GaitParams) -> Self {
        // Down a face steeper than the drive climbs, gravity carries the
        // body; where it does so faster than it rides, the face is run off.
        let steepest = params.body.steepest_ride();
        let steepest_descent = steepest.max(params.foot.max_slope_degrees.to_radians());
        let (least_normal_z, greatest_rise) = (steepest.cos(), floor.spacing * steepest.tan());
        let (least_descent_normal_z, greatest_drop) = (
            steepest_descent.cos(),
            floor.spacing * steepest_descent.tan(),
        );
        let count = floor.samples.len();
        let standable: Vec<bool> = (0..count)
            .map(|station| {
                floor.samples[station].is_some_and(|hit| {
                    let falling = hit.normal.s > 0.0;
                    hit.normal.z
                        >= if falling {
                            least_descent_normal_z
                        } else {
                            least_normal_z
                        }
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
                && rise <= greatest_rise + RIDE_TOLERANCE
                && -rise <= greatest_drop + RIDE_TOLERANCE;
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

/// Farthest apart two footholds of a lane can be and still be one piece of
/// ground, in metres: a few samples, so a missed cast does not split it.
const GROUND_GAP: f32 = 0.08;
/// Largest rise between neighbouring footholds of one piece of ground, in
/// metres.
const GROUND_STEP: f32 = 0.05;

/// One lane's footholds where a foot would touch them, in increasing `s`, so
/// a window is found by search rather than scanned for; and the unbroken
/// runs of ground they make.
struct LaneFootholds {
    d: f32,
    footholds: Vec<Foothold>,
    /// `(from, to)` along the path, in increasing `s`.
    runs: Vec<(f32, f32)>,
}

impl LaneFootholds {
    fn new(lane: &FootholdLane, radius: f32) -> Self {
        let contact =
            |point: &SurfacePoint| PathVec::new(point.s, lane.d, point.z) - point.normal * radius;
        let mut footholds = Vec::new();
        for surface in &lane.profile.surfaces {
            let (Some(first), Some(last)) = (surface.points.first(), surface.points.last()) else {
                continue;
            };
            let (first_s, last_s) = (contact(first).s, contact(last).s);
            footholds.extend(surface.points.iter().map(|point| {
                let at = contact(point);
                Foothold {
                    at,
                    normal: point.normal,
                    footing: surface.footing,
                    edge_margin: (at.s - first_s).min(last_s - at.s),
                }
            }));
        }
        footholds.sort_by(|a, b| a.at.s.total_cmp(&b.at.s));
        let mut runs: Vec<(f32, f32)> = Vec::new();
        for (index, foothold) in footholds.iter().enumerate() {
            let joins = index > 0 && {
                let before = &footholds[index - 1];
                foothold.at.s - before.at.s <= GROUND_GAP
                    && (foothold.at.z - before.at.z).abs() <= GROUND_STEP
            };
            match runs.last_mut() {
                Some(run) if joins => run.1 = foothold.at.s,
                _ => runs.push((foothold.at.s, foothold.at.s)),
            }
        }
        Self {
            d: lane.d,
            footholds,
            runs,
        }
    }

    /// The run of ground holding `s`, if any.
    fn run_at(&self, s: f32) -> Option<(f32, f32)> {
        let after = self.runs.partition_point(|run| run.0 <= s);
        after
            .checked_sub(1)
            .map(|index| self.runs[index])
            .filter(|run| s <= run.1)
    }

    /// The footholds from `s.start` up to `s.end`.
    fn within(&self, s: &Range<f32>) -> impl Iterator<Item = &Foothold> {
        let first = self
            .footholds
            .partition_point(|foothold| foothold.at.s < s.start);
        let end = s.end;
        self.footholds[first..]
            .iter()
            .take_while(move |foothold| foothold.at.s < end)
    }

    /// The foothold on this lane on the same surface as `other`, from
    /// another lane, if there is one: as near along the path as ground on
    /// one lane is joined, and where `other`'s slope carries it.
    fn same_surface_as(&self, other: &Foothold) -> Option<&Foothold> {
        let after = self
            .footholds
            .partition_point(|foothold| foothold.at.s < other.at.s);
        let nearest = [after.checked_sub(1), Some(after)]
            .into_iter()
            .flatten()
            .filter_map(|index| self.footholds.get(index))
            .min_by(|a, b| {
                (a.at.s - other.at.s)
                    .abs()
                    .total_cmp(&(b.at.s - other.at.s).abs())
            })?;
        let apart = nearest.at - other.at;
        let normal = other.normal;
        let carried = -(normal.s * apart.s + normal.d * apart.d) / normal.z.max(f32::EPSILON);
        let same = nearest.footing == other.footing
            && apart.s.abs() <= GROUND_GAP
            && (apart.z - carried).abs() <= GROUND_STEP
            && nearest.normal.dot(normal) >= SAME_SURFACE_DEGREES.to_radians().cos();
        same.then_some(nearest)
    }
}

/// A foothold `share` of the way from `near` to `far`, on one surface.
fn between(near: &Foothold, far: &Foothold, share: f32) -> Foothold {
    Foothold {
        at: near.at + (far.at - near.at) * share,
        normal: (near.normal * (1.0 - share) + far.normal * share).normalized(),
        footing: near.footing,
        edge_margin: near.edge_margin.min(far.edge_margin),
    }
}

/// `footholds` into `out`, at most one in each `spacing` along the path.
fn thinned(footholds: impl Iterator<Item = Foothold>, spacing: f32, out: &mut Vec<Foothold>) {
    let mut last_bucket = None;
    for foothold in footholds {
        let bucket = (foothold.at.s / spacing).floor() as i64;
        if last_bucket != Some(bucket) {
            last_bucket = Some(bucket);
            out.push(foothold);
        }
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
    params: &'a GaitParams,
    foothold_lanes: Vec<LaneFootholds>,
    floor: Line,
    rideway: Rideway,
    /// At each body station, the lowest and highest the bottom-sphere centre
    /// may be: on the floor, if any, and the spine below the ceiling, if any.
    /// Flights are checked against these by plain arithmetic.
    bounds: Vec<(f32, f32)>,
    /// At each body station, the lowest the bottom-sphere centre may be over
    /// what roofs the layer, if anything does and the body could pass over.
    over: Vec<Option<f32>>,
}

impl<'a> StripQuery<'a> {
    pub fn new(section: &'a Section, params: &'a GaitParams) -> Self {
        let layout = &section.layout;
        let (first, count) = layout.stations(section.origin, layout.body_spacing);
        let start = first as f32 * layout.body_spacing;
        let end = start + count.saturating_sub(1) as f32 * layout.body_spacing;
        let floor = Line::new(&section.body_floor, start, layout.body_spacing, end);
        let ceiling = Line::new(&section.ceiling, start, layout.body_spacing, end);
        let rideway = Rideway::new(&floor, &ceiling, params);
        let over = Line::new(&section.upper_floor, start, layout.body_spacing, end)
            .samples
            .iter()
            .map(|hit| hit.map(|hit| hit.z))
            .collect();
        let bounds = (0..floor.samples.len())
            .map(|station| {
                let low = floor.samples[station].map_or(f32::NEG_INFINITY, |hit| hit.z);
                let high = ceiling
                    .samples
                    .get(station)
                    .copied()
                    .flatten()
                    .map_or(f32::INFINITY, |hit| hit.z - params.body.spine());
                (low, high)
            })
            .collect();
        Self {
            bounds,
            over,
            params,
            foothold_lanes: section
                .foothold_lanes
                .iter()
                .map(|lane| LaneFootholds::new(lane, section.foot_radius))
                .collect(),
            floor,
            rideway,
        }
    }

    fn clears_flight(&self, phase: &Phase) -> bool {
        let Motion::Flight { takeoff, .. } = phase.motion else {
            unreachable!("a flight");
        };
        let (from, to) = (takeoff.s, phase.end_s());
        let end_zone = FLIGHT_END_ZONE.max(FLIGHT_END_TIME * takeoff.speed);
        let clearance = self.params.flight.clearance;
        self.floor.stations_between(from, to).all(|station| {
            let (floor, roof) = self.bounds[station];
            let s = self.floor.station_s(station);
            let z = phase.flight_height(s).expect("a flight");
            // Near its ends an arc may run along the floor it leaves or lands
            // on, and after its takeoff over floor falling away beneath both
            // that floor and the line the arc leaves along, as off a rounded
            // brow; but it may not meet another face: a plank's bevel or a
            // lip's rounded corner, met from the side, is struck, not landed
            // on.
            let same_surface = |end: f32| {
                let (Some(here), Some(there)) = (self.floor.at(s), self.floor.at(end)) else {
                    return true;
                };
                here.normal.dot(there.normal) >= SAME_SURFACE_DEGREES.to_radians().cos()
            };
            let falls_away = || {
                let (Some(here), Some(left)) = (self.floor.at(s), self.floor.at(from)) else {
                    return true;
                };
                let floor_grade = -left.normal.s / left.normal.z.max(f32::EPSILON);
                let arc_grade = takeoff.climb / takeoff.speed;
                here.z <= left.z + (s - from) * floor_grade.min(arc_grade) + FLIGHT_END_TOLERANCE
            };
            let lifting_off = s - from < end_zone && (same_surface(from) || falls_away());
            let coming_down = to - s < end_zone && same_surface(to);
            let margin = if lifting_off || coming_down {
                -FLIGHT_END_TOLERANCE
            } else {
                clearance
            };
            (z >= floor + margin && z <= roof)
                || self.over[station].is_some_and(|over| z >= over + margin)
        })
    }
}

impl SectionQuery for StripQuery<'_> {
    fn floor(&self, s: f32) -> Option<FloorHit> {
        self.floor.at(s)
    }

    fn sampled_to(&self) -> f32 {
        self.floor
            .station_s(self.floor.samples.len().saturating_sub(1))
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
        let climb = match (self.floor.at(from), self.floor.at(end)) {
            (Some(start), Some(finish)) => (finish.z - start.z).abs(),
            _ => 0.0,
        };
        Ride {
            end,
            vertical_travel: (self.rideway.travel_to(&self.floor, end)
                - self.rideway.travel_to(&self.floor, from)
                - climb)
                .max(0.0),
        }
    }

    fn footholds(&self, window: &FootholdWindow, out: &mut Vec<Foothold>) {
        for lane in &self.foothold_lanes {
            if window.d.contains(&lane.d) {
                thinned(lane.within(&window.s).copied(), window.spacing, out);
            }
        }
        let Some(line) = window.line.filter(|line| window.d.contains(line)) else {
            return;
        };
        let Some((near, far)) = self
            .foothold_lanes
            .windows(2)
            .find(|pair| pair[0].d < line && line < pair[1].d)
            .map(|pair| (&pair[0], &pair[1]))
        else {
            return;
        };
        let on_line = near.within(&window.s).filter_map(|foothold| {
            let other = far.same_surface_as(foothold)?;
            let share = (line - foothold.at.d) / (other.at.d - foothold.at.d);
            (0.0..=1.0)
                .contains(&share)
                .then(|| between(foothold, other, share))
        });
        thinned(on_line, window.spacing, out);
    }

    fn ground_reaches(&self, foot: &PathVec, s: f32) -> bool {
        let Some(lane) = self
            .foothold_lanes
            .iter()
            .min_by(|a, b| (a.d - foot.d).abs().total_cmp(&(b.d - foot.d).abs()))
        else {
            return false;
        };
        lane.run_at(foot.s)
            .is_some_and(|(from, to)| s >= from - GROUND_GAP && s <= to + GROUND_GAP)
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
