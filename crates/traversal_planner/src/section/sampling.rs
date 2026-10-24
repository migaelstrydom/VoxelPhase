use crate::params::GaitParams;
use crate::path::PathVec;

use super::strip::{FootholdLane, Profile, Section, StripLayout};
use super::surface::{Footing, Surface, SurfacePoint};

/// Which body a cast hit, as the caster tells them apart. Only compared, to
/// split surfaces where the body under them changes; never stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyKey(pub u64);

/// What a sphere cast stopped on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CastHit {
    /// Height of the sphere's centre where it stopped.
    pub z: f32,
    /// Unit contact normal, pointing from the surface to the sphere's centre.
    pub normal: PathVec,
    /// The body hit.
    pub body: BodyKey,
    pub footing: Footing,
}

/// How far an upward cast starts above the floor it rises from, in metres,
/// so it does not begin touching it.
const CEILING_START_GAP: f32 = 0.01;
/// How far a ceiling's normal must point down: a rising sphere that meets a
/// face leaning less than this has grazed a wall beside it, not met
/// something over it.
const CEILING_FACING: f32 = 0.05;
/// How far below the layer's height a floor must lie for a ceiling to be
/// looked for between them, in metres. Under that, a ceiling below the
/// layer's height leaves less headroom than any body has.
const CEILING_RECAST_DROP: f32 = 0.1;
/// How far a rising sphere must get before what it meets bounds the layer,
/// in metres: one stopped where it starts has no room there, and the layer
/// it followed has closed.
const LAYER_CLOSED: f32 = 1.0e-3;

/// Sphere casts in path coordinates: what a section is sampled from. The game
/// casts in its physics world; authored scenes cast against their shapes.
///
/// A sphere that starts inside something passes out of it unseen: a cast
/// meets only faces it moves toward.
pub trait SphereCaster {
    /// A sphere of `radius` centred at `(s, d, from)`, cast down onto the
    /// floor. `from` of infinity starts at the top of the caster's reach.
    fn cast_down(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit>;

    /// A sphere of `radius` centred at `(s, d, from)`, cast up into the
    /// ceiling.
    fn cast_up(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit>;
}

/// The body's layer at one station: the floor and ceiling of the open space
/// the body moves through, and the top of what roofs it over.
#[derive(Clone, Copy, Default)]
struct LayerStation {
    floor: Option<CastHit>,
    ceiling: Option<CastHit>,
    /// What a sphere cast down from the top of reach rests on, where that is
    /// above the ceiling: the top of an overhang, which the body may fly
    /// over but does not ride.
    upper_floor: Option<CastHit>,
}

/// Samples the strip around `origin` (the body's `s`), by the same rules
/// whatever the caster: the body floor and ceiling along the path with the
/// capsule's radius, footholds along every lane with a foot's.
///
/// The strip follows the body's layer: from the body's height `body_z` (its
/// bottom-sphere centre), station by station out from the body, each floor
/// is cast down from under the ceiling over the last one. Under an overhang
/// the floor is the ground beneath it, not the overhang's top.
pub fn sample_section(
    caster: &impl SphereCaster,
    layout: &StripLayout,
    params: &GaitParams,
    origin: f32,
    body_z: f32,
) -> Section {
    let body_radius = params.body.radius;
    let foot_radius = params.foot.radius;
    let least_foot_normal_z = params.foot.max_slope_degrees.to_radians().cos();

    let body_stations = stations(origin, layout, layout.body_spacing);
    let foot_stations = stations(origin, layout, layout.foot_spacing);

    let layer = body_layer(caster, &body_stations, origin, body_z, body_radius);
    let line = |hit: fn(&LayerStation) -> Option<CastHit>| {
        profile(
            body_stations.iter().copied().zip(layer.iter().map(hit)),
            |_| true,
        )
    };
    let body_floor = line(|station| station.floor);
    let ceiling = line(|station| station.ceiling);
    let upper_floor = line(|station| station.upper_floor);

    // A foot is cast from just under the ceiling of the body's layer at the
    // nearest body station.
    let foot_start = |s: f32| {
        let nearest = ((s - body_stations.first().copied().unwrap_or(s)) / layout.body_spacing)
            .round()
            .max(0.0) as usize;
        layer
            .get(nearest.min(layer.len().saturating_sub(1)))
            .and_then(|station| station.ceiling)
            .map_or(f32::INFINITY, |ceiling| {
                ceiling.z + body_radius - foot_radius
            })
    };
    let foothold_lanes = layout
        .lanes
        .iter()
        .map(|&d| FootholdLane {
            d,
            profile: profile(
                foot_stations
                    .iter()
                    .map(|&s| (s, caster.cast_down(s, d, foot_radius, foot_start(s)))),
                |hit| hit.normal.z >= least_foot_normal_z,
            ),
        })
        .collect();

    Section {
        layout: layout.clone(),
        origin,
        body_radius,
        foot_radius,
        body_floor,
        ceiling,
        upper_floor,
        foothold_lanes,
    }
}

/// The body's layer at every station, followed out from the body both ways:
/// each station's layer is the one over the floor before it.
fn body_layer(
    caster: &impl SphereCaster,
    stations: &[f32],
    origin: f32,
    body_z: f32,
    radius: f32,
) -> Vec<LayerStation> {
    let mut layer = vec![LayerStation::default(); stations.len()];
    let ahead = stations.partition_point(|&s| s < origin);
    let mut follow = |indices: &mut dyn Iterator<Item = usize>| {
        let mut z = body_z;
        for index in indices {
            let station = layer_station(caster, stations[index], z, radius);
            if let Some(floor) = station.floor {
                z = floor.z;
            }
            layer[index] = station;
        }
    };
    follow(&mut (ahead..stations.len()));
    follow(&mut (0..ahead).rev());
    layer
}

/// The layer at `s` of a body whose bottom-sphere centre was last at `z`.
///
/// Whatever a sphere rising from there meets bounds the layer: the floor is
/// cast down from under it, and anything on top of it is the upper floor.
/// Only a face that leans down is a ceiling; a wall corner grazed on the way
/// up still bounds the layer, but nothing hangs over the body there. Where
/// the sphere has no room to start, the layer has closed, and the station is
/// read from the top.
fn layer_station(caster: &impl SphereCaster, s: f32, z: f32, radius: f32) -> LayerStation {
    let rising = caster.cast_up(s, 0.0, radius, z);
    let closed = rising.is_some_and(|hit| hit.z < z + LAYER_CLOSED);
    let above = rising.filter(|_| !closed);
    let floor = caster.cast_down(s, 0.0, radius, above.map_or(f32::INFINITY, |above| above.z));
    let over = match floor {
        Some(floor) if closed || floor.z < z - CEILING_RECAST_DROP => {
            caster.cast_up(s, 0.0, radius, floor.z + CEILING_START_GAP)
        }
        _ => above,
    };
    let upper_floor = over.and_then(|over| {
        caster
            .cast_down(s, 0.0, radius, f32::INFINITY)
            .filter(|top| top.z > over.z)
    });
    LayerStation {
        floor,
        ceiling: over.filter(|hit| hit.normal.z <= -CEILING_FACING),
        upper_floor,
    }
}

/// The `s` of every sample from `behind` the origin to `ahead` of it, on the
/// grid of `spacing` fixed along the path.
fn stations(origin: f32, layout: &StripLayout, spacing: f32) -> Vec<f32> {
    let (first, count) = layout.stations(origin, spacing);
    (first..first + count as i64)
        .map(|index| index as f32 * spacing)
        .collect()
}

/// Groups the hits at each station that `keep` accepts into surfaces,
/// starting a new one wherever the body changes or a cast is missed or
/// rejected.
fn profile(
    hits: impl Iterator<Item = (f32, Option<CastHit>)>,
    keep: impl Fn(&CastHit) -> bool,
) -> Profile {
    let mut surfaces = Vec::new();
    let mut open: Option<(BodyKey, Surface)> = None;
    for (s, hit) in hits {
        let hit = hit.filter(|hit| keep(hit));
        let continues = matches!((&open, &hit), (Some((body, _)), Some(hit)) if *body == hit.body);
        if !continues {
            surfaces.extend(open.take().map(|(_, surface)| surface));
        }
        if let Some(hit) = hit {
            let point = SurfacePoint {
                s,
                z: hit.z,
                normal: hit.normal,
            };
            match &mut open {
                Some((_, surface)) => surface.points.push(point),
                None => {
                    open = Some((
                        hit.body,
                        Surface {
                            footing: hit.footing,
                            points: vec![point],
                        },
                    ))
                }
            }
        }
    }
    surfaces.extend(open.map(|(_, surface)| surface));
    Profile { surfaces }
}
