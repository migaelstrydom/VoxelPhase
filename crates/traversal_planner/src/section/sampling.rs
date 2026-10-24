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

/// Sphere casts in path coordinates: what a section is sampled from. The game
/// casts in its physics world; authored scenes cast against their shapes.
pub trait SphereCaster {
    /// A sphere of `radius` centred over `(s, d)`, cast down onto the floor.
    fn cast_down(&self, s: f32, d: f32, radius: f32) -> Option<CastHit>;

    /// A sphere of `radius` centred under `(s, d)` at the body's height, cast
    /// up into the ceiling.
    fn cast_up(&self, s: f32, d: f32, radius: f32) -> Option<CastHit>;
}

/// Samples the strip around `origin` (the body's `s`), by the same rules
/// whatever the caster: the body floor and ceiling along the path with the
/// capsule's radius, footholds along every lane with a foot's.
pub fn sample_section(
    caster: &impl SphereCaster,
    layout: &StripLayout,
    params: &GaitParams,
    origin: f32,
) -> Section {
    let body_radius = params.body.radius;
    let foot_radius = params.foot.radius;
    let least_foot_normal_z = params.foot.max_slope_degrees.to_radians().cos();

    let body_stations = stations(origin, layout, layout.body_spacing);
    let foot_stations = stations(origin, layout, layout.foot_spacing);

    let body_floor = profile(
        &body_stations,
        |s| caster.cast_down(s, 0.0, body_radius),
        |_| true,
    );
    let ceiling = profile(
        &body_stations,
        |s| caster.cast_up(s, 0.0, body_radius),
        |_| true,
    );
    let foothold_lanes = layout
        .lanes
        .iter()
        .map(|&d| FootholdLane {
            d,
            profile: profile(
                &foot_stations,
                |s| caster.cast_down(s, d, foot_radius),
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
        foothold_lanes,
    }
}

/// The `s` of every sample from `behind` the origin to `ahead` of it.
fn stations(origin: f32, layout: &StripLayout, spacing: f32) -> Vec<f32> {
    let start = origin - layout.behind;
    let count = ((layout.behind + layout.ahead) / spacing).round() as usize + 1;
    (0..count).map(|i| start + i as f32 * spacing).collect()
}

/// Casts at every station and groups the hits `keep` accepts into surfaces,
/// starting a new one wherever the body changes or a cast is missed or
/// rejected.
fn profile(
    stations: &[f32],
    cast: impl Fn(f32) -> Option<CastHit>,
    keep: impl Fn(&CastHit) -> bool,
) -> Profile {
    let mut surfaces = Vec::new();
    let mut open: Option<(BodyKey, Surface)> = None;
    for &s in stations {
        let hit = cast(s).filter(|hit| keep(hit));
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
