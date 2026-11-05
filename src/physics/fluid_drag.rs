//! How a fluid drags a body moving through it: the quadratic law
//! `F = ½ρ·Cd·A·|v|·v` over the silhouette the body shows the flow, and its
//! turning twin `τ = ½ρ·Cω·A·R³·|ω|·ω`, the mean silhouette turning at the
//! body's reach `R`.
//!
//! Every medium uses it, through [`DragArea::exposed`]: water
//! (`water::buoyancy`) over the parts it covers, and the world's air
//! ([`super::Atmosphere`]) over all of them. They differ only in density and
//! in how much of the body is in them.
//!
//! ```text
//!   body's envelope ──(shape changes)──▶ DragArea::of: Cd, mean silhouette,
//!                                        share shown, reach
//!   each substep: parts in the fluid, flow ──▶ DragArea::exposed ──▶ Exposure
//!                                                  ──▶ MediumDrag at density ρ
//! ```

use std::f32::consts::PI;

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;
use super::force_provider::MediumDrag;
use super::silhouette::Silhouette;

/// Drag coefficient of a sphere.
const SPHERE: f32 = 0.47;
/// Drag coefficient of a box, and of a convex hull.
const BOX: f32 = 1.05;
/// Drag coefficient of a capsule.
const CAPSULE: f32 = 0.82;
/// Spin drag coefficient `Cω`, over the mean silhouette at the reach.
///
/// A flat plate of face `S` and half-length `L` turning about its middle:
/// each strip at `r` pushes `½ρ·Cd·(ωr)²·dS` with arm `r`, `½ρ·Cd·ω²·S·L³/4`
/// in all. Its mean silhouette is `S/2`, so `Cω = Cd/2` for a plate's `Cd`.
pub const ANGULAR_COEFFICIENT: f32 = 0.6;
/// Speed through a fluid, m/s, below which a body has no flow direction and
/// shows the fluid its mean silhouette. The drag scales with the speed, so
/// which area stands in there makes no difference.
const FLOW_SPEED_EPSILON: f32 = 1e-3;

/// What a shape presents to a fluid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeDrag {
    /// Drag coefficient `Cd`.
    pub coefficient: f32,
    /// Projected area averaged over every direction, m².
    pub mean_area: f32,
}

impl ShapeDrag {
    /// What `shape` presents.
    pub fn of(shape: &ColliderShape) -> Self {
        // A convex body's projected area, averaged over every direction, is a
        // quarter of its surface (Cauchy).
        let (coefficient, surface) = match shape {
            ColliderShape::Sphere { radius } => (SPHERE, 4.0 * PI * radius * radius),
            ColliderShape::Box { half_extents: h } => {
                (BOX, 8.0 * (h.x * h.y + h.y * h.z + h.x * h.z))
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let r = *radius;
                let cylinder = (2.0 * (half_height - r)).max(0.0);
                (CAPSULE, 2.0 * PI * r * cylinder + 4.0 * PI * r * r)
            }
            ColliderShape::ConvexHull { hull } => (BOX, hull.surface_area()),
        };
        Self {
            coefficient,
            mean_area: 0.25 * surface,
        }
    }
}

/// A body's drag, measured from its shape whenever that changes.
///
/// A property of the body's shape as a whole, not of how it is cut up: a
/// crate of eight boxes drags as the one box they fill. Its mean silhouette
/// is the union of its parts' outlines averaged over directions
/// ([`Silhouette`]), so a gap between parts lets the flow through, and the
/// share of their own silhouettes the parts show is what scales them along
/// any one flow ([`Self::exposed`]). Its drag coefficient is the parts' own,
/// weighted by their area, or the one the body's bulk declares. It spins
/// against a fluid at its reach about its centre of mass, the furthest any
/// part reaches: a dumbbell turns at its ends.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DragArea {
    /// Drag coefficient `Cd`.
    coefficient: f32,
    /// Silhouette averaged over directions, m².
    mean_area: f32,
    /// The share of their own silhouettes the parts show, averaged over
    /// directions: 1 for one part, a half for a cube cut into eight, each
    /// hiding the others.
    shown: f32,
    /// The furthest any part reaches from the centre of mass, m.
    reach: f32,
}

/// The drag areas a body shows a fluid at one moment.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Exposure {
    /// `Cd·A` along the flow, m².
    pub linear: f32,
    /// `Cω·A·R³`, m⁵.
    pub angular: f32,
}

impl Exposure {
    /// The drag of a fluid of `density` on a body showing these areas, moving
    /// through it at `speed` and turning at `spin` (rad/s), towards the
    /// fluid's rest: the quadratic law linearised about the body's motion.
    pub fn drag(&self, density: f32, speed: f32, spin: f32) -> MediumDrag {
        let half_rho = 0.5 * density;
        MediumDrag {
            linear: half_rho * self.linear * speed,
            angular: half_rho * self.angular * spin,
            ..MediumDrag::NONE
        }
    }
}

impl DragArea {
    /// The drag of a body whose envelope is `parts`, each placed in the
    /// body's frame, measured from its centre of mass.
    pub fn of<'a>(parts: impl IntoIterator<Item = EnvelopePart<'a>>) -> Self {
        let parts: SmallVec<[EnvelopePart; 8]> = parts.into_iter().collect();
        let mut weighted_coefficient = 0.0;
        let mut total = 0.0;
        let mut reach: f32 = 0.0;
        for part in &parts {
            let drag = part.drag();
            weighted_coefficient += drag.coefficient * drag.mean_area;
            total += drag.mean_area;
            reach = reach.max(furthest(part));
        }
        if total <= 0.0 {
            return Self::default();
        }
        // One convex part shows all of its silhouette; only a body of
        // several needs measuring.
        let shown = if parts.len() == 1 {
            1.0
        } else {
            Silhouette::default().mean_shown(&parts)
        };
        Self {
            coefficient: weighted_coefficient / total,
            mean_area: shown * total,
            shown,
            reach,
        }
    }

    /// `Cd·A` over the mean silhouette, m².
    pub fn mean_linear(&self) -> f32 {
        self.coefficient * self.mean_area
    }

    /// `Cω·A·R³`, m⁵: the spin drag's area with the whole body in the fluid.
    pub fn angular(&self) -> f32 {
        ANGULAR_COEFFICIENT * self.mean_area * self.reach.powi(3)
    }

    /// What the body shows a fluid it moves through at `flow` (its velocity
    /// relative to the fluid, in the body's frame), with `parts` the body's
    /// envelope paired with how much of each part is in the fluid, 0 to 1.
    ///
    /// Each part shows its own silhouette along the flow, as much of it as is
    /// in the fluid, scaled by the share the parts show of each other: exact
    /// for a body cut on a grid, and the measured mean silhouette over every
    /// direction for any body. The spin drag is the share of the silhouette
    /// in the fluid.
    pub fn exposed<'a>(
        &self,
        parts: impl IntoIterator<Item = (EnvelopePart<'a>, f32)>,
        flow: Vector3<f32>,
    ) -> Exposure {
        let speed = flow.norm();
        let direction = (speed > FLOW_SPEED_EPSILON).then(|| flow / speed);
        let (mut immersed, mut whole) = (0.0, 0.0);
        for (part, share) in parts {
            let area = direction.map_or_else(
                || ShapeDrag::of(part.shape).mean_area,
                |d| {
                    part.shape
                        .projected_area(part.offset.rotation.inverse() * d)
                },
            );
            immersed += share * area;
            whole += area;
        }
        if whole <= 0.0 {
            return Exposure::default();
        }
        Exposure {
            linear: self.coefficient * self.shown * immersed,
            angular: self.angular() * immersed / whole,
        }
    }
}

/// The furthest any point of `part` lies from the body's origin, m.
fn furthest(part: &EnvelopePart) -> f32 {
    let offset = part.offset;
    let from_origin = |p: Point3<f32>| (offset * p).coords.norm();
    match part.shape {
        ColliderShape::Sphere { radius } => from_origin(Point3::origin()) + radius,
        ColliderShape::Box { half_extents: h } => (0..8)
            .map(|corner| {
                let sign = |bit: u32| if corner & (1 << bit) == 0 { -1.0 } else { 1.0 };
                from_origin(Point3::new(sign(0) * h.x, sign(1) * h.y, sign(2) * h.z))
            })
            .fold(0.0, f32::max),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let end = (half_height - radius).max(0.0);
            from_origin(Point3::new(0.0, end, 0.0)).max(from_origin(Point3::new(0.0, -end, 0.0)))
                + radius
        }
        ColliderShape::ConvexHull { hull } => hull
            .vertices
            .iter()
            .map(|v| from_origin(Point3::from(*v)))
            .fold(0.0, f32::max),
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Isometry3, Vector3};

    use super::*;

    /// A box of `half_extents` cut into `cuts` pieces along each axis, as
    /// shapes and their placements.
    fn cut_box(half_extents: Vector3<f32>, cuts: usize) -> (ColliderShape, Vec<Isometry3<f32>>) {
        let piece = half_extents / cuts as f32;
        let at = |k: usize, half: f32| -half * cuts as f32 + half * (2 * k + 1) as f32;
        let offsets = (0..cuts.pow(3))
            .map(|i| {
                Isometry3::translation(
                    at(i % cuts, piece.x),
                    at(i / cuts % cuts, piece.y),
                    at(i / (cuts * cuts), piece.z),
                )
            })
            .collect();
        (
            ColliderShape::Box {
                half_extents: piece,
            },
            offsets,
        )
    }

    fn parts<'a>(
        shape: &'a ColliderShape,
        offsets: &'a [Isometry3<f32>],
    ) -> impl Iterator<Item = EnvelopePart<'a>> + Clone {
        offsets.iter().map(move |o| part(shape, o, None))
    }

    /// A box drags and turns the same however it is cut: into eight boxes,
    /// twenty-seven, or a long box into two cubes. Its mean silhouette is the
    /// whole box's exact quarter-surface to within the directions' sampling,
    /// its reach is the same corner, and along any flow it shows what the
    /// whole box does.
    #[test]
    fn a_body_drags_as_its_shape_however_it_is_cut() {
        let flows = [
            Vector3::x(),
            Vector3::new(1.0, 2.0, 0.5),
            Vector3::new(-0.3, 0.2, 1.0),
        ];
        for (half_extents, cuts) in [
            (Vector3::repeat(0.5), 2),
            (Vector3::repeat(0.5), 3),
            (Vector3::new(0.5, 0.25, 0.25), 2),
        ] {
            let (whole_shape, whole_at) = cut_box(half_extents, 1);
            let (cut_shape, cut_at) = cut_box(half_extents, cuts);
            let whole = DragArea::of(parts(&whole_shape, &whole_at));
            let cut = DragArea::of(parts(&cut_shape, &cut_at));
            let h = half_extents;
            let cauchy = BOX * 2.0 * (h.x * h.y + h.y * h.z + h.x * h.z);
            assert!((whole.mean_linear() - cauchy).abs() < 1e-5 * cauchy);
            assert!(
                (cut.mean_linear() - whole.mean_linear()).abs() < 0.01 * whole.mean_linear(),
                "cut {cut:?} against {whole:?}"
            );
            assert!(
                (cut.reach - whole.reach).abs() < 1e-5,
                "cut {cut:?} against {whole:?}"
            );
            for flow in flows {
                let shows = |area: &DragArea, parts: &mut dyn Iterator<Item = EnvelopePart>| {
                    area.exposed(parts.map(|p| (p, 1.0)), flow).linear
                };
                let (cut_shows, whole_shows) = (
                    shows(&cut, &mut parts(&cut_shape, &cut_at)),
                    shows(&whole, &mut parts(&whole_shape, &whole_at)),
                );
                assert!(
                    (cut_shows - whole_shows).abs() < 0.01 * whole_shows,
                    "along {flow:?}: cut {cut_shows}, whole {whole_shows}"
                );
            }
        }
    }

    /// A sphere's silhouette is its great circle, from every side.
    #[test]
    fn a_sphere_drags_by_its_great_circle() {
        let sphere = ColliderShape::Sphere { radius: 0.3 };
        let area = DragArea::of([part(&sphere, &Isometry3::identity(), None)]).mean_linear();
        let exact = SPHERE * PI * 0.09;
        assert!(
            (area - exact).abs() < 0.01 * exact,
            "{area} against {exact}"
        );
    }

    /// Two balls on a bar drag by their two discs, not by the hull round
    /// both, and turn against a fluid at their ends.
    #[test]
    fn a_dumbbell_drags_by_its_balls_and_turns_at_its_ends() {
        let ball = ColliderShape::Sphere { radius: 0.1 };
        let one = DragArea::of([part(&ball, &Isometry3::identity(), None)]);
        let ends = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let dumbbell = DragArea::of(ends.iter().map(|o| part(&ball, o, None)));
        // The balls' circles are drawn circumscribed, about half a percent
        // over, and hide each other only end on.
        assert!(
            (dumbbell.mean_linear() - 2.0 * one.mean_linear()).abs() < 0.02 * one.mean_linear(),
            "dumbbell {dumbbell:?}, one ball {one:?}"
        );
        assert!((dumbbell.reach - 1.1).abs() < 1e-5, "{dumbbell:?}");
    }

    /// A body that declares its drag coefficient drags by it over the same
    /// silhouette, and turns as before: spin drag has its own coefficient.
    #[test]
    fn a_declared_coefficient_replaces_the_shapes() {
        let ball = ColliderShape::Sphere { radius: 0.3 };
        let at = Isometry3::identity();
        let own = DragArea::of([part(&ball, &at, None)]);
        let porous = DragArea::of([part(&ball, &at, Some(1.0))]);
        assert!((porous.mean_linear() - own.mean_linear() / SPHERE).abs() < 1e-6);
        assert_eq!(porous.angular(), own.angular());
    }

    /// A capsule moving along its axis shows the flow its caps' disc; across
    /// it, the disc and the cylinder's side; and at rest, its mean.
    #[test]
    fn a_capsule_moving_end_on_meets_less_drag_than_side_on() {
        let shape = ColliderShape::Capsule {
            half_height: 0.5,
            radius: 0.17,
        };
        let at = Isometry3::identity();
        let area = DragArea::of([part(&shape, &at, None)]);
        let along =
            |flow: Vector3<f32>| area.exposed([(part(&shape, &at, None), 1.0)], flow).linear;
        let (end_on, side_on, still) = (
            along(Vector3::y()),
            along(Vector3::x()),
            along(Vector3::zeros()),
        );
        assert!(
            end_on < still && still < side_on,
            "{end_on} {still} {side_on}"
        );
        assert!((side_on / end_on - (1.0 + 0.34 * 0.66 / (PI * 0.0289))).abs() < 1e-3);
        assert!((still - area.mean_linear()).abs() < 1e-6);
    }

    /// A body half in a fluid shows it the half that is in it, and spins
    /// against it by that half.
    #[test]
    fn a_body_half_in_a_fluid_shows_it_half() {
        let (shape, at) = cut_box(Vector3::repeat(0.5), 2);
        let area = DragArea::of(parts(&shape, &at));
        let flow = Vector3::new(0.3, 0.0, 1.0);
        let whole = area.exposed(parts(&shape, &at).map(|p| (p, 1.0)), flow);
        let lower = area.exposed(
            parts(&shape, &at).map(|p| {
                (
                    p,
                    if p.offset.translation.y < 0.0 {
                        1.0
                    } else {
                        0.0
                    },
                )
            }),
            flow,
        );
        assert!((lower.linear - 0.5 * whole.linear).abs() < 1e-5 * whole.linear);
        assert!((lower.angular - 0.5 * whole.angular).abs() < 1e-5 * whole.angular);
        assert!((whole.angular - area.angular()).abs() < 1e-6 * whole.angular);
    }

    fn part<'a>(
        shape: &'a ColliderShape,
        offset: &'a Isometry3<f32>,
        drag_coefficient: Option<f32>,
    ) -> EnvelopePart<'a> {
        EnvelopePart {
            shape,
            offset,
            drag_coefficient,
        }
    }
}
