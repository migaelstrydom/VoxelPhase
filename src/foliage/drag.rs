//! The soft part of foliage: leaves that slow what moves through them
//! instead of stopping it.
//!
//! ```text
//!   DragVolume (a clump's lobes, posed fixed or on its core's body)
//!        │  each substep, per body near one
//!        ▼
//!   FoliageDrag ──▶ MediumDrag towards the leaves' own velocity
//!   (SubstepForceProvider)
//! ```
//!
//! Nothing collides with leaves. Leaves are one more fluid, under the law
//! the air and water drag by (`physics::DragArea::exposed`): a body is
//! dragged by the silhouette of its own parts, each by how much of it is in
//! the leaves, measured at points spread through the part. A grenade thrown
//! through a shrub comes out slowed; the player wades through one.
//!
//! The drag acts on both sides. A body in a loose clump's leaves and the
//! clump are each pulled towards their shared, momentum-weighted velocity,
//! so a thrown clump slows as it drags what it passes and cannot haul a
//! heavier body along. A rooted shrub is immovable: what wades through it is
//! dragged towards rest. Only the linear drag is shared; the leaves' pull on
//! a body's spin has no reaction on the clump.
//!
//! The air's drag on a loose clump is the world's (`physics::Atmosphere`),
//! through the bulk its core declares: its whole lobes, not the core.

use std::sync::Arc;

use generational_arena::Arena;
use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;

use super::shape::FoliageShape;
use crate::physics::{
    Collider, ColliderShape, EnvelopePart, ForceContext, ForceOutput, MediumDrag, PhysicsWorld,
    RigidBody, RigidBodyHandle, SubstepForceProvider,
};

/// How hard leaves drag.
#[derive(Debug, Clone, Copy)]
pub struct LeafDrag {
    /// The leaves' density as a fluid, kg/m³, in the law every fluid drags
    /// by, `F = ½ρ·Cd·A·|v|·v`: a quarter of water's, so wading through a
    /// shrub is wading through shallow water.
    pub density: f32,
    /// The share of a character's stride leaves take wholly round it
    /// (`thicket`): what slows a wade where the drag alone, against a
    /// character's drive, does not.
    pub stride_loss: f32,
}

impl Default for LeafDrag {
    fn default() -> Self {
        Self {
            density: 250.0,
            stride_loss: 0.9,
        }
    }
}

/// Where a volume's lobes are.
#[derive(Debug, Clone, Copy)]
pub enum VolumeFrame {
    /// Rooted: lobes posed once, in the world.
    Fixed(Isometry3<f32>),
    /// Loose: lobes riding on the core's body, read afresh every substep.
    Body(RigidBodyHandle),
}

/// One clump's leaves, as drag sees them.
#[derive(Debug, Clone)]
pub struct DragVolume {
    /// The lobes, in the clump's frame.
    pub shape: Arc<FoliageShape>,
    /// Where that frame is.
    pub frame: VolumeFrame,
}

/// Where a frame is and how it moves.
#[derive(Debug, Clone, Copy)]
struct FrameMotion {
    pose: Isometry3<f32>,
    /// Velocity of the frame's origin.
    velocity: Vector3<f32>,
    spin: Vector3<f32>,
}

impl FrameMotion {
    fn of(body: &RigidBody) -> Self {
        Self {
            pose: Isometry3::from_parts(body.position().coords.into(), body.rotation()),
            velocity: body.linear_velocity(),
            spin: body.angular_velocity(),
        }
    }
}

/// A lobe posed in the world, with the velocity and spin of the leaves in it.
struct PosedLobe {
    centre: Point3<f32>,
    radius: f32,
    /// Velocity of the leaves at the lobe's centre.
    velocity: Vector3<f32>,
    /// Spin of the leaves.
    spin: Vector3<f32>,
}

impl DragVolume {
    /// Where this volume's frame is now, and how it moves: `None` if its
    /// body has gone.
    fn motion(
        &self,
        bodies: impl Fn(RigidBodyHandle) -> Option<FrameMotion>,
    ) -> Option<FrameMotion> {
        match self.frame {
            VolumeFrame::Fixed(pose) => Some(FrameMotion {
                pose,
                velocity: Vector3::zeros(),
                spin: Vector3::zeros(),
            }),
            VolumeFrame::Body(handle) => bodies(handle),
        }
    }

    fn owner(&self) -> Option<RigidBodyHandle> {
        match self.frame {
            VolumeFrame::Fixed(_) => None,
            VolumeFrame::Body(handle) => Some(handle),
        }
    }

    /// The lobes posed in the world, from the volume's pose and motion.
    fn posed(&self, motion: FrameMotion) -> impl Iterator<Item = PosedLobe> + '_ {
        self.shape.lobes.iter().map(move |lobe| {
            let centre = motion.pose * lobe.centre;
            let arm = centre - Point3::from(motion.pose.translation.vector);
            PosedLobe {
                centre,
                radius: lobe.radius,
                velocity: motion.velocity + motion.spin.cross(&arm),
                spin: motion.spin,
            }
        })
    }
}

/// How deep a ball of `radius` at `centre` is in a lobe, from 0 (clear of
/// it) to 1 (wholly inside): the overlap of the two along the line between
/// them, over the ball's diameter. A body bigger than the lobe is never
/// wholly in it.
fn immersion(centre: Point3<f32>, radius: f32, lobe: &PosedLobe) -> f32 {
    let distance = (centre - lobe.centre).magnitude();
    let overlap = lobe.radius + radius - distance;
    (overlap / (2.0 * radius))
        .clamp(0.0, 1.0)
        .min(lobe.radius / radius)
}

/// How deep a ball of `radius` at `centre` is in `volume`'s leaves now.
pub(crate) fn immersion_in(
    volume: &DragVolume,
    world: &PhysicsWorld,
    centre: Point3<f32>,
    radius: f32,
) -> f32 {
    let Some(motion) = volume.motion(|h| world.body(h).map(FrameMotion::of)) else {
        return 0.0;
    };
    volume
        .posed(motion)
        .map(|lobe| immersion(centre, radius, &lobe))
        .fold(0.0, f32::max)
}

/// Foliage's drag on every body near a clump, recomputed each substep.
pub struct FoliageDrag {
    drag: LeafDrag,
    volumes: Vec<DragVolume>,
    affected: Vec<RigidBodyHandle>,
}

impl FoliageDrag {
    /// Drag from `volumes` on whichever of `candidates` could reach one of
    /// them within a frame of `frame_dt`. A volume's own body is never
    /// dragged by its own leaves.
    pub fn new(
        drag: LeafDrag,
        volumes: Vec<DragVolume>,
        world: &PhysicsWorld,
        candidates: impl IntoIterator<Item = RigidBodyHandle>,
        frame_dt: f32,
    ) -> Self {
        let bounds: Vec<(Point3<f32>, f32)> = volumes
            .iter()
            .filter_map(|v| {
                let motion = v.motion(|h| world.body(h).map(FrameMotion::of))?;
                Some((
                    Point3::from(motion.pose.translation.vector),
                    v.shape.bounding_radius(),
                ))
            })
            .collect();
        let affected = if bounds.is_empty() {
            Vec::new()
        } else {
            candidates
                .into_iter()
                .filter(|&handle| {
                    let Some(body) = world.body(handle) else {
                        return false;
                    };
                    if !body.is_dynamic() {
                        return false;
                    }
                    let reach = world.reach(handle) + body.linear_velocity().magnitude() * frame_dt;
                    bounds.iter().any(|(centre, radius)| {
                        (body.position() - centre).magnitude() < radius + reach
                    })
                })
                .collect()
        };
        Self {
            drag,
            volumes,
            affected,
        }
    }

    /// Whether any body is near enough to be dragged this frame.
    pub fn is_idle(&self) -> bool {
        self.affected.is_empty()
    }
}

impl SubstepForceProvider for FoliageDrag {
    fn affected_bodies(&self) -> &[RigidBodyHandle] {
        &self.affected
    }

    fn compute_force(&self, handle: RigidBodyHandle, ctx: &ForceContext) -> ForceOutput {
        let mut out = ForceOutput::zero();
        let Some(body) = ctx.bodies.get(handle.0) else {
            return out;
        };
        let motion = |h: RigidBodyHandle| ctx.bodies.get(h.0).map(FrameMotion::of);
        for volume in &self.volumes {
            let Some(frame) = volume.motion(motion) else {
                continue;
            };
            let lobes: SmallVec<[PosedLobe; 8]> = volume.posed(frame).collect();
            let density = self.drag.density;
            if volume.owner() == Some(handle) {
                // Its own leaves, pushed back by every other body in them.
                for passer in self
                    .affected
                    .iter()
                    .filter(|&&other| other != handle)
                    .filter_map(|other| ctx.bodies.get(other.0))
                {
                    if let Some(drag) = in_leaves(passer, ctx.colliders, &lobes, density) {
                        let pull = Pull::between(drag, passer, Some(body.mass()));
                        out.drag = out.drag.combined(pull.on_leaves);
                    }
                }
            } else if let Some(drag) = in_leaves(body, ctx.colliders, &lobes, density) {
                let carrier = volume
                    .owner()
                    .and_then(|owner| ctx.bodies.get(owner.0))
                    .map(RigidBody::mass);
                out.drag = out
                    .drag
                    .combined(Pull::between(drag, body, carrier).on_body);
            }
        }
        out
    }
}

/// Width, m, over which a point goes from clear of a lobe to inside it, so a
/// part sliding into leaves is dragged smoothly rather than a sample at a
/// time.
const LEAF_EDGE: f32 = 0.1;

/// The drag of leaves on `body` were they immovable: towards their velocity
/// and spin where it is in them, over the silhouette of its parts, each by
/// the share of it in the leaves. `None` when no part is in them.
fn in_leaves(
    body: &RigidBody,
    colliders: &Arena<Collider>,
    lobes: &[PosedLobe],
    density: f32,
) -> Option<MediumDrag> {
    let (position, rotation) = (body.position(), body.rotation());
    let mut parts: SmallVec<[(EnvelopePart, f32); 8]> = SmallVec::new();
    let (mut weight, mut velocity, mut spin) = (0.0, Vector3::zeros(), Vector3::zeros());
    for part in body.envelope(colliders) {
        let pose = part.world_transform(position, rotation);
        let points = samples(part.shape);
        let mut share = 0.0;
        for point in &points {
            if let Some((depth, lobe)) = membership(pose * Point3::from(*point), lobes) {
                share += depth;
                weight += depth;
                velocity += lobe.velocity * depth;
                spin += lobe.spin * depth;
            }
        }
        parts.push((part, share / points.len() as f32));
    }
    if weight <= 0.0 {
        return None;
    }
    let (velocity, spin) = (velocity / weight, spin / weight);
    let flow = rotation.inverse() * (body.linear_velocity() - velocity);
    let turning = (body.angular_velocity() - spin).magnitude();
    let drag = body
        .drag_area()
        .exposed(parts, flow)
        .drag(density, flow.norm(), turning);
    Some(MediumDrag {
        velocity,
        spin,
        ..drag
    })
}

/// How far a point is in the leaves, 0 to 1, and the lobe it is deepest in.
/// Overlapping lobes are one mass of leaves, not two layers of them.
fn membership<'a>(point: Point3<f32>, lobes: &'a [PosedLobe]) -> Option<(f32, &'a PosedLobe)> {
    lobes
        .iter()
        .map(|lobe| {
            let inside = lobe.radius - (point - lobe.centre).magnitude();
            ((inside / LEAF_EDGE + 0.5).clamp(0.0, 1.0), lobe)
        })
        .filter(|(depth, _)| *depth > 0.0)
        .max_by(|a, b| a.0.total_cmp(&b.0))
}

/// Points spread through a shape, in its own frame: the centres of a 3×3×3
/// grid of cells over the box it fills, less those outside a round shape.
fn samples(shape: &ColliderShape) -> SmallVec<[Vector3<f32>; 27]> {
    let (centre, half) = match shape {
        ColliderShape::Sphere { radius } => (Vector3::zeros(), Vector3::repeat(*radius)),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => (
            Vector3::zeros(),
            Vector3::new(*radius, *half_height, *radius),
        ),
        ColliderShape::Box { half_extents } => (Vector3::zeros(), *half_extents),
        ColliderShape::ConvexHull { hull } => (hull.box_center, hull.box_half_extents),
    };
    let mut points = SmallVec::new();
    for i in -1..=1 {
        for j in -1..=1 {
            for k in -1..=1 {
                let cell = Vector3::new(i as f32, j as f32, k as f32) * (2.0 / 3.0);
                let point = centre + cell.component_mul(&half);
                if within(shape, point) {
                    points.push(point);
                }
            }
        }
    }
    if points.is_empty() {
        points.push(centre);
    }
    points
}

/// Whether a point in a shape's frame is inside it. A box is its box; a
/// hull is taken as the box it fills.
fn within(shape: &ColliderShape, point: Vector3<f32>) -> bool {
    match shape {
        ColliderShape::Sphere { radius } => point.magnitude() <= *radius,
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let end = (half_height - radius).max(0.0);
            let axis = Vector3::new(0.0, point.y.clamp(-end, end), 0.0);
            (point - axis).magnitude() <= *radius
        }
        ColliderShape::Box { .. } | ColliderShape::ConvexHull { .. } => true,
    }
}

/// Leaves' drag shared between a body and the clump carrying the leaves.
struct Pull {
    on_body: MediumDrag,
    on_leaves: MediumDrag,
}

impl Pull {
    /// `drag`, the leaves' drag on `body` were they immovable, shared with
    /// leaves whose clump weighs `carrier` kg (`None`: rooted, immovable).
    ///
    /// Two bodies of mass `m` and `M` dragged against each other by `k`
    /// close their relative velocity at `k(1/m + 1/M)`, about their shared
    /// momentum-weighted velocity `v̄`. Each is the same as being dragged
    /// towards `v̄` alone: the body by `k(1 + m/M)`, the clump by
    /// `k(1 + M/m)`. With the clump immovable, the body is dragged by `k`
    /// towards the leaves, as before.
    fn between(drag: MediumDrag, body: &RigidBody, carrier: Option<f32>) -> Self {
        let mass = body.mass();
        let Some(carrier) = carrier.filter(|m| m.is_finite() && *m > 0.0) else {
            return Self {
                on_body: drag,
                on_leaves: MediumDrag::NONE,
            };
        };
        let shared = (body.linear_velocity() * mass + drag.velocity * carrier) / (mass + carrier);
        Self {
            on_body: MediumDrag {
                linear: drag.linear * (1.0 + mass / carrier),
                velocity: shared,
                ..drag
            },
            on_leaves: MediumDrag {
                linear: drag.linear * (1.0 + carrier / mass),
                angular: 0.0,
                velocity: shared,
                spin: Vector3::zeros(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foliage::shape::Lobe;

    fn lobe_at(centre: Point3<f32>, radius: f32) -> PosedLobe {
        PosedLobe {
            centre,
            radius,
            velocity: Vector3::zeros(),
            spin: Vector3::zeros(),
        }
    }

    /// A ball is clear of a lobe it does not touch, wholly in one it sits in
    /// the middle of, and partly in one it straddles.
    #[test]
    fn immersion_runs_from_clear_to_inside() {
        let lobe = lobe_at(Point3::origin(), 0.5);
        assert_eq!(immersion(Point3::new(1.0, 0.0, 0.0), 0.1, &lobe), 0.0);
        assert_eq!(immersion(Point3::origin(), 0.1, &lobe), 1.0);
        let half = immersion(Point3::new(0.5, 0.0, 0.0), 0.1, &lobe);
        assert!((half - 0.5).abs() < 1e-5, "{half}");
        // Bigger than the lobe: never more in than the lobe can hold.
        assert!(immersion(Point3::origin(), 1.0, &lobe) <= 0.5);
    }

    /// A loose volume's lobes move with its body: a point off the centre
    /// of a spinning clump moves as the spin carries it.
    #[test]
    fn a_spinning_clump_carries_its_leaves_round() {
        let volume = DragVolume {
            shape: Arc::new(FoliageShape {
                lobes: vec![Lobe {
                    centre: Point3::new(1.0, 0.0, 0.0),
                    radius: 0.3,
                }],
            }),
            frame: VolumeFrame::Fixed(Isometry3::identity()),
        };
        let lobe = volume
            .posed(FrameMotion {
                pose: Isometry3::identity(),
                velocity: Vector3::zeros(),
                spin: Vector3::new(0.0, 2.0, 0.0),
            })
            .next()
            .unwrap();
        assert!((lobe.velocity - Vector3::new(0.0, 0.0, -2.0)).magnitude() < 1e-5);
    }
}
