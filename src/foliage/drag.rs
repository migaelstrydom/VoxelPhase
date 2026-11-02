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
//! Nothing collides with leaves. A body overlapping a lobe is dragged
//! towards the velocity of the leaves there, as water drags a body wading
//! through it: quadratic in the speed through them and proportional to the
//! body's cross-section and how deep into the lobe it is. A grenade thrown
//! through a shrub comes out slowed; the player wades through one.
//!
//! The leaves are not pushed back. A body pushing through a loose clump
//! meets its core, which is what moves it; the drag's reaction on the
//! clump is left out.
//!
//! A loose clump is also dragged by the air, quadratically, to a low
//! terminal speed: a ball of leaves is mostly air. That is what keeps a
//! blast's shove, an impulse whatever the mass, from firing a light clump
//! off at sixty metres a second, and it is why one falls floatily.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};

use super::shape::FoliageShape;
use crate::physics::{
    ForceContext, ForceOutput, MediumDrag, PhysicsWorld, RigidBody, RigidBodyHandle,
    SubstepForceProvider,
};

/// How hard leaves drag.
#[derive(Debug, Clone, Copy)]
pub struct LeafDrag {
    /// The drag's density, kg/m³, in the quadratic law `F = c·A·|v|·v` for a
    /// body of cross-section `A` wholly inside the leaves. Water's is about
    /// 500; leaves are a quarter of it, so wading through a shrub is wading
    /// through shallow water, and a grenade through one loses about a third
    /// of its speed.
    pub density: f32,
    /// Angular drag for each unit of linear drag, per square metre of the
    /// body's radius: how much leaves brushing past slow a body's spin.
    pub angular_per_linear: f32,
    /// The speed, m/s, a loose clump falls at through still air, where the
    /// air's drag on it matches its weight.
    pub terminal_speed: f32,
    /// The share of a character's stride leaves take wholly round it
    /// (`thicket`): what slows a wade where the drag alone, against a
    /// character's drive, does not.
    pub stride_loss: f32,
}

impl Default for LeafDrag {
    fn default() -> Self {
        Self {
            density: 120.0,
            angular_per_linear: 0.4,
            terminal_speed: 7.0,
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
    /// them within a frame of `frame_dt`, and the air's on every loose
    /// clump. A volume's own body is never dragged by its own leaves.
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
        let mut affected = affected;
        for owner in volumes.iter().filter_map(DragVolume::owner) {
            if !affected.contains(&owner) && world.body(owner).is_some_and(|b| b.is_dynamic()) {
                affected.push(owner);
            }
        }
        Self {
            drag,
            volumes,
            affected,
        }
    }

    /// The air's drag on a loose clump: quadratic, `m·g·|v|·v / v_t²`, so
    /// it balances the clump's weight at the terminal speed.
    fn air(&self, body: &RigidBody, gravity: f32) -> MediumDrag {
        let speed = body.linear_velocity().magnitude();
        let terminal = self.drag.terminal_speed;
        MediumDrag {
            linear: body.mass() * gravity * speed / (terminal * terminal),
            ..MediumDrag::NONE
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
        let lobes: Vec<PosedLobe> = self
            .volumes
            .iter()
            .filter(|v| v.owner() != Some(handle))
            .filter_map(|v| Some(v.posed(v.motion(motion)?).collect::<Vec<_>>()))
            .flatten()
            .collect();
        if self.volumes.iter().any(|v| v.owner() == Some(handle)) {
            out.drag = self.air(body, ctx.gravity_magnitude);
        }

        for &collider_handle in body.colliders() {
            let Some(collider) = ctx.colliders.get(collider_handle.0) else {
                continue;
            };
            let centre = collider.world_center(body.position(), body.rotation());
            let radius = collider.shape().bounding_radius();
            // The deepest lobe it is in: overlapping lobes are one mass of
            // leaves, not two layers of them.
            let Some((depth, lobe)) = lobes
                .iter()
                .map(|lobe| (immersion(centre, radius, lobe), lobe))
                .max_by(|a, b| a.0.total_cmp(&b.0))
            else {
                continue;
            };
            if depth <= 0.0 {
                continue;
            }
            let arm = centre - body.position();
            let velocity = body.linear_velocity() + body.angular_velocity().cross(&arm);
            let speed = (velocity - lobe.velocity).magnitude();
            let area = std::f32::consts::PI * radius * radius;
            let linear = self.drag.density * area * depth * speed;
            out.drag = out.drag.combined(MediumDrag {
                linear,
                angular: linear * self.drag.angular_per_linear * radius * radius,
                velocity: lobe.velocity,
                spin: lobe.spin,
            });
        }
        out
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
