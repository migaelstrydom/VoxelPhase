use nalgebra::{Point3, Vector3};

use crate::character::locomotion::{LedgePhase, LocomotionState};
use crate::character::LocomotionConfig;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyHandle};

use super::edge::{frame_of, SensedEdge, WorldEdge};

/// The hands' hold on a ledge: a ball joint from the edge's body (the world,
/// for terrain) to a point on the hanger's axis, `hang_reach` above its
/// centre. Only the edge's end moves: eased from where the body was caught to
/// the hang, kept under the hands as they shimmy, and lifted over the lip to
/// pull up, so every path is in the edge's frame. Each frame the end stands
/// where the body should be and is given the velocity that takes it to the
/// next point of its path, which the joint carries the body along at.
/// Rotation is free; the body's attitude control keeps it upright, and it
/// swings on the joint. Only a body that bears the hanger has an edge to hold
/// (`LedgeConfig::bearing_mass_ratio`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grip {
    constraint: ConstraintHandle,
    edge_body: Option<RigidBodyHandle>,
    /// Where the joint's edge end started its present motion (the catch's
    /// ease, or the pull-up's climb), in the edge's frame.
    from: Point3<f32>,
    /// Whether that motion is a pull-up.
    climbing: bool,
    /// Where the joint's edge end carries the body to by the end of this
    /// frame, in the edge's frame; the end is moved there next frame and set
    /// moving on.
    carried: Point3<f32>,
}

/// What [`Grip::reconcile`] reads this frame.
pub struct GripFrame<'a> {
    pub body: RigidBodyHandle,
    pub state: &'a LocomotionState,
    /// The edge the hands hold, as sensed this frame.
    pub sensed: Option<&'a SensedEdge>,
    pub config: &'a LocomotionConfig,
    pub dt: f32,
}

impl Grip {
    /// Make `grip` match the state: a joint while in a Ledge, on the edge
    /// held, its edge end placed for the phase; none otherwise.
    pub fn reconcile(grip: &mut Option<Grip>, world: &mut PhysicsWorld, frame: &GripFrame) {
        let held = match (frame.state, frame.sensed) {
            (LocomotionState::Ledge(ledge), Some(sensed)) => ledge
                .edge
                .in_world(world)
                .map(|at| (ledge.phase, sensed, at)),
            _ => None,
        };
        let Some((phase, sensed, at)) = held else {
            if let Some(old) = grip.take() {
                old.release(world);
            }
            return;
        };
        if grip.is_some_and(|g| g.edge_body != sensed.edge.body) {
            grip.take().unwrap().release(world);
        }
        let Some(to_edge) = frame_of(world, sensed.edge.body) else {
            return;
        };
        let Some(anchor) = world
            .body(frame.body)
            .map(|body| body.position() + body.rotation() * hang_anchor(frame.config))
        else {
            return;
        };
        if grip.is_none() {
            *grip = Grip::create(world, sensed, anchor, frame);
        }
        let Some(current) = grip.as_mut() else {
            return;
        };
        current.settle_at(world, current.carried);
        let now = to_edge * current.carried;
        if matches!(phase, LedgePhase::PullUp { .. }) && !current.climbing {
            current.from = current.carried;
            current.climbing = true;
        }
        let target = match phase {
            LedgePhase::Grab { elapsed } => {
                let t = smoothstep(elapsed / frame.config.ledge.settle);
                let from = to_edge * current.from;
                from + (hang_point(&at, frame.config) - from) * t
            }
            LedgePhase::Hang { .. } => hang_point(&at, frame.config),
            LedgePhase::PullUp { elapsed, duration } => {
                climb(&at, to_edge * current.from, frame.config).at(smoothstep(elapsed / duration))
            }
        };
        let velocity = if frame.dt > 0.0 {
            (target - now) / frame.dt
        } else {
            Vector3::zeros()
        };
        current.aim(world, velocity);
        current.carried = to_edge.inverse() * target;
    }

    /// Let go: the joint goes.
    fn release(self, world: &mut PhysicsWorld) {
        world.remove_constraint(self.constraint);
    }

    /// Whether the joint still holds: there, and not torn.
    pub fn holds(&self, world: &PhysicsWorld) -> bool {
        world
            .constraint(self.constraint)
            .is_some_and(|constraint| constraint.active)
    }

    fn create(
        world: &mut PhysicsWorld,
        sensed: &SensedEdge,
        anchor: Point3<f32>,
        frame: &GripFrame,
    ) -> Option<Self> {
        let from = frame_of(world, sensed.edge.body)?.inverse() * anchor;
        // Unbreakable: the solver's limit is an impulse per substep, and
        // moving the edge end each frame asks for more than any hit would.
        let constraint = world.create_constraint(ConstraintKind::BallJoint {
            body_a: sensed.edge.body,
            body_b: frame.body,
            local_anchor_a: from.coords,
            anchor_velocity: Vector3::zeros(),
            local_anchor_b: hang_anchor(frame.config),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        Some(Self {
            constraint,
            edge_body: sensed.edge.body,
            from,
            climbing: false,
            carried: from,
        })
    }

    /// Set the joint's edge end moving at `velocity` from where it stands,
    /// which is where the body was carried to last frame: the rows carry
    /// the body along at that speed, and the position correction meets only
    /// drift. Moved there in one step instead, the end would lead the body
    /// by a frame's travel, and the correction's velocity term would fight
    /// the move.
    fn aim(&self, world: &mut PhysicsWorld, velocity: Vector3<f32>) {
        if let Some(constraint) = world.constraint_mut(self.constraint) {
            if let ConstraintKind::BallJoint {
                anchor_velocity, ..
            } = &mut constraint.kind
            {
                *anchor_velocity = velocity;
            }
        }
    }

    /// Move the joint's edge end to `local`, in the joint's frame: where the
    /// body was carried to over the last frame.
    fn settle_at(&self, world: &mut PhysicsWorld, local: Point3<f32>) {
        if let Some(constraint) = world.constraint_mut(self.constraint) {
            if let ConstraintKind::BallJoint { local_anchor_a, .. } = &mut constraint.kind {
                *local_anchor_a = local.coords;
            }
        }
    }
}

/// The joint's end on the hanger, in its frame: on its axis, where its hands
/// hold above its centre.
fn hang_anchor(config: &LocomotionConfig) -> Vector3<f32> {
    Vector3::y() * config.ledge.hang_reach
}

/// Where the hanger's end of the joint hangs: level with the hands, out
/// from the face (which a bevelled lip sets back from the hands) so the
/// capsule hangs `face_gap` clear of it.
fn hang_point(edge: &WorldEdge, config: &LocomotionConfig) -> Point3<f32> {
    let ledge = &config.ledge;
    edge.face() + edge.outward.into_inner() * (config.collider_radius + ledge.face_gap)
}

/// A pull-up's path for the joint's hanger end: straight up from where the
/// climb began until the capsule's bottom clears the lip, then in over the
/// top to where the body stands.
fn climb(edge: &WorldEdge, from: Point3<f32>, config: &LocomotionConfig) -> Path {
    let ledge = &config.ledge;
    let up = Vector3::y() * ledge.hang_reach;
    let clear = edge.grip().y + ledge.lip_clearance + config.collider_half_height;
    let lifted = Point3::new(
        from.x,
        clear.max(from.y - ledge.hang_reach) + ledge.hang_reach,
        from.z,
    );
    let stand = edge.stand.map_or(lifted, |stand| stand + up);
    Path([from, lifted, stand])
}

/// Three points joined by straight runs.
struct Path([Point3<f32>; 3]);

impl Path {
    /// The point `t` of the way along, by length.
    fn at(&self, t: f32) -> Point3<f32> {
        let [a, b, c] = self.0;
        let (first, second) = ((b - a).norm(), (c - b).norm());
        let along = t.clamp(0.0, 1.0) * (first + second);
        if along <= first && first > 0.0 {
            a + (b - a) * (along / first)
        } else if second > 0.0 {
            b + (c - b) * ((along - first) / second)
        } else {
            c
        }
    }
}

fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
