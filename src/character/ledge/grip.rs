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
///
/// A body caught tipped over (a swimmer lying along its stroke, holding by
/// the head end of its axis) stands up about its own centre over
/// `LedgeConfig::settle`: the path is planned for the body upright, and the
/// end carries the lean of the hands' point off it, easing to nothing. The
/// attitude follows the same schedule ([`Grip::pitch`]), so the joint and the
/// attitude ask for one motion; asked to stand at once while the joint pins
/// the head, the two rigid rows fight, and the solver throws the body away.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grip {
    constraint: ConstraintHandle,
    edge_body: Option<RigidBodyHandle>,
    /// Where the joint's edge end started its present motion (the catch's
    /// ease, or the pull-up's climb), in the edge's frame, for the body
    /// upright.
    from: Point3<f32>,
    /// How the body stood when caught.
    caught: Tilt,
    /// Seconds since the catch, to the end of this frame.
    since_catch: f32,
    /// Whether that motion is a pull-up.
    climbing: bool,
    /// Where the joint's edge end carries the body to by the end of this
    /// frame, in the edge's frame; the end is moved there next frame and set
    /// moving on.
    carried: Point3<f32>,
}

/// How far a body's axis tips off upright, and which way.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Tilt {
    /// Radians from upright.
    pitch: f32,
    /// Level direction the axis tips towards; zero for a body upright.
    heading: Vector3<f32>,
}

impl Tilt {
    /// The tilt of a body whose axis (local +Y) points along `axis`.
    fn of(axis: Vector3<f32>) -> Self {
        let level = Vector3::new(axis.x, 0.0, axis.z);
        Self {
            pitch: axis.y.clamp(-1.0, 1.0).acos(),
            heading: level.try_normalize(1.0e-6).unwrap_or_else(Vector3::zeros),
        }
    }

    /// Where a point `reach` up the axis lies from where it would upright,
    /// with the body tipped `pitch` this way.
    fn lean(&self, pitch: f32, reach: f32) -> Vector3<f32> {
        let (sin, cos) = pitch.sin_cos();
        (self.heading * sin + Vector3::y() * (cos - 1.0)) * reach
    }
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
        let reach = frame.config.ledge.hang_reach;
        if matches!(phase, LedgePhase::PullUp { .. }) && !current.climbing {
            let lean = current.caught.lean(current.pitch(frame.config), reach);
            current.from = to_edge.inverse() * (now - lean);
            current.climbing = true;
        }
        current.since_catch += frame.dt;
        let lean = current.caught.lean(current.pitch(frame.config), reach);
        let upright = match phase {
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
        let target = upright + lean;
        let velocity = if frame.dt > 0.0 {
            (target - now) / frame.dt
        } else {
            Vector3::zeros()
        };
        current.aim(world, velocity);
        current.carried = to_edge.inverse() * target;
    }

    /// The pitch the body is held at by the end of this frame: the tilt it
    /// was caught at, eased upright over `LedgeConfig::settle`.
    pub fn pitch(&self, config: &LocomotionConfig) -> f32 {
        self.caught.pitch * (1.0 - smoothstep(self.since_catch / config.ledge.settle))
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
        let to_edge = frame_of(world, sensed.edge.body)?;
        let body = world.body(frame.body)?;
        let caught = Tilt::of(body.rotation() * Vector3::y());
        let reach = frame.config.ledge.hang_reach;
        let held = to_edge.inverse() * anchor;
        let from = to_edge.inverse() * (anchor - caught.lean(caught.pitch, reach));
        // Unbreakable: the solver's limit is an impulse per substep, and
        // moving the edge end each frame asks for more than any hit would.
        let constraint = world.create_constraint(ConstraintKind::BallJoint {
            body_a: sensed.edge.body,
            body_b: frame.body,
            local_anchor_a: held.coords,
            anchor_velocity: Vector3::zeros(),
            local_anchor_b: hang_anchor(frame.config),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        Some(Self {
            constraint,
            edge_body: sensed.edge.body,
            from,
            caught,
            since_catch: 0.0,
            climbing: false,
            carried: held,
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
