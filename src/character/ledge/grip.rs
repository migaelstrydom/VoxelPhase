use nalgebra::{Point3, Vector3};

use crate::character::locomotion::{LedgePhase, LocomotionState, StickOnEdge};
use crate::character::LocomotionConfig;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyHandle};

use super::edge::{frame_of, SensedEdge, WorldEdge};

/// The hands' hold on a ledge: a ball joint from the edge to a point on the
/// hanger's axis, `hang_reach` above its centre. Only the edge's end moves:
/// eased from where the body was caught to the hang, slid along the edge to
/// shimmy, and lifted over the lip to pull up, so every path is in the edge's
/// frame. Each frame the end stands where the body should be and is given
/// the velocity that takes it to the next point of its path, which the joint
/// carries the body along at. Rotation is free; the body's attitude control
/// keeps it upright, and it swings on the joint.
///
/// The joint is anchored to the edge's body only if that body outweighs the
/// hanger by `anchor_mass_ratio`, or does not move at all. A lighter body
/// (a crate, a platform's deck) would be the one the joint moved: a climb's
/// path held to it turns with it, drags the hanger round, and turns it
/// further, which flung an 8 kg box off the world. Its joint is anchored to
/// the world at the edge, following it, and the body takes the hanger's
/// weight at the grip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grip {
    constraint: ConstraintHandle,
    edge_body: Option<RigidBodyHandle>,
    /// The body the joint is anchored to: the edge's, or `None` for the
    /// world (static geometry, or an edge body too light to anchor to).
    anchored: Option<RigidBodyHandle>,
    /// Where the joint's edge end started its present motion (the catch's
    /// ease, or the pull-up's climb), in the edge's frame.
    from: Point3<f32>,
    /// Whether that motion is a pull-up.
    climbing: bool,
    /// Whether the hanger and the edge's body pass through each other. A
    /// light body the joint is not anchored to does for the whole hold: the
    /// world-anchored joint pulls the hanger towards the face, and contact
    /// would push the body away, the anchor following it, the hanger after
    /// it, a pump. Any moving body does from a climb's start: the path is
    /// built to clear the lip, and contacts along it would only shove the
    /// body out from under the climb.
    passing: bool,
    /// The hanger's body.
    hanger: RigidBodyHandle,
    /// Where the joint's edge end carries the body to by the end of this
    /// frame, in the joint's frame; the end is moved there next frame and
    /// set moving on.
    carried: Point3<f32>,
    /// The same point in the edge's frame: where along the edge the hands
    /// are, held as the edge moves however the joint is anchored.
    aimed: Point3<f32>,
}

/// What [`Grip::reconcile`] reads this frame.
pub struct GripFrame<'a> {
    pub body: RigidBodyHandle,
    pub state: &'a LocomotionState,
    /// The edge the hands hold, as sensed this frame.
    pub sensed: Option<&'a SensedEdge>,
    pub move_dir: Vector3<f32>,
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
        let Some(to_world) = frame_of(world, sensed.edge.body) else {
            return;
        };
        let Some(anchor) = world
            .body(frame.body)
            .map(|body| body.position() + body.rotation() * hang_anchor(frame.config))
        else {
            return;
        };
        let climbing = matches!(phase, LedgePhase::PullUp { .. });
        if grip.is_none() {
            *grip = Grip::create(world, sensed, anchor, frame);
        }
        let Some(current) = grip.as_mut() else {
            return;
        };
        let Some(to_joint) = frame_of(world, current.anchored) else {
            return;
        };
        current.settle_at(world, current.carried);
        let now = to_joint * current.carried;
        if climbing && !current.climbing {
            current.from = to_world.inverse() * now;
            current.climbing = true;
            current.pass_through(world);
        }
        let target = match phase {
            LedgePhase::Grab { elapsed } => {
                let t = smoothstep(elapsed / frame.config.ledge.settle);
                let from = to_world * current.from;
                from + (hang_point(&at, frame.config) - from) * t
            }
            LedgePhase::Hang { .. } => {
                let hang = hang_point(&at, frame.config);
                let along = at.along();
                let held = to_world * current.aimed;
                hang + along * (held - hang).dot(&along) + shimmy(sensed, frame) * frame.dt
            }
            LedgePhase::PullUp { elapsed, duration } => {
                climb(&at, to_world * current.from, frame.config).at(smoothstep(elapsed / duration))
            }
        };
        let velocity = if frame.dt > 0.0 {
            (target - now) / frame.dt
        } else {
            Vector3::zeros()
        };
        current.aim(world, velocity);
        current.carried = to_joint.inverse() * target;
        current.aimed = to_world.inverse() * target;
        current.bear_weight(world, frame, &at);
    }

    /// An edge body the joint is not anchored to takes the hanger's weight
    /// at the grip, one frame's worth.
    fn bear_weight(&self, world: &mut PhysicsWorld, frame: &GripFrame, edge: &WorldEdge) {
        let Some(edge_body) = self.edge_body.filter(|_| self.anchored.is_none()) else {
            return;
        };
        let Some(mass) = world.body(frame.body).map(|b| b.mass()) else {
            return;
        };
        let weight = world.config().gravity * (mass * frame.dt);
        world.apply_impulse_at_point(edge_body, weight, edge.grip());
    }

    /// Let go: the joint goes, and contact with the edge's body resumes.
    fn release(self, world: &mut PhysicsWorld) {
        world.remove_constraint(self.constraint);
        if let Some(edge) = self.edge_body.filter(|_| self.passing) {
            world.restore_contacts(self.hanger, edge);
        }
    }

    /// From now until the grip lets go, the hanger and a moving edge body
    /// pass through each other.
    fn pass_through(&mut self, world: &mut PhysicsWorld) {
        if let Some(edge) = self.moving_edge(world).filter(|_| !self.passing) {
            world.exempt_contacts(self.hanger, edge);
            self.passing = true;
        }
    }

    /// The edge's body, if it is one that moves.
    fn moving_edge(&self, world: &PhysicsWorld) -> Option<RigidBodyHandle> {
        self.edge_body
            .filter(|&edge| world.body(edge).is_some_and(|body| body.is_dynamic()))
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
        let hanger = world.body(frame.body)?.mass();
        let anchored = sensed.edge.body.filter(|&edge| {
            world.body(edge).is_some_and(|body| {
                !body.is_dynamic() || body.mass() >= hanger * frame.config.ledge.anchor_mass_ratio
            })
        });
        let carried = frame_of(world, anchored)?.inverse() * anchor;
        // Unbreakable: the solver's limit is an impulse per substep, and
        // moving the edge end each frame asks for more than any hit would.
        let constraint = world.create_constraint(ConstraintKind::BallJoint {
            body_a: anchored,
            body_b: frame.body,
            local_anchor_a: carried.coords,
            anchor_velocity: Vector3::zeros(),
            local_anchor_b: hang_anchor(frame.config),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        let mut grip = Self {
            constraint,
            edge_body: sensed.edge.body,
            anchored,
            from,
            climbing: false,
            passing: false,
            hanger: frame.body,
            carried,
            aimed: from,
        };
        if anchored.is_none() {
            grip.pass_through(world);
        }
        Some(grip)
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

/// The shimmy's velocity: along the edge with the stick, where the edge
/// carries on that way.
fn shimmy(sensed: &SensedEdge, frame: &GripFrame) -> Vector3<f32> {
    let ledge = &frame.config.ledge;
    let outward = sensed.at.outward.into_inner();
    if StickOnEdge::of(frame.move_dir, outward, ledge) != StickOnEdge::Along {
        return Vector3::zeros();
    }
    let along = sensed.at.along();
    let way = frame.move_dir.dot(&along);
    let room = if way > 0.0 {
        sensed.room[1]
    } else {
        sensed.room[0]
    };
    if room {
        along * way.signum() * ledge.shimmy_speed
    } else {
        Vector3::zeros()
    }
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
