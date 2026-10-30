//! Which bodies shock propagation may scale through: those in static
//! equilibrium on what holds them up.

use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::super::conditioning::ManifoldConditions;
use super::contact_graph::ContactGraph;
use super::wrench_cone::ContactWrenchCone;

/// Speed at which a carrying contact counts as opening, in m/s: it carries
/// nothing however hard it was pressed last frame. Closing as fast, it is
/// striking the body, with an impulse last frame's do not hold.
const OPENING_SPEED: f32 = 0.05;

/// How far a load may fall outside the carriers' wrench cone, as a share of
/// its size, and still count as held. A load held only within a looser 5 %
/// let a knocked arch creep up for as long as it was watched.
const CONE_TOLERANCE: f64 = 0.02;

/// How far a body's load may drift from the one its verdict was solved for,
/// as a share of its size, before it is solved again.
const REUSE_LOAD_DRIFT: f32 = 0.02;

/// How far its carrying contacts may drift, per contact, before its verdict
/// is solved again, in metres.
const REUSE_POINT_DRIFT: f32 = 0.001;

/// Frames a verdict is kept for a body that no longer comes up.
const FORGET_AFTER_FRAMES: u64 = 600;

/// Unscales every pair whose lower body is not in static equilibrium on
/// feasible impulses from the ground side.
///
/// Scaled impulses move bodies exactly as physical impulses would that carry
/// each withheld share of a push down through the supports to the ground
/// (`docs/SHOCK_PROPAGATION_STATICS.md`). Those impulses are a contact
/// solution only where every body they pass through can be held by its
/// carriers: its contacts with the ground or with bodies no higher than it,
/// none of them opening or striking it, every impulse pushing and within its
/// friction. A body is *grounded* when its weight and the loads on it from
/// higher bodies lie in its carriers' wrench cone, every carrier below it is
/// grounded, and no driven body is about to strike it.
#[derive(Default)]
pub(super) struct StaticsGate {
    /// Each body's last solved verdict and what it was solved for.
    verdicts: FxHashMap<RigidBodyHandle, Verdict>,
    /// Bodies a driven body will strike this frame through a contact that has
    /// not met yet.
    struck: FxHashSet<RigidBodyHandle>,
    /// This frame's grounded bodies.
    grounded: FxHashMap<RigidBodyHandle, bool>,
    /// This frame's met manifolds touching each body with a depth.
    touching: FxHashMap<RigidBodyHandle, SmallVec<[usize; 8]>>,
    /// Bodies in order of depth, lowest first.
    order: Vec<RigidBodyHandle>,
    /// The body being judged's carriers.
    cone: ContactWrenchCone,
    /// Frames gated.
    frame: u64,
}

/// A wrench-cone verdict, kept until the body's load or carriers move.
struct Verdict {
    /// The load force and torque about the centre of mass it was solved for.
    load: (Vector3<f32>, Vector3<f32>),
    /// Carrying contacts, and the sum of their lever arms.
    carriers: usize,
    levers: Vector3<f32>,
    holds: bool,
    /// Frame last used.
    seen: u64,
}

/// What holds a body up and what it must hold, from this frame's contacts.
struct Statics {
    /// Weight plus every push from higher bodies, about the centre of mass.
    load: (Vector3<f32>, Vector3<f32>),
    /// What the carriers gave the body last frame.
    held: (Vector3<f32>, Vector3<f32>),
    carriers: usize,
    levers: Vector3<f32>,
    /// Whether every dynamic carrier lower than the body is grounded.
    stands_on_grounded: bool,
}

impl StaticsGate {
    /// Sets the scales of every pair whose lower body is not grounded to 1.
    pub(super) fn apply(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        graph: &ContactGraph,
        gravity: Vector3<f32>,
        dt: f32,
        conditions: &mut ManifoldConditions,
    ) {
        self.frame += 1;
        self.collect_touching(bodies, manifolds, graph);

        for i in 0..self.order.len() {
            let body = self.order[i];
            let statics = self.statics_of(body, bodies, manifolds, graph, gravity * dt);
            let grounded = !self.struck.contains(&body)
                && statics.is_some_and(|s| s.stands_on_grounded && self.holds(body, &s));
            self.grounded.insert(body, grounded);
        }

        for (index, manifold) in manifolds.iter().enumerate() {
            let scales = conditions.shock_scales[index];
            if scales == (1.0, 1.0) {
                continue;
            }
            let header = &manifold.header;
            let lower = match header.body_a {
                Some(a) if scales.0 < 1.0 => a,
                _ => header.body_b,
            };
            if !self.grounded.get(&lower).copied().unwrap_or(false) {
                conditions.shock_scales[index] = (1.0, 1.0);
            }
        }

        let frame = self.frame;
        if frame % 120 == 0 {
            self.verdicts
                .retain(|_, verdict| frame - verdict.seen < FORGET_AFTER_FRAMES);
        }
    }

    /// Every met manifold, under each dynamic body the graph gave a depth,
    /// and those bodies lowest first; and the bodies a driven body is about
    /// to strike.
    fn collect_touching(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        graph: &ContactGraph,
    ) {
        self.touching.clear();
        self.grounded.clear();
        self.struck.clear();
        for (index, manifold) in manifolds.iter().enumerate() {
            if !manifold.has_met() {
                self.collect_struck(bodies, manifold);
                continue;
            }
            let header = &manifold.header;
            for side in [header.body_a, Some(header.body_b)].into_iter().flatten() {
                if graph.depth.contains_key(&side) && !is_ground(bodies, Some(side)) {
                    self.touching.entry(side).or_default().push(index);
                }
            }
        }
        self.order.clear();
        self.order.extend(self.touching.keys().copied());
        self.order.sort_by_key(|body| graph.depth[body]);
    }

    /// Both bodies of a pair that has not met, if one of them is driven and
    /// they are closing: the impulse the strike brings is in no warm start.
    fn collect_struck(&mut self, bodies: &Arena<RigidBody>, manifold: &SolverManifold) {
        let header = &manifold.header;
        if !is_driven(bodies, header.body_a) && !is_driven(bodies, Some(header.body_b)) {
            return;
        }
        let velocity_at = |body: Option<RigidBodyHandle>, point| {
            body.and_then(|h| bodies.get(h.0))
                .map_or(Vector3::zeros(), |b| b.velocity_at(point))
        };
        let closing = manifold.contacts.iter().any(|c| {
            let relative =
                velocity_at(Some(header.body_b), c.point) - velocity_at(header.body_a, c.point);
            relative.dot(&c.normal) < -OPENING_SPEED
        });
        if closing {
            self.struck
                .extend([header.body_a, Some(header.body_b)].into_iter().flatten());
        }
    }

    /// The body's load and carriers, with its carriers' wrench cone left in
    /// `self.cone`; `None` if nothing carries it, or a carrier is striking it.
    fn statics_of(
        &mut self,
        body: RigidBodyHandle,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        graph: &ContactGraph,
        step_gravity: Vector3<f32>,
    ) -> Option<Statics> {
        let depth = graph.depth[&body];
        let rigid = bodies.get(body.0)?;
        let centre = rigid.position();
        let mut statics = Statics {
            load: (step_gravity * rigid.mass(), Vector3::zeros()),
            held: (Vector3::zeros(), Vector3::zeros()),
            carriers: 0,
            levers: Vector3::zeros(),
            stands_on_grounded: true,
        };
        self.cone.clear();
        for &index in &self.touching[&body] {
            let manifold = &manifolds[index];
            let header = &manifold.header;
            let (other, into_body) = if header.body_b == body {
                (header.body_a, 1.0)
            } else {
                (Some(header.body_b), -1.0)
            };
            let other_depth = other.and_then(|h| graph.depth.get(&h).copied());
            let carries = is_ground(bodies, other) || other_depth.is_some_and(|d| d <= depth);
            for contact in manifold.contacts.iter().filter(|c| c.gap <= 0.0) {
                let normal = contact.normal * into_body;
                let lever = contact.point - centre;
                let impulse = (contact.normal * contact.warm_normal_impulse
                    + contact.warm_friction_impulse_ws)
                    * into_body;
                if !carries {
                    statics.load.0 += impulse;
                    statics.load.1 += lever.cross(&impulse);
                    continue;
                }
                let other_velocity = other
                    .and_then(|h| bodies.get(h.0))
                    .map_or(Vector3::zeros(), |b| b.velocity_at(contact.point));
                let separating = (rigid.velocity_at(contact.point) - other_velocity).dot(&normal);
                if separating > OPENING_SPEED {
                    continue;
                }
                if separating < -OPENING_SPEED {
                    return None;
                }
                self.cone.add_contact(normal, lever, header.friction);
                statics.carriers += 1;
                statics.levers += lever;
                statics.held.0 += impulse;
                statics.held.1 += lever.cross(&impulse);
            }
            let lower_carrier = other.filter(|_| other_depth.is_some_and(|d| d < depth));
            if let Some(lower) = lower_carrier.filter(|&h| !is_ground(bodies, Some(h))) {
                statics.stands_on_grounded &= self.grounded.get(&lower).copied().unwrap_or(false);
            }
        }
        (statics.carriers > 0).then_some(statics)
    }

    /// Whether the carriers in `self.cone` hold the body's load: certified by
    /// last frame's impulses, reused from a verdict on the same load, or
    /// solved.
    fn holds(&mut self, body: RigidBodyHandle, statics: &Statics) -> bool {
        if self.certified(statics) {
            return true;
        }
        let (force, torque) = statics.load;
        let size = force.norm() + torque.norm();
        if let Some(verdict) = self.verdicts.get_mut(&body) {
            let drift = (verdict.load.0 - force).norm() + (verdict.load.1 - torque).norm();
            if verdict.carriers == statics.carriers
                && drift <= REUSE_LOAD_DRIFT * size
                && (verdict.levers - statics.levers).norm()
                    <= REUSE_POINT_DRIFT * statics.carriers as f32
            {
                verdict.seen = self.frame;
                return verdict.holds;
            }
        }
        let holds = self.cone.contains(-force, -torque, CONE_TOLERANCE);
        self.verdicts.insert(
            body,
            Verdict {
                load: statics.load,
                carriers: statics.carriers,
                levers: statics.levers,
                holds,
                seen: self.frame,
            },
        );
        holds
    }

    /// Whether a positive multiple of what the carriers gave last frame
    /// balances the load. Those impulses pushed and kept within friction, so
    /// any positive multiple of them does too: if one balances the load, it
    /// lies in the cone. Under scaling the carriers held the body's weight
    /// and only part of the loads above it, hence a multiple.
    fn certified(&self, statics: &Statics) -> bool {
        let reach = self.cone.reach();
        let held = (statics.held.0, statics.held.1 / reach);
        let load = (statics.load.0, statics.load.1 / reach);
        let dot = |a: &(Vector3<f32>, Vector3<f32>), b: &(Vector3<f32>, Vector3<f32>)| {
            a.0.dot(&b.0) + a.1.dot(&b.1)
        };
        let held_size = dot(&held, &held);
        let load_size = dot(&load, &load);
        if held_size < 1e-12 || load_size < 1e-12 {
            return false;
        }
        let multiple = (-dot(&held, &load) / held_size).max(0.0);
        let residual = (held.0 * multiple + load.0, held.1 * multiple + load.1);
        let tolerance = CONE_TOLERANCE as f32;
        dot(&residual, &residual) <= tolerance * tolerance * load_size
    }
}

/// Whether `body` is immovable: static geometry, or a body that is not dynamic.
fn is_ground(bodies: &Arena<RigidBody>, body: Option<RigidBodyHandle>) -> bool {
    body.is_none_or(|h| bodies.get(h.0).is_none_or(|b| !b.is_dynamic()))
}

/// Whether `body` drives itself through its supports, as a character does.
fn is_driven(bodies: &Arena<RigidBody>, body: Option<RigidBodyHandle>) -> bool {
    body.and_then(|h| bodies.get(h.0))
        .is_some_and(|b| b.support_drive().is_some())
}
