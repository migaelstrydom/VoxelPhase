//! The allowance: the design's one sanctioned non-conservative authority, in
//! the one module it is permitted to live in.
//!
//! ```text
//!   DriveCommand ─┬─ budget  ─┐
//!                 ├─ verbs   ─┼──► apply_allowances ──► RigidBody velocity
//!                 └─ steer   ─┘         │
//!   SupportSets ────────────────────────┴──► AllowanceLedger
//! ```
//!
//! A tangential row needs a normal impulse to bound it, so a body with nothing
//! holding it up has no drive authority of any kind — see
//! `solve_friction_impulse`'s early return. That is the mechanism behaving
//! correctly and it is also the whole airborne half of a character controller:
//! jumping, jump shaping, air steering and turning on the spot all ask for
//! momentum no contact can supply. This module is where that momentum is
//! conjured, and the accounting that comes with it (R8,
//! `docs/TRACTION_DRIVE_DESIGN.md` §6.3).
//!
//! Three rules keep it honest:
//!
//! - **Opt-in.** A body with no [`Allowance`] gets none of this. Every crate,
//!   prop and corpse in the game is in that category, and so is a character
//!   whose actuator has been taken away.
//! - **Bounded.** Every authority here has a declared per-entity ceiling in
//!   [`Allowance`], in the units of the thing it bounds.
//! - **Measured.** Everything spent is recorded in
//!   [`AllowanceLedger`](super::ledger::AllowanceLedger) and printed, because
//!   R8 asks that "deliberate" be a property of the code rather than of the
//!   document.
//!
//! One authority here is *not* an allowance and is deliberately routed through
//! the same call: a jump with a Support Set behind it is delivered as impulse
//! pairs at the contact points, so the platform it pushes off is pushed down
//! and tipped. Only a jump the Support Set cannot deliver falls back on the
//! budget — decision D2a.

use generational_arena::Arena;
use nalgebra::{Point3, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::command::NormalVerbs;
use super::ledger::AllowanceLedger;
use super::support::SupportSets;

/// A per-entity budget of non-conservative authority.
///
/// Each field is a ceiling in the units of what it bounds, and each is zero by
/// default: an allowance grants nothing until someone writes a number into it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Allowance {
    /// Linear acceleration this body may conjure while nothing holds it up,
    /// in m/s². Air steering; spent only while the Support Set is empty.
    pub air_accel: f32,
    /// Angular acceleration this body may conjure about its support axis, in
    /// rad/s².
    ///
    /// Spent whether or not the body is supported, because a character's
    /// ground yaw has nowhere else to come from: a torsional row is bounded by
    /// `μ·N·r` and a capsule's contact patch has no radius (§6.2). A body that
    /// declares a patch radius *and* a yaw allowance gets both authorities;
    /// declaring one or the other is almost always what is meant.
    pub yaw_accel: f32,
    /// Largest speed along the support axis a jump may establish when there is
    /// no Support Set to deliver it through, in m/s.
    ///
    /// Decision D2a: an unsupported jump is not dropped, it is conjured. This
    /// is the largest single non-conservative impulse in the game — a coyote
    /// hop off a platform torques it differently from an edge walk, and that
    /// is a cost the requirement accepts by name. Zero forbids it outright.
    pub unsupported_jump_speed: f32,
}

impl Allowance {
    /// The airborne authority of a character: steer, turn, and jump off
    /// nothing when support has just been lost.
    pub fn character(air_accel: f32, yaw_accel: f32, unsupported_jump_speed: f32) -> Self {
        Self {
            air_accel,
            yaw_accel,
            unsupported_jump_speed,
        }
    }
}

/// The non-conservative half of one frame's drive command.
///
/// Carried inside `DriveCommand` rather than beside it, so that a body's
/// budget and what it is asking to spend arrive together and no caller can
/// push one without the other.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AllowanceCommand {
    /// This entity's declared budget. `None` — the default — is a body that
    /// was granted no allowance at all.
    pub budget: Option<Allowance>,
    /// The frame's discrete verbs, consumed on the first substep.
    pub verbs: NormalVerbs,
    /// Rate at which the linear target may be steered toward while nothing
    /// holds the body up, in m/s². Clamped to the budget's `air_accel`.
    ///
    /// `None` is a body that has asked not to be steered this frame — a
    /// committed long jump, whose arc is already ballistic.
    pub steer_accel: Option<f32>,
}

impl AllowanceCommand {
    /// True when this command asks for nothing at all, for a body that would
    /// be granted nothing anyway.
    pub fn is_inert(&self) -> bool {
        self.budget.is_none() && self.verbs.is_inert()
    }

    /// Take the frame's discrete verbs, leaving the continuous authority.
    ///
    /// Edge-triggered by nature: a jump commanded once must fire once, on the
    /// first substep of the frame it was commanded, or its height would depend
    /// on how many substeps that frame happened to run.
    pub fn take_verbs(&mut self) -> NormalVerbs {
        std::mem::take(&mut self.verbs)
    }
}

/// Everything one body's allowance does in one substep, resolved before any of
/// it is applied.
///
/// Separated from the application because a supported jump writes to the
/// partner bodies as well as the jumper, and the arena cannot be borrowed
/// mutably twice.
struct AllowancePlan {
    body: RigidBodyHandle,
    /// Axis the discrete verbs and the yaw act along.
    axis: Vector3<f32>,
    /// One share of a jump per supporting contact, each an impulse exchange
    /// with whatever is on the other end of it.
    jump: SmallVec<[JumpShare; 4]>,
    /// Impulse to apply at the jumper's centre of mass with no partner at all.
    conjured_jump: Vector3<f32>,
    /// Impulse removed from the body by jump shaping.
    shaping: Vector3<f32>,
    /// Impulse conjured to steer the body through the air.
    steer: Vector3<f32>,
    /// Angular velocity change conjured about `axis`, and the angular impulse
    /// it stood for.
    yaw: (f32, f32),
}

/// One supporting contact's share of a jump: the impulse the jumper receives
/// there, and the body that receives the other half of it.
struct JumpShare {
    impulse: Vector3<f32>,
    point: Point3<f32>,
    partner: Option<RigidBodyHandle>,
}

/// Spend one substep's allowances.
///
/// `first_substep` gates the edge-triggered verbs — a jump fires once per
/// frame, not once per substep, or its height would follow the frame rate.
/// `world_up` is the axis a body with no support falls back on; a world with
/// no gravity has none, and there the vertical verbs are inert because there
/// is no direction they could mean anything along.
pub fn apply_allowances(
    bodies: &mut Arena<RigidBody>,
    supports: &SupportSets,
    world_up: Option<UnitVector3<f32>>,
    dt: f32,
    first_substep: bool,
    ledger: &mut AllowanceLedger,
) {
    let mut plans: Vec<AllowancePlan> = Vec::new();

    for (index, body) in bodies.iter() {
        let handle = RigidBodyHandle(index);
        if !body.is_dynamic() || body.inv_mass() <= 0.0 {
            continue;
        }
        if body.allowance_command().is_inert() {
            continue;
        }
        let Some(plan) = plan_body(body, handle, supports, world_up, dt, first_substep) else {
            continue;
        };
        plans.push(plan);
    }

    for plan in plans {
        apply_plan(bodies, &plan, ledger);
        if first_substep {
            if let Some(body) = bodies.get_mut(plan.body.0) {
                body.take_drive_verbs();
            }
        }
    }
}

/// Work out what one body's allowance does this substep.
fn plan_body(
    body: &RigidBody,
    handle: RigidBodyHandle,
    supports: &SupportSets,
    world_up: Option<UnitVector3<f32>>,
    dt: f32,
    first_substep: bool,
) -> Option<AllowancePlan> {
    let command = body.allowance_command();
    let drive = body.support_drive();
    let support = supports.get(handle).filter(|set| !set.is_empty());

    // The axis every vertical verb and the yaw act along: what holds the body
    // up if anything does, and the world's up if nothing does. A slope is
    // therefore jumped off along the slope (§10.2), and a body in free fall
    // still knows which way is up.
    let axis = match support {
        Some(set) => set.mean_normal(),
        None => world_up.map(|up| up.into_inner())?,
    };

    let mut plan = AllowancePlan {
        body: handle,
        axis,
        jump: SmallVec::new(),
        conjured_jump: Vector3::zeros(),
        shaping: Vector3::zeros(),
        steer: Vector3::zeros(),
        yaw: (0.0, 0.0),
    };

    let mass = body.mass();
    let mut speed_along = body.linear_velocity().dot(&axis);

    if first_substep && !command.verbs.is_inert() {
        // A jump establishes a speed along the axis rather than adding one, so
        // that a jump taken while walking downhill is the same jump as one
        // taken from a standstill. It never brakes a body already leaving
        // faster than it asked for.
        if let Some(speed) = command.verbs.impulse {
            match support {
                Some(set) => {
                    let delta = speed - speed_along;
                    if delta > 0.0 {
                        let contacts = set.contacts();
                        let share = mass * delta / contacts.len() as f32;
                        for contact in contacts {
                            plan.jump.push(JumpShare {
                                impulse: axis * share,
                                point: contact.point,
                                partner: contact.partner,
                            });
                        }
                        speed_along = speed;
                    }
                }
                None => {
                    // D2a: no support, so the impulse comes out of the budget
                    // or not at all. A budget of zero is not a jump clamped to
                    // a standstill — it is no jump, and a body falling past a
                    // ledge must go on falling.
                    let ceiling = command
                        .budget
                        .map(|budget| budget.unsupported_jump_speed)
                        .unwrap_or(0.0);
                    let established = speed.min(ceiling);
                    if ceiling > 0.0 && established > speed_along {
                        plan.conjured_jump = axis * (mass * (established - speed_along));
                        speed_along = established;
                    }
                }
            }
        }

        // Shaping is a projection of the body's own velocity, so it is
        // non-conservative whatever the Support Set says and needs a budget to
        // exist at all.
        if command.budget.is_some() && !command.verbs.projection.is_identity() {
            let shaped = command.verbs.projection.applied_to(speed_along);
            plan.shaping = axis * (mass * (shaped - speed_along));
        }
    }

    if let (Some(budget), Some(drive)) = (command.budget, drive) {
        if support.is_none() {
            if let Some(rate) = command.steer_accel {
                plan.steer = mass * steer_delta(body, drive.linear_target, axis, rate, budget, dt);
            }
        }

        if budget.yaw_accel > 0.0 {
            let desired = drive.angular_target.dot(&axis);
            let current = body.angular_velocity().dot(&axis);
            let limit = budget.yaw_accel * dt;
            let delta = (desired - current).clamp(-limit, limit);
            let inv_inertia_about_axis = axis.dot(&(body.world_inv_inertia() * axis));
            let angular_impulse = if inv_inertia_about_axis > 1e-9 {
                delta.abs() / inv_inertia_about_axis
            } else {
                0.0
            };
            plan.yaw = (delta, angular_impulse);
        }
    }

    Some(plan)
}

/// The velocity change air steering asks for, across the plane perpendicular
/// to `axis`.
///
/// The axis itself is left to gravity and the jump verbs: steering a fall is a
/// different verb from steering a run, and nothing in the game has it.
fn steer_delta(
    body: &RigidBody,
    target: Vector3<f32>,
    axis: Vector3<f32>,
    rate: f32,
    budget: Allowance,
    dt: f32,
) -> Vector3<f32> {
    let rate = rate.min(budget.air_accel);
    if rate <= 0.0 {
        return Vector3::zeros();
    }
    let across = |v: Vector3<f32>| v - axis * v.dot(&axis);
    let error = across(target) - across(body.linear_velocity());
    let step = rate * dt;
    if error.magnitude() <= step {
        error
    } else {
        error.normalize() * step
    }
}

/// Apply one body's plan, and record what it cost.
fn apply_plan(bodies: &mut Arena<RigidBody>, plan: &AllowancePlan, ledger: &mut AllowanceLedger) {
    for share in plan.jump.iter() {
        if let Some(body) = bodies.get_mut(plan.body.0) {
            body.apply_impulse_at_point(share.impulse, share.point);
        }
        if let Some(partner) = share.partner {
            if let Some(body) = bodies.get_mut(partner.0) {
                if body.is_dynamic() {
                    body.apply_impulse_at_point(-share.impulse, share.point);
                }
            }
        }
    }
    if !plan.jump.is_empty() {
        ledger.record_supported_jump(plan.body);
    }

    let Some(body) = bodies.get_mut(plan.body.0) else {
        return;
    };

    if plan.conjured_jump.magnitude_squared() > 0.0 {
        body.apply_impulse(plan.conjured_jump);
        ledger.record_unsupported_jump(plan.body, plan.conjured_jump.magnitude());
    }
    if plan.shaping.magnitude_squared() > 0.0 {
        body.apply_impulse(plan.shaping);
        ledger.record_shaping(plan.body, plan.shaping.magnitude());
    }
    if plan.steer.magnitude_squared() > 0.0 {
        body.apply_impulse(plan.steer);
        ledger.record_steer(plan.body, plan.steer.magnitude());
    }
    if plan.yaw.0 != 0.0 {
        body.set_angular_velocity(body.angular_velocity() + plan.axis * plan.yaw.0);
        ledger.record_yaw(plan.body, plan.yaw.1);
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Point3};

    use super::*;
    use crate::physics::body::{RigidBodyDesc, SupportDrive};
    use crate::physics::drive::support::{tests::manifold, SupportResolver};

    fn up() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(Vector3::y()))
    }

    fn down() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(-Vector3::y()))
    }

    /// One substep of the game's budget: four to a sixtieth of a second.
    const DT: f32 = 1.0 / 240.0;

    /// An arena of unit-mass, unit-inertia bodies, so an impulse reads as a
    /// velocity and an angular impulse as a spin.
    fn bodies(count: usize) -> Arena<RigidBody> {
        let mut arena = Arena::new();
        for _ in 0..count {
            let mut body = RigidBody::new(RigidBodyDesc::dynamic());
            body.set_mass_properties(1.0, Matrix3::identity());
            arena.insert(body);
        }
        arena
    }

    fn handle(arena: &Arena<RigidBody>, index: usize) -> RigidBodyHandle {
        RigidBodyHandle(arena.iter().nth(index).unwrap().0)
    }

    fn budgeted() -> AllowanceCommand {
        AllowanceCommand {
            budget: Some(Allowance::character(8.0, 500.0, 7.0)),
            verbs: NormalVerbs::default(),
            steer_accel: Some(8.0),
        }
    }

    fn jumping(speed: f32) -> AllowanceCommand {
        AllowanceCommand {
            verbs: NormalVerbs {
                impulse: Some(speed),
                ..Default::default()
            },
            ..budgeted()
        }
    }

    fn walking(linear: Vector3<f32>) -> SupportDrive {
        SupportDrive {
            linear_target: linear,
            angular_target: Vector3::zeros(),
            gain: 1.0,
            patch_radius: 0.0,
        }
    }

    fn velocity(arena: &Arena<RigidBody>, body: RigidBodyHandle) -> Vector3<f32> {
        arena.get(body.0).unwrap().linear_velocity()
    }

    /// One substep, with the frame's edge-triggered verbs live.
    fn first_substep(
        arena: &mut Arena<RigidBody>,
        supports: &SupportSets,
        ledger: &mut AllowanceLedger,
    ) {
        apply_allowances(arena, supports, up(), DT, true, ledger);
    }

    /// An unsupported jump comes out of the budget, and is capped by it.
    #[test]
    fn a_jump_with_no_support_is_conjured_and_capped() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena
            .get_mut(body.0)
            .unwrap()
            .set_allowance_command(jumping(10.0));

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &SupportSets::default(), &mut ledger);

        assert_eq!(velocity(&arena, body).y, 7.0);
        assert_eq!(ledger.usage(body).unwrap().unsupported_jumps, 1);
    }

    /// No budget, no jump: the whole authority is opt-in.
    #[test]
    fn a_body_with_no_allowance_cannot_jump_off_nothing() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena
            .get_mut(body.0)
            .unwrap()
            .set_allowance_command(AllowanceCommand {
                budget: None,
                ..jumping(7.0)
            });

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &SupportSets::default(), &mut ledger);

        assert_eq!(velocity(&arena, body).y, 0.0);
        assert!(ledger.usage(body).is_none());
    }

    /// A supported jump is an impulse exchange: the jumper leaves at the speed
    /// it asked for and the thing it left is pushed the other way.
    #[test]
    fn a_supported_jump_pushes_the_thing_it_leaves() {
        let mut arena = bodies(2);
        let (deck, jumper) = (handle(&arena, 0), handle(&arena, 1));
        arena
            .get_mut(jumper.0)
            .unwrap()
            .set_allowance_command(jumping(7.0));

        let manifolds = vec![manifold(Some(deck), jumper, &[Vector3::y()])];
        let supports = SupportResolver::default().resolve(&manifolds, down());

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &supports, &mut ledger);

        assert!((velocity(&arena, jumper).y - 7.0).abs() < 1e-4);
        assert!((velocity(&arena, deck).y + 7.0).abs() < 1e-4);
        let usage = ledger.usage(jumper).unwrap();
        assert_eq!(usage.supported_jumps, 1);
        assert_eq!(usage.unsupported_jumps, 0);
    }

    /// The verbs are edge-triggered: a frame's jump fires on its first substep
    /// and never again, whatever the substep count.
    #[test]
    fn a_jump_fires_once_per_frame() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena
            .get_mut(body.0)
            .unwrap()
            .set_allowance_command(jumping(7.0));

        let mut ledger = AllowanceLedger::default();
        for substep in 0..4 {
            apply_allowances(
                &mut arena,
                &SupportSets::default(),
                up(),
                DT,
                substep == 0,
                &mut ledger,
            );
        }

        assert_eq!(velocity(&arena, body).y, 7.0);
        assert_eq!(ledger.usage(body).unwrap().unsupported_jumps, 1);
    }

    /// Shaping is proportional to the velocity it acts on, and acts on a rise
    /// only.
    #[test]
    fn shaping_cuts_a_rise_and_leaves_a_fall_alone() {
        for (initial, expected) in [(7.0_f32, 3.15_f32), (-7.0, -7.0)] {
            let mut arena = bodies(1);
            let body = handle(&arena, 0);
            let rigid = arena.get_mut(body.0).unwrap();
            rigid.set_linear_velocity(Vector3::new(0.0, initial, 0.0));
            let mut command = budgeted();
            command.verbs.projection.scale = 0.45;
            rigid.set_allowance_command(command);

            let mut ledger = AllowanceLedger::default();
            first_substep(&mut arena, &SupportSets::default(), &mut ledger);

            let measured = velocity(&arena, body).y;
            assert!(
                (measured - expected).abs() < 1e-4,
                "{measured} vs {expected}"
            );
        }
    }

    /// Air steering is bounded by the budget, spends only across the support
    /// axis, and stops when it arrives.
    #[test]
    fn air_steering_ramps_at_its_budget_and_leaves_the_axis_alone() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        let rigid = arena.get_mut(body.0).unwrap();
        rigid.set_linear_velocity(Vector3::new(0.0, -3.0, 0.0));
        rigid.set_support_drive(walking(Vector3::new(5.0, 0.0, 0.0)));
        rigid.set_allowance_command(budgeted());

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &SupportSets::default(), &mut ledger);

        let measured = velocity(&arena, body);
        assert!((measured.x - 8.0 * DT).abs() < 1e-5, "{measured:?}");
        assert_eq!(measured.y, -3.0, "the fall is not the steering's business");
        assert!(ledger.usage(body).unwrap().steer_impulse > 0.0);
    }

    /// A body standing on something steers with its feet, not its budget.
    #[test]
    fn a_supported_body_does_not_air_steer() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        let rigid = arena.get_mut(body.0).unwrap();
        rigid.set_support_drive(walking(Vector3::new(5.0, 0.0, 0.0)));
        rigid.set_allowance_command(budgeted());

        let manifolds = vec![manifold(None, body, &[Vector3::y()])];
        let supports = SupportResolver::default().resolve(&manifolds, down());

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &supports, &mut ledger);

        assert_eq!(velocity(&arena, body), Vector3::zeros());
    }

    /// A jump leaves along the support normal, so a slope is jumped off at an
    /// angle (§10.2).
    #[test]
    fn a_jump_leaves_along_the_support_normal() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena
            .get_mut(body.0)
            .unwrap()
            .set_allowance_command(jumping(7.0));

        let slope = Vector3::new(1.0, 1.0, 0.0).normalize();
        let mut manifolds = vec![manifold(None, body, &[slope])];
        manifolds[0].contacts[0].point = Point3::origin();
        let supports = SupportResolver::default().resolve(&manifolds, down());

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &supports, &mut ledger);

        let measured = velocity(&arena, body);
        assert!((measured.dot(&slope) - 7.0).abs() < 1e-4);
        assert!(measured.x > 4.0, "and gains a direction it did not have");
    }

    /// The yaw allowance turns a body about the axis holding it up, at its
    /// declared rate.
    #[test]
    fn the_yaw_allowance_spins_the_body_at_its_budget() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        let rigid = arena.get_mut(body.0).unwrap();
        rigid.set_support_drive(SupportDrive {
            angular_target: Vector3::new(0.0, 4.0, 0.0),
            ..walking(Vector3::zeros())
        });
        rigid.set_allowance_command(AllowanceCommand {
            budget: Some(Allowance::character(8.0, 120.0, 7.0)),
            ..Default::default()
        });

        let mut ledger = AllowanceLedger::default();
        first_substep(&mut arena, &SupportSets::default(), &mut ledger);

        let spin = arena.get(body.0).unwrap().angular_velocity().y;
        assert!((spin - 120.0 * DT).abs() < 1e-5, "{spin}");
        assert!(ledger.usage(body).unwrap().yaw_impulse > 0.0);
    }
}
