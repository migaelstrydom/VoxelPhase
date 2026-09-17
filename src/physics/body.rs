//! Rigid body representation.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

use super::constraint::types::ConstraintHandle;
use super::drive::allowance::AllowanceCommand;
use super::drive::command::NormalVerbs;
use super::handle::ColliderHandle;
use super::math::{integrate_orientation, skew, transform_inertia_tensor};

/// Type of rigid body determining how it participates in physics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyType {
    /// Affected by forces and collisions. Fully simulated.
    Dynamic,
    /// Moved by user code, affects dynamic bodies but not affected by them.
    Kinematic,
    /// Never moves. Used for static level geometry.
    Static,
}

/// Descriptor for creating a rigid body.
#[derive(Debug, Clone)]
pub struct RigidBodyDesc {
    pub body_type: BodyType,
    pub position: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub linear_velocity: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
}

impl Default for RigidBodyDesc {
    fn default() -> Self {
        Self {
            body_type: BodyType::Dynamic,
            position: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            linear_damping: 0.0,
            angular_damping: 0.05,
            gravity_scale: 1.0,
        }
    }
}

impl RigidBodyDesc {
    pub fn dynamic() -> Self {
        Self::default()
    }

    /// A body that never moves and is never moved.
    ///
    /// Distinct from the static *geometry* supplied by `StaticGeometry`: this
    /// is an ordinary collider that happens to have infinite mass, so it takes
    /// part in the body-vs-body narrowphase and can support dynamic bodies.
    /// Use it for immovable props; use `StaticGeometry` for the level itself.
    pub fn static_body() -> Self {
        Self {
            body_type: BodyType::Static,
            ..Self::default()
        }
    }

    pub fn position(mut self, position: Point3<f32>) -> Self {
        self.position = position;
        self
    }

    pub fn linear_velocity(mut self, velocity: Vector3<f32>) -> Self {
        self.linear_velocity = velocity;
        self
    }

    pub fn linear_damping(mut self, damping: f32) -> Self {
        self.linear_damping = damping;
        self
    }

    pub fn angular_damping(mut self, damping: f32) -> Self {
        self.angular_damping = damping;
        self
    }

    pub fn rotation(mut self, rotation: UnitQuaternion<f32>) -> Self {
        self.rotation = rotation;
        self
    }

    pub fn angular_velocity(mut self, velocity: Vector3<f32>) -> Self {
        self.angular_velocity = velocity;
        self
    }

    pub fn gravity_scale(mut self, scale: f32) -> Self {
        self.gravity_scale = scale;
        self
    }
}

/// A support-anchored drive command, as the traction rows read it.
///
/// Both targets are **relative to the support**: "five metres a second
/// forward" means five metres a second across whatever is holding the body up,
/// so a passenger asking for zero on a running deck is carried by it rather
/// than braked against the world.
#[derive(Debug, Clone, Copy)]
pub struct SupportDrive {
    /// Target velocity of the body relative to its supports.
    pub linear_target: Vector3<f32>,
    /// Target angular velocity of the body relative to its supports.
    pub angular_target: Vector3<f32>,
    /// Factor separating this body's drive budget from the contact's grip.
    ///
    /// `1.0` — the default — drives exactly as hard as it grips. Above that
    /// the body pushes harder than the surface honestly permits; see
    /// `Actuator::drive_gain` and `docs/TRACTION_DRIVE_DESIGN.md` §11.
    pub gain: f32,
    /// Radius of the contact patch this body's supports stand for, in metres.
    ///
    /// The bound on a torsional row is `μ·N·r`, and `r` is the one term the
    /// engine cannot derive: a contact is a point, and a manifold's several
    /// points already resist spin through their own tangential rows. So the
    /// entity declares it, and a body that does not — every character, every
    /// crate — has an inert torsional row. See §6.2.
    pub patch_radius: f32,
}

/// What a body's actuator pushes against, and the engine state that delivers
/// it.
///
/// One field, two variants, and that is the point: a body may not both carry
/// traction rows at its supports and a world-anchored motor row through the
/// solve. Overlapping the two would hand the body roughly twice the authority
/// its actuator declares and leave the row's bounds meaning nothing, so the
/// overlap is made unrepresentable rather than merely avoided.
/// `PhysicsWorld::set_body_drive` is the only writer.
#[derive(Debug, Clone)]
pub enum BodyDrive {
    /// Reaction goes into whatever holds the body up, through the tangential
    /// rows at its support contacts. The command itself; the rows are planned
    /// from it once per frame by `physics::drive::plan`.
    Support(SupportDrive),
    /// Reaction goes into the world. The command itself lives in the
    /// constraint arena as world-anchored motor rows; this is the handle to
    /// them, so the drive can be retired with the body or replaced by a
    /// support drive.
    Medium(ConstraintHandle),
}

/// A rigid body in the physics simulation.
#[derive(Debug)]
pub struct RigidBody {
    body_type: BodyType,

    // Transform
    position: Point3<f32>,
    rotation: UnitQuaternion<f32>,

    // Velocities
    linear_velocity: Vector3<f32>,
    angular_velocity: Vector3<f32>,

    // Damping
    linear_damping: f32,
    angular_damping: f32,

    // Gravity
    gravity_scale: f32,

    // Mass properties (computed from attached colliders)
    mass: f32,
    inv_mass: f32,
    local_inertia: Matrix3<f32>,
    inv_local_inertia: Matrix3<f32>,

    // Force/torque accumulators. External systems set these each frame;
    // they are NOT cleared per substep so forces like buoyancy integrate
    // alongside gravity across all substeps.
    force: Vector3<f32>,
    torque: Vector3<f32>,

    // Drag coefficients set by external systems (e.g. water buoyancy).
    // Applied per substep using current velocity, so drag tracks the body's
    // actual speed even as it changes within a frame.
    // F_drag = -linear_drag_coeff * v, τ_drag = -angular_drag_coeff * ω.
    linear_drag_coeff: f32,
    angular_drag_coeff: f32,

    // Attached colliders
    colliders: Vec<ColliderHandle>,

    /// How this body converts a drive command into momentum, if it is driven
    /// at all. Exactly one form at a time — see [`BodyDrive`].
    ///
    /// One of R11's three surviving drive-aware sites, and the one the other
    /// two exist to service. It survives because a command has to be stored
    /// between the frame that writes it and the plan that reads it, and the
    /// body is what both of them are about. Nothing in `integrate_forces`,
    /// `integrate_bodies` or CCD reads it: `Some` and `None` behave
    /// identically everywhere except in `physics::drive`, which is what makes
    /// a driven body an ordinary rigid body through the whole solve.
    drive: Option<BodyDrive>,
    /// Fraction of the tangential budget this body may draw at contacts that
    /// are not holding it up. `1.0` — the default — grips everything it
    /// touches equally.
    ///
    /// The actuator's declaration, not the material's: it scales what *this*
    /// body draws and leaves the contact's own coefficient alone, so a crate
    /// the body leans on keeps its own grip. A character sets it near zero so
    /// jumps along vertical surfaces are not grabbed.
    non_support_grip: f32,
    /// What this body may conjure where no contact can deliver it, and what it
    /// has been asked to spend that on this frame.
    ///
    /// The default grants nothing, which is what every body in the world has
    /// unless an actuator declared otherwise. See
    /// [`crate::physics::drive::allowance`].
    allowance: AllowanceCommand,
}

/// Newton steps allowed on the gyroscopic update per frame.
///
/// One is the usual recommendation and is enough while `|ω|·dt` is small. It
/// is not small for a piece of debris off an explosion, which can leave at
/// tens of radians a second and take most of a radian in a single frame; one
/// step there loses a third of the body's angular momentum over a few seconds,
/// so a tumbling shard visibly slows to a stop in mid-air. Newton converges
/// quadratically, so the extra steps are nearly free and usually not taken.
const GYROSCOPIC_ITERATIONS: usize = 4;

/// Change in ω, in rad/s, below which the gyroscopic iteration has converged.
const GYROSCOPIC_TOLERANCE: f32 = 1e-6;

impl RigidBody {
    pub(crate) fn new(desc: RigidBodyDesc) -> Self {
        Self {
            body_type: desc.body_type,
            position: desc.position,
            rotation: desc.rotation,
            linear_velocity: desc.linear_velocity,
            angular_velocity: desc.angular_velocity,
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            gravity_scale: desc.gravity_scale,
            mass: 0.0,
            inv_mass: 0.0,
            local_inertia: Matrix3::zeros(),
            inv_local_inertia: Matrix3::zeros(),
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
            linear_drag_coeff: 0.0,
            angular_drag_coeff: 0.0,
            colliders: Vec::new(),
            drive: None,
            non_support_grip: 1.0,
            allowance: AllowanceCommand::default(),
        }
    }

    // === Accessors ===

    pub fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type == BodyType::Kinematic
    }

    pub fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    pub fn position(&self) -> Point3<f32> {
        self.position
    }

    pub fn rotation(&self) -> UnitQuaternion<f32> {
        self.rotation
    }

    pub fn linear_velocity(&self) -> Vector3<f32> {
        self.linear_velocity
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        self.angular_velocity
    }

    /// Velocity of the material point of this body that is currently at
    /// `world_point`.
    ///
    /// Linear velocity plus the spin about the centre. A point on a rotating
    /// platform moves even when the platform's centre does not, which is the
    /// whole reason anything standing on it has to ask per point rather than
    /// per body.
    pub fn velocity_at(&self, world_point: Point3<f32>) -> Vector3<f32> {
        self.linear_velocity + self.angular_velocity.cross(&(world_point - self.position))
    }

    pub fn kinetic_energy(&self) -> f32 {
        if self.inv_mass == 0.0 {
            return 0.0;
        }
        let linear = 0.5 * self.mass * self.linear_velocity.magnitude_squared();
        let world_inertia = transform_inertia_tensor(&self.local_inertia, &self.rotation);
        let angular = 0.5
            * self
                .angular_velocity
                .dot(&(world_inertia * self.angular_velocity));
        linear + angular
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    /// How much of the world's gravity this body feels. A grenade thrown on a
    /// lighter arc than the world's own runs at a scale below 1.
    pub fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders
    }

    /// Get the world-space inverse inertia tensor.
    pub fn world_inv_inertia(&self) -> Matrix3<f32> {
        if self.inv_mass == 0.0 {
            return Matrix3::zeros();
        }
        transform_inertia_tensor(&self.inv_local_inertia, &self.rotation)
    }

    // === Mutators ===

    pub fn set_position(&mut self, position: Point3<f32>) {
        self.position = position;
    }

    pub fn set_rotation(&mut self, rotation: UnitQuaternion<f32>) {
        self.rotation = rotation;
    }

    pub fn set_linear_velocity(&mut self, velocity: Vector3<f32>) {
        self.linear_velocity = velocity;
    }

    pub fn set_linear_velocity_y(&mut self, y: f32) {
        self.linear_velocity.y = y;
    }

    pub fn set_angular_velocity(&mut self, velocity: Vector3<f32>) {
        self.angular_velocity = velocity;
    }

    /// Drive this body toward `drive`'s support-relative targets, with the
    /// reaction landing on whatever holds it up.
    ///
    /// Nothing happens here and nothing happens during force integration: the
    /// command is read by the traction planner, which turns it into a target
    /// for the tangential row at each supporting contact. The solver can
    /// therefore oppose it with the same impulses that oppose everything else,
    /// and the reaction lands on the support. Replaces any drive the body
    /// already had.
    pub(crate) fn set_support_drive(&mut self, drive: SupportDrive) {
        self.drive = Some(BodyDrive::Support(drive));
    }

    /// Record that this body's drive is delivered by the medium rows behind
    /// `constraint`. Replaces any drive the body already had.
    pub(crate) fn set_medium_drive(&mut self, constraint: ConstraintHandle) {
        self.drive = Some(BodyDrive::Medium(constraint));
    }

    /// Take this body out of service, returning whatever drive it had so the
    /// caller can retire the state that backs it.
    pub(crate) fn take_drive(&mut self) -> Option<BodyDrive> {
        self.drive.take()
    }

    /// This body's support-anchored command, if that is the anchor it declared.
    /// A medium anchor answers `None` — its command is solver rows.
    pub(crate) fn support_drive(&self) -> Option<SupportDrive> {
        match &self.drive {
            Some(BodyDrive::Support(drive)) => Some(*drive),
            _ => None,
        }
    }

    /// The rows this body's drive is delivered by, if it has a medium anchor.
    pub fn medium_drive(&self) -> Option<ConstraintHandle> {
        match self.drive {
            Some(BodyDrive::Medium(handle)) => Some(handle),
            _ => None,
        }
    }

    /// What this body may draw at a contact outside its Support Set, as a
    /// fraction of the tangential budget. Clamped to `[0, 1]`.
    pub fn set_non_support_grip(&mut self, grip: f32) {
        self.non_support_grip = grip.clamp(0.0, 1.0);
    }

    pub fn non_support_grip(&self) -> f32 {
        self.non_support_grip
    }

    /// Record this frame's non-conservative authority and what it is to be
    /// spent on. Replaces whatever the previous frame left.
    pub(crate) fn set_allowance_command(&mut self, command: AllowanceCommand) {
        self.allowance = command;
    }

    /// What this body may conjure, and what it has been asked to conjure.
    pub(crate) fn allowance_command(&self) -> AllowanceCommand {
        self.allowance
    }

    /// Consume the frame's edge-triggered verbs, leaving the continuous half
    /// of the allowance in place.
    pub(crate) fn take_drive_verbs(&mut self) -> NormalVerbs {
        self.allowance.take_verbs()
    }

    /// Scale the diagonal elements of the local inertia tensor.
    ///
    /// Call after collider attachment to adjust how resistant the body is to
    /// angular acceleration on each axis. For example, `Vector3::new(1, 50, 1)`
    /// makes the body very resistant to yaw torques from contacts while leaving
    /// pitch and roll unchanged.
    pub fn scale_local_inertia(&mut self, scale: Vector3<f32>) {
        self.local_inertia.m11 *= scale.x;
        self.local_inertia.m22 *= scale.y;
        self.local_inertia.m33 *= scale.z;
        if let Some(inv) = self.local_inertia.try_inverse() {
            self.inv_local_inertia = inv;
        }
    }

    /// Apply an instantaneous linear impulse at the center of mass.
    pub fn apply_impulse(&mut self, impulse: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.linear_velocity += impulse * self.inv_mass;
        }
    }

    /// Apply an instantaneous impulse at a world-space point.
    pub fn apply_impulse_at_point(&mut self, impulse: Vector3<f32>, point: Point3<f32>) {
        if self.inv_mass > 0.0 {
            self.linear_velocity += impulse * self.inv_mass;
            let r = point - self.position;
            let angular_impulse = r.cross(&impulse);
            self.angular_velocity += self.world_inv_inertia() * angular_impulse;
        }
    }

    /// Apply an instantaneous angular impulse (torque × dt) directly.
    pub fn apply_angular_impulse(&mut self, angular_impulse: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.angular_velocity += self.world_inv_inertia() * angular_impulse;
        }
    }

    /// Set the force accumulator directly.
    ///
    /// The accumulator is NOT cleared per substep, so the force integrates
    /// across all substeps within a frame. The caller is responsible for
    /// updating or clearing these each frame.
    pub fn set_force(&mut self, force: Vector3<f32>) {
        self.force = force;
    }

    /// Set the torque accumulator directly.
    ///
    /// The accumulator is NOT cleared per substep, so the torque integrates
    /// across all substeps within a frame. The caller is responsible for
    /// updating or clearing these each frame.
    pub fn set_torque(&mut self, torque: Vector3<f32>) {
        self.torque = torque;
    }

    /// Set drag coefficients applied per substep using the body's current
    /// velocity. This ensures drag tracks actual speed even as the body
    /// accelerates within a frame (e.g. a light body bobbing in water).
    ///
    /// `linear`: drag force magnitude per unit velocity (N·s/m).
    /// `angular`: drag torque magnitude per unit angular velocity (N·m·s/rad).
    pub fn set_drag(&mut self, linear: f32, angular: f32) {
        self.linear_drag_coeff = linear;
        self.angular_drag_coeff = angular;
    }

    // === Internal methods ===

    pub(crate) fn add_collider(&mut self, handle: ColliderHandle) {
        self.colliders.push(handle);
    }

    /// Remove a collider from this body's list. Returns true if found.
    pub(crate) fn remove_collider(&mut self, handle: ColliderHandle) -> bool {
        if let Some(pos) = self.colliders.iter().position(|h| *h == handle) {
            self.colliders.swap_remove(pos);
            true
        } else {
            false
        }
    }

    /// Update mass properties from attached colliders.
    /// Called by PhysicsWorld when colliders are added/removed.
    pub(crate) fn set_mass_properties(&mut self, mass: f32, local_inertia: Matrix3<f32>) {
        self.mass = mass;
        self.local_inertia = local_inertia;

        if self.body_type == BodyType::Dynamic && mass > 0.0 {
            self.inv_mass = 1.0 / mass;
            // Invert the inertia tensor
            if let Some(inv) = local_inertia.try_inverse() {
                self.inv_local_inertia = inv;
            } else {
                self.inv_local_inertia = Matrix3::zeros();
            }
        } else {
            // Static and kinematic bodies have infinite mass
            self.inv_mass = 0.0;
            self.inv_local_inertia = Matrix3::zeros();
        }
    }

    /// Integrate velocities from gravity, accumulated forces and drag.
    ///
    /// Knows nothing about drives. Both anchors are solved rows now — the
    /// support anchor through the tangential rows at its contacts, the medium
    /// anchor through world-anchored motor rows — so there is no pre-solve
    /// chase to keep out of gravity's way and no target to shift back
    /// afterwards.
    pub(crate) fn integrate_forces(&mut self, dt: f32, gravity: Vector3<f32>) {
        if self.body_type != BodyType::Dynamic {
            return;
        }

        // Apply all external effects: gravity, accumulated forces, drag.
        self.linear_velocity += gravity * self.gravity_scale * dt;
        self.linear_velocity += self.force * self.inv_mass * dt;
        if self.linear_drag_coeff > 0.0 {
            let decay = (-self.linear_drag_coeff * self.inv_mass * dt).exp();
            self.linear_velocity *= decay;
        }

        self.angular_velocity += self.world_inv_inertia() * self.torque * dt;
        if self.angular_drag_coeff > 0.0 {
            // Use the average inverse inertia (trace(I_inv)/3) so that
            // angular drag scales with rotational inertia, not mass.
            let inv_i = self.world_inv_inertia();
            let avg_inv_inertia = (inv_i.m11 + inv_i.m22 + inv_i.m33) / 3.0;
            let decay = (-self.angular_drag_coeff * avg_inv_inertia * dt).exp();
            self.angular_velocity *= decay;
        }

        // Gyroscopic correction: Euler's equation for rigid body rotation is
        //   I·dω/dt = τ_ext − ω × (I·ω)
        // Without the cross term, angular momentum drifts as the body rotates,
        // causing spurious precession/nutation (especially for elongated shapes
        // with very different principal moments).
        self.apply_gyroscopic_correction(dt);

        self.linear_velocity *= (1.0 - self.linear_damping.min(1.0)).powf(dt);
        self.angular_velocity *= (1.0 - self.angular_damping.min(1.0)).powf(dt);
    }

    /// Apply the gyroscopic torque correction: −ω × (I·ω).
    ///
    /// Solved implicitly, by one Newton step on Euler's equations in the
    /// body's own frame, because the explicit form is not merely inaccurate —
    /// it is unstable. Free rotation of a body whose three principal moments
    /// differ has a genuine instability about the intermediate axis, and an
    /// explicit step *adds* energy on every frame of it: a brick spun about a
    /// mixed axis in empty space, with no gravity and no contact and angular
    /// damping working against it, wound itself from 27 rad/s to 119 rad/s in
    /// ten seconds and kept going. Clamping the step, which is what this used
    /// to do, bounds how fast it winds up and not whether it does.
    ///
    /// Solving for the ω that satisfies the equation at the *end* of the step
    /// instead is stable at any spin and any timestep, and holds |I·ω| — the
    /// quantity free rotation must conserve — steady.
    ///
    /// Reference: Catto, "Numerical Methods", GDC 2015.
    fn apply_gyroscopic_correction(&mut self, dt: f32) {
        if self.angular_velocity.magnitude_squared() < 1e-12 {
            return;
        }

        // Euler's equations are diagonal in the body frame, which is where
        // the inertia tensor is constant and the Newton step is cheap.
        let start = self
            .rotation
            .inverse_transform_vector(&self.angular_velocity);
        let mut omega = start;

        for _ in 0..GYROSCOPIC_ITERATIONS {
            let momentum = self.local_inertia * omega;
            // g(ω) = I·(ω − ω₀) + dt · ω × I·ω, the equation to satisfy at the
            // *end* of the step, and its derivative with respect to ω.
            let residual = self.local_inertia * (omega - start) + omega.cross(&momentum) * dt;
            let jacobian =
                self.local_inertia + (skew(&omega) * self.local_inertia - skew(&momentum)) * dt;
            let Some(step) = jacobian.try_inverse() else {
                // A body with no inertia about some axis — nothing to precess.
                return;
            };
            let correction = step * residual;
            omega -= correction;
            if correction.magnitude_squared() <= GYROSCOPIC_TOLERANCE * GYROSCOPIC_TOLERANCE {
                break;
            }
        }

        if !omega.iter().all(|v| v.is_finite()) {
            return;
        }

        // The implicit step is stable where the explicit one was not, and it
        // pays for that by damping: a fast tumble lost a third of its angular
        // momentum over half a minute, so debris visibly wound down in
        // mid-air. But the gyroscopic term does no work and applies no
        // torque — free rotation conserves |I·ω| exactly — so the size of the
        // answer is known in advance and only its direction had to be solved
        // for. Putting the magnitude back makes the step lossless.
        let before = (self.local_inertia * start).magnitude();
        let after = (self.local_inertia * omega).magnitude();
        if after > 1e-9 {
            omega *= before / after;
        }

        self.angular_velocity = self.rotation.transform_vector(&omega);
    }

    /// Integrate positions from velocities.
    pub(crate) fn integrate_velocities(&mut self, dt: f32) {
        if self.body_type == BodyType::Static {
            return;
        }

        self.position += self.linear_velocity * dt;
        self.rotation = integrate_orientation(self.rotation, self.angular_velocity, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dynamic_body() -> RigidBody {
        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        body.set_mass_properties(1.0, Matrix3::identity());
        body
    }

    /// A body whose three principal moments all differ, which is the only
    /// case free rotation is interesting in: a brick, not a ball.
    fn tumbling_body(spin: Vector3<f32>) -> RigidBody {
        // Damping off, so that what the measurements see is the integrator
        // and not a decay the body was asked for.
        let mut body = RigidBody::new(
            RigidBodyDesc::dynamic()
                .angular_velocity(spin)
                .angular_damping(0.0)
                .linear_damping(0.0),
        );
        body.set_mass_properties(
            24.0,
            Matrix3::from_diagonal(&Vector3::new(0.31, 0.36, 0.40)),
        );
        body
    }

    /// Angular momentum and rotational energy, the two quantities a body
    /// spinning with nothing acting on it must keep.
    fn rotational_state(body: &RigidBody) -> (f32, f32) {
        let inertia = transform_inertia_tensor(&body.local_inertia, &body.rotation);
        let momentum = inertia * body.angular_velocity;
        (
            momentum.magnitude(),
            0.5 * body.angular_velocity.dot(&momentum),
        )
    }

    /// Spin a body in empty space for `seconds`. Reports what became of its
    /// angular momentum, as a ratio of where it started, and the fastest it
    /// ever span.
    fn spun_freely(spin: Vector3<f32>, seconds: f32) -> (f32, f32) {
        const DT: f32 = 1.0 / 60.0;
        let mut body = tumbling_body(spin);
        let (momentum, _) = rotational_state(&body);
        let mut peak = body.angular_velocity.magnitude();
        for _ in 0..(seconds / DT) as usize {
            body.integrate_forces(DT, Vector3::zeros());
            body.integrate_velocities(DT);
            peak = peak.max(body.angular_velocity.magnitude());
        }
        (rotational_state(&body).0 / momentum, peak)
    }

    /// The largest angular speed a body holding this much angular momentum
    /// can possibly have: all of it about its easiest axis to spin.
    fn fastest_possible_spin(body: &RigidBody, momentum: f32) -> f32 {
        let smallest = body
            .local_inertia
            .diagonal()
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        momentum / smallest
    }

    /// The bug this guards against took a machine down. A body whose
    /// principal moments differ has a genuine instability about its
    /// intermediate axis, and the gyroscopic term used to be integrated
    /// explicitly, which *adds* energy on every frame of it. A brick spun
    /// about a mixed axis wound itself from 27 rad/s past 100 in ten seconds
    /// with nothing touching it; in the game a blown-off piece of ice climbed
    /// until its swept bounding box covered the level, and the terrain query
    /// that followed exhausted memory.
    ///
    /// Free rotation applies no torque, so angular momentum is the invariant
    /// to hold — and holding it is what bounds everything else. A body may
    /// legitimately trade angular *speed* between its axes as it tumbles, up
    /// to the ratio of its largest principal moment to its smallest, but it
    /// can never exceed what its momentum allows. Spinning near the
    /// intermediate axis, where the tumble is genuinely unstable, is the case
    /// that says so.
    #[test]
    fn a_tumbling_body_does_not_wind_itself_up() {
        for spin in [
            Vector3::new(-12.0, -18.0, 16.0),
            Vector3::new(1.0, 30.0, 1.0),
            // About the intermediate axis, the unstable one.
            Vector3::new(0.2, 60.0, 0.1),
            Vector3::new(40.0, 40.0, 40.0),
        ] {
            let body = tumbling_body(spin);
            let (momentum, _) = rotational_state(&body);
            let ceiling = fastest_possible_spin(&body, momentum);

            let (kept, peak) = spun_freely(spin, 30.0);
            assert!(
                (kept - 1.0).abs() < 0.01,
                "spun about {spin:?}, angular momentum became {kept:.4} of what it was"
            );
            assert!(
                peak <= ceiling * 1.01,
                "spun about {spin:?}, reached {peak:.1} rad/s where its momentum allows {ceiling:.1}"
            );
        }
    }

    /// Spun about one of its own principal axes a body just keeps spinning:
    /// there is no gyroscopic term at all, so this is the case that says the
    /// correction stays out of the way when it has nothing to correct.
    #[test]
    fn a_body_spun_about_a_principal_axis_is_left_alone() {
        for axis in 0..3 {
            let mut spin = Vector3::zeros();
            spin[axis] = 11.0;
            let (kept, peak) = spun_freely(spin, 30.0);
            assert!((kept - 1.0).abs() < 1e-3, "axis {axis}: momentum {kept}");
            assert!((peak - 11.0).abs() < 1e-2, "axis {axis}: peak {peak}");
        }
    }

    /// Force integration no longer knows what a drive is. Both anchors are
    /// solved rows, so a body carrying a support command falls exactly as an
    /// undriven one does — the whole reason the target shift could go.
    #[test]
    fn a_support_command_does_not_reach_force_integration() {
        let mut body = dynamic_body();
        body.set_support_drive(SupportDrive {
            linear_target: Vector3::new(10.0, 0.0, 0.0),
            angular_target: Vector3::new(0.0, 3.0, 0.0),
            gain: 5.0,
            patch_radius: 0.0,
        });

        let gravity = Vector3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 240.0;
        for _ in 0..4 {
            body.integrate_forces(dt, gravity);
        }

        assert_eq!(body.linear_velocity().x, 0.0);
        assert_eq!(body.angular_velocity().y, 0.0);
        let expected_y = gravity.y * dt * 4.0;
        assert!(
            (body.linear_velocity().y - expected_y).abs() < 0.01,
            "gravity should integrate untouched, got {} expected {}",
            body.linear_velocity().y,
            expected_y
        );
    }

    #[test]
    fn set_body_velocity_does_not_create_drive() {
        let mut body = dynamic_body();
        body.set_linear_velocity(Vector3::new(5.0, 0.0, 0.0));

        // Simulate friction reducing velocity
        body.set_linear_velocity(Vector3::new(2.0, 0.0, 0.0));
        body.integrate_forces(1.0 / 240.0, Vector3::zeros());

        // Without a drive, velocity should stay near 2.0 (no re-acceleration)
        assert!(
            (body.linear_velocity().x - 2.0).abs() < 0.1,
            "velocity should not re-accelerate without a drive: got {}",
            body.linear_velocity().x
        );
    }

    #[test]
    fn gyroscopic_no_effect_on_principal_axis() {
        // Spinning about a principal axis should produce zero gyroscopic torque.
        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        let inertia = Matrix3::from_diagonal(&Vector3::new(10.0, 1.0, 10.0));
        body.set_mass_properties(1.0, inertia);
        body.set_angular_velocity(Vector3::new(0.0, 5.0, 0.0));

        let omega_before = body.angular_velocity();
        body.integrate_forces(1.0 / 240.0, Vector3::zeros());
        let omega_after = body.angular_velocity();

        // Only damping should change ω, no gyroscopic drift
        let diff = omega_after - omega_before * (1.0 - 0.05_f32).powf(1.0 / 240.0);
        assert!(
            diff.magnitude() < 1e-4,
            "principal-axis spin should be unaffected by gyroscopic term, diff={diff:?}"
        );
    }

    #[test]
    fn gyroscopic_correction_reduces_cross_axis_drift() {
        // A body with I_y << I_xz and angular velocity in both X and Y
        // should get a gyroscopic correction that prevents drift into Z.
        let mut body = RigidBody::new(RigidBodyDesc::dynamic().angular_damping(0.0));
        let inertia = Matrix3::from_diagonal(&Vector3::new(10.0, 0.1, 10.0));
        body.set_mass_properties(1.0, inertia);
        body.set_angular_velocity(Vector3::new(1.0, 0.5, 0.0));

        let dt = 1.0 / 240.0;
        body.integrate_forces(dt, Vector3::zeros());

        // The gyroscopic term ω × (I·ω) has a Z component:
        //   (1, 0.5, 0) × (10, 0.05, 0) = (0, 0, 1·0.05 − 0.5·10) = (0, 0, −4.95)
        // So the correction should push ω_z in the positive direction.
        assert!(
            body.angular_velocity().z > 0.0,
            "gyroscopic correction should create Z component, got ω_z={}",
            body.angular_velocity().z
        );
    }
}
