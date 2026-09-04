//! Rigid body representation.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

use super::constraint::types::ConstraintHandle;
use super::handle::ColliderHandle;
use super::math::{integrate_orientation, transform_inertia_tensor};

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

/// Per-substep velocity drive for externally controlled bodies.
///
/// Instead of directly overwriting a body's velocity (which fights the solver),
/// the drive applies a clamped acceleration toward a target each substep.
/// The solver then applies contact impulses on top, allowing equilibrium
/// when pushing heavy objects.
#[derive(Debug, Clone, Copy)]
pub struct VelocityDrive {
    /// Target velocity the body accelerates toward.
    pub target: Vector3<f32>,
    /// Maximum acceleration magnitude (units/s²).
    pub max_accel: f32,
}

/// What a body's actuator pushes against, and the engine state that delivers
/// it.
///
/// One field, two variants, and that is the point: a body may not both chase a
/// velocity target before the solve and carry a world-anchored motor row
/// through it. Overlapping the two would hand the body roughly twice the
/// authority its actuator declares and leave the row's bounds meaning nothing,
/// so the overlap is made unrepresentable rather than merely avoided.
/// `PhysicsWorld::set_body_drive` is the only writer.
#[derive(Debug, Clone)]
pub enum BodyDrive {
    /// Reaction goes into whatever holds the body up. Applied during force
    /// integration as a bounded chase toward the target — the pre-solve form
    /// the traction drive replaces.
    Support {
        /// Linear target and its acceleration budget.
        linear: VelocityDrive,
        /// Angular target and its acceleration budget.
        angular: VelocityDrive,
    },
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
}

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

    /// Drive this body toward `linear` and `angular` with the reaction landing
    /// on whatever holds it up.
    ///
    /// The drive is applied during force integration, before the solver, so
    /// the solver can oppose it via contact impulses and reach equilibrium
    /// when pushing heavy objects. Replaces any drive the body already had.
    pub(crate) fn set_support_drive(
        &mut self,
        linear: Vector3<f32>,
        max_accel: f32,
        angular: Vector3<f32>,
        angular_max_accel: f32,
    ) {
        self.drive = Some(BodyDrive::Support {
            linear: VelocityDrive {
                target: linear,
                max_accel,
            },
            angular: VelocityDrive {
                target: angular,
                max_accel: angular_max_accel,
            },
        });
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

    /// The linear half of a support-anchored drive, if that is what this body
    /// has. A medium anchor answers `None` — its command is solver rows.
    fn support_drive(&self) -> Option<VelocityDrive> {
        match &self.drive {
            Some(BodyDrive::Support { linear, .. }) => Some(*linear),
            _ => None,
        }
    }

    /// The angular half of a support-anchored drive.
    fn support_angular_drive(&self) -> Option<VelocityDrive> {
        match &self.drive {
            Some(BodyDrive::Support { angular, .. }) => Some(*angular),
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

    /// Integrate velocities from forces and velocity drives.
    pub(crate) fn integrate_forces(&mut self, dt: f32, gravity: Vector3<f32>) {
        if self.body_type != BodyType::Dynamic {
            return;
        }

        // Apply velocity drive first, against the clean previous-substep
        // velocity. A medium-anchored drive is a set of solver rows and has
        // nothing to do here.
        if let Some(drive) = self.support_drive() {
            let delta = drive.target - self.linear_velocity;
            let delta_mag = delta.magnitude();
            let max_delta = drive.max_accel * dt;
            if delta_mag > 1e-6 {
                let scale = (max_delta / delta_mag).min(1.0);
                self.linear_velocity += delta * scale;
            }
        }

        // Record velocity after drive, before external effects.
        let vel_after_drive = self.linear_velocity;

        // Apply all external effects: gravity, accumulated forces, drag.
        self.linear_velocity += gravity * self.gravity_scale * dt;
        self.linear_velocity += self.force * self.inv_mass * dt;
        if self.linear_drag_coeff > 0.0 {
            let decay = (-self.linear_drag_coeff * self.inv_mass * dt).exp();
            self.linear_velocity *= decay;
        }

        // Apply angular velocity drive (same pattern as linear drive).
        if let Some(drive) = self.support_angular_drive() {
            let delta = drive.target - self.angular_velocity;
            let delta_mag = delta.magnitude();
            let max_delta = drive.max_accel * dt;
            if delta_mag > 1e-6 {
                let scale = (max_delta / delta_mag).min(1.0);
                self.angular_velocity += delta * scale;
            }
        }

        let ang_vel_after_drive = self.angular_velocity;

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

        // Shift drive targets by external forces (gravity, accumulated forces,
        // drag, torques, gyroscopic correction) so drives don't fight these
        // effects across substeps. Damping is excluded — it's a resistive effect
        // that drives should actively overcome.
        if let Some(BodyDrive::Support { linear, angular }) = &mut self.drive {
            linear.target += self.linear_velocity - vel_after_drive;
            angular.target += self.angular_velocity - ang_vel_after_drive;
        }

        // Apply damping after target shift — drives fight damping intentionally.
        self.linear_velocity *= (1.0 - self.linear_damping.min(1.0)).powf(dt);
        self.angular_velocity *= (1.0 - self.angular_damping.min(1.0)).powf(dt);
    }

    /// Apply the gyroscopic torque correction: −ω × (I·ω).
    ///
    /// Uses explicit Euler with a magnitude clamp to prevent instability at
    /// large angular velocities or timesteps.
    fn apply_gyroscopic_correction(&mut self, dt: f32) {
        let omega_sq = self.angular_velocity.magnitude_squared();
        if omega_sq < 1e-12 {
            return;
        }

        let world_inertia = transform_inertia_tensor(&self.local_inertia, &self.rotation);
        let angular_momentum = world_inertia * self.angular_velocity;
        let gyro_torque = self.angular_velocity.cross(&angular_momentum);

        if gyro_torque.magnitude_squared() < 1e-12 {
            return;
        }

        let correction = self.world_inv_inertia() * gyro_torque * dt;

        // Clamp the correction to a fraction of |ω| to keep explicit Euler stable.
        let corr_mag = correction.magnitude();
        let max_corr = omega_sq.sqrt() * 0.125;
        if corr_mag > max_corr {
            self.angular_velocity -= correction * (max_corr / corr_mag);
        } else {
            self.angular_velocity -= correction;
        }
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

    #[test]
    fn velocity_drive_does_not_fight_gravity() {
        let mut body = dynamic_body();
        body.set_linear_velocity(Vector3::new(0.0, 0.0, 0.0));
        // Drive targets zero on all axes with high max_accel.
        body.set_support_drive(Vector3::zeros(), 5000.0, Vector3::zeros(), 5000.0);

        let gravity = Vector3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 240.0;

        // Simulate 4 substeps (typical frame).
        for _ in 0..4 {
            body.integrate_forces(dt, gravity);
        }

        // Y should accumulate gravity over all 4 substeps. The drive target
        // shifts with gravity each substep, so the drive doesn't fight it.
        let expected_y = gravity.y * dt * 4.0;
        assert!(
            (body.linear_velocity().y - expected_y).abs() < 0.01,
            "Y should reflect full gravity accumulation, got {} expected {}",
            body.linear_velocity().y,
            expected_y
        );
    }

    #[test]
    fn velocity_drive_clamps_to_max_accel() {
        let mut body = dynamic_body();
        body.set_linear_velocity(Vector3::zeros());
        // Target is 10 m/s but max_accel limits how fast we get there
        body.set_support_drive(Vector3::new(10.0, 0.0, 0.0), 50.0, Vector3::zeros(), 50.0);

        let dt = 1.0 / 240.0;
        body.integrate_forces(dt, Vector3::zeros());

        // max_delta = 50 * (1/240) ≈ 0.208, should not reach 10.0
        let vx = body.linear_velocity().x;
        assert!(
            vx < 0.25 && vx > 0.15,
            "velocity should be clamped by max_accel: got {vx}"
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
