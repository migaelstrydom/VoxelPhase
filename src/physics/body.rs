//! Rigid body representation.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

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
#[derive(Debug, Clone)]
pub struct VelocityDrive {
    /// Target velocity the body accelerates toward.
    pub target: Vector3<f32>,
    /// Maximum acceleration magnitude (units/s²).
    pub max_accel: f32,
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

    // Force/torque accumulators (cleared each step)
    force: Vector3<f32>,
    torque: Vector3<f32>,

    // Attached colliders
    colliders: Vec<ColliderHandle>,

    /// Optional per-substep velocity drive for externally controlled bodies.
    velocity_drive: Option<VelocityDrive>,
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
            colliders: Vec::new(),
            velocity_drive: None,
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

    /// Set a velocity drive that accelerates toward `target` each substep.
    ///
    /// The drive is applied during force integration, before the solver.
    /// This replaces direct velocity setting for externally controlled bodies
    /// (player, moving platforms) so that the solver can properly oppose the
    /// drive when pushing heavy objects.
    pub fn set_velocity_drive(&mut self, target: Vector3<f32>, max_accel: f32) {
        self.velocity_drive = Some(VelocityDrive { target, max_accel });
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

    // === Internal methods ===

    pub(crate) fn add_collider(&mut self, handle: ColliderHandle) {
        self.colliders.push(handle);
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

        // Apply gravity
        self.linear_velocity += gravity * self.gravity_scale * dt;

        // Apply velocity drive on horizontal axes only. The drive accelerates
        // toward the target each substep so the solver can oppose it at contacts.
        // Vertical velocity is left to gravity and the solver.
        if let Some(drive) = &self.velocity_drive {
            let dx = drive.target.x - self.linear_velocity.x;
            let dz = drive.target.z - self.linear_velocity.z;
            let delta_mag = (dx * dx + dz * dz).sqrt();
            let max_delta = drive.max_accel * dt;
            if delta_mag > 1e-6 {
                let scale = (max_delta / delta_mag).min(1.0);
                self.linear_velocity.x += dx * scale;
                self.linear_velocity.z += dz * scale;
            }
        }

        // Apply accumulated forces
        self.linear_velocity += self.force * self.inv_mass * dt;
        self.angular_velocity += self.world_inv_inertia() * self.torque * dt;

        // Gyroscopic correction: Euler's equation for rigid body rotation is
        //   I·dω/dt = τ_ext − ω × (I·ω)
        // Without the cross term, angular momentum drifts as the body rotates,
        // causing spurious precession/nutation (especially for elongated shapes
        // with very different principal moments).
        self.apply_gyroscopic_correction(dt);

        // Apply damping
        self.linear_velocity *= (1.0 - self.linear_damping.min(1.0)).powf(dt);
        self.angular_velocity *= (1.0 - self.angular_damping.min(1.0)).powf(dt);

        // Clear accumulators
        self.force = Vector3::zeros();
        self.torque = Vector3::zeros();
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
    fn velocity_drive_only_affects_horizontal() {
        let mut body = dynamic_body();
        body.set_linear_velocity(Vector3::new(0.0, -2.0, 0.0));
        body.set_velocity_drive(Vector3::new(5.0, 0.0, 3.0), 5000.0);

        let gravity = Vector3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 240.0;
        body.integrate_forces(dt, gravity);

        // X and Z should move toward drive target
        assert!(
            (body.linear_velocity().x - 5.0).abs() < 0.1,
            "X should reach drive target, got {}",
            body.linear_velocity().x,
        );
        assert!(
            (body.linear_velocity().z - 3.0).abs() < 0.1,
            "Z should reach drive target, got {}",
            body.linear_velocity().z,
        );

        // Y should only have gravity applied, not driven
        let expected_y = -2.0 + gravity.y * dt;
        assert!(
            (body.linear_velocity().y - expected_y).abs() < 0.01,
            "Y should reflect gravity only, got {} expected {}",
            body.linear_velocity().y,
            expected_y
        );
    }

    #[test]
    fn velocity_drive_clamps_to_max_accel() {
        let mut body = dynamic_body();
        body.set_linear_velocity(Vector3::zeros());
        // Target is 10 m/s but max_accel limits how fast we get there
        body.set_velocity_drive(Vector3::new(10.0, 0.0, 0.0), 50.0);

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
        let mut body = RigidBody::new(
            RigidBodyDesc::dynamic().angular_damping(0.0),
        );
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
