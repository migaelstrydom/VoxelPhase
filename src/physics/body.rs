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

    pub fn kinematic() -> Self {
        Self {
            body_type: BodyType::Kinematic,
            ..Self::default()
        }
    }

    pub fn position(mut self, position: Point3<f32>) -> Self {
        self.position = position;
        self
    }

    pub fn rotation(mut self, rotation: UnitQuaternion<f32>) -> Self {
        self.rotation = rotation;
        self
    }

    pub fn linear_velocity(mut self, velocity: Vector3<f32>) -> Self {
        self.linear_velocity = velocity;
        self
    }

    pub fn angular_velocity(mut self, velocity: Vector3<f32>) -> Self {
        self.angular_velocity = velocity;
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
        }
    }

    // === Accessors ===

    pub fn body_type(&self) -> BodyType {
        self.body_type
    }

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

    pub fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    pub fn force(&self) -> Vector3<f32> {
        self.force
    }

    pub fn torque(&self) -> Vector3<f32> {
        self.torque
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

    pub fn set_angular_velocity(&mut self, velocity: Vector3<f32>) {
        self.angular_velocity = velocity;
    }

    /// Apply a force at the center of mass (no torque generated).
    pub fn apply_force(&mut self, force: Vector3<f32>) {
        self.force += force;
    }

    /// Apply a force at a world-space point, generating both force and torque.
    pub fn apply_force_at_point(&mut self, force: Vector3<f32>, point: Point3<f32>) {
        self.force += force;
        let r = point - self.position;
        self.torque += r.cross(&force);
    }

    /// Apply a torque (no linear force).
    pub fn apply_torque(&mut self, torque: Vector3<f32>) {
        self.torque += torque;
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

    /// Apply an instantaneous angular impulse.
    pub fn apply_angular_impulse(&mut self, impulse: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.angular_velocity += self.world_inv_inertia() * impulse;
        }
    }

    // === Internal methods ===

    pub(crate) fn add_collider(&mut self, handle: ColliderHandle) {
        self.colliders.push(handle);
    }

    pub(crate) fn remove_collider(&mut self, handle: ColliderHandle) {
        self.colliders.retain(|h| *h != handle);
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

    /// Integrate velocities from forces.
    pub(crate) fn integrate_forces(&mut self, dt: f32, gravity: Vector3<f32>) {
        if self.body_type != BodyType::Dynamic {
            return;
        }

        // Apply gravity
        self.linear_velocity += gravity * self.gravity_scale * dt;

        // Apply accumulated forces
        self.linear_velocity += self.force * self.inv_mass * dt;
        self.angular_velocity += self.world_inv_inertia() * self.torque * dt;

        // Apply damping
        self.linear_velocity *= 1.0 - self.linear_damping.min(1.0);
        self.angular_velocity *= 1.0 - self.angular_damping.min(1.0);

        // Clear accumulators
        self.force = Vector3::zeros();
        self.torque = Vector3::zeros();
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
