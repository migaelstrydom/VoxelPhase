use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage, VecStorage};

use crate::collision::Sphere;
use crate::model::Model;
use crate::rendering::camera::Camera;

// Physics Components
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Position(pub Vector3<f32>);

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Velocity(pub Vector3<f32>);

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Acceleration(pub Vector3<f32>);

/// Gravity strength for an entity. Applied as downward acceleration.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Gravity(pub f32);

/// Motion state for CCD collision detection.
///
/// Stores previous and predicted positions for swept collision queries.
/// Updated by MotionPredictionSystem, consumed by collision systems.
#[derive(Component, Debug, Clone)]
#[storage(VecStorage)]
pub struct MotionState {
    /// Position at the start of the frame (before movement).
    pub prev: Point3<f32>,
    /// Predicted position at the end of the frame (before collision resolution).
    pub predicted: Point3<f32>,
}

impl MotionState {
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            prev: position,
            predicted: position,
        }
    }
}

// Collision Components

/// Collision shape for an entity. Currently supports sphere only.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Collider {
    pub shape: Sphere,
}

impl Collider {
    pub fn sphere(radius: f32) -> Self {
        Self {
            shape: Sphere::new(radius),
        }
    }
}

// Sensor + IK Components

/// Purpose tag for terrain probes and IK targets.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProbePurpose {
    FootLeft,
    FootRight,
    HandLeft,
    HandRight,
    Wall,
    Ledge,
}

/// Probe shape used for terrain queries.
#[derive(Debug, Clone, Copy)]
pub enum ProbeShape {
    /// Ray-style probe implemented as a thin swept sphere.
    Ray { radius: f32 },
    /// Swept sphere probe with explicit radius.
    SphereSweep { radius: f32 },
}

/// A terrain probe defined in local space.
#[derive(Debug, Clone, Copy)]
pub struct Probe {
    pub purpose: ProbePurpose,
    /// Local-space origin relative to the entity.
    pub local_origin: Vector3<f32>,
    /// Local-space direction (does not need to be normalized).
    pub local_direction: Vector3<f32>,
    /// Probe length in world units.
    pub length: f32,
    pub shape: ProbeShape,
}

impl Probe {
    pub fn ray(
        purpose: ProbePurpose,
        local_origin: Vector3<f32>,
        local_direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Self {
        Self {
            purpose,
            local_origin,
            local_direction,
            length,
            shape: ProbeShape::Ray { radius },
        }
    }

    pub fn sphere_sweep(
        purpose: ProbePurpose,
        local_origin: Vector3<f32>,
        local_direction: Vector3<f32>,
        length: f32,
        radius: f32,
    ) -> Self {
        Self {
            purpose,
            local_origin,
            local_direction,
            length,
            shape: ProbeShape::SphereSweep { radius },
        }
    }
}

/// Sensor definitions for terrain awareness and IK target acquisition.
#[derive(Component, Debug, Clone)]
#[storage(VecStorage)]
pub struct SensorSet {
    pub probes: Vec<Probe>,
}

impl SensorSet {
    pub fn new(probes: Vec<Probe>) -> Self {
        Self { probes }
    }
}

/// A raw contact candidate returned by terrain probes.
#[derive(Debug, Clone)]
pub struct ContactCandidate {
    pub purpose: ProbePurpose,
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
    pub distance: f32,
}

/// All contact candidates produced for an entity this frame.
#[derive(Component, Debug, Default)]
#[storage(VecStorage)]
pub struct ContactCandidates {
    pub candidates: Vec<ContactCandidate>,
}

/// An IK target derived from contact candidates.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct IKTarget {
    pub purpose: ProbePurpose,
    pub target: Point3<f32>,
    pub normal: Vector3<f32>,
    pub weight: f32,
}

/// All IK targets for an entity.
#[derive(Component, Debug, Default)]
#[storage(VecStorage)]
pub struct IKTargets {
    pub targets: Vec<IKTarget>,
}

/// Desired pelvis height computed from IK targets.
#[derive(Component, Debug, Clone, Copy, Default)]
#[storage(VecStorage)]
pub struct PelvisTarget {
    pub target_y: f32,
    pub has_contact: bool,
}

/// Physical properties for collision response.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct PhysicsBody {
    /// Bounciness (0 = no bounce, 1 = perfect bounce).
    pub restitution: f32,
    /// Friction coefficient.
    pub friction: f32,
}

impl Default for PhysicsBody {
    fn default() -> Self {
        Self {
            restitution: 0.2,
            friction: 0.8,
        }
    }
}

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

/// A model instance referencing a shared Model definition.
#[derive(Component)]
#[storage(VecStorage)]
pub struct ModelInstance {
    /// The model definition (shared across instances).
    pub model: Arc<Model>,
}

impl ModelInstance {
    pub fn new(model: Arc<Model>) -> Self {
        Self { model }
    }
}

#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Renderable;

#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct CameraComponent(pub Camera);
