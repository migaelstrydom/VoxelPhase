use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage, VecStorage};

use crate::model::{MeshPrimitive, Model, Transform};
use crate::physics::constraint::ConstraintHandle;
use crate::physics::RigidBodyHandle;
use crate::rendering::camera::Camera;
use crate::rendering::material::SurfaceModulation;

// Physics Components
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Position(pub Vector3<f32>);

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Velocity(pub Vector3<f32>);

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

/// 3D orientation as a unit quaternion.
/// Synced from physics for entities with RigidBodyComponent.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Orientation(pub UnitQuaternion<f32>);

impl Default for Orientation {
    fn default() -> Self {
        Self(UnitQuaternion::identity())
    }
}

/// Links an entity to a rigid body in the physics world.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct RigidBodyComponent(pub RigidBodyHandle);

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

/// What a [`PosedModel`]'s poses are measured in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseFrame {
    /// The world: for a rig, which works out where its joints are there.
    World,
    /// The entity's own frame, at its position and turned as it is: for
    /// parts that move on a body the physics carries.
    Entity,
}

/// A model drawn part by part, each part at its own pose: a creature made
/// of rigid pieces that turn and slide against one another (an eye in its
/// socket, a lid over it, a shin on a knee, a shell on its hinge).
///
/// The model is uploaded once and only the poses change, so the pieces may
/// be as finely modelled as they like. The system that owns the motion
/// writes the poses.
#[derive(Component)]
#[storage(VecStorage)]
pub struct PosedModel {
    /// The model, each part modelled about its own joint.
    pub model: Arc<Model>,
    /// Each part's pose, in part order. A part past the end is drawn at
    /// rest.
    pub poses: Vec<Transform>,
    /// What the poses are measured in.
    pub frame: PoseFrame,
}

impl PosedModel {
    /// `model` with every part at rest in `frame`.
    pub fn at_rest(model: Arc<Model>, frame: PoseFrame) -> Self {
        let poses = vec![Transform::default(); model.parts.len()];
        Self {
            model,
            poses,
            frame,
        }
    }
}

/// A mesh whose shape changes every frame, rebuilt by the system that owns
/// that shape and drawn in the entity's frame, as a [`ModelInstance`] is. Its
/// primitives are uploaded afresh each frame, so it is for small meshes that
/// deform: a trampoline's bed, not a level's worth of props.
#[derive(Component, Default)]
#[storage(VecStorage)]
pub struct ProceduralMesh {
    /// The mesh's primitives, in the entity's own frame.
    pub primitives: Vec<MeshPrimitive>,
}

#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Renderable;

/// A mesh drawn with the terrain's own surface, for terrain that moves: a
/// piece a blast cut loose. Its texture is projected from its own frame, so it
/// rides with it rather than swimming across it.
#[derive(Component)]
#[storage(VecStorage)]
pub struct TerrainMeshInstance {
    /// The mesh, around the entity's position. Its primitives' materials are
    /// not read: terrain's surface replaces them.
    pub model: Arc<Model>,
    /// Added to model position to address the texture: the world position the
    /// mesh's origin had while it was still ground.
    pub anchor: Vector3<f32>,
}

/// Per-instance override of the surface parameters of the entity's model.
///
/// Materials are shared between every instance of a model, so anything that
/// varies per entity and per frame — a grenade heating up as it flies — cannot
/// live in the material. This component carries that variation and is folded in
/// at draw time. An entity without one draws its materials exactly as authored.
#[derive(Component, Debug, Default, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct MaterialModulation(pub SurfaceModulation);

/// Anchors an entity to a fixed world-space position via physics constraints.
///
/// When the terrain beneath any of its anchor points is destroyed, its
/// constraints are removed, a body welded into the ground stops passing through
/// it (`RigidBody::ignores_static`), and this component is removed — releasing
/// the body to fall freely.
#[derive(Component)]
#[storage(DenseVecStorage)]
pub struct TerrainAnchored {
    /// The constraints holding the body to the ground (a fixed weld, or a
    /// pin and a KeepUpright), all removed on release.
    pub constraints: Vec<ConstraintHandle>,
    /// World-space positions to check for terrain solidity. The body is
    /// released as soon as the ground at any one of them is gone: a post needs
    /// one, under its foot; a stone bedded in the ground is loose once the
    /// ground under any part of it goes.
    pub anchor_points: Vec<nalgebra::Point3<f32>>,
    /// Model to swap in when released (e.g. to remove a rope visual that
    /// only makes sense while the constraint is active).
    pub released_model: Option<Arc<Model>>,
}

impl std::fmt::Debug for TerrainAnchored {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TerrainAnchored")
            .field("constraints", &self.constraints)
            .field("anchor_points", &self.anchor_points)
            .field("released_model", &self.released_model.is_some())
            .finish()
    }
}

#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct CameraComponent(pub Camera);
