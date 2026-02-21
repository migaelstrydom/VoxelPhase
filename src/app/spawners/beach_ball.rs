use std::sync::Arc;

use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::geometry::{generate_magic_sphere_vertices, generate_sphere_indices, MagicSphereConfig};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

const RADIUS: f32 = 0.5;
const SEGMENTS: u32 = 32;
const RINGS: u32 = 24;

/// Picks a random value in [lo, hi].
fn rand_range(lo: f32, hi: f32) -> f32 {
    lo + rand::random::<f32>() * (hi - lo)
}

/// Builds a unique beach ball model with a randomised colour pattern.
///
/// Parameter ranges are constrained to produce consistently attractive spirals:
/// - 2–5 spiral arms with moderate tightness for readable patterns
/// - Complementary/triadic accent offsets for colour contrast
/// - Subtle colour variation to avoid rainbow noise
fn build_model(material: MaterialId) -> Arc<Model> {
    let config = MagicSphereConfig {
        base_hue: rand::random::<f32>(),
        spiral_frequency: rand_range(2.0, 5.0),
        spiral_tightness: rand_range(1.5, 4.0),
        accent_hue_offset: rand_range(0.2, 0.45),
        color_variation: rand_range(0.1, 0.35),
        glow_intensity: rand_range(0.85, 1.15),
    };

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: generate_magic_sphere_vertices(RADIUS, SEGMENTS, RINGS, &config),
        indices: generate_sphere_indices(SEGMENTS, RINGS),
        material,
    }])];

    Arc::new(Model::flat(parts))
}

/// Spawns a beach ball entity with bouncy physics and a randomised colour pattern.
pub fn spawn_beach_ball(
    world: &mut World,
    initial_pos: nalgebra::Point3<f32>,
    material: MaterialId,
) -> Entity {
    let model = build_model(material);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();

        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.001)
            .angular_damping(0.002);

        let body_handle = physics.0.create_body(body_desc);

        let collider_desc = ColliderDesc::sphere(RADIUS)
            .density(1.0)
            .restitution(0.6)
            .friction(0.5);

        physics.0.attach_collider(body_handle, collider_desc);

        body_handle
    };

    world
        .create_entity()
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Orientation::default())
        .with(RigidBodyComponent(body_handle))
        .with(ModelInstance::new(model))
        .with(Renderable)
        .build()
}
