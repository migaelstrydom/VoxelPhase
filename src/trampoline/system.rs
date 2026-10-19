//! The bed's drawn shape, kept up with its body.

use nalgebra::Vector3;
use specs::{Component, Join, ReadExpect, ReadStorage, System, VecStorage, WriteStorage};

use super::bed_mesh::{bed_mesh, springs_mesh};
use super::shape::TrampolineShape;
use crate::components::ProceduralMesh;
use crate::model::MeshPrimitive;
use crate::physics::RigidBodyHandle;
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// A trampoline, on its frame's entity: the two bodies, the shape they were
/// laid out from, and what its bed and springs are drawn in.
#[derive(Component)]
#[storage(VecStorage)]
pub struct Trampoline {
    /// The frame: hoop and legs, the body the bed is sprung from.
    pub frame: RigidBodyHandle,
    /// The bed.
    pub bed: RigidBodyHandle,
    /// The dimensions both bodies and every drawn part were laid out from.
    pub shape: TrampolineShape,
    /// Added to a point in the trampoline's ground frame to put it in the
    /// frame body's own frame, whose origin is its centre of mass.
    pub ground_to_body: Vector3<f32>,
    /// What the bed is drawn in.
    pub bed_material: MaterialId,
    /// What the springs are drawn in.
    pub spring_material: MaterialId,
}

impl Trampoline {
    /// How far the bed's collider has sunk below its rest, in metres, in the
    /// frame's own up: negative when it has sprung above it. `None` if
    /// either body is gone.
    pub fn sag(&self, physics: &PhysicsResource) -> Option<f32> {
        let frame = physics.world.body(self.frame)?;
        let bed = physics.world.body(self.bed)?;
        let local = frame.rotation().inverse() * (bed.position() - frame.position());
        let rest = self.shape.bed_rest_centre().coords + self.ground_to_body;
        Some(rest.y - local.y)
    }

    /// The bed and its springs as drawn with the bed's collider `sag` below
    /// its rest.
    pub fn bed_primitives(&self, sag: f32) -> Vec<MeshPrimitive> {
        let (vertices, indices) = bed_mesh(&self.shape, sag, self.ground_to_body);
        let (spring_vertices, spring_indices) = springs_mesh(&self.shape, sag, self.ground_to_body);
        vec![
            MeshPrimitive {
                vertices,
                indices,
                material: self.bed_material,
            },
            MeshPrimitive {
                vertices: spring_vertices,
                indices: spring_indices,
                material: self.spring_material,
            },
        ]
    }
}

/// Rebuilds each trampoline's bed and springs where its bed is this frame.
pub struct TrampolineBedSystem;

impl<'a> System<'a> for TrampolineBedSystem {
    type SystemData = (
        ReadExpect<'a, PhysicsResource>,
        ReadStorage<'a, Trampoline>,
        WriteStorage<'a, ProceduralMesh>,
    );

    fn run(&mut self, (physics, trampolines, mut meshes): Self::SystemData) {
        for (trampoline, mesh) in (&trampolines, &mut meshes).join() {
            let Some(sag) = trampoline.sag(&physics) else {
                continue;
            };
            mesh.primitives = trampoline.bed_primitives(sag);
        }
    }
}
