//! A cube of one solid metal, from the [`Metal`] list.
//!
//! The densities are the real ones, and they span forty-fold. A knee-high cube
//! of osmium or tungsten weighs over half a tonne; the player can lean on one
//! and it will not notice, and a blast that throws a crate across the level
//! only nudges it. The same cube of lithium weighs fourteen kilograms and
//! floats.
//!
//! Bevelled for the same reason ice is: a polished face shows the sun from one
//! direction only, and the narrow facets along the edges are what catch it
//! from every other. Its `+Z` face is stamped with the element it is made of.
//!
//! ```text
//!   MetalCubeDef ──▶ Metal ──┬──▶ substance ──┬──▶ ColliderDesc (box)
//!                            │                ├──▶ body material ──┐
//!                            └──▶ stamp ──────┴──▶ face material ──┤
//!                    bevelled_box ──▶ body + stamped face ─────────┴──▶ Model
//! ```

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::super::shared::bevelled_box::bevelled_box;
use super::super::shared::orientation::Yaw;
use super::super::shared::textures::seed_from_position;
use super::super::{MaterialCtx, Spawnable};
use super::metal::Metal;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::ColliderSubstance;
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 128;

/// The stamped face's texture is finer than the body's: the smallest line of
/// the stamp has to hold its shape a couple of metres away.
const STAMP_TEXTURE_SIZE: u32 = 512;

/// The face the stamp is on, in the cube's own frame.
const STAMPED_FACE: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

/// How far each edge is cut back, as a fraction of the half-edge. Finer than
/// ice's: machined metal has a small chamfer, and a wide one reads as a die.
const BEVEL_FRACTION: f32 = 0.08;

#[derive(Deserialize)]
pub struct MetalCubeDef {
    pub pos: (f32, f32, f32),
    #[serde(default)]
    pub metal: Metal,
    /// Edge length, in metres.
    #[serde(default = "MetalCubeDef::default_size")]
    pub size: f32,
    /// Rotation about `+Y`, in degrees.
    #[serde(default)]
    pub yaw: f32,
}

impl MetalCubeDef {
    /// Knee height.
    pub fn default_size() -> f32 {
        0.3
    }

    fn half_extents(&self) -> Vector3<f32> {
        Vector3::repeat(self.size * 0.5)
    }
}

impl Spawnable for MetalCubeDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let substance = self.metal.substance();
        let seed = seed_from_position(self.pos, 0);
        Ok(vec![
            ctx.patterned(&substance, &pattern::POLISHED_METAL, seed, TEXTURE_SIZE)?,
            ctx.engraved(
                &substance,
                &pattern::POLISHED_METAL,
                seed,
                STAMP_TEXTURE_SIZE,
                &self.metal.stamp(),
            )?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let centre = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let rotation = Yaw::degrees(self.yaw).rotation();
        let half_extents = self.half_extents();
        let model = metal_cube_model(half_extents, materials[0], materials[1]);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(centre)
                    .rotation(rotation)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );

            // The full box, as ice does: the faces sit on it exactly and only
            // the chamfers fall a hair inside.
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(half_extents).of(&self.metal.substance()),
            );

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

/// The cube as drawn: bevelled, with one tile of texture across each face,
/// and the stamped face as a primitive of its own.
fn metal_cube_model(half_extents: Vector3<f32>, body: MaterialId, stamp: MaterialId) -> Arc<Model> {
    let (body_mesh, stamp_mesh) = metal_cube_meshes(half_extents);
    let primitive = |(vertices, indices), material| MeshPrimitive {
        vertices,
        indices,
        material,
    };
    Arc::new(Model::flat(vec![ModelPart::new(vec![
        primitive(body_mesh, body),
        primitive(stamp_mesh, stamp),
    ])]))
}

type Mesh = (Vec<Vertex>, Vec<u32>);

/// The body of the cube, and its stamped face with the texture laid across
/// exactly `0..1` from edge to edge of the flat, `v` running down.
fn metal_cube_meshes(half_extents: Vector3<f32>) -> (Mesh, Mesh) {
    let bevel = half_extents.min() * BEVEL_FRACTION;
    let uv_scale = 0.5 / half_extents.max();
    let (vertices, indices) = bevelled_box(half_extents, bevel, uv_scale);
    let inner = half_extents - Vector3::repeat(bevel);

    let mut body = MeshBuilder::default();
    let mut face = MeshBuilder::default();
    for triangle in indices.chunks_exact(3) {
        let corners: [Vertex; 3] = std::array::from_fn(|i| vertices[triangle[i] as usize]);
        if (corners[0].normal - STAMPED_FACE).norm() < 1e-4 {
            face.triangle(corners.map(|mut vertex| {
                vertex.tex_coords.x = (vertex.pos.x + inner.x) / (2.0 * inner.x);
                vertex.tex_coords.y = (inner.y - vertex.pos.y) / (2.0 * inner.y);
                vertex
            }));
        } else {
            body.triangle(corners);
        }
    }
    (body.finish(), face.finish())
}

/// Triangles of flat-shaded vertices, each with its own copies.
#[derive(Default)]
struct MeshBuilder {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

impl MeshBuilder {
    fn triangle(&mut self, corners: [Vertex; 3]) {
        let base = self.vertices.len() as u32;
        self.vertices.extend(corners);
        self.indices.extend([base, base + 1, base + 2]);
    }

    fn finish(self) -> Mesh {
        (self.vertices, self.indices)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The stamp gets one face, the flat of `+Z`, and all of it: two
    /// triangles whose texture runs corner to corner.
    #[test]
    fn the_stamp_covers_one_flat_face_exactly() {
        let (body, (vertices, indices)) = metal_cube_meshes(Vector3::repeat(0.15));
        assert_eq!(indices.len(), 6);
        assert_eq!(body.1.len() / 3, 6 * 2 + 12 * 2 + 8 - 2);
        for vertex in &vertices {
            assert_eq!(vertex.normal, STAMPED_FACE);
            for uv in [vertex.tex_coords.x, vertex.tex_coords.y] {
                assert!(uv.abs() < 1e-5 || (uv - 1.0).abs() < 1e-5, "uv {uv}");
            }
        }
    }

    /// Seen from outside the face, `u` runs right and `v` runs down, or the
    /// stamp reads mirrored.
    #[test]
    fn the_stamp_reads_the_right_way_round() {
        let (_, (vertices, _)) = metal_cube_meshes(Vector3::repeat(0.15));
        for vertex in &vertices {
            // Looking down -Z at the +Z face, +X is to the right and +Y up.
            assert_eq!(vertex.tex_coords.x > 0.5, vertex.pos.x > 0.0);
            assert_eq!(vertex.tex_coords.y > 0.5, vertex.pos.y < 0.0);
        }
    }
}
