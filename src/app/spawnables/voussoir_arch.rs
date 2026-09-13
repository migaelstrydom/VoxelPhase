//! Voussoir arch spawnable — semicircular masonry arch that locks under gravity.
//!
//! Creates a classic Roman-style arch from wedge-shaped stone blocks (voussoirs)
//! arranged in a semicircle, supported by two heavy abutment pillars. The arch
//! holds together purely through compressive forces and friction.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, cuboid_model, SolidFace};
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 256;

/// Base seed for the arch's stone. Dressed stone from one quarry should look
/// like dressed stone from one quarry.
const LIMESTONE_SEED: u32 = 77;

/// How many distinct blocks the arch is cut from.
///
/// One seed for every piece would be fourteen baked textures; one seed for all
/// of them is the same photograph fourteen times, which is what an arch of
/// identical stones looks like. A few variants cycled around the ring buys the
/// difference for the cost of a few tiles.
const STONE_VARIANTS: u32 = 3;

#[derive(Deserialize)]
pub struct VoussoirArchDef {
    /// World position of the ground-level center of the arch opening.
    pub base: (f32, f32, f32),
    /// Inner radius of the arch (half-width of the opening).
    #[serde(default = "VoussoirArchDef::default_inner_radius")]
    pub inner_radius: f32,
    /// Radial thickness of each voussoir block.
    #[serde(default = "VoussoirArchDef::default_thickness")]
    pub thickness: f32,
    /// Depth of the arch along the Z axis.
    #[serde(default = "VoussoirArchDef::default_depth")]
    pub depth: f32,
    /// Number of voussoir blocks (odd gives a centered keystone).
    #[serde(default = "VoussoirArchDef::default_num_voussoirs")]
    pub num_voussoirs: u32,
    /// Height of the abutment pillars supporting the arch.
    #[serde(default = "VoussoirArchDef::default_abutment_height")]
    pub abutment_height: f32,
    #[serde(default = "VoussoirArchDef::default_density")]
    pub density: f32,
    #[serde(default = "VoussoirArchDef::default_friction")]
    pub friction: f32,
}

impl VoussoirArchDef {
    pub fn default_inner_radius() -> f32 {
        2.0
    }
    pub fn default_thickness() -> f32 {
        0.5
    }
    pub fn default_depth() -> f32 {
        1.2
    }
    pub fn default_num_voussoirs() -> u32 {
        11
    }
    pub fn default_abutment_height() -> f32 {
        1.5
    }
    pub fn default_density() -> f32 {
        2000.0
    }
    pub fn default_friction() -> f32 {
        0.8
    }

    fn total_pieces(&self) -> usize {
        self.num_voussoirs as usize + 2
    }

    /// The voussoirs: dressed limestone, with the friction and density a level
    /// authors. Declared once so the collider and the stone they are rendered
    /// with cannot disagree about what they are.
    fn voussoir_substance(&self) -> Substance {
        substance::LIMESTONE
            .with_density(self.density)
            .with_friction(self.friction)
    }

    /// The abutment pillars: the same stone at twice the density, so they stay
    /// put under the arch's thrust. Twice as heavy and identical to look at,
    /// which is the separation the substance library exists to allow.
    fn abutment_substance(&self) -> Substance {
        self.voussoir_substance().with_density(self.density * 2.0)
    }
}

impl Spawnable for VoussoirArchDef {
    fn material_count(&self) -> usize {
        self.total_pieces()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let count = self.total_pieces();
        let mut mats = Vec::with_capacity(count);

        // One material per piece, drawing on `STONE_VARIANTS` textures: the
        // voussoirs and the abutments are the same weathered limestone, so
        // the cache hands the same tile to every piece that shares a variant.
        // The pieces differ in density, which the cache key rightly ignores —
        // a heavier stone is not a different-looking one.
        for i in 0..self.num_voussoirs as usize {
            mats.push(ctx.patterned(
                &self.voussoir_substance(),
                &pattern::WEATHERED_STONE,
                stone_seed(i as u32),
                TEXTURE_SIZE,
            )?);
        }
        for i in 0..2 {
            mats.push(ctx.patterned(
                &self.abutment_substance(),
                &pattern::WEATHERED_STONE,
                stone_seed(self.num_voussoirs + i),
                TEXTURE_SIZE,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let n = self.num_voussoirs;
        let inner_r = self.inner_radius;
        let outer_r = inner_r + self.thickness;
        let half_depth = self.depth / 2.0;
        let angle_step = std::f32::consts::PI / n as f32;

        // Arch center of curvature sits at the top of the abutment pillars.
        let center = Vector3::new(self.base.0, self.base.1 + self.abutment_height, self.base.2);

        let mut entities = Vec::with_capacity(self.total_pieces());

        // --- Voussoirs ---
        for i in 0..n {
            let angle_start = i as f32 * angle_step;
            let angle_end = (i + 1) as f32 * angle_step;

            let (arch_verts, faces) =
                voussoir_geometry(inner_r, outer_r, half_depth, angle_start, angle_end);

            let centroid =
                arch_verts.iter().copied().sum::<Vector3<f32>>() / arch_verts.len() as f32;
            let local_verts: Vec<_> = arch_verts.iter().map(|v| v - centroid).collect();

            let pos = Point3::new(
                center.x + centroid.x,
                center.y + centroid.y,
                center.z + centroid.z,
            );

            let hull = Arc::new(build_convex_hull(&local_verts, &faces));
            let model = convex_solid_model(&local_verts, &faces, materials[i as usize]);

            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::convex_hull(hull).of(&self.voussoir_substance()),
                );
                body_handle
            };

            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation::default())
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        }

        // --- Abutment pillars ---
        let abutment_he =
            Vector3::new(self.thickness / 2.0, self.abutment_height / 2.0, half_depth);

        for (i, side) in [1.0f32, -1.0].iter().enumerate() {
            let x = center.x + side * (inner_r + self.thickness / 2.0);
            let y = self.base.1 + self.abutment_height / 2.0;
            let z = center.z;
            let pos = Point3::new(x, y, z);
            let mat_idx = n as usize + i;

            let model = cuboid_model(abutment_he, materials[mat_idx]);

            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(abutment_he).of(&self.abutment_substance()),
                );
                body_handle
            };

            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation::default())
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        }

        entities
    }
}

/// The stone a given piece was cut from, cycling around the ring.
fn stone_seed(piece: u32) -> u32 {
    LIMESTONE_SEED.wrapping_add(piece % STONE_VARIANTS)
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Vertices and faces of a single voussoir (truncated wedge).
///
/// Computed in arch-local space where the arch center of curvature is at the
/// origin. The voussoir spans from `angle_start` to `angle_end` (radians,
/// measured counter-clockwise from the +X axis in the XY plane).
///
/// Returns 8 vertices and 6 quadrilateral faces.
fn voussoir_geometry(
    inner_radius: f32,
    outer_radius: f32,
    half_depth: f32,
    angle_start: f32,
    angle_end: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let cos_s = angle_start.cos();
    let sin_s = angle_start.sin();
    let cos_e = angle_end.cos();
    let sin_e = angle_end.sin();

    let vertices = vec![
        // Front face (z = +half_depth)
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, half_depth), // 0: inner-start
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, half_depth), // 1: outer-start
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, half_depth), // 2: outer-end
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, half_depth), // 3: inner-end
        // Back face (z = -half_depth)
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, -half_depth), // 4: inner-start
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, -half_depth), // 5: outer-start
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, -half_depth), // 6: outer-end
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, -half_depth), // 7: inner-end
    ];

    let faces = vec![
        // Front face (+Z). Opposite: any back vertex.
        SolidFace {
            vertex_indices: vec![0, 1, 2, 3],
            opposite_vertex: 4,
        },
        // Back face (-Z). Opposite: any front vertex.
        SolidFace {
            vertex_indices: vec![7, 6, 5, 4],
            opposite_vertex: 0,
        },
        // Start radial face. Opposite: an end-side vertex.
        SolidFace {
            vertex_indices: vec![0, 4, 5, 1],
            opposite_vertex: 3,
        },
        // End radial face. Opposite: a start-side vertex.
        SolidFace {
            vertex_indices: vec![3, 2, 6, 7],
            opposite_vertex: 0,
        },
        // Outer face. Opposite: an inner vertex.
        SolidFace {
            vertex_indices: vec![1, 5, 6, 2],
            opposite_vertex: 0,
        },
        // Inner face. Opposite: an outer vertex.
        SolidFace {
            vertex_indices: vec![0, 3, 7, 4],
            opposite_vertex: 1,
        },
    ];

    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Limestone texture generation
// ---------------------------------------------------------------------------
