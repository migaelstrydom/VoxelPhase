//! Shrub spawnable: a ball of leaves on the ground.
//!
//! Rooted, it is drawn and drags whatever passes through it, but has no
//! body: nothing collides with leaves. A blast near it tears it loose
//! (`foliage::FoliageUprootSystem`). Loose, it is a clump on a light core
//! that tumbles, lands dead and stops. Either way its shape and skin grow
//! from a seed taken from where it stands, so a shrub is the same shrub
//! every load.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::seed_from_ground;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::foliage::{Foliage, LeafCore, Rooting, ShrubSpec, SkinSpec};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Leaves are matte, with a faint sheen off the tops of the sprigs.
const LEAF_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};

/// How far a rooted shrub stands sunk into the ground, as a share of its
/// radius: it grows out of the soil, not on top of it.
const ROOTED_SINK: f32 = 0.2;

#[derive(Deserialize)]
pub struct ShrubDef {
    /// Ground point (x, z) under its middle. Y is the terrain's there.
    pub pos: (f32, f32),
    /// Radius of the clump, in metres.
    #[serde(default = "ShrubDef::default_radius")]
    pub radius: f32,
    /// Whether it starts torn loose.
    #[serde(default)]
    pub loose: bool,
    /// Its height overall, in metres: taller than its width a column, lower
    /// a mound. `None` is a round shrub.
    #[serde(default)]
    pub height: Option<f32>,
}

impl ShrubDef {
    pub fn default_radius() -> f32 {
        ShrubSpec::default().radius
    }

    fn seed(&self) -> u32 {
        seed_from_ground(self.pos, 0)
    }
}

impl Spawnable for ShrubDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // The greens ride on the skin's vertices.
        let leaves = Material::coloured(Colour::WHITE)
            .with_finish(LEAF_FINISH)
            .as_foliage();
        Ok(vec![ctx.materials.register(leaves)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ground = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        let Some(ground) = ground else {
            return Vec::new();
        };

        let seed = self.seed();
        let spec = ShrubSpec::with_radius(self.radius);
        let spec = if self.loose { spec } else { spec.rooted() };
        let spec = self.height.map_or(spec, |h| spec.with_height(h));
        let shape = Arc::new(spec.grow(seed));
        let skin = SkinSpec::default().grow(&shape, seed);
        let yaw = (seed % 360) as f32 * std::f32::consts::PI / 180.0;
        let core = LeafCore::default();
        let (pose, sunk) = if self.loose {
            let pose = core.resting_pose(&shape, Point3::new(self.pos.0, ground, self.pos.1), yaw);
            (pose, 0.0)
        } else {
            let sunk = self.radius * ROOTED_SINK;
            let pose = Isometry3::from_parts(
                Translation3::new(self.pos.0, ground + shape.depth_below() - sunk, self.pos.1),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw),
            );
            (pose, sunk)
        };

        let mut clump = Foliage::new(shape, Rooting::Rooted { pose, sunk });
        let body = self.loose.then(|| {
            let mut physics = world.write_resource::<PhysicsResource>();
            clump.core.spawn(&mut physics.world, &clump.shape, pose)
        });
        if let Some(body) = body {
            clump.rooting = Rooting::on(body);
        }

        let mut entity = world
            .create_entity()
            .with(Position(pose.translation.vector))
            .with(Orientation(pose.rotation))
            .with(ModelInstance::new(skin.model(materials[0])))
            .with(clump)
            .with(Renderable);
        if let Some(body) = body {
            entity = entity
                .with(RigidBodyComponent(body))
                .with(Velocity(Vector3::zeros()));
        }
        vec![entity.build()]
    }
}
