//! Dolos spawnable — concrete breakwater armour unit.
//!
//! A dolos has a central shank with a fluke at each end, the two flukes
//! perpendicular to each other. The resulting shape interlocks with its
//! neighbours on a breakwater and is notoriously awkward to stack flat —
//! which makes it a fun obstacle.
//!
//! Laid down with the shank level, it would stand on the tip of its upright
//! fluke with everything else in the air, and topple. It is spawned already
//! tipped onto that fluke's edge and the far end of the level one, which is
//! where it comes to rest.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::compound_cuboid_model;
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::physical_finish::PhysicalSurface;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct DolosDef {
    /// Ground point under the dolos: its lowest point rests here.
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees. A rotated segment adds its own yaw.
    #[serde(default)]
    pub yaw: f32,
    /// End-to-end length of the central shank.
    #[serde(default = "DolosDef::default_shank_length")]
    pub shank_length: f32,
    /// End-to-end length of each perpendicular fluke.
    #[serde(default = "DolosDef::default_fluke_length")]
    pub fluke_length: f32,
    /// Side length of the square cross-section of all three bars.
    #[serde(default = "DolosDef::default_thickness")]
    pub thickness: f32,
    /// Concrete density by default.
    #[serde(default = "DolosDef::default_density")]
    pub density: f32,
    #[serde(default = "DolosDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "DolosDef::default_friction")]
    pub friction: f32,
}

impl DolosDef {
    pub fn default_shank_length() -> f32 {
        1.4
    }
    pub fn default_fluke_length() -> f32 {
        1.0
    }
    pub fn default_thickness() -> f32 {
        0.42
    }
    /// Concrete.
    pub fn default_density() -> f32 {
        2400.0
    }
    pub fn default_restitution() -> f32 {
        0.1
    }
    pub fn default_friction() -> f32 {
        0.8
    }

    /// The one declaration of what this object is made of. The collider takes
    /// the coefficients and the material takes the finish and grain, so the two
    /// cannot drift apart.
    ///
    /// Cast concrete, but with every coefficient left authored: a dolos exists
    /// to be tuned, and how it grips and bounces is the point of placing one.
    /// Only its appearance comes from the library.
    /// The tip about the shank's cross axis that brings it to rest: the
    /// upright fluke's inner bottom edge and the far bottom edge of the shank
    /// and level fluke both on the ground.
    fn resting_tilt(&self) -> UnitQuaternion<f32> {
        let (half_shank, half_fluke, half_thk) = (
            self.shank_length * 0.5,
            self.fluke_length * 0.5,
            self.thickness * 0.5,
        );
        // Both edges in the shank's (y, z) plane.
        let upright_edge = (-half_fluke, -(half_shank - 2.0 * half_thk));
        let level_edge = (-half_thk, half_shank);
        let tilt = (upright_edge.0 - level_edge.0).atan2(upright_edge.1 - level_edge.1);
        // atan2 of two negatives lands in the third quadrant; the tip is the
        // acute angle, and none at all if the upright fluke does not reach
        // below the shank.
        let tilt = (tilt + std::f32::consts::PI).max(0.0);
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tilt)
    }

    fn substance(&self) -> Substance {
        substance::CONCRETE.with_physics(PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        })
    }
}

impl Spawnable for DolosDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ctx.patterned(
            &self.substance(),
            &pattern::CONCRETE,
            seed_from_position(self.pos, 0),
            TEXTURE_SIZE,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];

        let half_shank = self.shank_length * 0.5;
        let half_fluke = self.fluke_length * 0.5;
        let half_thk = self.thickness * 0.5;

        // Shank runs along Z. Flukes sit flush at each end, offset inward by
        // half the thickness so the bar ends meet cleanly.
        let shank_he = Vector3::new(half_thk, half_thk, half_shank);
        let shank_offset = Vector3::zeros();

        let fluke_a_he = Vector3::new(half_fluke, half_thk, half_thk);
        let fluke_a_offset = Vector3::new(0.0, 0.0, half_shank - half_thk);

        // Second fluke along Y — perpendicular to the first.
        let fluke_b_he = Vector3::new(half_thk, half_fluke, half_thk);
        let fluke_b_offset = Vector3::new(0.0, 0.0, -(half_shank - half_thk));

        let boxes = [
            (shank_he, shank_offset),
            (fluke_a_he, fluke_a_offset),
            (fluke_b_he, fluke_b_offset),
        ];
        let model = compound_cuboid_model(&boxes, material);

        let rotation = Yaw::degrees(self.yaw).rotation() * self.resting_tilt();
        let lowest = boxes
            .iter()
            .flat_map(|(he, offset)| corners(he).map(move |c| rotation * (offset + c)))
            .map(|c| c.y)
            .fold(f32::INFINITY, f32::min);
        let initial_pos = Point3::new(self.pos.0, self.pos.1 - lowest, self.pos.2);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .rotation(rotation)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.02);

            let body_handle = physics.world.create_body(body_desc);

            for &(half_extents, offset) in &boxes {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(half_extents)
                        .offset_translation(offset)
                        .of(&self.substance()),
                );
            }

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(Vector3::new(
                initial_pos.x,
                initial_pos.y,
                initial_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

/// The eight corners of a box with these half-extents, about its centre.
fn corners(he: &Vector3<f32>) -> impl Iterator<Item = Vector3<f32>> + '_ {
    (0..8).map(move |i| {
        Vector3::new(
            if i & 1 == 0 { -he.x } else { he.x },
            if i & 2 == 0 { -he.y } else { he.y },
            if i & 4 == 0 { -he.z } else { he.z },
        )
    })
}
