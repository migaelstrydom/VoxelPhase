//! Table spawnable — compound body with top slab + 4 legs.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::compound_cuboid_model;
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::shared::textures::TextureRng;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{CompoundFracture, FractureJoint};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct TableDef {
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees. A rotated segment adds its own yaw.
    #[serde(default)]
    pub yaw: f32,
    #[serde(default = "TableDef::default_top_half_extents")]
    pub top_half_extents: (f32, f32, f32),
    #[serde(default = "TableDef::default_leg_half_extents")]
    pub leg_half_extents: (f32, f32, f32),
    #[serde(default = "TableDef::default_density")]
    pub density: f32,
    #[serde(default = "TableDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "TableDef::default_friction")]
    pub friction: f32,
}

impl TableDef {
    pub fn default_top_half_extents() -> (f32, f32, f32) {
        (1.0, 0.1, 0.6)
    }
    pub fn default_leg_half_extents() -> (f32, f32, f32) {
        (0.12, 0.35, 0.12)
    }
    pub fn default_density() -> f32 {
        600.0
    }
    pub fn default_restitution() -> f32 {
        0.1
    }
    pub fn default_friction() -> f32 {
        0.5
    }

    /// The one declaration of this object's physics. The collider takes the
    /// coefficients and the material takes the finish they imply, so the two
    /// cannot drift apart.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        }
    }
}

impl TableDef {
    fn top_he(&self) -> Vector3<f32> {
        Vector3::new(
            self.top_half_extents.0,
            self.top_half_extents.1,
            self.top_half_extents.2,
        )
    }

    fn leg_he(&self) -> Vector3<f32> {
        Vector3::new(
            self.leg_half_extents.0,
            self.leg_half_extents.1,
            self.leg_half_extents.2,
        )
    }

    fn total_height(&self) -> f32 {
        self.leg_he().y * 2.0 + self.top_he().y * 2.0
    }
}

impl Spawnable for TableDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_table_wood(seed_from_position(self.pos, 0));
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture).with_derived_finish(self.surface());
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let top_he = self.top_he();
        let leg_he = self.leg_he();
        let table_height = self.total_height();

        let top_y = table_height * 0.5 - top_he.y;
        let leg_y = -top_he.y;
        let leg_x = top_he.x - leg_he.x;
        let leg_z = top_he.z - leg_he.z;

        let leg_positions = [
            Vector3::new(-leg_x, leg_y, -leg_z),
            Vector3::new(leg_x, leg_y, -leg_z),
            Vector3::new(-leg_x, leg_y, leg_z),
            Vector3::new(leg_x, leg_y, leg_z),
        ];

        // Build compound model
        let mut boxes = vec![(top_he, Vector3::new(0.0, top_y, 0.0))];
        for &pos in &leg_positions {
            boxes.push((leg_he, pos));
        }
        let model = compound_cuboid_model(&boxes, materials[0]);

        // Create compound physics body
        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .rotation(Yaw::degrees(self.yaw).rotation())
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(top_he)
                    .offset_translation(Vector3::new(0.0, top_y, 0.0))
                    .with_physical_surface(self.surface()),
            );

            for &pos in &leg_positions {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(leg_he)
                        .offset_translation(pos)
                        .with_physical_surface(self.surface()),
                );
            }

            body_handle
        };

        // 5 children: [0] top slab, [1..4] legs.
        // 4 joints: each leg connects to the top.
        let joint_threshold = 10.0;
        let fracture = CompoundFracture {
            joints: vec![
                FractureJoint {
                    child_a: 0,
                    child_b: 1,
                    threshold: joint_threshold,
                },
                FractureJoint {
                    child_a: 0,
                    child_b: 2,
                    threshold: joint_threshold,
                },
                FractureJoint {
                    child_a: 0,
                    child_b: 3,
                    threshold: joint_threshold,
                },
                FractureJoint {
                    child_a: 0,
                    child_b: 4,
                    threshold: joint_threshold,
                },
            ],
            child_count: 5,
            material: materials[0],
        };

        vec![world
            .create_entity()
            .with(Position(Vector3::new(
                initial_pos.x,
                initial_pos.y,
                initial_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(Yaw::degrees(self.yaw).rotation()))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(fracture)
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Procedural polished wood texture
// ---------------------------------------------------------------------------

fn generate_table_wood(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let tone = rng.pick(3);
    let base = match tone {
        0 => Rgb::new(0.55, 0.38, 0.20),
        1 => Rgb::new(0.35, 0.22, 0.12),
        _ => Rgb::new(0.68, 0.52, 0.28),
    };

    let hue_offset = rng.range(-0.04, 0.04);
    let seed_grain = rng.u32();
    let seed_rings = rng.u32();
    let seed_knots = rng.u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let grain = fbm_2d_periodic(u * 6.0, v * 28.0, 4, 0.5, 2.0, seed_grain, Some(6));
            let variation =
                fbm_2d_periodic(u * 3.0, v * 3.0, 3, 0.45, 2.0, seed_grain + 1, Some(3));

            let ring_distort = fbm_2d_periodic(u * 4.0, v * 4.0, 2, 0.4, 2.0, seed_rings, Some(4));
            let ring_u = u - 0.5 + ring_distort * 0.15;
            let ring_v = (v - 0.5) * 3.0;
            let ring_dist = (ring_u * ring_u + ring_v * ring_v).sqrt();
            let ring = ((ring_dist * 30.0).sin() * 0.5 + 0.5).powf(6.0) * 0.08;

            let knot_noise = fbm_2d_periodic(u * 2.0, v * 2.0, 3, 0.6, 2.0, seed_knots, Some(2));
            let knot = if knot_noise > 0.72 {
                (knot_noise - 0.72) / 0.28 * 0.2
            } else {
                0.0
            };

            let wood_factor = 0.80 + grain * 0.20;
            let hue_shift = (variation - 0.5) * 0.06 + hue_offset;

            let mut c = Rgb::new(
                (base.r + hue_shift) * wood_factor,
                (base.g + hue_shift * 0.5) * wood_factor,
                base.b * wood_factor,
            );

            c = c.scale(1.0 - ring);
            c = c.scale(1.0 - knot);

            let eu = (u - 0.5).abs() * 2.0;
            let ev = (v - 0.5).abs() * 2.0;
            let edge = ((eu.max(ev) - 0.90) / 0.10).clamp(0.0, 1.0);
            c = c.scale(1.0 - edge * 0.15);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
