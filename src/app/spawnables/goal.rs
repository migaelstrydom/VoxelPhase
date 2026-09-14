//! Goal spawnable — the beacon that ends a level.
//!
//! Two entities, because the two halves answer to different things. The plinth
//! is static geometry the player can stand on; the beacon above it is a pure
//! visual carrying the [`Goal`] component, and is what brightens when the level
//! is finished.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{compound_cuboid_model, cuboid_model};
use super::{MaterialCtx, Spawnable};
use crate::components::{MaterialModulation, ModelInstance, Position, Renderable};
use crate::core::error::EngineResult;
use crate::lighting::PointLight;
use crate::objective::Goal;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{
    Emission, Material, MaterialId, SurfaceFinish, SurfaceModulation,
};
use crate::systems::PhysicsResource;

/// Half-height of the plinth. Low enough to walk onto without a jump.
const PLINTH_HALF_HEIGHT: f32 = 0.15;

/// Plinth footprint as a fraction of the goal radius: the disc is the part you
/// stand on, the radius is the part that counts as arriving.
const PLINTH_FOOTPRINT: f32 = 0.55;

/// Half-thickness of the central column.
const COLUMN_HALF_WIDTH: f32 = 0.12;

/// Half-height of the central column. Tall enough to clear most props and be
/// picked out across a level.
const COLUMN_HALF_HEIGHT: f32 = 3.0;

/// Posts standing around the plinth rim, with the column at the centre.
const RIM_POSTS: usize = 6;
const POST_HALF_WIDTH: f32 = 0.08;
const POST_HALF_HEIGHT: f32 = 0.45;

/// Beacon colour: a cool green that no material in the library competes with.
const BEACON_COLOUR: Colour = Colour {
    r: 0.35,
    g: 1.0,
    b: 0.55,
    a: 1.0,
};

/// Plinth colour: plain pale stone, so the beacon is the only bright thing.
const PLINTH_COLOUR: Colour = Colour {
    r: 0.62,
    g: 0.62,
    b: 0.66,
    a: 1.0,
};

/// Emitted luminance of the beacon, above the bloom threshold.
const GLOW: f32 = 2.4;
const RIM_STRENGTH: f32 = 0.4;
const RIM_POWER: f32 = 2.5;

/// The beacon lights the ground it stands on, so it reads at night and from
/// inside a cavern.
const LIGHT_INTENSITY: f32 = 1.0;
const LIGHT_RANGE: f32 = 10.0;

const PLINTH_DENSITY: f32 = 2400.0;
const PLINTH_FRICTION: f32 = 0.8;

#[derive(Deserialize)]
pub struct GoalDef {
    /// Ground point at the foot of the beacon.
    pub pos: (f32, f32, f32),

    /// Horizontal distance that counts as arriving.
    #[serde(default = "GoalDef::default_radius")]
    pub radius: f32,

    /// Gems that must be caught before the goal opens.
    #[serde(default)]
    pub required_gems: u32,
}

impl GoalDef {
    pub fn default_radius() -> f32 {
        2.0
    }

    fn plinth_half_extents(&self) -> Vector3<f32> {
        let half = self.radius * PLINTH_FOOTPRINT;
        Vector3::new(half, PLINTH_HALF_HEIGHT, half)
    }
}

impl Spawnable for GoalDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let plinth_texture = ctx.textures.create_solid_colour(PLINTH_COLOUR)?;
        let plinth = Material::textured(plinth_texture).with_finish(SurfaceFinish::MATTE);

        let beacon_texture = ctx.textures.create_solid_colour(Colour::WHITE)?;
        let beacon = Material::textured(beacon_texture)
            .with_finish(SurfaceFinish::POLISHED)
            .with_emission(Emission {
                colour: BEACON_COLOUR,
                strength: GLOW,
                rim_strength: RIM_STRENGTH,
                rim_power: RIM_POWER,
            });

        Ok(vec![
            ctx.materials.register(plinth),
            ctx.materials.register(beacon),
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ground = Vector3::new(self.pos.0, self.pos.1, self.pos.2);
        let half_extents = self.plinth_half_extents();

        let plinth_centre = ground + Vector3::new(0.0, PLINTH_HALF_HEIGHT, 0.0);
        let plinth_model = cuboid_model(half_extents, materials[0]);

        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics
                .world
                .create_body(RigidBodyDesc::static_body().position(Point3::from(plinth_centre)));
            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(half_extents)
                    .density(PLINTH_DENSITY)
                    .restitution(0.0)
                    .friction(PLINTH_FRICTION),
            );
        }

        let plinth_entity = world
            .create_entity()
            .with(Position(plinth_centre))
            .with(ModelInstance::new(plinth_model))
            .with(Renderable)
            .build();

        // The beacon's own origin is the ground point, so the `Goal` radius is
        // measured from where the player's feet have to be.
        let beacon_model = compound_cuboid_model(&beacon_boxes(half_extents.x), materials[1]);

        let beacon_entity = world
            .create_entity()
            .with(Position(ground))
            .with(ModelInstance::new(beacon_model))
            .with(Renderable)
            .with(MaterialModulation(SurfaceModulation::IDENTITY))
            .with(Goal {
                radius: self.radius,
                required_gems: self.required_gems,
            })
            .with(PointLight::new(BEACON_COLOUR, LIGHT_INTENSITY, LIGHT_RANGE))
            .build();

        vec![plinth_entity, beacon_entity]
    }
}

/// The column and its ring of posts, as half-extents and offsets from the
/// beacon's ground origin.
fn beacon_boxes(plinth_half: f32) -> Vec<(Vector3<f32>, Vector3<f32>)> {
    let deck = PLINTH_HALF_HEIGHT * 2.0;

    let mut boxes = vec![(
        Vector3::new(COLUMN_HALF_WIDTH, COLUMN_HALF_HEIGHT, COLUMN_HALF_WIDTH),
        Vector3::new(0.0, deck + COLUMN_HALF_HEIGHT, 0.0),
    )];

    let ring = (plinth_half - POST_HALF_WIDTH).max(POST_HALF_WIDTH);
    for i in 0..RIM_POSTS {
        let angle = TAU * i as f32 / RIM_POSTS as f32;
        boxes.push((
            Vector3::new(POST_HALF_WIDTH, POST_HALF_HEIGHT, POST_HALF_WIDTH),
            Vector3::new(
                ring * angle.cos(),
                deck + POST_HALF_HEIGHT,
                ring * angle.sin(),
            ),
        ));
    }

    boxes
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every piece of the beacon must stand on top of the plinth, not inside
    /// it: a post sunk into the deck is invisible from the ground.
    #[test]
    fn the_beacon_sits_above_the_plinth_deck() {
        let deck = PLINTH_HALF_HEIGHT * 2.0;
        for (half, offset) in beacon_boxes(1.1) {
            assert!(
                offset.y - half.y >= deck - 1e-6,
                "a beacon box reaches down to {}",
                offset.y - half.y
            );
        }
    }

    #[test]
    fn the_plinth_is_smaller_than_the_goal_radius() {
        let def = GoalDef {
            pos: (0.0, 0.0, 0.0),
            radius: GoalDef::default_radius(),
            required_gems: 0,
        };
        assert!(def.plinth_half_extents().x < def.radius);
    }
}
