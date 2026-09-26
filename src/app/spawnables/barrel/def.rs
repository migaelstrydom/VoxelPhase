//! A sealed wooden barrel: oak staves round a hollow of air.
//!
//! The barrel's hull is its contact envelope; its bulk is declared apart from
//! it, as a shell of oak two centimetres thick. So it weighs what an empty
//! barrel weighs and floats as high as one does, where a hull of solid oak the
//! same size would weigh six times as much and float more than half under.
//!
//! ```text
//!   BarrelProfile ─┬─ hull(STAVES) ────────▶ contact collider (oak coefficients)
//!                  ├─ hull, inset(STAVE) ──▶ BulkShape::hollow ─▶ mass, buoyancy
//!                  └─ side / head meshes ──▶ model (stave + head materials)
//! ```

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::mesh::{head_meshes, side_mesh};
use super::profile::BarrelProfile;
use super::texture::{head_texture, side_texture, HEAD_SIZE, SIDE_HEIGHT, SIDE_WIDTH, STAVES};
use crate::app::spawnables::shared::textures::seed_from_position;
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{BulkShape, ColliderDesc, ColliderShape, RigidBodyDesc, Volume};
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

/// Radius at the belly, the barrel's widest point.
pub const BARREL_RADIUS: f32 = 0.30;

/// A small barrel: 0.9 m head to head, 0.6 m across the belly.
const PROFILE: BarrelProfile = BarrelProfile {
    half_height: 0.45,
    belly_radius: BARREL_RADIUS,
    head_radius: 0.26,
};

/// Thickness of the staves and heads.
const STAVE_THICKNESS: f32 = 0.02;

/// Seasoned oak, kg/m³: lighter than the library's green-timber figure.
const OAK_DENSITY: f32 = 600.0;

/// Rings of the contact hull from head to head. Five keeps the bulge to
/// within a couple of millimetres at 80 vertices.
const HULL_RINGS: usize = 5;
/// Render mesh resolution round the barrel and from head to head.
const MESH_SEGMENTS: u32 = 48;
const MESH_RINGS: u32 = 13;

#[derive(Deserialize)]
pub struct BarrelDef {
    /// Centre of the barrel, standing on a head.
    pub pos: (f32, f32, f32),
}

impl BarrelDef {
    /// Oak's grip, bounce, finish and grain. The density here is what the
    /// collider carries, which the declared bulk overrides.
    fn substance(&self) -> Substance {
        substance::OAK.with_density(OAK_DENSITY)
    }
}

/// The barrel's bulk: an oak shell `STAVE_THICKNESS` thick, sealed round air.
///
/// The inner volume is the same shape inset, so the shell has the same wall
/// everywhere, and its mass and inertia are the outer solid's less the air's.
fn barrel_bulk() -> BulkShape {
    BulkShape::hollow(
        outer_volume(),
        Volume::centred(hull_shape(&PROFILE.inset(STAVE_THICKNESS))),
        OAK_DENSITY,
    )
}

/// Everything the barrel encloses, staves and air together: what it
/// displaces.
fn outer_volume() -> Volume {
    Volume::centred(hull_shape(&PROFILE))
}

/// A barrel profile as a hull shape, one flat face per stave.
fn hull_shape(profile: &BarrelProfile) -> ColliderShape {
    ColliderShape::ConvexHull {
        hull: Arc::new(profile.hull(STAVES, HULL_RINGS)),
    }
}

impl Spawnable for BarrelDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = seed_from_position(self.pos, 0);

        let side =
            ctx.textures
                .create_from_rgba(SIDE_WIDTH, SIDE_HEIGHT, &side_texture(seed), true)?;
        let head =
            ctx.textures
                .create_from_rgba(HEAD_SIZE, HEAD_SIZE, &head_texture(seed), true)?;

        Ok(vec![
            ctx.materials.register(self.substance().material(side)),
            ctx.materials.register(self.substance().material(head)),
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let position = Point3::new(self.pos.0, self.pos.1, self.pos.2);

        let (side_vertices, side_indices) = side_mesh(&PROFILE, MESH_SEGMENTS, MESH_RINGS);
        let (head_vertices, head_indices) = head_meshes(&PROFILE, MESH_SEGMENTS);
        let model = Arc::new(Model::flat(vec![ModelPart::new(vec![
            MeshPrimitive {
                vertices: side_vertices,
                indices: side_indices,
                material: materials[0],
            },
            MeshPrimitive {
                vertices: head_vertices,
                indices: head_indices,
                material: materials[1],
            },
        ])]));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(position)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.01)
                    .bulk(barrel_bulk()),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::convex_hull(Arc::new(PROFILE.hull(STAVES, HULL_RINGS)))
                    .of(&self.substance()),
            );
            body
        };

        vec![world
            .create_entity()
            .with(Position(position.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use nalgebra::UnitQuaternion;

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{PhysicsConfig, PhysicsWorld, SubstepForceProvider};
    use crate::water::buoyancy::{lift, BuoyancyForceProvider, StillWater, FLUID_DENSITY};

    fn solid_bulk() -> BulkShape {
        BulkShape::solid(outer_volume(), OAK_DENSITY)
    }

    fn displaced_volume() -> f32 {
        outer_volume().shape.compute_mass(1.0)
    }

    #[test]
    fn an_empty_barrel_weighs_what_a_small_barrel_does() {
        let mass = barrel_bulk().mass().expect("the barrel declares its mass");
        assert!((15.0..25.0).contains(&mass), "the barrel weighs {mass} kg");
    }

    #[test]
    fn an_empty_barrel_is_far_lighter_than_a_solid_one() {
        let hollow = barrel_bulk().mass().unwrap();
        let solid = solid_bulk().mass().unwrap();
        assert!(
            solid > 4.0 * hollow,
            "solid {solid} kg against hollow {hollow} kg"
        );
    }

    /// Drop `bulk` on its side into still water, let it settle, and return
    /// the fraction of its outer volume left under the surface.
    fn settled_submerged_fraction(bulk: BulkShape) -> f32 {
        const DT: f32 = 1.0 / 240.0;
        const SURFACE: f32 = 5.0;

        let water = StillWater {
            surface: SURFACE,
            floor: SURFACE - 50.0,
        };
        // Sleep would freeze the barrel wherever it first slowed, mid-bob;
        // the game's sleep tracker asks the water, and this test has none.
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let on_side = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, SURFACE, 0.0))
                .rotation(on_side)
                .bulk(bulk),
        );
        world.attach_collider(
            body,
            ColliderDesc::convex_hull(Arc::new(PROFILE.hull(STAVES, HULL_RINGS))),
        );
        world.wake_body(body);

        let floor = FlatQuadGeometry::new(20.0);
        let buoyancy = BuoyancyForceProvider::new(&water, vec![body]);
        let providers: [&dyn SubstepForceProvider; 1] = [&buoyancy];
        let mut debug = DebugLines::default();
        for _ in 0..(5.0 / DT) as usize {
            world.update_contacts(DT, 1, &floor, &[], &mut debug);
            world.substep(DT, &floor, &providers);
        }

        let settled = world.body(body).expect("the barrel");
        assert!(
            settled.linear_velocity().norm() < 0.01,
            "still moving at {:?}",
            settled.linear_velocity()
        );

        lift(&world, body, &water) / (FLUID_DENSITY * 9.81 * displaced_volume())
    }

    /// Archimedes: at rest a floating body displaces its own mass of water.
    fn floating_fraction(bulk: &BulkShape) -> f32 {
        bulk.mass().unwrap() / (FLUID_DENSITY * displaced_volume())
    }

    #[test]
    fn an_empty_barrel_floats_high() {
        let fraction = settled_submerged_fraction(barrel_bulk());
        assert!(
            fraction < 0.2,
            "{:.0}% of the barrel is under water",
            fraction * 100.0
        );
        let expected = floating_fraction(&barrel_bulk());
        assert!(
            (fraction - expected).abs() < 0.01,
            "floats {fraction} under, where its mass says {expected}"
        );
    }

    #[test]
    fn a_solid_barrel_floats_low() {
        let fraction = settled_submerged_fraction(solid_bulk());
        assert!(
            fraction > 0.5,
            "{:.0}% of a solid barrel is under water",
            fraction * 100.0
        );
    }
}
