//! A pump: a stone bulb with a standpipe, welded into the bed of a pond,
//! that lifts the pond's water and throws it from spouts round the pipe's
//! head.
//!
//! ```text
//!      ↖ ↑ ↗ jets       spouts: round the pipe's head
//!        ║ standpipe
//!      ╭─╨─╮
//!   ~~~│   │~~~ water
//!      ╰───╯ bulb       intake: low on the bulb, just over the bed
//!   ▒▒▒▒▒▒▒▒▒▒▒ bed     weld, let go when the bed under it is blasted
//! ```
//!
//! The pumping is the water's (`crate::pump`, `water::field::pumps`): loose,
//! it throws water whichever way its spouts point for as long as its intake
//! lies under water.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::ColliderSurface;
use super::shared::rose_stone::{self, MATERIAL_COUNT, STONE};
use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{painted_model, Ellipsoid, SculptedPart, Warp};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::pump::{Spout, WaterPump};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// How much of the bulb's radius its centre stands above the bed: the rest
/// of its lower half is bedded.
const BULB_STAND: f32 = 0.5;
/// How far below the bulb's centre the intake is, as a share of its radius:
/// just clear of the bed.
const INTAKE_DROP: f32 = 0.35;
/// The standpipe's radius, as a share of the bulb's.
const PIPE_RADIUS: f32 = 0.26;
/// Spouts round the pipe's head. Four, so a quarter turn changes nothing.
const SPOUTS: usize = 4;
/// How steeply a spout throws its water, above level, radians.
const SPOUT_PITCH: f32 = 1.0;
/// How far out from the pipe's axis a spout opens, as a share of the
/// bulb's radius.
const SPOUT_REACH: f32 = 0.5;

/// A pump standing in a pond.
#[derive(Deserialize)]
pub struct PumpDef {
    /// Position (x, z). It stands on the bed there.
    pub pos: (f32, f32),
    /// Radius of its bulb, m.
    #[serde(default = "PumpDef::default_radius")]
    pub radius: f32,
    /// Height of the pipe's head above the bed, m. Water deeper than this
    /// hides its jets.
    #[serde(default = "PumpDef::default_height")]
    pub height: f32,
    /// The most water it moves, m³/s.
    #[serde(default = "PumpDef::default_rate")]
    pub rate: f32,
    /// How fast its water leaves a spout, m/s.
    #[serde(default = "PumpDef::default_jet_speed")]
    pub jet_speed: f32,
}

impl PumpDef {
    pub fn default_radius() -> f32 {
        0.45
    }
    pub fn default_height() -> f32 {
        1.8
    }
    /// Enough that each spout's stream is as thick as a wrist: a thinner
    /// one is too clear to see.
    pub fn default_rate() -> f32 {
        0.4
    }
    pub fn default_jet_speed() -> f32 {
        3.5
    }

    /// Height of the pipe's head above the bulb's centre, m.
    fn head(&self) -> f32 {
        self.height - BULB_STAND * self.radius
    }

    /// The way spout `index` faces, level, in the body's frame.
    fn spout_facing(index: usize) -> Vector3<f32> {
        let azimuth = TAU * index as f32 / SPOUTS as f32;
        Vector3::new(azimuth.cos(), 0.0, azimuth.sin())
    }

    /// How it looks, drawn in `materials` (registered from
    /// [`rose_stone::materials`], in order), round its bulb's centre with
    /// its pipe up.
    pub fn model(&self, materials: &[MaterialId]) -> Model {
        let r = self.radius;
        let head = self.head();
        let looks = rose_stone::Looks::of(materials);
        let (rose, gold) = (looks.rose(), looks.gold());
        let pipe = PIPE_RADIUS * r;
        let foot = 0.8 * r;
        let mut parts = vec![
            // The bulb, lobed like a gourd.
            SculptedPart::new(Ellipsoid::new((r, 0.92 * r, r), (0.0, 0.0, 0.0)), rose).warped(
                Warp::Scallop {
                    lobes: 8,
                    bulge: 0.06,
                    lift: 0.0,
                },
            ),
            // A collar where the pipe leaves it.
            SculptedPart::new(
                Ellipsoid::new((0.5 * r, 0.1 * r, 0.5 * r), (0.0, 0.9 * r, 0.0)),
                gold,
            ),
            // The standpipe, narrowing as it rises.
            SculptedPart::new(
                Ellipsoid::new(
                    (pipe, 0.5 * (head - foot) + pipe, pipe),
                    (0.0, 0.5 * (foot + head), 0.0),
                ),
                rose,
            )
            .warped(Warp::Taper {
                along: Vector3::y(),
                tip: 0.75,
            }),
            // A knop half-way up it.
            SculptedPart::new(
                Ellipsoid::new(
                    (1.5 * pipe, 0.8 * pipe, 1.5 * pipe),
                    (0.0, 0.5 * (foot + head), 0.0),
                ),
                rose,
            ),
            // The head the spouts open from, and a bud on top of it.
            SculptedPart::new(
                Ellipsoid::new(
                    (SPOUT_REACH * r, 0.3 * r, SPOUT_REACH * r),
                    (0.0, head, 0.0),
                ),
                rose,
            ),
            SculptedPart::new(
                Ellipsoid::new((0.2 * r, 0.3 * r, 0.2 * r), (0.0, head + 0.4 * r, 0.0)),
                gold,
            ),
        ];
        // A gilt mouth at each spout.
        for index in 0..SPOUTS {
            let at = Vector3::y() * head + Self::spout_facing(index) * SPOUT_REACH * r;
            parts.push(SculptedPart::new(Ellipsoid::ball(0.11 * r, at), gold));
        }
        painted_model(
            &parts,
            &rose_stone::mottle(r),
            &rose_stone::contact_shade(r),
        )
    }

    /// Its pump, in the frame of the bulb's centre.
    fn pump(&self) -> WaterPump {
        let (sin, cos) = SPOUT_PITCH.sin_cos();
        let spouts = (0..SPOUTS)
            .map(|index| {
                let facing = Self::spout_facing(index);
                Spout {
                    outlet: Point3::from(
                        Vector3::y() * self.head() + facing * (SPOUT_REACH + 0.11) * self.radius,
                    ),
                    velocity: (facing * cos + Vector3::y() * sin) * self.jet_speed,
                }
            })
            .collect();
        WaterPump {
            intake: Point3::new(0.0, -INTAKE_DROP * self.radius, 0.0),
            spouts,
            rate: self.rate,
        }
    }
}

impl Spawnable for PumpDef {
    fn validate(&self) -> Result<(), String> {
        if self.radius <= 0.0 {
            return Err(format!(
                "a pump's radius must be positive, not {}",
                self.radius
            ));
        }
        if self.height < 2.0 * self.radius {
            return Err(format!(
                "a pump's pipe must clear its bulb: height {} is under twice its radius {}",
                self.height, self.radius
            ));
        }
        if self.rate < 0.0 || self.jet_speed <= 0.0 {
            return Err("a pump's rate cannot be negative and its jet must move".into());
        }
        Ok(())
    }

    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(rose_stone::materials()
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let bed = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        let Some(bed) = bed else {
            return Vec::new();
        };
        let r = self.radius;
        let centre = Point3::new(self.pos.0, bed + BULB_STAND * r, self.pos.1);
        let pipe = PIPE_RADIUS * r;
        // The pipe's capsule, from inside the bulb to the top of its head.
        let (pipe_foot, pipe_top) = (0.5 * r, self.head() + 0.3 * r);

        let (body, weld, moved) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            // Welded with its foot in the bed, which it passes through
            // until released: the weld holds it where the bed would.
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(centre)
                    .linear_damping(0.02)
                    .angular_damping(0.1)
                    .ignores_static(true),
            );
            physics
                .world
                .attach_collider(body, ColliderDesc::sphere(r).with_physical_surface(STONE));
            physics.world.attach_collider(
                body,
                ColliderDesc::capsule(0.5 * (pipe_top - pipe_foot), pipe)
                    .offset_translation(Vector3::y() * 0.5 * (pipe_top + pipe_foot))
                    .with_physical_surface(STONE),
            );
            // The body's origin goes to its centre of mass, up the pipe
            // from the bulb's centre; everything else follows it.
            let moved = physics.world.recenter_on_colliders(body);
            let weld = physics.world.create_constraint(ConstraintKind::world_fixed(
                body,
                centre,
                -moved,
                &UnitQuaternion::identity(),
                0.0,
                f32::MAX,
            ));
            (body, weld, moved)
        };

        let mut model = self.model(materials);
        for vertex in model
            .parts
            .iter_mut()
            .flat_map(|part| part.primitives.iter_mut())
            .flat_map(|primitive| primitive.vertices.iter_mut())
        {
            vertex.pos -= moved;
        }
        let mut pump = self.pump();
        pump.recentre(moved);

        vec![world
            .create_entity()
            .with(Position(centre.coords + moved))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(model)))
            .with(Renderable)
            .with(pump)
            .with(TerrainAnchored {
                constraints: vec![weld],
                anchor_points: vec![Point3::new(self.pos.0, bed - 0.1, self.pos.1)],
                released_model: None,
            })
            .build()]
    }
}

#[cfg(all(test, feature = "water_scenarios"))]
mod tests {
    use specs::{RunNow, World};

    use super::*;
    use crate::app::world_builder::WorldBuilder;
    use crate::level::{resolve_placements, Level};
    use crate::level_check::build_terrain;
    use crate::pump::PumpSystem;
    use crate::systems::{PhysicsSyncSystem, TerrainAnchorSystem};
    use crate::terrain::BlastConfig;
    use crate::water::WaterWorld;

    const DT: f32 = 1.0 / 60.0;
    /// The pond's surface, m.
    const POND: f32 = -1.5;

    /// A crater holding a pond.
    const LEVEL: &str = r#"
        Level(
            name: "Pump",
            segments: [(
                name: "main",
                terrain: Terrain(
                    voxel_size: 0.5,
                    bounds: (min: (-32.0, -16.0, -32.0), max: (32.0, 16.0, 32.0)),
                    base_height: 0.0,
                    material_layers: [(depth: 999.0, material: Rock)],
                    features: [Crater(center: (0.0, 0.0), radius: 20.0, depth: 6.0)],
                ),
            )],
            placements: [Root(segment: "main")],
            player_spawn: (25.0, 2.0, 25.0),
            water: Some((bodies: [Pool(seed: (0.0, 0.0), surface_level: -1.5)])),
        )
    "#;

    /// The game's world for [`LEVEL`], and what steps its bodies.
    struct Pond {
        world: World,
        physics: PhysicsSyncSystem,
    }

    impl Pond {
        fn new() -> Self {
            let mut level: Level = ron::from_str(LEVEL).expect("the pond level parses");
            level.frames = resolve_placements(&level).expect("its placement resolves");
            let terrain = build_terrain(&level);
            let water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
            let mut world = WorldBuilder::new()
                .with_default_resources()
                .build()
                .expect("a world with no device attached builds infallibly");
            world.insert(terrain);
            world.insert(water);
            world.insert(crate::time::Time::fixed(DT));
            Self {
                world,
                physics: PhysicsSyncSystem::default(),
            }
        }

        /// The bed's height at `x` on the pond's east-west line.
        fn bed(&self, x: f32) -> f32 {
            self.world
                .read_resource::<TerrainWorld>()
                .mesh_surface_height_at(x, 0.0)
                .unwrap()
        }

        /// Where along that line the bed lies `depth` under the pond's
        /// surface, coming in from the east bank.
        fn where_deep(&self, depth: f32) -> f32 {
            (0..200)
                .map(|step| 20.0 - 0.1 * step as f32)
                .find(|&x| self.bed(x) < POND - depth)
                .expect("the pond is that deep somewhere")
        }

        fn spawn_pump(&mut self, x: f32) -> Entity {
            let pump = PumpDef {
                pos: (x, 0.0),
                radius: PumpDef::default_radius(),
                height: PumpDef::default_height(),
                rate: PumpDef::default_rate(),
                jet_speed: PumpDef::default_jet_speed(),
            };
            pump.spawn(&mut self.world, &[MaterialId(0); MATERIAL_COUNT])[0]
        }

        /// Run the systems a pump lives by for `seconds`.
        fn run(&mut self, seconds: f32) {
            for _ in 0..(seconds / DT).round() as usize {
                self.physics.run_now(&self.world);
                TerrainAnchorSystem.run_now(&self.world);
                PumpSystem.run_now(&self.world);
                let terrain = self.world.read_resource::<TerrainWorld>();
                let mut water = self.world.write_resource::<WaterWorld>();
                water.on_terrain_update(&terrain);
                water.step(DT);
                drop((terrain, water));
                self.world.maintain();
            }
        }

        /// How much water its pumps are moving, m³/s.
        fn pumped(&self) -> f32 {
            let water = self.world.read_resource::<WaterWorld>();
            water.field().jets().map(|jet| jet.discharge).sum()
        }

        fn height(&self, entity: Entity) -> f32 {
            self.world
                .read_storage::<Position>()
                .get(entity)
                .unwrap()
                .0
                .y
        }

        /// Whether the water stands over `entity`'s pump's intake.
        fn intake_is_under_water(&self, entity: Entity) -> bool {
            let position = self.world.read_storage::<Position>().get(entity).unwrap().0;
            let turn = self
                .world
                .read_storage::<Orientation>()
                .get(entity)
                .unwrap()
                .0;
            let pumps = self.world.read_storage::<WaterPump>();
            let intake = Point3::from(position) + turn * pumps.get(entity).unwrap().intake.coords;
            let water = self.world.read_resource::<WaterWorld>();
            water
                .field()
                .level_at(intake)
                .is_some_and(|level| level > intake.y)
        }
    }

    #[test]
    fn a_pump_standing_in_a_pond_throws_its_rate_from_four_spouts() {
        let mut pond = Pond::new();
        let x = pond.where_deep(1.0);
        pond.spawn_pump(x);
        pond.run(1.0);
        let water = pond.world.read_resource::<WaterWorld>();
        assert_eq!(water.field().jets().count(), SPOUTS);
        drop(water);
        assert!((pond.pumped() - PumpDef::default_rate()).abs() < 1.0e-3);
    }

    #[test]
    fn a_standing_pump_sleeps_through_its_own_recoil() {
        let mut pond = Pond::new();
        let x = pond.where_deep(1.0);
        let pump = pond.spawn_pump(x);
        let stood = pond.height(pump);
        pond.run(4.0);
        assert!(pond.pumped() > 0.0);
        let body = pond
            .world
            .read_storage::<RigidBodyComponent>()
            .get(pump)
            .unwrap()
            .0;
        let physics = pond.world.read_resource::<PhysicsResource>();
        assert!(physics.world.is_sleeping(body), "its jets keep it awake");
        assert!((pond.height(pump) - stood).abs() < 1.0e-3);
    }

    #[test]
    fn a_pump_on_the_dry_bank_is_still() {
        let mut pond = Pond::new();
        pond.spawn_pump(25.0);
        pond.run(1.0);
        assert_eq!(pond.pumped(), 0.0);
    }

    #[test]
    fn a_pump_blasted_loose_falls_and_pumps_only_while_its_intake_is_wet() {
        let mut pond = Pond::new();
        let x = pond.where_deep(1.0);
        let pump = pond.spawn_pump(x);
        pond.run(0.5);
        let stood = pond.height(pump);
        let held_before = pond.world.read_resource::<WaterWorld>().volume();

        let bed = pond.bed(x);
        {
            let mut terrain = pond.world.write_resource::<TerrainWorld>();
            terrain.detonate(Point3::new(x, bed - 0.5, 0.0), &BlastConfig::default());
            terrain.update();
        }
        pond.run(6.0);

        assert!(
            !pond.world.read_storage::<TerrainAnchored>().contains(pump),
            "the blast under it did not release it"
        );
        assert!(
            pond.height(pump) < stood - 0.3,
            "it stands at {} where it stood at {stood}",
            pond.height(pump)
        );
        assert_eq!(
            pond.pumped() > 0.0,
            pond.intake_is_under_water(pump),
            "it pumps exactly when its intake is under water"
        );
        // Pumping made no water: the pond holds what it held, less only
        // what the field's own ledger accounts for.
        let water = pond.world.read_resource::<WaterWorld>();
        let ledger = water.field().ledger();
        assert_eq!(ledger.sourced, 0.0);
        let drift = water.volume() + ledger.sunk + ledger.lost - held_before;
        assert!(
            drift.abs() < 0.05 * held_before,
            "the pond changed by {drift} m³"
        );
    }
}
