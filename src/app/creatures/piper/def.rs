//! The piper as a spawnable: its body, its lobbing, and the model its
//! sculpture is put together into.
//!
//! ```text
//!   bag, pipes, head, limbs ──sculpt──▶ Model: BAG, FIXTURES, CHEEKS, EYES
//!   body() ──▶ LobberBody: which parts swell, puff and look ──▶ Lobber
//!   spawn ──▶ a damped ball kept upright, a PosedModel in its own frame,
//!             all-round perception, a Lobber, Health
//! ```

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::frame::{MUZZLE, RADIUS};
use super::pipes::Pipe;
use super::shaping::{placed, vector};
use super::{bag, head, limbs, pipes};
use crate::animation::FootSide;
use crate::app::creatures::likeness;
use crate::app::creatures::palette::{self, Paint, BESTIARY};
use crate::app::creatures::sculpted::{all_round, ground_under, Exhibit};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Gaze, Lobber, LobberBody, Swell, Throw};
use crate::damage::{Health, Ragdoll};
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// The name its look is kept under.
const KIND: &str = "piper";

/// Kilograms to the cubic metre: a bag of air and skin.
const DENSITY: f32 = 500.0;

/// Nearest and furthest it throws, m, seconds between throws, and seconds a
/// bomb is in the air.
const RANGE: (f32, f32) = (4.0, 24.0);
const INTERVAL: f32 = 3.5;
const FLIGHT_TIME: f32 = 1.9;

/// Hit points it can take.
const HEALTH: f32 = 25.0;

/// The parts of its model, and how many there are.
const BAG: usize = 0;
const FIXTURES: usize = 1;
const CHEEKS: usize = 2;
const EYES: [usize; 2] = [3, 4];
const PARTS: usize = 5;

/// The bag's size just after a throw and as the next is due, as scales of
/// the size it is sculpted.
const SLACK: (f32, f32, f32) = (0.94, 0.84, 0.94);
const TIGHT: (f32, f32, f32) = (1.07, 1.09, 1.07);
/// The cheeks' size between puffs and at the top of one.
const CHEEKS_SLACK: f32 = 0.84;
const CHEEKS_TIGHT: f32 = 1.1;

/// A piper.
#[derive(Deserialize)]
pub struct PiperDef {
    /// Position (x, z). It squats on the ground there.
    pub pos: (f32, f32),
}

impl PiperDef {
    /// The parts of its model that show a throw coming.
    fn body() -> LobberBody {
        let sockets = head::sockets();
        let eye = |which: usize| {
            Some(Gaze {
                part: EYES[which],
                socket: sockets[which].0,
                ahead: sockets[which].1,
                swivel: 0.6,
            })
        };
        LobberBody {
            bag: Swell {
                part: BAG,
                seat: Vector3::new(0.0, -RADIUS, 0.0),
                slack: vector(SLACK),
                tight: vector(TIGHT),
            },
            cheeks: Some(Swell {
                part: CHEEKS,
                seat: head::in_head(head::CHEEK_ROOT),
                slack: Vector3::repeat(CHEEKS_SLACK),
                tight: Vector3::repeat(CHEEKS_TIGHT),
            }),
            eyes: [eye(0), eye(1)],
            frame: Some(FIXTURES),
        }
    }

    /// How it looks, against the palette's slots, about the middle of the
    /// ball it collides as: its parts at [`BAG`], [`FIXTURES`], [`CHEEKS`]
    /// and [`EYES`].
    pub fn sculpt() -> Model {
        let belly = bag::belly();
        let drones = pipes::drones(&belly);
        let lips = head::in_head(head::LIPS);
        let blow = head::bow() * Vector3::new(0.0, -0.5, 0.87).normalize();
        let small = pipes::small_pipes(&belly, lips, blow);
        let stocks: Vec<&Pipe> = drones
            .iter()
            .chain(&small)
            .filter(|pipe| pipe.stock.is_some())
            .collect();

        let form = bag::form(&belly, &stocks);
        let in_head = |mesh: MeshPrimitive| placed(mesh, head::bow(), vector(head::HEAD));

        let mut fixtures: Vec<MeshPrimitive> =
            drones.iter().chain(&small).flat_map(Pipe::meshes).collect();
        fixtures.push(in_head(head::head()));
        fixtures.push(in_head(head::neck()));
        fixtures.extend(limbs::leg(FootSide::Left));
        fixtures.extend(limbs::leg(FootSide::Right));
        fixtures.extend(limbs::hand(&small[0]));

        let mut parts: [Vec<MeshPrimitive>; PARTS] = Default::default();
        parts[BAG] = vec![bag::hide(&form), bag::thread(&belly)];
        parts[FIXTURES] = fixtures;
        parts[CHEEKS] = vec![in_head(head::cheeks())];
        for eye in EYES {
            parts[eye] = vec![head::eye()];
        }
        Model::flat(parts.into_iter().map(ModelPart::new).collect())
    }

    /// Its parts' poses `fullness` of the way from one throw to the next,
    /// watching `target` (in its own frame).
    fn poses(fullness: f32, target: Option<Vector3<f32>>) -> Vec<Transform> {
        let mut poses = vec![Transform::default(); PARTS];
        let throw = Throw {
            fullness,
            since: None,
            until: None,
        };
        Self::body().pose(&mut poses, throw, 0.0, target);
        poses
    }

    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        // Whoever is looking at it, it is looking back.
        let watcher = Some(Vector3::new(2.4, 1.0, 3.6));
        let shown = |name: &'static str, poses: Vec<Transform>| Exhibit {
            name,
            model: model.clone(),
            poses,
            lift: RADIUS,
            reach: 1.3,
        };
        let mut spent = Self::poses(0.0, None);
        Self::body().slump(&mut spent);
        vec![
            shown("slack", Self::poses(0.0, watcher)),
            shown("swollen", Self::poses(1.0, watcher)),
            shown("spent", spent),
        ]
    }
}

impl Spawnable for PiperDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let Some(ground) = ground_under(world, self.pos) else {
            return Vec::new();
        };
        let at = Point3::new(self.pos.0, ground + RADIUS, self.pos.1);
        let (body, keep_upright) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            // It sits where it is put: heavy damping, so a shove rocks it
            // and it settles.
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(at)
                    .linear_damping(0.4)
                    .angular_damping(0.98),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::sphere(RADIUS)
                    .density(DENSITY)
                    .friction(1.2)
                    .restitution(0.0),
            );
            // A ball rolls off down any slope, drones and all. Kept
            // upright while it lives; let go when it bursts, so it slumps.
            let keep_upright = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            (body, keep_upright)
        };
        let lobber = Lobber {
            range: RANGE,
            interval: INTERVAL,
            remaining: INTERVAL,
            since_throw: None,
            flight_time: FLIGHT_TIME,
            muzzle: Vector3::y() * MUZZLE,
            body: Some(Self::body()),
        };
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            // Posed as it starts, so that a level looked at without being
            // run shows it.
            .with(PosedModel {
                model: likeness::model(KIND, materials, Self::sculpt),
                poses: Self::poses(lobber.fullness(), None),
                frame: PoseFrame::Entity,
            })
            .with(Renderable)
            .with(all_round(RANGE.1))
            .with(lobber)
            .with(Health::new(HEALTH, 4.0))
            .with(Ragdoll::new(vec![keep_upright]))
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn it_is_tens_of_thousands_of_triangles_with_a_pose_for_every_part() {
        let started = Instant::now();
        let model = PiperDef::sculpt();
        println!("sculpted in {:?}", started.elapsed());
        assert_eq!(model.parts.len(), PiperDef::poses(0.5, None).len());
        let mut triangles = 0;
        for (part, name) in model
            .parts
            .iter()
            .zip(["bag", "fixtures", "cheeks", "eye", "eye"])
        {
            let each: Vec<usize> = part
                .primitives
                .iter()
                .map(|m| m.indices.len() / 3)
                .collect();
            println!("  {name}: {each:?}");
            triangles += each.iter().sum::<usize>();
        }
        println!("a piper is {triangles} triangles");
        assert!(
            (30_000..200_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn its_tallest_drone_ends_at_the_muzzle_and_it_sits_on_the_ground() {
        let belly = bag::belly();
        let bass = &pipes::drones(&belly)[0];
        let rim = bass
            .meshes()
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .map(|vertex| vertex.pos.y)
            .fold(f32::MIN, f32::max);
        assert!((rim - MUZZLE).abs() < 0.02, "its mouth is at {rim}");
        assert!(bass.tip.xz().norm() < 1e-6);

        let model = PiperDef::sculpt();
        let low = model.parts[FIXTURES]
            .primitives
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .map(|vertex| vertex.pos.y)
            .fold(f32::MAX, f32::min);
        assert!((low + RADIUS).abs() < 0.015, "its feet are at {low}");
    }

    #[test]
    fn its_bag_is_bigger_swollen_than_slack_and_never_leaves_the_ground() {
        let bag = |fullness: f32| PiperDef::poses(fullness, None)[BAG].clone();
        let (slack, swollen) = (bag(0.0), bag(1.0));
        assert!(swollen.scale.y > slack.scale.y + 0.1);
        assert!(swollen.scale.x > slack.scale.x);
        for pose in [slack, swollen] {
            let seat = pose
                .to_matrix()
                .transform_point(&Point3::new(0.0, -RADIUS, 0.0));
            assert!((seat.y + RADIUS).abs() < 1e-5, "it sits at {}", seat.y);
        }
        // Its eyes are in its head, not where they were modelled.
        let poses = PiperDef::poses(0.5, None);
        for eye in EYES {
            assert!((poses[eye].translation - vector(head::HEAD)).norm() < 0.15);
        }
    }
}
