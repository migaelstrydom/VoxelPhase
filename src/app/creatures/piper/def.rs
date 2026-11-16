//! The piper as a spawnable: its body, its lobbing, and the model its
//! sculpture is put together into.
//!
//! ```text
//!   bag, pipes, head, limbs ──sculpt──▶ Model: BAG, LEGS, HEAD, EYES,
//!                                              DRONES, CHANTER
//!   head ──cheeks──▶ Bulge: the cheeks blown out of the face
//!   body() ──▶ LobberBody: which parts swell, ride the bag, puff and look
//!   spawn ──▶ a damped ball kept upright, a PosedModel in its own frame,
//!             its cheeks a ProceduralMesh, all-round perception, a
//!             Lobber, Health
//! ```

use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::bag::{COLLAR, CORE};
use super::frame::{MUZZLE, RADIUS};
use super::pipes::Pipe;
use super::shaping::{placed, surface, vector};
use super::{bag, head, limbs, pipes};
use crate::animation::FootSide;
use crate::app::creatures::likeness;
use crate::app::creatures::palette::{self, Paint, BESTIARY};
use crate::app::creatures::sculpted::{all_round, ground_under, Exhibit};
use crate::app::sculpt::Clay;
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, ProceduralMesh, Renderable, RigidBodyComponent,
    Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{
    Cheeks, Gaze, Lobber, LobberBody, Puffing, Rider, Showing, Strut, Swell, Throw,
};
use crate::damage::{Health, Ragdoll};
use crate::model::{Bulge, MeshPrimitive, Model, ModelPart, Transform};
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

/// The parts of its model, and how many there are: the bag; the legs it
/// squats on; the head with its neck and blowpipe; the eyes; the drones;
/// the chanter with the hand on it; the arm, in two, its elbow on a thigh.
const BAG: usize = 0;
const LEGS: usize = 1;
const HEAD: usize = 2;
const EYES: [usize; 2] = [3, 4];
const DRONES: [usize; 3] = [5, 6, 7];
const CHANTER: usize = 8;
const FOREARM: usize = 9;
const UPPER_ARM: usize = 10;
const PARTS: usize = 11;

/// The bag's size just after a throw and as the next is due, as scales of
/// the size it is sculpted.
const SLACK: (f32, f32, f32) = (0.94, 0.84, 0.94);
const TIGHT: (f32, f32, f32) = (1.07, 1.09, 1.07);
/// How far its cheeks are blown between puffs and at the top of the
/// hardest, as shares of the way from let down to as sculpted.
const CHEEKS_SLACK: f32 = 0.3;
const CHEEKS_TIGHT: f32 = 1.05;

/// A piper.
#[derive(Deserialize)]
pub struct PiperDef {
    /// Position (x, z). It squats on the ground there.
    pub pos: (f32, f32),
}

impl PiperDef {
    /// The parts of its model that show a throw coming, its cheeks blown
    /// out of its face as `cheeks`.
    fn body(cheeks: Arc<Bulge>) -> LobberBody {
        let sockets = head::sockets();
        let eye = |which: usize| {
            Some(Gaze {
                part: EYES[which],
                socket: sockets[which].0,
                ahead: sockets[which].1,
                swivel: 0.6,
            })
        };
        let belly = bag::belly();
        let small = Self::small_pipes(&belly);
        let mut riders: Vec<Rider> = pipes::drones(&belly)
            .iter()
            .zip(DRONES)
            .map(|(drone, part)| fastened(&belly, part, drone.foot))
            .collect();
        riders.push(fastened(&belly, CHANTER, small[0].foot));
        // The arm rests its elbow on a thigh, and its forearm turns there
        // to keep hold of the chanter, its upper arm to stay in the bag.
        let joints = limbs::arm_joints(&small[0]);
        let struts = vec![
            Strut {
                part: FOREARM,
                pivot: joints.elbow,
                reach: joints.wrist,
                held_by: Some(CHANTER),
            },
            Strut {
                part: UPPER_ARM,
                pivot: joints.elbow,
                reach: joints.shoulder,
                held_by: None,
            },
        ];
        LobberBody {
            bag: Swell {
                part: BAG,
                seat: Vector3::new(0.0, -RADIUS, 0.0),
                slack: vector(SLACK),
                tight: vector(TIGHT),
            },
            // Its neck is gathered in the bag's mouth, at the top of the
            // collar, which stands out of the belly as the belly does there.
            head: Some(Rider {
                part: HEAD,
                anchor: bag::throat(),
                out: fastened(&belly, HEAD, vector(COLLAR)).out,
            }),
            cheeks: Some(Cheeks {
                bulge: cheeks,
                slack: CHEEKS_SLACK,
                tight: CHEEKS_TIGHT,
            }),
            eyes: [eye(0), eye(1)],
            riders,
            struts,
            frame: Some(LEGS),
        }
    }

    /// The chanter and the blowpipe's two lengths.
    fn small_pipes(belly: &Clay) -> [Pipe; 3] {
        let lips = head::in_head(head::LIPS);
        let blow = head::bow() * Vector3::new(0.0, -0.5, 0.87).normalize();
        pipes::small_pipes(belly, lips, blow)
    }

    /// How it looks, against the palette's slots, about the middle of the
    /// ball it collides as: its parts at [`BAG`], [`LEGS`], [`HEAD`],
    /// [`EYES`], [`DRONES`], [`CHANTER`], [`FOREARM`] and [`UPPER_ARM`].
    /// Its cheeks are apart from it ([`head::cheeks`]).
    pub fn sculpt() -> Model {
        let belly = bag::belly();
        let drones = pipes::drones(&belly);
        let small = Self::small_pipes(&belly);
        let stocks: Vec<&Pipe> = drones
            .iter()
            .chain(&small)
            .filter(|pipe| pipe.stock.is_some())
            .collect();

        let form = bag::form(&belly, &stocks);
        let in_head = |mesh: MeshPrimitive| placed(mesh, head::bow(), vector(head::HEAD));

        let mut parts: [Vec<MeshPrimitive>; PARTS] = Default::default();
        parts[BAG] = vec![bag::hide(&form), bag::thread(&belly)];
        parts[LEGS] = limbs::leg(FootSide::Left);
        parts[LEGS].extend(limbs::leg(FootSide::Right));
        // The blowpipe is held in the lips: it goes where the head goes,
        // its stock sliding in and out of the bag.
        parts[HEAD] = vec![in_head(head::head()), in_head(head::neck())];
        parts[HEAD].extend(small[1..].iter().flat_map(Pipe::meshes));
        for eye in EYES {
            parts[eye] = vec![head::eye()];
        }
        for (drone, part) in drones.iter().zip(DRONES) {
            parts[part] = drone.meshes();
        }
        let arm = limbs::arm(&small[0]);
        parts[CHANTER] = small[0].meshes();
        parts[CHANTER].push(arm.hand);
        parts[FOREARM] = vec![arm.forearm];
        parts[UPPER_ARM] = vec![arm.upper_arm];
        Model::flat(parts.into_iter().map(ModelPart::new).collect())
    }

    /// Its parts' poses and its cheeks, blown out of `cheeks`, `fullness`
    /// of the way from one throw to the next, its cheeks `blown`, watching
    /// `target` (in its own frame).
    fn poses(
        cheeks: Arc<Bulge>,
        fullness: f32,
        blown: f32,
        target: Option<Vector3<f32>>,
    ) -> (Vec<Transform>, Vec<MeshPrimitive>) {
        let mut poses = vec![Transform::default(); PARTS];
        let mut shaped = Vec::new();
        let now = Showing {
            throw: Throw {
                fullness,
                since: None,
                until: None,
            },
            clock: 0.0,
            blown,
            target,
        };
        Self::body(cheeks).pose(&mut poses, &mut shaped, &now);
        (poses, shaped)
    }

    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        let cheeks = likeness::bulge_sculpted(KIND, head::cheeks);
        // Whoever is looking at it, it is looking back.
        let watcher = Some(Vector3::new(2.4, 1.0, 3.6));
        let shown =
            |name: &'static str, (poses, shaped): (Vec<Transform>, Vec<MeshPrimitive>)| Exhibit {
                name,
                model: model.clone(),
                poses,
                shaped,
                lift: RADIUS,
                reach: 1.3,
            };
        let mut spent = (vec![Transform::default(); PARTS], Vec::new());
        Self::body(cheeks.clone()).slump(&mut spent.0, &mut spent.1);
        vec![
            shown("slack", Self::poses(cheeks.clone(), 0.0, 0.0, watcher)),
            shown("swollen", Self::poses(cheeks, 1.0, 1.0, watcher)),
            shown("spent", spent),
        ]
    }
}

/// A rider of `part` fastened into the bag at `at`: standing out of the
/// belly as the belly does there.
fn fastened(belly: &Clay, part: usize, at: Vector3<f32>) -> Rider {
    let core = vector(CORE);
    Rider {
        part,
        anchor: at,
        out: surface(belly, core, at - core).1,
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
        let cheeks = likeness::bulge(KIND, materials, head::cheeks);
        let lobber = Lobber {
            range: RANGE,
            interval: INTERVAL,
            remaining: INTERVAL,
            since_throw: None,
            flight_time: FLIGHT_TIME,
            muzzle: Vector3::y() * MUZZLE,
            body: Some(Self::body(cheeks.clone())),
            puffing: Puffing::default(),
        };
        // Posed as it starts, so that a level looked at without being run
        // shows it.
        let (poses, shaped) = Self::poses(cheeks, lobber.fullness(), 0.0, None);
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(PosedModel {
                model: likeness::model(KIND, materials, Self::sculpt),
                poses,
                frame: PoseFrame::Entity,
            })
            .with(ProceduralMesh { primitives: shaped })
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
        assert_eq!(model.parts.len(), PARTS);
        let mut triangles = 0;
        for (part, name) in model.parts.iter().zip([
            "bag",
            "legs",
            "head",
            "eye",
            "eye",
            "bass",
            "tenor",
            "stump",
            "chanter",
            "forearm",
            "upper arm",
        ]) {
            let each: Vec<usize> = part
                .primitives
                .iter()
                .map(|m| m.indices.len() / 3)
                .collect();
            println!("  {name}: {each:?}");
            triangles += each.iter().sum::<usize>();
        }
        let cheeks = head::cheeks().blown().indices.len() / 3;
        println!("  cheeks: {cheeks}");
        triangles += cheeks;
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
        let low = model.parts[LEGS]
            .primitives
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .map(|vertex| vertex.pos.y)
            .fold(f32::MAX, f32::min);
        assert!((low + RADIUS).abs() < 0.015, "its feet are at {low}");
    }

    #[test]
    fn its_bag_is_bigger_swollen_than_slack_and_never_leaves_the_ground() {
        let cheeks = Arc::new(head::cheeks());
        let posed = |fullness: f32| PiperDef::poses(cheeks.clone(), fullness, 0.5, None).0;
        let bag = |fullness: f32| posed(fullness)[BAG].clone();
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
        let poses = posed(0.5);
        for eye in EYES {
            assert!((poses[eye].translation - vector(head::HEAD)).norm() < 0.15);
        }
    }
}
