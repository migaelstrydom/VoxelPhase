//! Scree: slivers and chips that fall on a flight of their own and crumble
//! where they land. Never a physics body, so they cost the solver nothing and
//! bring none of a thin body's contact trouble.
//!
//! ```text
//!   RubbleSpawnSystem ──▶ FallingScree { Flight, Crumble size } + Position,
//!                         Orientation, TerrainMeshInstance
//!   ScreeSystem, per frame:
//!       Flight::step ── gravity, spin, a ray along the frame's motion ──▶
//!           Flying  ──▶ Position, Orientation
//!           Landed  ──▶ a crumble at the hit, entity deleted
//!           Expired ──▶ entity deleted (out of time, or below the terrain)
//! ```

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{
    Builder, Component, Entities, Join, LazyUpdate, Read, ReadExpect, System, VecStorage,
    WriteStorage,
};

use super::dust::{Crumble, CrumbleSize};
use crate::components::{Orientation, Position};
use crate::sensing::ProbeTarget;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// How scree flies.
#[derive(Debug, Clone, Copy)]
pub struct ScreeRules {
    /// Downward acceleration, m/s².
    pub gravity: f32,
    /// Seconds a piece may fly before it is removed, landed or not.
    pub lifetime: f32,
    /// How far below the terrain's lowest point a piece is given up on, m.
    pub floor_margin: f32,
    /// The least upward share of a surface's normal for the piece to crumble
    /// on it: ground. Steeper than this is a wall or a ceiling, which the
    /// piece glances off and goes on falling.
    pub ground_normal_y: f32,
    /// Frames running a piece may glance off walls before it crumbles where it
    /// is: wedged in a notch, it would otherwise hang there until its time ran
    /// out.
    pub max_glances: u32,
}

impl Default for ScreeRules {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            lifetime: 4.0,
            floor_margin: 4.0,
            ground_normal_y: 0.4,
            max_glances: 6,
        }
    }
}

/// Where a piece of scree is and how it moves.
#[derive(Debug, Clone)]
pub struct Flight {
    /// Its centroid, in the world.
    pub position: Point3<f32>,
    pub orientation: UnitQuaternion<f32>,
    /// m/s.
    pub velocity: Vector3<f32>,
    /// Angular velocity in world axes, rad/s.
    pub spin: Vector3<f32>,
    /// Seconds since it broke off.
    pub age: f32,
    /// Frames running it has glanced off a wall or a ceiling.
    glances: u32,
    /// Its mesh's extreme points in its own frame, one per direction of a
    /// 26-direction hull: enough to tell how far it reaches along any
    /// direction without walking every vertex.
    support: Vec<Vector3<f32>>,
}

/// What a step of flight came to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    Flying,
    /// It reached the ground at this point.
    Landed(Point3<f32>),
    /// Out of time, or fallen past the floor.
    Expired,
}

impl Flight {
    /// A piece whose mesh, around its centroid at `position`, has `vertices`.
    pub fn new(
        position: Point3<f32>,
        velocity: Vector3<f32>,
        spin: Vector3<f32>,
        vertices: impl IntoIterator<Item = Vector3<f32>>,
    ) -> Self {
        Self {
            position,
            orientation: UnitQuaternion::identity(),
            velocity,
            spin,
            age: 0.0,
            glances: 0,
            support: support_points(vertices),
        }
    }

    /// How far the piece reaches from its centroid along unit `direction`.
    fn reach(&self, direction: Vector3<f32>) -> f32 {
        self.support
            .iter()
            .map(|p| (self.orientation * p).dot(&direction))
            .fold(0.0, f32::max)
    }

    /// Move `dt` seconds on. It lands when the point of it leading along the
    /// frame's motion would pass through ground: a surface facing up. A wall
    /// or a ceiling takes the part of its velocity going into it, and it falls
    /// on. `floor_y` is the height below which nothing is left to land on.
    pub fn step(
        &mut self,
        dt: f32,
        rules: &ScreeRules,
        ground: &impl ProbeTarget,
        floor_y: f32,
    ) -> Outcome {
        self.velocity.y -= rules.gravity * dt;
        let delta = self.velocity * dt;
        let distance = delta.norm();
        if distance > f32::EPSILON {
            let direction = delta / distance;
            let length = distance + self.reach(direction);
            if let Some(hit) = ground.raycast(self.position, direction, length) {
                self.glances += 1;
                if hit.normal.y >= rules.ground_normal_y || self.glances > rules.max_glances {
                    return Outcome::Landed(hit.point);
                }
                self.velocity -= hit.normal * self.velocity.dot(&hit.normal).min(0.0);
                return self.age_by(dt, rules, floor_y);
            }
        }
        self.glances = 0;
        self.position += delta;
        self.orientation = UnitQuaternion::from_scaled_axis(self.spin * dt) * self.orientation;
        self.age_by(dt, rules, floor_y)
    }

    fn age_by(&mut self, dt: f32, rules: &ScreeRules, floor_y: f32) -> Outcome {
        self.age += dt;
        if self.age > rules.lifetime || self.position.y < floor_y {
            Outcome::Expired
        } else {
            Outcome::Flying
        }
    }
}

/// The extreme vertex along each of 26 directions: the faces, edges and
/// corners of a cube.
fn support_points(vertices: impl IntoIterator<Item = Vector3<f32>>) -> Vec<Vector3<f32>> {
    let directions: Vec<Vector3<f32>> = (0..27)
        .filter(|&k| k != 13)
        .map(|k| {
            Vector3::new(
                (k / 9) as f32 - 1.0,
                ((k / 3) % 3) as f32 - 1.0,
                (k % 3) as f32 - 1.0,
            )
        })
        .collect();
    let mut best: Vec<Option<(f32, Vector3<f32>)>> = vec![None; directions.len()];
    for v in vertices {
        for (slot, d) in best.iter_mut().zip(&directions) {
            let along = v.dot(d);
            if slot.is_none_or(|(b, _)| along > b) {
                *slot = Some((along, v));
            }
        }
    }
    let mut points: Vec<Vector3<f32>> = best.into_iter().flatten().map(|(_, v)| v).collect();
    points.sort_by(|a, b| a.as_slice().partial_cmp(b.as_slice()).unwrap());
    points.dedup();
    points
}

/// A piece of terrain falling as scree.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct FallingScree {
    pub flight: Flight,
    /// How big a crumble it makes where it lands.
    pub size: CrumbleSize,
}

/// Flies every piece of scree, and crumbles the ones that land.
#[derive(Default)]
pub struct ScreeSystem {
    rules: ScreeRules,
    crumble: Crumble,
}

impl<'a> System<'a> for ScreeSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, LazyUpdate>,
        WriteStorage<'a, FallingScree>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
    );

    fn run(
        &mut self,
        (entities, time, terrain, lazy, mut scree, mut positions, mut orientations): Self::SystemData,
    ) {
        let Some(terrain) = terrain else {
            return;
        };
        let dt = time.delta_seconds();
        let floor_y = terrain.bounds().min.y - self.rules.floor_margin;
        for (entity, piece) in (&entities, &mut scree).join() {
            match piece.flight.step(dt, &self.rules, &*terrain, floor_y) {
                Outcome::Flying => {
                    if let Some(position) = positions.get_mut(entity) {
                        position.0 = piece.flight.position.coords;
                    }
                    if let Some(orientation) = orientations.get_mut(entity) {
                        orientation.0 = piece.flight.orientation;
                    }
                }
                Outcome::Landed(at) => {
                    lazy.create_entity(&entities)
                        .with(Position(at.coords))
                        .with(self.crumble.emitter(piece.size))
                        .build();
                    let _ = entities.delete(entity);
                }
                Outcome::Expired => {
                    let _ = entities.delete(entity);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensing::ProbeHit;

    /// Flat ground at a height, infinite.
    struct Floor(f32);

    impl ProbeTarget for Floor {
        fn raycast(
            &self,
            origin: Point3<f32>,
            direction: Vector3<f32>,
            length: f32,
        ) -> Option<ProbeHit> {
            if direction.y >= 0.0 || origin.y < self.0 {
                return None;
            }
            let t = (origin.y - self.0) / -direction.y;
            (t <= length).then(|| ProbeHit {
                t: t / length,
                point: origin + direction * t,
                normal: Vector3::y(),
            })
        }
    }

    /// A cube of half-side `h`, as the corners of a mesh.
    fn cube(h: f32) -> Vec<Vector3<f32>> {
        (0..8)
            .map(|k| {
                Vector3::new(
                    if k & 1 == 0 { -h } else { h },
                    if k & 2 == 0 { -h } else { h },
                    if k & 4 == 0 { -h } else { h },
                )
            })
            .collect()
    }

    fn fly(flight: &mut Flight, ground: &Floor) -> (Outcome, usize) {
        let rules = ScreeRules::default();
        for frame in 1..1000 {
            let outcome = flight.step(1.0 / 60.0, &rules, ground, -100.0);
            if outcome != Outcome::Flying {
                return (outcome, frame);
            }
        }
        panic!("still flying after 1000 frames");
    }

    /// Dropped from 4.5 m with its underside half a metre below its centre,
    /// it lands when the underside meets the floor: after falling 4 m, about
    /// 0.9 s.
    #[test]
    fn a_piece_lands_when_its_underside_reaches_the_ground() {
        let mut flight = Flight::new(
            Point3::new(0.0, 4.5, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            cube(0.5),
        );
        let (outcome, frame) = fly(&mut flight, &Floor(0.0));
        assert!(
            matches!(outcome, Outcome::Landed(p) if p.y == 0.0),
            "{outcome:?}"
        );
        let seconds = frame as f32 / 60.0;
        assert!(
            (seconds - (2.0 * 4.0 / 9.81f32).sqrt()).abs() < 0.05,
            "{seconds} s"
        );
    }

    /// Thrown down fast enough to cross the floor between two frames, it
    /// still lands on it.
    #[test]
    fn a_fast_piece_does_not_pass_through_the_ground() {
        let mut flight = Flight::new(
            Point3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, -300.0, 0.0),
            Vector3::zeros(),
            cube(0.1),
        );
        let (outcome, frame) = fly(&mut flight, &Floor(0.0));
        assert!(matches!(outcome, Outcome::Landed(_)));
        assert_eq!(frame, 1);
    }

    /// With nothing below it, it is given up on once its time is out.
    #[test]
    fn a_piece_with_nothing_below_expires() {
        let mut flight = Flight::new(
            Point3::new(0.0, 1.0, 0.0),
            Vector3::zeros(),
            Vector3::zeros(),
            cube(0.1),
        );
        let (outcome, frame) = fly(&mut flight, &Floor(-1.0e6));
        assert_eq!(outcome, Outcome::Expired);
        let seconds = frame as f32 / 60.0;
        assert!(
            (seconds - ScreeRules::default().lifetime).abs() < 0.05,
            "{seconds} s"
        );
    }

    /// Flat ground at a height, and a ceiling over all of it at another.
    struct Ceiling {
        floor: f32,
        ceiling: f32,
    }

    impl ProbeTarget for Ceiling {
        fn raycast(
            &self,
            origin: Point3<f32>,
            direction: Vector3<f32>,
            length: f32,
        ) -> Option<ProbeHit> {
            if direction.y > 0.0 {
                let t = (self.ceiling - origin.y) / direction.y;
                return (t <= length).then(|| ProbeHit {
                    t: t / length,
                    point: origin + direction * t,
                    normal: -Vector3::y(),
                });
            }
            Floor(self.floor).raycast(origin, direction, length)
        }
    }

    /// Thrown up against a ceiling just over it, it glances off and lands on
    /// the floor below rather than crumbling against the ceiling.
    #[test]
    fn a_piece_thrown_into_a_ceiling_falls_to_the_floor() {
        let mut flight = Flight::new(
            Point3::new(0.0, 2.0, 0.0),
            Vector3::new(1.0, 3.0, 0.0),
            Vector3::zeros(),
            cube(0.25),
        );
        let ground = Ceiling {
            floor: 0.0,
            ceiling: 2.3,
        };
        let rules = ScreeRules::default();
        let mut outcome = Outcome::Flying;
        for _ in 0..1000 {
            outcome = flight.step(1.0 / 60.0, &rules, &ground, -100.0);
            if outcome != Outcome::Flying {
                break;
            }
        }
        assert!(
            matches!(outcome, Outcome::Landed(p) if p.y == 0.0),
            "{outcome:?}"
        );
        assert!(flight.position.x > 0.0, "it kept its sideways motion");
    }

    /// A slab turned on edge reaches further down than one lying flat, so it
    /// lands sooner.
    #[test]
    fn how_far_a_piece_reaches_turns_with_it() {
        let slab: Vec<Vector3<f32>> = cube(1.0)
            .into_iter()
            .map(|v| Vector3::new(v.x * 2.0, v.y * 0.25, v.z))
            .collect();
        let mut flight = Flight::new(Point3::origin(), Vector3::zeros(), Vector3::zeros(), slab);
        let down = -Vector3::y();
        assert!((flight.reach(down) - 0.25).abs() < 1e-5);
        flight.orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        assert!((flight.reach(down) - 2.0).abs() < 1e-5);
    }
}
