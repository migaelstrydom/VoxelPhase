//! Roller — a great thorned seed pod that hunts by rolling.
//!
//! The first creature, chosen because it needs no skeleton. It reuses the
//! brain, perception and steering layers wholesale and adds only a rolling
//! body.
//!
//! **A roller cannot be killed.** It has no `Health` and no `Flammable`, so
//! nothing in `crate::damage` touches it. You deal with one by outrunning it or
//! by blowing a hole in the ground and stranding it — and since explosion
//! knockback is applied to `Velocity` independently of damage, a grenade is
//! still the tool for the job, just as a way to *move* the creature rather than
//! to destroy it.
//!
//! That is a deliberate design choice, not an unfinished one. It play-tests
//! better than the alternative: trapping a roller uses the terrain destruction
//! the game is built around, where shooting it until it pops uses none of it.
//! It also avoids committing to a death model — corpses, ragdolls, respawns —
//! before there is a reason to prefer one.
//!
//! The consequence to keep in mind is that rollers are permanent. Nothing
//! despawns them, so a level's population only ever grows.
//!
//! It tumbles, so it has no front and no top, and is sculpted to be looked
//! at from every side (`visual_bench roller`):
//!
//! ```text
//!   husk     dry, fibrous leather in lobes from stem to crown, a rib     (Husk)
//!            along each and a boss raised under every thorn; split
//!            along one seam, its lips curling back from a bed of pith
//!   seeds    seated in the pith in rows, flattened, wet and red          (Wet)
//!   thorns   horn, one to a boss, their tips on the sphere it rolls as;  (Horn)
//!            some short, some broken to stumps
//!   stem     dried, with a curl of dead leaf; a withered crown opposite  (Husk)
//! ```

use std::f32::consts::{PI, TAU};
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::sculpted::Exhibit;
use crate::app::sculpt::{
    scatter, Clay, Ellipsoid, Lump, Mottle, Relief, Spots, Strand, VertexPaint,
};
use crate::app::spawnables::MaterialCtx;
use crate::app::spawnables::Spawnable;
use crate::character::CharacterIntent;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{AlertTelegraph, Brain, Lunge, Perception, Roller};
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_perlin_3d;

/// The name its look is kept under.
const KIND: &str = "roller";

/// The radius it is sculpted at, m: a roller of another size is this one
/// scaled.
const SCULPTED_RADIUS: f32 = 0.6;
/// The sculpture's own radius, for the lengths below.
const R: f32 = SCULPTED_RADIUS;

/// Lobes round the pod, from stem to crown.
const LOBES: u32 = 7;
/// Places a thorn is tried, set out by the golden angle; those in the
/// split, on the stem or on the crown are left out.
const THORN_PLACES: u32 = 170;
/// The turn between one thorn and the next that never lines two up.
const GOLDEN_ANGLE: f32 = 2.399_963;
/// How far the boss under a thorn stands out of the husk, m, and how thick
/// the thorn is where it leaves it.
const BOSS: f32 = 0.04;
const THORN_GIRTH: f32 = 0.021;
/// Side of the grid the husk is skinned from, m, and the seeds.
const HUSK_GRAIN: f32 = 0.009;
const SEED_GRAIN: f32 = 0.0042;
/// From one seed to the next along a row, and from row to row, m; and how
/// far under the husk's skin the pith they are seated in lies.
const SEED_PITCH: f32 = 0.0155;
const ROW_PITCH: f32 = 0.03;
const PITH_SUNK: f32 = 0.026;

/// The husk (linear): sallow leather, its lobes greener or browner, the
/// pith its split shows, and the dried crown.
const HUSK: (f32, f32, f32) = (0.2, 0.155, 0.05);
const HUSK_GREEN: (f32, f32, f32) = (0.13, 0.155, 0.045);
const HUSK_BROWN: (f32, f32, f32) = (0.16, 0.095, 0.04);
const PITH: (f32, f32, f32) = (0.56, 0.44, 0.24);
const DRIED: (f32, f32, f32) = (0.08, 0.055, 0.03);
/// A seed (linear), and its juice.
const SEED: (f32, f32, f32) = (0.27, 0.008, 0.018);
const JUICE: (f32, f32, f32) = (0.17, 0.008, 0.014);

/// Stone, so a roller is heavy enough to be a threat and to shrug off a
/// grazing blast.
const DENSITY: f32 = 2400.0;

/// Contact friction. High, because friction is what turns drive torque into
/// travel — a slick roller would spin in place. Shared with `Roller::new`,
/// which derives the traction limit on its drive torque from it.
const FRICTION: f32 = 1.2;

/// Its lunge. Between 6 and 9 m of what it hunts, unless rolling away from
/// it faster than 0.5 m/s (just past it, or backing off), it surges along
/// the line to it toward 7.5 m/s for 2.2 s, as fast as the ground's grip
/// lets it. On grass, about 4 m/s², a heavy hit needs that run-up: it
/// reaches 6.5 m/s or more by the time it strikes a body standing in its
/// way, which knocks it down; a body walking into it is struck at its own
/// speed as well, which kills. Stepping aside beats it. It rests 1 s
/// before it lunges again, so it comes again almost as soon as it has
/// rolled past.
const LUNGE: Lunge = Lunge {
    nearest: 6.0,
    range: 9.0,
    closing: -0.5,
    speed: 7.5,
    seconds: 2.2,
    rest: 1.0,
};

/// How far from what it hunts it circles once close, m. Its brain holds
/// that within 15% either way, which keeps it inside the band its lunge
/// surges from: it backs off there while it rests, and surges again with
/// the run-up a heavy hit needs.
const CIRCLING: f32 = 7.5;

/// How far it hears what it hunts, m: it sees only where it rolls, and
/// rolling past or circling it, it would otherwise lose what it is after.
/// Clear of the furthest it circles and surges from.
const HEARING: f32 = 11.0;

/// A rolling boulder creature.
///
/// Position is `(x, z)` — the spawn height is taken from the terrain, since a
/// creature authored at a hand-picked Y would either hang in the air or start
/// embedded once the terrain around it is reshaped.
#[derive(Deserialize)]
pub struct RollerDef {
    pub pos: (f32, f32),

    #[serde(default = "RollerDef::default_radius")]
    pub radius: f32,
    /// Flat-ground top speed in m/s. The player walks at 5.0, so the default
    /// is a shade slower — a roller should be escapable on the flat and
    /// terrifying downhill.
    #[serde(default = "RollerDef::default_speed")]
    pub speed: f32,
    /// Seconds to reach `speed` from rest on the flat. Lower is more
    /// aggressive; below roughly `speed / (friction * g)` the ground cannot
    /// transmit the extra torque and the value stops having an effect.
    #[serde(default = "RollerDef::default_spin_up_time")]
    pub spin_up_time: f32,
    /// How far it can see. Hearing range is close and fixed.
    #[serde(default = "RollerDef::default_sight_range")]
    pub sight_range: f32,
}

/// Rises smoothly from 0 to 1 as `t` does.
fn smooth(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn vector(v: (f32, f32, f32)) -> Vector3<f32> {
    Vector3::new(v.0, v.1, v.2)
}

fn tuple(v: Vector3<f32>) -> (f32, f32, f32) {
    (v.x, v.y, v.z)
}

/// The way out from the pod's middle `round` radians round its stem from
/// +Z toward +X, and `up` radians above its waist.
fn way(round: f32, up: f32) -> Vector3<f32> {
    Vector3::new(up.cos() * round.sin(), up.sin(), up.cos() * round.cos())
}

/// How far from the pod's middle `clay`'s skin is, the way of `toward`.
fn reach(clay: &Clay, toward: Vector3<f32>) -> f32 {
    let (mut inside, mut outside) = (0.0, R);
    for _ in 0..22 {
        let middle = 0.5 * (inside + outside);
        if clay.distance(toward * middle) < 0.0 {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    inside
}

fn empty(paint: Paint) -> MeshPrimitive {
    MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material: paint.id(),
    }
}

/// `mesh`'s triangles any of whose corners `keep` holds of, and the
/// vertices they use.
fn kept(mesh: &MeshPrimitive, keep: impl Fn(&Vertex) -> bool) -> MeshPrimitive {
    let mut some = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material: mesh.material,
    };
    let mut slots = vec![u32::MAX; mesh.vertices.len()];
    for triangle in mesh.indices.chunks_exact(3) {
        if !triangle
            .iter()
            .any(|index| keep(&mesh.vertices[*index as usize]))
        {
            continue;
        }
        for &index in triangle {
            let slot = &mut slots[index as usize];
            if *slot == u32::MAX {
                *slot = some.vertices.len() as u32;
                some.vertices.push(mesh.vertices[index as usize]);
            }
            some.indices.push(*slot);
        }
    }
    some
}

/// One place along the split.
#[derive(Clone, Copy)]
struct Parting {
    /// The way out from the pod's middle to it.
    out: Vector3<f32>,
    /// How far out the husk's skin was there before it split, m.
    skin: f32,
    /// Half the split's width there, m.
    half: f32,
}

impl Parting {
    /// The way across the split here, along the husk.
    fn across(&self) -> Vector3<f32> {
        Vector3::y().cross(&self.out).normalize()
    }

    /// A point `aside` metres across the split from its middle and `proud`
    /// metres out of the husk's skin.
    fn point(&self, aside: f32, proud: f32) -> Vector3<f32> {
        (self.out * self.skin + self.across() * aside).normalize() * (self.skin + proud)
    }
}

/// A thorn: the way it points, and how far out the husk's skin is under it.
#[derive(Clone, Copy)]
struct ThornPlace {
    out: Vector3<f32>,
    skin: f32,
}

/// The pod as its parts are set out on it: worked out once, for the husk,
/// the seeds and the thorns to agree on.
struct Pod {
    /// The pod's body, before anything is raised on it or cut out of it.
    body: Clay,
    /// The way round the stem of each lobe's crest, radians: where its ribs
    /// run.
    ribs: Vec<f32>,
    /// The split, from its end near the crown to its end near the stem.
    split: Vec<Parting>,
    /// What the split takes out of the husk, as a form: for asking how near
    /// the split something is.
    gash: Clay,
    thorns: Vec<ThornPlace>,
}

impl Pod {
    fn new() -> Self {
        let ribs: Vec<f32> = (0..LOBES)
            // A furrow between two lobes lies along +Z: where it splits.
            .map(|lobe| (lobe as f32 + 0.5 + 0.16 * scatter(lobe, 1)) / LOBES as f32 * TAU)
            .collect();
        let body = Self::body(&ribs);
        let split = Self::split(&body);
        let mut gash = Clay::new(0.0);
        for lump in Self::cuts(&split) {
            gash = gash.add(lump, PITH);
        }
        let thorns = (0..THORN_PLACES)
            .filter_map(|index| {
                let height = 1.0 - 2.0 * (index as f32 + 0.5) / THORN_PLACES as f32;
                let round = index as f32 * GOLDEN_ANGLE + 0.25 * scatter(index, 11);
                // Thorns grow along the ribs: each is drawn most of the way
                // to the nearest, which leaves the furrows bare.
                let rib = ribs
                    .iter()
                    .map(|rib| (round - rib + PI).rem_euclid(TAU) - PI)
                    .min_by(|a, b| a.abs().total_cmp(&b.abs()))
                    .unwrap_or(0.0);
                let round = round - 0.58 * rib;
                let up = (height + 0.03 * scatter(index, 12)).clamp(-1.0, 1.0).asin();
                let out = way(round, up);
                let skin = reach(&body, out);
                // None in the split, on the stem or on the crown.
                let clear = gash.distance(out * skin) > 0.045 && out.y < 0.95 && out.y > -0.955;
                clear.then_some(ThornPlace { out, skin })
            })
            .collect();
        Self {
            body,
            ribs,
            split,
            gash,
            thorns,
        }
    }

    /// The pod's body: a lobe from stem to crown for each rib, about a
    /// core, no two the same size.
    fn body(ribs: &[f32]) -> Clay {
        let mut clay = Clay::new(0.045).add(Lump::ball(0.335, (0.0, 0.0, 0.0)), HUSK);
        for (lobe, &round) in ribs.iter().enumerate() {
            let lobe = lobe as u32;
            let swell = 1.0 + 0.1 * scatter(lobe, 2);
            let out = way(round, 0.0);
            let colour = match lobe % 3 {
                0 => HUSK,
                1 => HUSK_GREEN,
                _ => HUSK_BROWN,
            };
            let shape = Ellipsoid::new(
                (0.155 * swell, 0.4 + 0.02 * scatter(lobe, 3), 0.25 * swell),
                tuple(out * 0.18 + Vector3::y() * 0.02 * scatter(lobe, 4)),
            )
            .yawed(round);
            clay = clay.add(Lump::Ball(shape), colour);
        }
        clay
    }

    /// Where the split runs: down the furrow along +Z, wandering, widest
    /// about the pod's waist.
    fn split(body: &Clay) -> Vec<Parting> {
        let places = 16;
        (0..=places)
            .map(|place| {
                let along = place as f32 / places as f32;
                let up = -0.98 + 1.8 * along;
                let round = 0.07 * (2.6 * up + 0.5).sin();
                let out = way(round, up);
                let wide = (1.0 - ((up + 0.08) / 0.92).powi(2)).max(0.0).powf(1.2);
                Parting {
                    out,
                    skin: reach(body, out),
                    half: (0.01 + 0.072 * wide) * (1.0 + 0.22 * scatter(place, 5)),
                }
            })
            .collect()
    }

    /// The lumps cut out of the husk to open the split.
    fn cuts(split: &[Parting]) -> Vec<Lump> {
        let sunk = |parting: &Parting| parting.point(0.0, -0.3 * parting.half);
        split
            .windows(2)
            .map(|pair| {
                Lump::limb(
                    tuple(sunk(&pair[0])),
                    tuple(sunk(&pair[1])),
                    (pair[0].half, pair[1].half),
                )
            })
            .collect()
    }

    /// The split where it passes nearest the way of `toward`.
    fn parting(&self, toward: Vector3<f32>) -> &Parting {
        let nearest = self
            .split
            .iter()
            .max_by(|a, b| a.out.dot(&toward).total_cmp(&b.out.dot(&toward)));
        nearest.expect("the split has places along it")
    }

    /// The husk: the body with a rib raised along each lobe's crest, its
    /// split's lips curled back, a boss under every thorn, a collar round
    /// the stem and a withered crown opposite. Its skin is dry and fibrous,
    /// drawn into wrinkles that run from stem to crown.
    fn husk(&self) -> Clay {
        let mut clay = self.body.clone().smoothing(0.025);
        for &round in &self.ribs {
            let steps = 12;
            let keel = |step: u32| {
                let up = -1.3 + 2.6 * step as f32 / steps as f32;
                let out = way(round + 0.05 * (2.3 * up + round).sin(), up);
                // Fading out toward the stem and the crown.
                let girth = 0.011 + 0.013 * up.cos().max(0.0).sqrt();
                (out * (reach(&self.body, out) + 0.003), girth)
            };
            for step in 0..steps {
                let ((from, thick), (to, thicker)) = (keel(step), keel(step + 1));
                clay = clay.add(Lump::limb(tuple(from), tuple(to), (thick, thicker)), HUSK);
            }
        }
        // The lips: a roll of rind along each side of the split.
        clay = clay.smoothing(0.03);
        for side in [-1.0, 1.0] {
            for pair in self.split.windows(2) {
                let lip = |parting: &Parting| {
                    (
                        parting.point(side * (parting.half + 0.01), 0.004),
                        0.007 + 0.14 * parting.half,
                    )
                };
                let ((from, thick), (to, thicker)) = (lip(&pair[0]), lip(&pair[1]));
                clay = clay.add(Lump::limb(tuple(from), tuple(to), (thick, thicker)), PITH);
            }
        }
        clay = clay.smoothing(0.032);
        for (index, thorn) in self.thorns.iter().enumerate() {
            let girth = 0.05 + 0.008 * scatter(index as u32, 13);
            clay = clay.add(
                Lump::limb(
                    tuple(thorn.out * (thorn.skin - 0.04)),
                    tuple(thorn.out * (thorn.skin + BOSS)),
                    (girth, 0.5 * girth),
                ),
                HUSK,
            );
        }
        // A collar where the stem leaves it,
        let top = reach(&self.body, Vector3::y());
        clay = clay.smoothing(0.02);
        for knob in 0..5 {
            let round = knob as f32 / 5.0 * TAU + 0.4;
            let at = Vector3::new(0.06 * round.sin(), top + 0.004, 0.06 * round.cos());
            clay = clay.add(
                Lump::ball(0.03 + 0.006 * scatter(knob, 14), tuple(at)),
                HUSK_BROWN,
            );
        }
        // and the crown the flower left, dried to six teeth round a pit.
        let foot = reach(&self.body, -Vector3::y());
        for tooth in 0..6 {
            let round = (tooth as f32 + 0.2 * scatter(tooth, 15)) / 6.0 * TAU;
            let aside = Vector3::new(round.sin(), 0.0, round.cos());
            let lean = 0.08 + 0.02 * scatter(tooth, 16);
            clay = clay.add(
                Lump::limb(
                    tuple(aside * 0.05 - Vector3::y() * (foot - 0.01)),
                    tuple(aside * lean - Vector3::y() * (foot + 0.075)),
                    (0.03, 0.007),
                ),
                DRIED,
            );
        }
        clay = clay
            .relief(Relief::Folds {
                along: Vector3::y(),
                frequency: 30.0,
                depth: 0.0045,
                seed: 20,
            })
            .relief(Relief::Scales {
                size: 0.034,
                depth: 0.003,
            })
            .relief(Relief::Lumps {
                frequency: 11.0,
                depth: 0.006,
                seed: 21,
            })
            .cut(Lump::ball(0.036, (0.0, -foot - 0.012, 0.0)), 0.01);
        for lump in Self::cuts(&self.split) {
            clay = clay.cut(lump, 0.012);
        }
        clay
    }

    /// The husk's skin, painted: russet on one side and dried brown at
    /// its ends, bruised, scabbed, scuffed on its bosses, dirt in its
    /// furrows, grass-stained, and flushed and run with juice at the split.
    fn husk_skin(&self, husk: &Clay) -> MeshPrimitive {
        let mut mesh = husk.skin(HUSK_GRAIN, Paint::Husk.id());
        Mottle {
            frequency: 7.0,
            value: 0.3,
            warmth: 0.25,
            seed: 22,
        }
        .paint(&mut mesh);
        // Bruises, scabs, and the green of what it has rolled through.
        Spots {
            frequency: 5.5,
            presence: 0.55,
            radius: (0.25, 0.5),
            tint: Vector3::new(0.42, 0.27, 0.3),
            salt: 23,
        }
        .paint(&mut mesh);
        Spots {
            frequency: 24.0,
            presence: 0.3,
            radius: (0.12, 0.3),
            tint: Vector3::new(0.45, 0.36, 0.28),
            salt: 25,
        }
        .paint(&mut mesh);
        Spots {
            frequency: 8.0,
            presence: 0.2,
            radius: (0.2, 0.45),
            tint: Vector3::new(0.6, 1.0, 0.45),
            salt: 24,
        }
        .paint(&mut mesh);
        // Pores.
        Spots {
            frequency: 64.0,
            presence: 0.45,
            radius: (0.25, 0.45),
            tint: Vector3::new(0.55, 0.47, 0.4),
            salt: 30,
        }
        .paint(&mut mesh);
        let (pith, juice) = (vector(PITH), vector(JUICE));
        let (dirt, russet, dry) = (
            Vector3::new(0.36, 0.28, 0.2),
            Vector3::new(0.24, 0.075, 0.04),
            Vector3::new(0.1, 0.065, 0.035),
        );
        let noise = |at: Vector3<f32>, seed: u32| fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, seed);
        for vertex in &mut mesh.vertices {
            let at = vertex.pos;
            let out = at.normalize();
            let mut colour = vertex.color.xyz();
            // Fibres, lighter and darker, from stem to crown.
            colour *= 1.0 + 0.4 * fbm_perlin_3d(at.x * 34.0, at.y * 3.5, at.z * 34.0, 2, 0.5, 31);
            // Reddened on the side the sun had, and dried at stem and crown.
            colour = colour.lerp(&russet, 0.65 * smooth(0.5 + 1.8 * noise(at * 1.7, 26)));
            let ends = smooth((out.y.abs() - 0.72 + 0.12 * noise(at * 5.0, 27)) / 0.25);
            colour = colour.lerp(&dry, 0.75 * ends);
            // Dirt where it is closed in, and its bosses rubbed pale.
            let closed = smooth((0.95 - vertex.ao) / 0.35);
            colour = colour.component_mul(&Vector3::repeat(1.0).lerp(&dirt, closed));
            let proud = at.norm() - reach(&self.body, out);
            let rubbed = smooth((proud - 0.006) / 0.03) * (1.0 - closed);
            colour = colour.lerp(&(pith * 0.6), 0.45 * rubbed);
            // Flushed toward the split, as a fruit is that is ready, and
            // run with what has come out of it.
            let near = self.gash.distance(at);
            let flush = smooth(1.0 - near / 0.14);
            colour = colour
                .component_mul(&Vector3::repeat(1.0).lerp(&Vector3::new(1.2, 0.5, 0.42), flush));
            let run = smooth(1.0 - near / 0.12) * smooth((noise(at * 9.0, 28) - 0.02) / 0.16);
            colour = colour.lerp(&(juice * 1.3), 0.8 * run);
            // Its rind pale where it has torn, and soaked red deep in.
            let torn = smooth(1.0 - (near - 0.002) / 0.012);
            let parting = self.parting(out);
            let deep = smooth((parting.skin - at.norm()) / (0.35 * parting.half + 0.006));
            let raw = Vector3::new(0.4, 0.2, 0.1) * (0.8 + 0.4 * noise(at * 30.0, 29));
            let rind = raw.lerp(&juice, deep);
            colour = colour.lerp(&rind, torn);
            vertex.color = Vector4::new(colour.x, colour.y, colour.z, 1.0);
        }
        mesh
    }

    /// What the split shows: a bed of pith, and the seeds seated in it in
    /// rows, each flattened against the next, with here and there one gone.
    fn seeds(&self, husk: &Clay) -> [MeshPrimitive; 2] {
        // The bed: rounded across, its top a little under the husk's skin.
        let bed = |parting: &Parting| (0.9 * parting.half).max(0.008);
        let mut pith = Clay::new(0.01);
        for pair in self.split.windows(2) {
            let sunk = |parting: &Parting| parting.point(0.0, -(bed(parting) + PITH_SUNK));
            pith = pith.add(
                Lump::limb(
                    tuple(sunk(&pair[0])),
                    tuple(sunk(&pair[1])),
                    (bed(&pair[0]), bed(&pair[1])),
                ),
                PITH,
            );
        }
        let pith = pith.relief(Relief::Lumps {
            frequency: 36.0,
            depth: 0.003,
            seed: 35,
        });

        let mut seeds = Clay::new(0.002);
        let mut key = 0;
        for pair in self.split.windows(2) {
            let run = (pair[1].point(0.0, 0.0) - pair[0].point(0.0, 0.0)).norm();
            let steps = (run / SEED_PITCH).round().max(1.0) as u32;
            for step in 0..steps {
                let t = (step as f32 + 0.5) / steps as f32;
                let parting = Parting {
                    out: pair[0].out.lerp(&pair[1].out, t).normalize(),
                    skin: pair[0].skin + (pair[1].skin - pair[0].skin) * t,
                    half: pair[0].half + (pair[1].half - pair[0].half) * t,
                };
                // As many rows as the bed has room for, spread to fill it.
                let round = bed(&parting);
                let room = (round - 0.01).max(0.0);
                let rows = (room / (0.8 * ROW_PITCH)).floor() as i32;
                let spread = if rows > 0 {
                    room / rows as f32
                } else {
                    ROW_PITCH
                };
                // Each seed the broader where the rows lie further apart.
                let broad = (spread / ROW_PITCH).clamp(0.85, 1.45);
                for row in -rows..=rows {
                    key += 1;
                    if scatter(key, 30) > 0.88 {
                        continue;
                    }
                    let aside = row as f32 * spread + 0.002 * scatter(key, 31);
                    let seat = (round * round - aside * aside).max(0.0).sqrt() - round - PITH_SUNK;
                    let at = parting.point(aside, seat + 0.004 + 0.003 * scatter(key, 32));
                    // On end in its seat and leaning a little: broad across
                    // the row, and flat where it is pressed to the next.
                    let leaning = at.normalize()
                        + Vector3::new(scatter(key, 35), scatter(key, 36), scatter(key, 37)) * 0.18;
                    let radial = leaning.normalize();
                    let across = Vector3::y().cross(&radial).normalize();
                    let size = 1.0 + 0.16 * scatter(key, 33);
                    let (thick, tall) = (0.0088 * size, 0.009 * size);
                    for half in [-1.0, 1.0] {
                        let foot = at + across * half * 0.0055 * broad * size;
                        seeds = seeds.add(
                            Lump::limb(
                                tuple(foot),
                                tuple(foot + radial * tall),
                                (thick, 0.85 * thick),
                            ),
                            SEED,
                        );
                    }
                }
            }
        }

        // The pith, less what is under the husk or faces into the pod.
        let mut bedding = kept(&pith.skin(0.007, Paint::Husk.id()), |vertex| {
            husk.distance(vertex.pos) > -0.004 && vertex.normal.dot(&vertex.pos.normalize()) > -0.2
        });
        let juice = vector(JUICE);
        for vertex in &mut bedding.vertices {
            // Soaked red round every seed, and paler between.
            let soaked = smooth(1.0 - seeds.distance(vertex.pos) / 0.016);
            let fibre = 0.8
                + 0.4
                    * fbm_perlin_3d(
                        vertex.pos.x * 60.0,
                        vertex.pos.y * 8.0,
                        vertex.pos.z * 60.0,
                        2,
                        0.5,
                        38,
                    );
            let colour = (vertex.color.xyz() * 0.8 * fibre).lerp(&(juice * 1.6), 0.9 * soaked);
            vertex.color = Vector4::new(colour.x, colour.y, colour.z, 1.0);
        }

        // The seeds, less what of each is sunk in the pith.
        let mut seated = kept(&seeds.skin(SEED_GRAIN, Paint::Wet.id()), |vertex| {
            pith.distance(vertex.pos) > -0.003
        });
        // Each one near black where it is pressed against the next, and
        // garnet on its crown, where the light gets into it.
        for vertex in &mut seated.vertices {
            let crown = smooth((vertex.normal.dot(&vertex.pos.normalize()) - 0.25) / 0.7);
            let pressed = smooth((0.92 - vertex.ao) / 0.4);
            let glow = Vector3::new(1.0 + 0.7 * crown, 1.0 + 1.8 * crown, 1.0 + 0.8 * crown);
            let colour = vertex.color.xyz().component_mul(&glow) * (1.0 - 0.75 * pressed);
            vertex.color = Vector4::new(colour.x, colour.y, colour.z, 1.0);
        }
        Mottle {
            frequency: 26.0,
            value: 0.35,
            warmth: 0.15,
            seed: 34,
        }
        .paint(&mut seated);
        [bedding, seated]
    }

    /// Its thorns: horn, dark at the root and pale at the tip, hooked a
    /// little, some short and some broken to stumps.
    fn horn(&self) -> MeshPrimitive {
        let mut mesh = empty(Paint::Horn);
        let (root, shaft, point) = (
            Vector3::new(0.07, 0.04, 0.025),
            Vector3::new(0.33, 0.2, 0.08),
            Vector3::new(0.82, 0.7, 0.48),
        );
        for (index, thorn) in self.thorns.iter().enumerate() {
            let key = index as u32;
            // Most reach the sphere it rolls as; some stop short of it.
            let lot = 0.5 + 0.5 * scatter(key, 41);
            let tip_out = if lot < 0.74 {
                R
            } else {
                R * (0.86 + 0.11 * (0.5 + 0.5 * scatter(key, 42)))
            };
            let broken = (lot > 0.9).then(|| 0.3 + 0.3 * (0.5 + 0.5 * scatter(key, 43)));
            // Hooked the way it rolls, more or less.
            let round = Vector3::y().cross(&thorn.out);
            let hook = (round.try_normalize(1e-3).unwrap_or_else(Vector3::x)
                + Vector3::new(scatter(key, 44), scatter(key, 45), scatter(key, 46)) * 0.7)
                * 0.03;
            let foot = thorn.out * (thorn.skin + BOSS - 0.016);
            let tip = (thorn.out * tip_out + hook).normalize() * tip_out;
            let bend = foot + (tip - foot) * 0.5 - hook * 0.35;
            let girth = THORN_GIRTH * (1.0 + 0.18 * scatter(key, 47));
            let mut path = vec![(foot, girth), (bend, 0.48 * girth), (tip, 0.0)];
            if let Some(left) = broken {
                // Snapped: what is left of it, ending ragged.
                let end = foot + (bend - foot) * 2.0 * left;
                let thick = girth * (1.0 - 0.52 * 2.0 * left);
                path = vec![(foot, girth), (end, thick)];
            }
            Strand::through(path)
                .sides(8)
                .rings(3)
                .lay(&mut mesh, |along| {
                    let colour = if broken.is_some() {
                        root.lerp(&shaft, along)
                    } else if along < 0.45 {
                        root.lerp(&shaft, along / 0.45)
                    } else {
                        shaft.lerp(&point, (along - 0.45) / 0.55)
                    };
                    (colour, 0.45 + 0.55 * smooth(along * 3.0))
                });
        }
        // Horn is streaked along its grain.
        Mottle {
            frequency: 55.0,
            value: 0.2,
            warmth: 0.12,
            seed: 48,
        }
        .paint(&mut mesh);
        mesh
    }

    /// Its stem, dried and snapped short, and a dead leaf curled on it.
    fn stem(&self) -> MeshPrimitive {
        let mut mesh = empty(Paint::Husk);
        let top = reach(&self.body, Vector3::y());
        let at = |x: f32, up: f32, z: f32| Vector3::new(x, top + up, z);
        // Gnarled: thick at each joint it grew by, and kinked there.
        let path = vec![
            (at(0.0, -0.03, 0.0), 0.03),
            (at(0.004, 0.022, 0.002), 0.019),
            (at(0.014, 0.05, 0.012), 0.023),
            (at(0.026, 0.078, 0.008), 0.015),
            (at(0.052, 0.102, 0.022), 0.019),
            (at(0.078, 0.112, 0.05), 0.013),
            (at(0.11, 0.128, 0.058), 0.015),
        ];
        let (bark, snapped) = (
            Vector3::new(0.12, 0.085, 0.05),
            Vector3::new(0.42, 0.33, 0.2),
        );
        Strand::through(path)
            .sides(9)
            .rings(3)
            .lay(&mut mesh, |along| {
                let ringed = 1.0 + 0.3 * (along * 52.0).sin();
                let colour = if along > 0.995 {
                    snapped
                } else {
                    bark * ringed
                };
                (colour, 0.5 + 0.5 * smooth(along * 2.5))
            });
        Leaf {
            root: at(0.01, 0.012, -0.018),
            along: Vector3::new(-0.5, 0.0, -0.86).normalize(),
            length: 0.3,
            width: 0.16,
        }
        .lay(&mut mesh);
        mesh
    }

    /// Threads of pith still holding across the split, and one that has
    /// given way.
    fn threads(&self) -> MeshPrimitive {
        let mut mesh = empty(Paint::Husk);
        let pith = vector(PITH) * 0.8;
        for (thread, place) in [5usize, 8, 9, 12].into_iter().enumerate() {
            let key = thread as u32;
            let parting = &self.split[place];
            let slant = 0.045 * scatter(key, 51);
            let end = |side: f32| {
                parting.point(side * (parting.half + 0.004), -0.004) + Vector3::y() * side * slant
            };
            let sag = parting.point(0.02 * scatter(key, 52), 0.002 + 0.008 * scatter(key, 53));
            let thick = 0.0022 + 0.0008 * scatter(key, 54);
            let path = if thread == 2 {
                // Snapped: what is left of it lies back on the seeds.
                vec![
                    (end(-1.0), 1.6 * thick),
                    (sag - Vector3::y() * 0.02, 0.5 * thick),
                ]
            } else {
                vec![
                    (end(-1.0), 1.6 * thick),
                    (sag, 0.5 * thick),
                    (end(1.0), 1.5 * thick),
                ]
            };
            Strand::through(path)
                .sides(5)
                .rings(4)
                .lay(&mut mesh, |along| {
                    let taut = 1.0 - (2.0 * along - 1.0).abs();
                    (pith * (0.8 + 0.4 * taut), 1.0)
                });
        }
        mesh
    }
}

/// A dead leaf dried into a curl: twisted, crinkled, its edges rolled in
/// over its midrib and rolled tightest at its tip.
struct Leaf {
    /// Where its stalk is.
    root: Vector3<f32>,
    /// The way it runs from there, level.
    along: Vector3<f32>,
    /// Its length and its width at its widest, m.
    length: f32,
    width: f32,
}

impl Leaf {
    /// Ribs along the leaf and across it.
    const ROWS: u32 = 30;
    const COLUMNS: u32 = 12;
    /// Half how thick it is, m.
    const HALF_THICK: f32 = 0.0012;

    /// Lay both its faces onto `mesh`.
    fn lay(&self, mesh: &mut MeshPrimitive) {
        let level = self.along.cross(&Vector3::y()).normalize();
        let (blade, scorched, vein) = (
            Vector3::new(0.17, 0.085, 0.028),
            Vector3::new(0.05, 0.03, 0.015),
            Vector3::new(0.38, 0.25, 0.1),
        );
        for face in [1.0f32, -1.0] {
            let first = mesh.vertices.len() as u32;
            for row in 0..=Self::ROWS {
                let u = row as f32 / Self::ROWS as f32;
                // Its midrib arches off the pod, clear of the bosses, and
                // its tip turns up again.
                let arch = 0.3 * (PI * u.powf(0.8)).sin() + 0.1 * u * u * u;
                let midrib = self.root
                    + self.along * self.length * u * (1.0 - 0.12 * u)
                    + Vector3::y() * self.length * arch;
                // Twisted along its length as it dried.
                let twist = 0.55 * (3.1 * u + 0.4).sin();
                let side = level * twist.cos() + Vector3::y() * twist.sin();
                let up = Vector3::y() * twist.cos() - level * twist.sin();
                // Broad near its stalk, drawn out to a point, its edge
                // toothed and torn.
                let half = 0.5
                    * self.width
                    * (PI * u.powf(0.62)).sin().max(0.0).powf(0.8)
                    * (1.0 + 0.07 * (u * 75.0).sin() + 0.1 * (u * 19.0 + 1.0).sin());
                let curl = 0.5 + 2.0 * u * u;
                let bow = half.max(1e-5) / curl;
                for column in 0..=Self::COLUMNS {
                    let v = column as f32 / Self::COLUMNS as f32 * 2.0 - 1.0;
                    let turned = v * curl;
                    let normal = up * turned.cos() - side * turned.sin();
                    let crinkle = 0.004 * (v * 8.0 + u * 27.0).sin() * (u * 16.0).cos();
                    let pos = midrib
                        + side * bow * turned.sin()
                        + up * bow * (1.0 - turned.cos())
                        + normal * (crinkle + face * Self::HALF_THICK);
                    // Veins off the midrib toward its edge, and scorched
                    // where it is thinnest.
                    let ribbed = ((v.abs() * 1.3 - u * 2.2) * 15.0).sin().abs();
                    let veined = smooth(1.0 - ribbed / 0.3).max(smooth(1.0 - v.abs() * 7.0));
                    let patchy = fbm_perlin_3d(u * 6.0, v * 3.0, 0.0, 3, 0.5, 61);
                    let mut colour = blade * (1.0 + 0.6 * patchy);
                    colour = colour.lerp(&scorched, smooth(v.abs() * 1.5 - 0.55));
                    colour = colour.lerp(&vein, 0.6 * veined);
                    mesh.vertices.push(Vertex {
                        pos,
                        color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
                        tex_coords: Vector2::zeros(),
                        normal: normal * face,
                        ao: if face > 0.0 { 0.9 } else { 0.6 },
                    });
                }
            }
            let across = Self::COLUMNS + 1;
            for row in 0..Self::ROWS {
                for column in 0..Self::COLUMNS {
                    let a = first + row * across + column;
                    let (b, c, d) = (a + 1, a + across + 1, a + across);
                    if face > 0.0 {
                        mesh.indices.extend([a, b, d, b, c, d]);
                    } else {
                        mesh.indices.extend([a, d, b, b, d, c]);
                    }
                }
            }
        }
    }
}

impl RollerDef {
    pub fn default_radius() -> f32 {
        SCULPTED_RADIUS
    }
    pub fn default_speed() -> f32 {
        4.5
    }
    pub fn default_spin_up_time() -> f32 {
        0.7
    }
    pub fn default_sight_range() -> f32 {
        25.0
    }

    /// Its look against the palette's slots, at [`SCULPTED_RADIUS`]: one
    /// rigid part, about the middle of the sphere it rolls as.
    pub fn sculpt() -> Model {
        let pod = Pod::new();
        let husk = pod.husk();
        let [pith, seeds] = pod.seeds(&husk);
        Model::flat(vec![ModelPart::new(vec![
            pod.husk_skin(&husk),
            pith,
            seeds,
            pod.horn(),
            pod.stem(),
            pod.threads(),
        ])])
    }

    /// `sculpted` as a roller of `radius`.
    fn sized(sculpted: &Model, radius: f32) -> Model {
        let scale = radius / SCULPTED_RADIUS;
        let mut model = sculpted.clone();
        let meshes = model.parts.iter_mut().flat_map(|part| &mut part.primitives);
        for vertex in meshes.flat_map(|mesh| &mut mesh.vertices) {
            vertex.pos *= scale;
        }
        model
    }

    /// How this roller looks in a level whose bestiary materials are
    /// `materials`. Every roller of the sculpted size shares one model.
    fn look(&self, materials: &[MaterialId]) -> Arc<Model> {
        let sculpted = likeness::model(KIND, materials, Self::sculpt);
        if (self.radius - SCULPTED_RADIUS).abs() < 1e-4 {
            sculpted
        } else {
            Arc::new(Self::sized(&sculpted, self.radius))
        }
    }

    /// Mass of the sphere, used to size the drive torque.
    fn mass(&self) -> f32 {
        (4.0 / 3.0) * PI * self.radius.powi(3) * DENSITY
    }

    /// The states it is shown in by tools: it has only the one, so it is
    /// shown as it comes to rest on each of its sides.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        let turned = |name, pitch: f32, yaw: f32| Exhibit {
            name,
            model: model.clone(),
            poses: vec![Transform {
                rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw)
                    * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch),
                ..Transform::default()
            }],
            shaped: Vec::new(),
            lift: SCULPTED_RADIUS,
            reach: 0.62,
        };
        vec![
            turned("portrait", 0.25, 0.45),
            turned("stem", 1.25, 0.9),
            turned("crown", -1.9, 0.6),
            turned("flank", 0.4, 3.6),
        ]
    }
}

impl Spawnable for RollerDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        // No terrain beneath the authored position means the level moved and
        // this creature's spot went with it. Dropping it silently beats
        // spawning something that falls forever.
        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let initial_pos = Point3::new(self.pos.0, surface_y + self.radius, self.pos.1);

        let model = self.look(materials);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(initial_pos)
                    .gravity_scale(1.0)
                    // Low linear damping so momentum carries downhill, but
                    // enough angular damping that a roller which loses its
                    // target coasts to a stop instead of spinning forever.
                    .linear_damping(0.05)
                    .angular_damping(0.4),
            );
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::sphere(self.radius)
                    .density(DENSITY)
                    .friction(FRICTION)
                    .restitution(0.1),
            );
            body_handle
        };

        // It circles what it hunts at `CIRCLING`, always the same way round,
        // and attacks by its lunge: a boulder that turned back every second
        // and a half would read as one that cannot make up its mind.
        // Relentless because a roller has no health to break off over —
        // `next_behaviour` would reach the same conclusion from a missing
        // `Health`, but saying it here means the intent survives someone later
        // giving rollers health back.
        let brain = Brain {
            strafe_interval: f32::INFINITY,
            ..Brain::hunter(CIRCLING).relentless()
        };

        vec![world
            .create_entity()
            .with(Position(initial_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            // Intent is the seam: `BrainSystem` writes it, `RollerLocomotionSystem`
            // reads it. Neither knows about the other.
            .with(CharacterIntent::default())
            .with(Perception {
                hearing_range: HEARING,
                ..Perception::ground_creature(self.sight_range)
            })
            // A sphere has no face to pull, so the alert beat is expressed
            // as a hop and a shiver instead.
            .with(AlertTelegraph::ground_creature())
            .with(brain)
            .with(
                Roller::new(
                    self.radius,
                    self.mass(),
                    self.speed,
                    self.spin_up_time,
                    FRICTION,
                )
                .with_lunge(LUNGE),
            )
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// How far from the model's middle its furthest vertex in `paint` is.
    fn furthest(model: &Model, paint: Paint) -> f32 {
        let meshes = model.parts.iter().flat_map(|part| &part.primitives);
        meshes
            .filter(|mesh| mesh.material == paint.id())
            .flat_map(|mesh| &mesh.vertices)
            .map(|vertex| vertex.pos.norm())
            .fold(0.0, f32::max)
    }

    #[test]
    fn it_is_tens_of_thousands_of_triangles_and_rolls_on_the_tips_of_its_thorns() {
        let model = RollerDef::sculpt();
        let mut triangles = 0;
        for mesh in model.parts.iter().flat_map(|part| &part.primitives) {
            println!(
                "  {:?}: {} triangles",
                mesh.material,
                mesh.indices.len() / 3
            );
            triangles += mesh.indices.len() / 3;
            let count = mesh.vertices.len() as u32;
            assert!(mesh.indices.iter().all(|index| *index < count));
            for vertex in &mesh.vertices {
                assert!(vertex.pos.iter().all(|x| x.is_finite()));
                // Nothing of it stands out of the sphere it collides as.
                assert!(
                    vertex.pos.norm() < SCULPTED_RADIUS + 1e-3,
                    "part of it is {} m from its middle",
                    vertex.pos.norm()
                );
            }
        }
        println!("a roller is {triangles} triangles");
        assert!(
            (30_000..200_000).contains(&triangles),
            "{triangles} triangles"
        );
        let tips = furthest(&model, Paint::Horn);
        assert!(
            (SCULPTED_RADIUS - tips).abs() < 1e-3,
            "its thorns reach {tips} m"
        );
    }

    #[test]
    fn most_thorns_reach_the_sphere_and_they_stand_all_round_it() {
        let pod = Pod::new();
        assert!(pod.thorns.len() > 100, "{} thorns", pod.thorns.len());
        // Whichever way is down, a thorn points near enough to it.
        for index in 0..400u32 {
            let down = way(
                index as f32 * GOLDEN_ANGLE,
                (1.0 - index as f32 / 200.0).asin(),
            );
            let nearest = pod
                .thorns
                .iter()
                .map(|thorn| thorn.out.dot(&down))
                .fold(-1.0, f32::max);
            assert!(
                nearest > 0.75,
                "no thorn within {} of a side",
                nearest.acos()
            );
        }
    }

    #[test]
    fn a_roller_of_another_size_is_the_same_pod_scaled() {
        let sculpted = RollerDef::sculpt();
        let big = RollerDef::sized(&sculpted, 0.9);
        assert!((furthest(&big, Paint::Horn) - 0.9).abs() < 2e-3);
        assert_eq!(big.parts.len(), sculpted.parts.len());
    }
}
