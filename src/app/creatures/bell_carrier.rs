//! Bell-carrier — a small naked thing under a great bronze bell. It holds
//! the bell up off its own back by the rim, and looks out through the hole
//! where the casting broke.
//!
//! The armoured one. Nothing hurts it: a blast rings the bell and shoves it,
//! and it comes on. It is heavy and slow, so it is dealt with by where it is
//! made to walk: off a ledge, into a pit, into deep water. `visual_bench
//! bell_carrier` shows it.
//!
//! ```text
//!   bell     cast bronze, hollow: crown and loop, shoulder, waist, sound
//!            bow and lip; cast bands and a worn inscription; dented,
//!            cracked from the lip, a piece broken out of it       (Head)
//!   within   a face pressed to the break, an eye that follows you and
//!            its lids; eight fingers hooked under the rim         (Head, Eye, Hinge)
//!   clapper  iron, hung from the crown, swinging between the knees (Hinge)
//!   legs     bare thighs, shins and feet, filthy and bruised      (the bones)
//! ```

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use rustc_hash::FxHashMap;
use serde::Deserialize;
use specs::{Entity, World};

use super::anatomy::{self, EyeLook};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{bare_legs, Peck, StiltWalker};
use crate::animation::stilts::{Carriage, Dress, Drive, Joint, StiltRigConfig, Worn};
use crate::animation::{FootPlacerConfig, FootSide};
use crate::app::sculpt::{scatter, Blush, Clay, Grime, Lump, Mottle, Relief, VertexPaint};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

/// The name its look is kept under.
const KIND: &str = "bell_carrier";

/// Bronze: it sinks like one, and a blast barely moves it.
const DENSITY: f32 = 2600.0;
/// Rad/s².
const TURN_AUTHORITY: f32 = 260.0;
/// Seconds of rocking back before it rams, and before it can again.
const RAM_WINDUP: f32 = 0.8;
const RAM_RECOVERY: f32 = 1.2;

/// The bell's half-width at its mouth, m.
const BELL_RADIUS: f32 = 0.42;
/// How far below the head's middle the bell's lip hangs, m: at its
/// carrier's knees.
const LIP_DROP: f32 = 0.56;
/// How thick the bell's wall is, m.
const WALL: f32 = 0.03;
/// Side of the grid the bell is skinned from, m.
const BELL_GRAIN: f32 = 0.0115;
/// How far the bell lists to one side, radians: it is not held level.
const LIST: f32 = 0.045;

/// Bronze where it is rubbed bright, where it has only aged, and where it
/// has gone black under its crust; and the dark of the bell's inside
/// (linear). On a metal these tint what it mirrors.
const BRIGHT: Vector3<f32> = Vector3::new(0.95, 0.66, 0.33);
const AGED: Vector3<f32> = Vector3::new(0.42, 0.26, 0.12);
const BLACKENED: Vector3<f32> = Vector3::new(0.1, 0.075, 0.05);
const SOOT: Vector3<f32> = Vector3::new(0.035, 0.028, 0.022);
/// Verdigris, the crust that grows on bronze: where it is thick and pale,
/// and where it is thin and dark (linear).
const VERDIGRIS: Vector3<f32> = Vector3::new(0.2, 0.38, 0.3);
const VERDIGRIS_THIN: Vector3<f32> = Vector3::new(0.045, 0.085, 0.065);
/// Iron gone black (linear).
const IRON: (f32, f32, f32) = (0.085, 0.075, 0.07);

/// What is cast round the bell's waist: "I call the living, I mourn the
/// dead."
const INSCRIPTION: &str = "+VIVOS VOCO+MORTVOS PLANGO";
/// The heights the inscription stands between, about the head's middle, m.
const LETTERS: (f32, f32) = (-0.3, -0.205);
/// How thick a letter's stroke is cast, m, and how far its middle is sunk
/// under the bell's surface.
const STROKE: f32 = 0.0085;
const STROKE_SUNK: f32 = 0.0035;

/// Where the piece is broken out of the bell: how far round from straight
/// ahead, radians, and how high, m.
const BREAK: (f32, f32) = (0.06, -0.02);
/// Radius of the eye that looks out of it, m, and how far behind the bell's
/// wall its middle sits.
const EYE_RADIUS: f32 = 0.045;
const EYE_SET_BACK: f32 = 0.07;
/// Skin that lives in the dark (linear).
const PALLOR: (f32, f32, f32) = (0.25, 0.16, 0.14);

/// How long the hands that hold the rim are, m; how far each joint of
/// their fingers is curled round it, radians; and how far they lean out
/// from plumb with the flare of the wall they lie against.
const HAND: f32 = 0.3;
const GRIP_CURL: f32 = 0.9;
const GRIP_LEAN: f32 = 0.62;
/// Side of the grid the hands are skinned from, m.
const HAND_GRAIN: f32 = 0.0065;

/// Where the clapper hangs from, in the head's frame, and how far below
/// that its ball is, m.
const STAPLE: (f32, f32, f32) = (0.0, 0.26, -0.04);
const CLAPPER_DROP: f32 = 0.81;

/// What the years have done to one point of the bell's skin.
struct Tarnish {
    /// The tint of the bronze there.
    bronze: Vector3<f32>,
    /// The colour of the crust over it.
    crust: Vector3<f32>,
    /// How much of a crust there is, 0 to 1.
    crusted: f32,
}

/// How crusted a point of the bell's skin must be for the crust to cover
/// it.
const CRUST_EDGE: f32 = 0.5;

/// A corner of a triangle of the bell's skin as it is cut along the edge
/// of the crust.
#[derive(Clone, Copy)]
enum Corner {
    /// One of the skin's vertices.
    At(u32),
    /// A point this share of the way from one vertex of the skin to
    /// another, the lower-numbered first.
    Between(u32, u32, f32),
}

/// One of the two meshes the bell's skin is cut into.
struct Cut {
    mesh: MeshPrimitive,
    /// Where each vertex of the skin has gone in the mesh, if it has.
    kept: Vec<Option<u32>>,
    /// Where the cut across each edge of the skin has gone in the mesh.
    made: FxHashMap<(u32, u32), u32>,
    /// The colour this mesh takes from a point's tarnish.
    tint: fn(&Tarnish) -> Vector3<f32>,
}

impl Cut {
    /// Nothing yet of `skin`, to be drawn in `material`.
    fn of(skin: &MeshPrimitive, material: MaterialId, tint: fn(&Tarnish) -> Vector3<f32>) -> Self {
        Self {
            mesh: MeshPrimitive {
                vertices: Vec::new(),
                indices: Vec::new(),
                material,
            },
            kept: vec![None; skin.vertices.len()],
            made: FxHashMap::default(),
            tint,
        }
    }

    /// The mesh's vertex for `corner`, made if it is not there yet.
    fn vertex(&mut self, skin: &MeshPrimitive, tarnish: &[Tarnish], corner: Corner) -> u32 {
        let found = match corner {
            Corner::At(index) => self.kept[index as usize],
            Corner::Between(from, to, _) => self.made.get(&(from, to)).copied(),
        };
        if let Some(found) = found {
            return found;
        }
        let (mut vertex, tint) = match corner {
            Corner::At(index) => (
                skin.vertices[index as usize],
                (self.tint)(&tarnish[index as usize]),
            ),
            Corner::Between(from, to, share) => {
                let (a, b) = (skin.vertices[from as usize], skin.vertices[to as usize]);
                let mut vertex = a;
                vertex.pos = a.pos.lerp(&b.pos, share);
                vertex.normal = a
                    .normal
                    .lerp(&b.normal, share)
                    .try_normalize(1e-9)
                    .unwrap_or(a.normal);
                vertex.ao = a.ao + (b.ao - a.ao) * share;
                let tint = (self.tint)(&tarnish[from as usize])
                    .lerp(&(self.tint)(&tarnish[to as usize]), share);
                (vertex, tint)
            }
        };
        vertex.color.x = tint.x;
        vertex.color.y = tint.y;
        vertex.color.z = tint.z;
        let made = self.mesh.vertices.len() as u32;
        self.mesh.vertices.push(vertex);
        match corner {
            Corner::At(index) => self.kept[index as usize] = Some(made),
            Corner::Between(from, to, _) => {
                self.made.insert((from, to), made);
            }
        }
        made
    }

    /// Add the triangle with these corners.
    fn triangle(&mut self, skin: &MeshPrimitive, tarnish: &[Tarnish], corners: [Corner; 3]) {
        for corner in corners {
            let index = self.vertex(skin, tarnish, corner);
            self.mesh.indices.push(index);
        }
    }
}

/// The bell's skin cut in two along the edge of its crust: its bronze, and
/// the crust over it. A triangle the edge crosses is cut where it crosses,
/// so the crust ends in a line and not in the teeth of the skin's grid.
fn cut_at_the_crust(skin: &MeshPrimitive, tarnish: &[Tarnish]) -> Vec<MeshPrimitive> {
    let mut bronze = Cut::of(skin, Paint::Bronze.id(), |tarnish| tarnish.bronze);
    let mut crust = Cut::of(skin, Paint::Husk.id(), |tarnish| tarnish.crust);
    let crusted = |index: u32| tarnish[index as usize].crusted;
    // Where the crust ends on the way from one vertex to another.
    let edge = |from: u32, to: u32| {
        let share = (CRUST_EDGE - crusted(from)) / (crusted(to) - crusted(from));
        if from < to {
            Corner::Between(from, to, share)
        } else {
            Corner::Between(to, from, 1.0 - share)
        }
    };
    for triangle in skin.indices.chunks_exact(3) {
        let corners = [triangle[0], triangle[1], triangle[2]];
        let covered = corners.map(|index| crusted(index) >= CRUST_EDGE);
        let count = covered.iter().filter(|covered| **covered).count();
        if count == 0 || count == 3 {
            let whole = if count == 3 { &mut crust } else { &mut bronze };
            whole.triangle(skin, tarnish, corners.map(Corner::At));
            continue;
        }
        // One corner is alone on its side of the edge: its side keeps the
        // tip of the triangle, and the other side the rest.
        let lone = covered
            .iter()
            .position(|covered| *covered == (count == 1))
            .unwrap_or(0);
        let [a, b, c] = [
            corners[lone],
            corners[(lone + 1) % 3],
            corners[(lone + 2) % 3],
        ];
        let (ab, ac) = (edge(a, b), edge(a, c));
        let (tip, rest) = if count == 1 {
            (&mut crust, &mut bronze)
        } else {
            (&mut bronze, &mut crust)
        };
        tip.triangle(skin, tarnish, [Corner::At(a), ab, ac]);
        rest.triangle(skin, tarnish, [ab, Corner::At(b), Corner::At(c)]);
        rest.triangle(skin, tarnish, [ab, Corner::At(c), ac]);
    }
    vec![bronze.mesh, crust.mesh]
}

/// A bell-carrier.
#[derive(Deserialize)]
pub struct BellCarrierDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

/// A mesh turned by `turn` and then moved by `shift`.
fn placed(
    mut mesh: MeshPrimitive,
    turn: UnitQuaternion<f32>,
    shift: Vector3<f32>,
) -> MeshPrimitive {
    for vertex in &mut mesh.vertices {
        vertex.pos = turn * vertex.pos + shift;
        vertex.normal = turn * vertex.normal;
    }
    mesh
}

/// 0 below `low`, 1 above `high`, eased between.
fn ease(low: f32, high: f32, x: f32) -> f32 {
    let t = ((x - low) / (high - low)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The strokes of a letter as cast: each a run of points in a square, x
/// across and y up, both 0 to 1.
fn strokes(letter: char) -> &'static [&'static [(f32, f32)]] {
    match letter {
        '+' => &[&[(0.5, 0.08), (0.5, 0.92)], &[(0.08, 0.5), (0.92, 0.5)]],
        'A' => &[
            &[(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)],
            &[(0.22, 0.4), (0.78, 0.4)],
        ],
        'C' => &[&[
            (1.0, 0.8),
            (0.5, 1.0),
            (0.0, 0.7),
            (0.0, 0.3),
            (0.5, 0.0),
            (1.0, 0.2),
        ]],
        'G' => &[&[
            (1.0, 0.8),
            (0.5, 1.0),
            (0.0, 0.7),
            (0.0, 0.3),
            (0.5, 0.0),
            (1.0, 0.2),
            (1.0, 0.5),
            (0.55, 0.5),
        ]],
        'I' => &[&[(0.5, 0.0), (0.5, 1.0)]],
        'L' => &[&[(0.0, 1.0), (0.0, 0.0), (1.0, 0.0)]],
        'M' => &[&[(0.0, 0.0), (0.0, 1.0), (0.5, 0.35), (1.0, 1.0), (1.0, 0.0)]],
        'N' => &[&[(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)]],
        'O' => &[&[
            (0.5, 0.0),
            (0.0, 0.3),
            (0.0, 0.7),
            (0.5, 1.0),
            (1.0, 0.7),
            (1.0, 0.3),
            (0.5, 0.0),
        ]],
        'P' => &[&[
            (0.0, 0.0),
            (0.0, 1.0),
            (0.85, 0.95),
            (0.9, 0.55),
            (0.0, 0.45),
        ]],
        'R' => &[&[
            (0.0, 0.0),
            (0.0, 1.0),
            (0.8, 0.95),
            (0.9, 0.6),
            (0.0, 0.5),
            (1.0, 0.0),
        ]],
        'S' => &[&[
            (1.0, 0.8),
            (0.5, 1.0),
            (0.0, 0.75),
            (1.0, 0.25),
            (0.5, 0.0),
            (0.0, 0.2),
        ]],
        'T' => &[&[(0.0, 1.0), (1.0, 1.0)], &[(0.5, 1.0), (0.5, 0.0)]],
        'V' => &[&[(0.0, 1.0), (0.5, 0.0), (1.0, 1.0)]],
        _ => &[],
    }
}

impl BellCarrierDef {
    /// Its proportions: short legs, and the bell over everything above them.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.2,
            lower_leg_length: 0.2,
            hip_width: 0.12,
            leg_radius: 0.04,
            haunch_height: 0.05,
            haunch_radius: 0.08,
            neck_length: 0.4,
            neck_radius: 0.05,
            head_radius: BELL_RADIUS,
            rest_pitch: 0.0,
            rear_pitch: -0.25,
            strike_pitch: 0.35,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.08,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: 1.0,
            foot_placer: FootPlacerConfig::default(),
            // A bronze bell on short legs: it waddles, rolling well over
            // each planted foot and swinging past it, slow to answer and
            // slow to settle.
            carriage: Carriage {
                dip: 0.12,
                sway: 0.17,
                lean: 0.05,
                stride_speed: 1.8,
                heft: 0.04,
                frequency: 7.0,
                damping: 0.22,
                ..Carriage::default()
            },
        }
    }

    /// The bell as its founder meant it, solid, about the head's middle:
    /// a long waist under a round shoulder, flaring through the sound bow
    /// to the lip.
    fn casting() -> Clay {
        let mut bell = Clay::new(0.05)
            .add(
                Lump::mass((0.215, 0.14, 0.215), (0.0, 0.22, 0.0)),
                (1.0, 1.0, 1.0),
            )
            .add(
                Lump::limb((0.0, 0.1, 0.0), (0.0, -0.33, 0.0), (0.215, 0.275)),
                (1.0, 1.0, 1.0),
            );
        // The flare, turned as a stack of discs: how far down, how wide,
        // and how deep each is.
        for (drop, radius, depth) in [
            (0.33, 0.28, 0.11),
            (0.385, 0.298, 0.09),
            (0.43, 0.322, 0.08),
            (0.468, 0.352, 0.075),
            (0.5, 0.385, 0.075),
            (LIP_DROP - 0.012, BELL_RADIUS, 0.1),
        ] {
            bell = bell.add(
                Lump::mass((radius, depth, radius), (0.0, -drop, 0.0)),
                (1.0, 1.0, 1.0),
            );
        }
        bell
    }

    /// The point on the casting's outside `azimuth` radians round from
    /// straight ahead at `height`, and the way out of the bronze there.
    fn on_bell(casting: &Clay, azimuth: f32, height: f32) -> (Vector3<f32>, Vector3<f32>) {
        let out = Vector3::new(azimuth.sin(), 0.0, azimuth.cos());
        let axis = Vector3::y() * height;
        // Walk out from the axis to where the bronze ends.
        let (mut inside, mut outside) = (0.0, 0.7);
        for _ in 0..24 {
            let middle = 0.5 * (inside + outside);
            if casting.distance(axis + out * middle) < 0.0 {
                inside = middle;
            } else {
                outside = middle;
            }
        }
        let point = axis + out * inside;
        let slope = |along: Vector3<f32>| {
            casting.distance(point + along * 0.004) - casting.distance(point - along * 0.004)
        };
        let normal = Vector3::new(
            slope(Vector3::x()),
            slope(Vector3::y()),
            slope(Vector3::z()),
        )
        .try_normalize(1e-9)
        .unwrap_or(out);
        (point, normal)
    }

    /// `bell` with what was cast onto it: bands round the shoulder, the
    /// waist and the sound bow, the inscription between two of them, and
    /// the crown it was hung by.
    fn cast_on(mut bell: Clay, casting: &Clay) -> Clay {
        let bronze = (1.0, 1.0, 1.0);
        // A band is the rim of a disc standing out of the wall: how high it
        // is, how far it stands out and how wide it is.
        bell = bell.smoothing(0.004);
        for (height, proud, wide) in [
            (0.13, 0.005, 0.008),
            (0.105, 0.004, 0.006),
            (LETTERS.1 + 0.03, 0.005, 0.008),
            (LETTERS.0 - 0.03, 0.005, 0.008),
            (-0.375, 0.004, 0.006),
            (-0.4, 0.006, 0.009),
            (-0.425, 0.004, 0.006),
            (-LIP_DROP + 0.035, 0.004, 0.008),
        ] {
            let radius = Self::on_bell(casting, 0.0, height).0.z + proud;
            let depth = wide / (2.0 * proud / radius).sqrt();
            bell = bell.add(
                Lump::mass((radius, depth, radius), (0.0, height, 0.0)),
                bronze,
            );
        }

        // The inscription: each letter in its own cell round the waist,
        // some of its strokes worn nearly away.
        let cell = TAU / INSCRIPTION.len() as f32;
        let tall = LETTERS.1 - LETTERS.0;
        for (place, letter) in INSCRIPTION.chars().enumerate() {
            let middle = (place as f32 + 0.5) * cell - 2.2;
            let at = |(across, up): (f32, f32)| {
                let (point, out) = Self::on_bell(
                    casting,
                    middle + (across - 0.5) * 0.62 * cell,
                    LETTERS.0 + up * tall,
                );
                let sunk = point - out * STROKE_SUNK;
                (sunk.x, sunk.y, sunk.z)
            };
            for (run, stroke) in strokes(letter).iter().enumerate() {
                let worn = 0.5 + 0.5 * scatter(place as u32 * 7 + run as u32, 53);
                let thick = STROKE * (1.0 - 0.45 * worn * worn);
                for pair in stroke.windows(2) {
                    bell = bell.add(Lump::limb(at(pair[0]), at(pair[1]), (thick, thick)), bronze);
                }
            }
        }

        // The crown: a plate on the shoulder, and on it the loop the bell
        // hung by, with a cannon braced to each side of it.
        bell = bell
            .smoothing(0.012)
            .add(Lump::mass((0.115, 0.03, 0.115), (0.0, 0.352, 0.0)), bronze);
        let (ring, tube, centre) = (0.058, 0.021, 0.44);
        let round = |turn: f32| (ring * turn.cos(), centre + ring * turn.sin(), 0.0);
        let links = 12;
        for link in 0..links {
            let turn = |link: i32| TAU * link as f32 / links as f32;
            bell = bell.add(
                Lump::limb(round(turn(link)), round(turn(link + 1)), (tube, tube)),
                bronze,
            );
        }
        for side in [-1.0, 1.0] {
            bell = bell.add(
                Lump::limb(
                    (0.105 * side, 0.35, 0.0),
                    (0.05 * side, 0.455, 0.0),
                    (0.022, 0.017),
                ),
                bronze,
            );
        }
        bell
    }

    /// `bell` hollowed, opened at the mouth, and as the years have left it:
    /// pitted in the casting, dented, a piece broken out of its waist and
    /// cracked from the lip up through the break.
    fn broken(bell: Clay, casting: &Clay) -> Clay {
        let at = |azimuth: f32, height: f32| {
            let (point, _) = Self::on_bell(casting, azimuth, height);
            (point.x, point.y, point.z)
        };
        let beyond = |azimuth: f32, height: f32, by: f32| {
            let (point, out) = Self::on_bell(casting, azimuth, height);
            let centre = point + out * by;
            (centre.x, centre.y, centre.z)
        };
        // An uneven casting, and the mouth cut off flat.
        let far = 6.0;
        let mut bell = bell
            .relief(Relief::Lumps {
                frequency: 7.0,
                depth: 0.0007,
                seed: 54,
            })
            .hollow(WALL)
            .cut(Lump::ball(far, (0.0, -LIP_DROP - far, 0.0)), 0.004);

        // The break: bites out of the wall that overlap, so the edge
        // between two is a tooth of bronze.
        let (round, high) = BREAK;
        for (aside, up, bite) in [
            (0.0, 0.0, 0.062),
            (0.24, 0.03, 0.04),
            (-0.22, -0.035, 0.036),
            (0.1, -0.065, 0.034),
            (-0.1, 0.055, 0.03),
            (0.3, -0.04, 0.02),
        ] {
            bell = bell.cut(Lump::ball(bite, at(round + aside, high + up)), 0.002);
        }
        // The crack: from the lip up into the break and on out of the top
        // of it, narrowing as it climbs.
        let crack = [
            (0.36, -LIP_DROP - 0.01, 0.016),
            (0.31, -0.5, 0.013),
            (0.37, -0.43, 0.011),
            (0.27, -0.36, 0.01),
            (0.31, -0.28, 0.009),
            (0.21, -0.2, 0.0085),
            (0.19, -0.1, 0.0085),
            (-0.02, 0.03, 0.0075),
            (-0.1, 0.1, 0.007),
            (-0.05, 0.16, 0.006),
            (-0.14, 0.23, 0.005),
        ];
        for pair in crack.windows(2) {
            let (from, to) = (pair[0], pair[1]);
            bell = bell.cut(
                Lump::limb(at(from.0, from.1), at(to.0, to.1), (from.2, to.2)),
                0.0,
            );
        }
        // Chips out of the lip.
        for (azimuth, bite) in [(0.39, 0.024), (-1.9, 0.016), (2.6, 0.02), (-0.7, 0.012)] {
            bell = bell.cut(Lump::ball(bite, beyond(azimuth, -LIP_DROP, 0.004)), 0.003);
        }
        // Dents: where something far larger than a fist has met it.
        for (azimuth, height, size, deep) in [
            (1.1, -0.1, 0.2, 0.011),
            (-0.75, -0.36, 0.14, 0.009),
            (2.5, 0.05, 0.16, 0.01),
            (-2.2, -0.2, 0.24, 0.012),
            (0.5, 0.22, 0.1, 0.007),
        ] {
            bell = bell.cut(Lump::ball(size, beyond(azimuth, height, size - deep)), 0.02);
        }
        // Blowholes the founder never filled.
        for flaw in 0..14u32 {
            let azimuth = 3.1 * scatter(flaw, 55);
            let height = -0.12 + 0.4 * scatter(flaw, 56);
            let size = 0.013 + 0.005 * scatter(flaw, 57);
            bell = bell.cut(
                Lump::ball(size, beyond(azimuth, height, 0.55 * size)),
                0.002,
            );
        }
        bell
    }

    /// What the years have done to the bell at one point of its skin: the
    /// tint of its bronze there, the colour of the crust over it, and how
    /// much of a crust there is, 0 to 1.
    fn tarnish(vertex: &Vertex, casting: &Clay, wept_from: Vector3<f32>) -> Tarnish {
        let p = vertex.pos;
        // How far the surface stands out of the bell as it was meant: above
        // it on a band or a letter, below it in a dent, and a wall's
        // thickness below it inside.
        let proud = casting.distance(p);
        if proud < -0.6 * WALL {
            return Tarnish {
                bronze: SOOT,
                crust: VERDIGRIS_THIN,
                crusted: 0.0,
            };
        }
        let closed = (1.0 - vertex.ao) / 0.75;
        let blotch = fbm_perlin_3d(p.x * 4.5, p.y * 4.5, p.z * 4.5, 3, 0.55, 51);
        let run = fbm_perlin_3d(p.x * 30.0, p.y * 2.4, p.z * 30.0, 2, 0.5, 52);
        let fine = fbm_perlin_3d(p.x * 60.0, p.y * 60.0, p.z * 60.0, 2, 0.5, 58);
        // Rain sits on the shoulder and on the top of each band.
        let ledge = vertex.normal.y.max(0.0);
        // What has wept from the break and run down from it.
        let under = wept_from.y - p.y;
        let beside = Vector3::new(p.x - wept_from.x, 0.0, p.z - wept_from.z).norm();
        let wept = ease(0.0, 0.05, under)
            * ease(0.4, 0.08, under)
            * ease(0.06, 0.015, beside + 0.03 * run)
            * (0.55 + 0.45 * fine);
        // Rubbed: whatever stands proud, and the lip all round, where knees
        // knock it and hands hold it.
        let lip = ease(0.11, 0.0, p.y + LIP_DROP);
        let rubbed = (ease(0.0012, 0.0045, proud) * (0.75 + 0.25 * fine)
            + lip * (0.6 + 0.4 * blotch))
            .clamp(0.0, 1.0)
            * (1.0 - 0.6 * closed);
        let crusted = ease(
            0.45,
            0.95,
            1.5 * closed
                + 0.55 * blotch
                + 0.3 * run
                + 0.3 * ledge * (0.5 + blotch)
                + 0.2 * fine
                + 0.8 * wept
                - 1.1 * rubbed,
        );
        // Under the edge of its crust the bronze has gone black.
        let bronze = (AGED * (1.0 + 0.25 * fine))
            .lerp(&BLACKENED, ease(0.0, 0.5, crusted))
            .lerp(&BRIGHT, rubbed);
        let thick =
            ease(0.5, 1.0, crusted) * (0.5 + 0.5 * fine) * (0.6 + 0.4 * blotch) + 0.25 * wept;
        Tarnish {
            bronze,
            crust: VERDIGRIS_THIN.lerp(&VERDIGRIS, thick.clamp(0.0, 1.0)),
            crusted,
        }
    }

    /// The bell: its bronze, and the verdigris crusted over it wherever
    /// nothing rubs and rain has sat.
    fn bell() -> Vec<MeshPrimitive> {
        let casting = Self::casting();
        let bell = Self::broken(Self::cast_on(casting.clone(), &casting), &casting);
        let skin = bell.skin(BELL_GRAIN, Paint::Bronze.id());
        let (wept_from, _) = Self::on_bell(&casting, BREAK.0, BREAK.1 - 0.05);
        let tarnish: Vec<Tarnish> = skin
            .vertices
            .iter()
            .map(|vertex| Self::tarnish(vertex, &casting, wept_from))
            .collect();
        cut_at_the_crust(&skin, &tarnish)
    }

    /// The clapper, about the staple it hangs from: a rod of black iron, a
    /// ball, and the flight under it that shows below the lip.
    fn clapper() -> MeshPrimitive {
        let ball = -CLAPPER_DROP;
        let mut mesh = Clay::new(0.02)
            // The eye it hangs by.
            .add(Lump::ball(0.022, (0.0, 0.0, 0.0)), IRON)
            .add(
                Lump::limb((0.0, 0.0, 0.0), (0.0, ball + 0.05, 0.0), (0.011, 0.015)),
                IRON,
            )
            .add(Lump::mass((0.047, 0.055, 0.047), (0.0, ball, 0.0)), IRON)
            .add(
                Lump::limb(
                    (0.0, ball - 0.03, 0.0),
                    (0.0, ball - 0.115, 0.0),
                    (0.017, 0.024),
                ),
                IRON,
            )
            // Forged, not turned: nothing on it is quite round.
            .relief(Relief::Lumps {
                frequency: 22.0,
                depth: 0.0035,
                seed: 59,
            })
            .skin(0.009, Paint::Tin.id());
        // Rust in its pits, and bright where it strikes.
        for vertex in &mut mesh.vertices {
            let p = vertex.pos;
            let pit = fbm_perlin_3d(p.x * 40.0, p.y * 40.0, p.z * 40.0, 3, 0.5, 60);
            let rust = ease(-0.1, 0.5, pit + (1.0 - vertex.ao));
            let struck = ease(0.035, 0.047, Vector3::new(p.x, 0.0, p.z).norm())
                * ease(0.05, 0.0, (p.y - ball).abs());
            let iron = Vector3::new(IRON.0, IRON.1, IRON.2);
            let colour = iron
                .lerp(&Vector3::new(0.2, 0.075, 0.03), rust)
                .lerp(&Vector3::new(0.5, 0.48, 0.45), struck);
            vertex.color.x = colour.x;
            vertex.color.y = colour.y;
            vertex.color.z = colour.z;
        }
        mesh
    }

    /// A hand hooked under the rim from inside, `azimuth` radians round
    /// from straight ahead: its wrist up in the bell, its palm laid to the
    /// flare of the wall, its fingers under the lip and up the outside.
    fn grip(casting: &Clay, side: FootSide, azimuth: f32) -> MeshPrimitive {
        let mut hand = anatomy::flesh(&anatomy::hand(HAND, GRIP_CURL, side), HAND_GRAIN);
        // Fingers that have held bronze for years: dark with it to the
        // knuckle.
        for vertex in &mut hand.vertices {
            let along = (vertex.pos.z / HAND - 0.4).clamp(0.0, 1.0);
            let dirt = 1.0 - 0.4 * along;
            vertex.color.x *= dirt * 0.92;
            vertex.color.y *= dirt * 0.9;
            vertex.color.z *= dirt * 0.8;
        }
        // The fingers run down the wall, leaning out as it flares, and the
        // palm faces the bronze.
        let out = Vector3::new(azimuth.sin(), 0.0, azimuth.cos());
        let (lean_sin, lean_cos) = GRIP_LEAN.sin_cos();
        let fingers = out * lean_sin - Vector3::y() * lean_cos;
        let palm = out * lean_cos + Vector3::y() * lean_sin;
        let turn = UnitQuaternion::from_basis_unchecked(&[fingers.cross(&palm), -palm, fingers]);
        // The middle of the lip's underside, and the part of the fingers
        // that lies under it: the middle of their middle bones.
        let (rim, _) = Self::on_bell(casting, azimuth, -LIP_DROP + 0.002);
        let under = rim - out * 0.3 * WALL - Vector3::y() * 0.02;
        let hooked = Vector3::new(0.0, -0.213 * HAND, 0.667 * HAND);
        placed(hand, turn, under - turn * hooked)
    }

    /// The face pressed to the break: a brow and a cheek round the socket
    /// the eye sits in, the rest lost in the dark.
    fn face(socket: Vector3<f32>, out: Vector3<f32>) -> MeshPrimitive {
        let at = |ahead: f32, up: f32, aside: f32| {
            let side = Vector3::y().cross(&out);
            let p = socket + out * ahead + Vector3::y() * up + side * aside;
            (p.x, p.y, p.z)
        };
        let r = EYE_RADIUS;
        let clay = Clay::new(0.03)
            .add(Lump::ball(2.3 * r, at(-1.5 * r, -0.2 * r, 0.0)), PALLOR)
            // The brow, heavy over the eye, and the cheekbone under it.
            .add(
                Lump::limb(
                    at(0.25 * r, 1.25 * r, -1.5 * r),
                    at(0.3 * r, 1.15 * r, 1.5 * r),
                    (0.6 * r, 0.5 * r),
                ),
                PALLOR,
            )
            .add(
                Lump::mass((1.3 * r, 0.7 * r, 0.9 * r), at(0.0, -1.5 * r, 0.3 * r)),
                PALLOR,
            )
            .relief(Relief::Folds {
                along: Vector3::y().cross(&out),
                frequency: 40.0,
                depth: 0.002,
                seed: 61,
            })
            .cut(Lump::ball(1.04 * r, at(0.0, 0.0, 0.0)), 0.006);
        let mut mesh = anatomy::flesh(&clay, 0.011);
        // Raw round the eye it never shuts for long.
        Blush {
            at: socket + out * r,
            radius: 2.2 * r,
            tint: Vector3::new(0.95, 0.6, 0.6),
        }
        .paint(&mut mesh);
        // Only what is nearest the break has any light on it.
        for vertex in &mut mesh.vertices {
            let ahead = (vertex.pos - socket).dot(&out);
            let lit = 0.12 + 0.6 * ease(-0.2 * r, 1.3 * r, ahead);
            vertex.color.x *= lit;
            vertex.color.y *= lit;
            vertex.color.z *= lit;
        }
        mesh
    }

    /// Its legs: filthy from the ground up, and bruised across the shins
    /// where the lip knocks them at every step.
    fn legs(rig: &StiltRigConfig) -> Vec<(Worn, Vec<MeshPrimitive>)> {
        let mut legs = bare_legs(rig, 0.062, 0.185);
        let filth = Grime {
            below: -rig.lower_leg_length + 0.04,
            fade: 0.17,
            tint: Vector3::new(0.5, 0.4, 0.31),
        };
        let caked = Mottle {
            frequency: 28.0,
            value: 0.22,
            warmth: 0.05,
            seed: 62,
        };
        let bruise = Blush {
            at: Vector3::new(0.0, -0.055, 0.03),
            radius: 0.05,
            tint: Vector3::new(0.62, 0.42, 0.6),
        };
        for (worn, meshes) in &mut legs {
            for mesh in meshes.iter_mut() {
                match worn.joint {
                    Joint::Shin(_) => {
                        filth.paint(mesh);
                        caked.paint(mesh);
                        bruise.paint(mesh);
                    }
                    // Up under the bell, where no light comes.
                    Joint::Thigh(_) => {
                        for vertex in &mut mesh.vertices {
                            let dark = 1.0 - 0.55 * ease(-0.2, 0.0, vertex.pos.y);
                            vertex.color.x *= dark;
                            vertex.color.y *= dark;
                            vertex.color.z *= dark;
                        }
                    }
                    _ => caked.paint(mesh),
                }
            }
        }
        legs
    }

    /// Its dress, against the palette's slots.
    pub fn sculpt() -> Dress {
        let rig = Self::rig();
        let casting = Self::casting();
        // Everything that is the bell's lists with it.
        let list = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), LIST);
        let listed = |mesh: MeshPrimitive| placed(mesh, list, Vector3::zeros());

        let (hole, out) = Self::on_bell(&casting, BREAK.0, BREAK.1);
        let level = Vector3::new(out.x, 0.0, out.z).normalize();
        let socket = hole - level * (WALL + EYE_SET_BACK);

        let eye = Joint::Eye {
            socket: list * socket,
            swivel: 0.5,
        };
        let lid = |shut: f32| Joint::Hinge {
            pivot: list * socket,
            axis: Vector3::x(),
            drive: Drive::Lid { shut },
        };
        // The clapper hangs plumb when the bell rears back off it.
        let staple = Vector3::new(STAPLE.0, STAPLE.1, STAPLE.2);
        let hung = Joint::Hinge {
            pivot: list * staple,
            axis: Vector3::x(),
            drive: Drive::Strike {
                open: -rig.rear_pitch,
            },
        };

        let mut pieces = vec![
            (
                Worn::on(Joint::Head),
                Self::bell().into_iter().map(listed).collect(),
            ),
            (
                Worn::on(Joint::Head),
                vec![
                    listed(Self::grip(&casting, FootSide::Left, 1.02)),
                    listed(Self::grip(&casting, FootSide::Right, -1.2)),
                    listed(Self::face(socket, level)),
                ],
            ),
            (Worn::on(hung), vec![Self::clapper()]),
            (
                Worn::on(eye),
                vec![anatomy::eyeball(
                    EYE_RADIUS,
                    EyeLook {
                        iris: (0.3, 0.33, 0.28),
                        iris_angle: 0.5,
                        pupil_share: 0.5,
                        bloodshot: 0.9,
                    },
                )],
            ),
            // A lid too heavy to hold up, and the bag under the eye.
            (
                Worn::on(lid(FRAC_PI_2 - 0.8)),
                vec![anatomy::eyelid(EYE_RADIUS, 1.38, false, PALLOR)],
            ),
            (
                Worn::on(lid(-0.3)),
                vec![anatomy::eyelid(EYE_RADIUS, 1.12, true, PALLOR)],
            ),
        ];
        pieces.extend(Self::legs(&rig));
        Dress::of(pieces)
    }

    /// Its dress against the palette's slots, sculpted once: for tools
    /// that draw it in the palette's own materials.
    pub fn dress() -> Arc<Dress> {
        likeness::dress_sculpted(KIND, Self::sculpt)
    }

    fn walker(&self, materials: &[MaterialId]) -> StiltWalker {
        StiltWalker {
            at: self.pos,
            rig: Self::rig(),
            dress: likeness::dress(KIND, materials, Self::sculpt),
            speed: 2.0,
            sight_range: 16.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 18.0,
                windup: RAM_WINDUP,
                recovery: RAM_RECOVERY,
            }),
            // Nothing breaks the bell.
            health: None,
            brain: |standoff| Brain::hunter(standoff).relentless(),
        }
    }
}

impl Spawnable for BellCarrierDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        self.walker(materials).spawn(world)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn its_dress_is_tens_of_thousands_of_triangles_and_every_part_is_worn() {
        let started = Instant::now();
        let dress = BellCarrierDef::sculpt();
        println!("sculpted in {:?}", started.elapsed());
        assert_eq!(dress.worn.len(), dress.model.parts.len());
        for (worn, part) in dress.worn.iter().zip(&dress.model.parts) {
            let triangles: Vec<usize> = part
                .primitives
                .iter()
                .map(|m| m.indices.len() / 3)
                .collect();
            let joint = format!("{:?}", worn.joint);
            println!(
                "  {}: {triangles:?}",
                joint.split([' ', '(']).next().unwrap_or("")
            );
        }
        let triangles = dress.triangles();
        println!("a bell-carrier is {triangles} triangles");
        assert!(
            (30_000..200_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn the_bell_is_a_mouth_wide_and_hangs_a_hand_above_the_ground() {
        let rig = BellCarrierDef::rig();
        let bell = BellCarrierDef::bell();
        let skin = || bell.iter().flat_map(|mesh| &mesh.vertices);
        let low = skin().map(|v| v.pos.y).fold(f32::MAX, f32::min);
        let wide = skin().map(|v| v.pos.xz().norm()).fold(f32::MIN, f32::max);
        // The head's middle rides this far above the ground.
        let head = rig.standing_height() + rig.haunch_height + rig.neck_length;
        let clear = head + low;
        assert!((0.18..0.28).contains(&clear), "the lip clears by {clear} m");
        assert!((wide - BELL_RADIUS).abs() < 0.02, "its mouth is {wide} m");
    }
}
