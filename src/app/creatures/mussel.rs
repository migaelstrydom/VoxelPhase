//! Mussel — a shell the size of a cart, lying open on the path with
//! something wet and patient inside it, and a pearl.
//!
//! The ambush. It does nothing until someone steps between its valves, and
//! then it shuts on them, hard, and holds shut a while before it opens
//! again. Seen, it is walked round or blasted; it is for the path taken at
//! a run. `visual_bench mussel` shows it open and shut.
//!
//! ```text
//!   part          what it is                                      moves as
//!   ────────────  ──────────────────────────────────────────────  ────────
//!   lower valve   shell, its nacre, the ligament along the hinge,  Bed
//!                 byssus threads out to the ground
//!   mantle        the flesh lining it, drawn into folds, with a    Flesh
//!                 thin dark frill and feelers along its edge
//!   body          the mound, its foot, and the pearl               Flesh
//!   adductor      the muscle that shuts it                         Muscle
//!   upper valve   shell and nacre, barnacles, its own frill        Jaw
//!   legs          somebody's, bare, one part each                  Catch
//! ```
//!
//! It is sculpted lying shut, about the middle of the slab it collides as,
//! with its hinge along the back (−Z) and its beak to −X; the trap's rig
//! (`creature::snap_trap`) opens it.

use std::f32::consts::{FRAC_PI_2, TAU};

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::anatomy::{self, LIMB_GRAIN};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::sculpted::{all_round, ground_under, Exhibit};
use crate::animation::FootSide;
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, Strand, Veins, VertexPaint,
};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Cycle, Hinge, Moment, SnapTrap, TrapPart, TrapRig};
use crate::damage::Health;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_perlin_3d;

/// The name its look is kept under.
const KIND: &str = "mussel";

/// Half the slab it collides as: the lower valve alone, low enough to step
/// onto. The upper one is only drawn, or nothing could walk into the trap.
const SLAB: (f32, f32, f32) = (0.75, 0.05, 0.6);
/// Kilograms to the cubic metre: shell, and it stays where it lies.
const DENSITY: f32 = 2600.0;
/// Distance from its middle within which it bites, m.
const BITE_RADIUS: f32 = 0.6;
/// Hit points a bite takes off.
const BITE: f32 = 30.0;
/// How long a bite takes it: it shuts in a blink and is slow to open.
const CYCLE: Cycle = Cycle {
    slam: 0.07,
    hold: 2.5,
    creak: 2.2,
};
/// How far the upper valve stands open, radians.
const GAPE: f32 = 1.0;
/// Hit points it can take.
const HEALTH: f32 = 70.0;

/// The ground it lies on, under the slab's middle, m.
const GROUND: f32 = -SLAB.1;
/// Height of the plane its valves meet in.
const SEAM: f32 = 0.09;
/// A point between the valves well inside the shell: where its outline is
/// looked for from.
const MIDDLE: (f32, f32) = (0.1, 0.0);
/// The beak the shell grew from, at the hinge's end.
const UMBO: (f32, f32, f32) = (-0.72, SEAM, -0.45);
/// How thick a valve is, m, and the nacre laid inside that.
const WALL: f32 = 0.03;
const NACRE: f32 = 0.006;
/// Side of the grid each valve is skinned from, m: the lower lies mostly
/// under its flesh.
const UPPER_GRAIN: f32 = 0.0145;
const LOWER_GRAIN: f32 = 0.016;
/// Side of the grid the upper valve's nacre is skinned from, m: fine enough
/// to carry its growth lines.
const NACRE_GRAIN: f32 = 0.011;
/// Side of the grid its flesh is skinned from, m.
const FLESH_GRAIN: f32 = 0.011;

/// The shell's skin, where it has not worn or flaked (linear).
const PERIOSTRACUM: (f32, f32, f32) = (0.035, 0.045, 0.08);
/// The flesh (linear): the mantle, the mound, the frill at the mantle's
/// edge, the foot from its root to its tip, and the muscle.
const MANTLE: (f32, f32, f32) = (0.66, 0.34, 0.14);
const MOUND: (f32, f32, f32) = (0.72, 0.45, 0.24);
const FRILL: (f32, f32, f32) = (0.085, 0.03, 0.04);
const FOOT_ROOT: (f32, f32, f32) = (0.56, 0.25, 0.07);
const FOOT_TIP: (f32, f32, f32) = (0.72, 0.56, 0.36);
const MUSCLE: (f32, f32, f32) = (0.82, 0.68, 0.55);
/// Where the foot ends, and its byssus leaves it.
const FOOT_END: (f32, f32, f32) = (-0.26, SEAM + 0.1, 0.37);
/// The pearl (linear), its radius, m, and where it is seated.
const PEARL: (f32, f32, f32) = (0.92, 0.88, 0.82);
const PEARL_RADIUS: f32 = 0.1;
const PEARL_AT: (f32, f32, f32) = (0.17, SEAM + 0.115, 0.1);

/// A leg of whoever it caught: thigh and shin, m, the thigh's girth at the
/// hip, and the foot.
const THIGH: f32 = 0.43;
const SHIN: f32 = 0.43;
const LEG_GIRTH: f32 = 0.072;
const FOOT_LENGTH: f32 = 0.24;
/// How far down a thigh the lips have it, and how high an ankle is over
/// its sole, m.
const THIGH_WITHIN: f32 = 0.2;
const ANKLE: f32 = 0.075;

/// A mussel.
#[derive(Deserialize)]
pub struct MusselDef {
    /// Position (x, z). It lies on the ground there.
    pub pos: (f32, f32),
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

/// Where `clay`'s surface is met walking from `from`, inside it, the way
/// of `toward`.
fn surface(clay: &Clay, from: Vector3<f32>, toward: Vector3<f32>) -> Vector3<f32> {
    let (mut inside, mut outside) = (0.0, 2.0);
    for _ in 0..24 {
        let middle = 0.5 * (inside + outside);
        if clay.distance(from + toward * middle) < 0.0 {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    from + toward * inside
}

/// `mesh` in two: the triangles at least `corners` of whose corners `pick`
/// holds of, and the rest.
fn parted(
    mesh: &MeshPrimitive,
    corners: usize,
    pick: impl Fn(Vector3<f32>) -> bool,
) -> (MeshPrimitive, MeshPrimitive) {
    let picked: Vec<bool> = mesh.vertices.iter().map(|v| pick(v.pos)).collect();
    let empty = || MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material: mesh.material,
    };
    let mut halves = [empty(), empty()];
    let mut placed = [
        vec![u32::MAX; mesh.vertices.len()],
        vec![u32::MAX; mesh.vertices.len()],
    ];
    for triangle in mesh.indices.chunks_exact(3) {
        let votes = triangle.iter().filter(|i| picked[**i as usize]).count();
        let half = if votes >= corners { 0 } else { 1 };
        for &index in triangle {
            let slot = &mut placed[half][index as usize];
            if *slot == u32::MAX {
                *slot = halves[half].vertices.len() as u32;
                halves[half].vertices.push(mesh.vertices[index as usize]);
            }
            halves[half].indices.push(*slot);
        }
    }
    let [picked, rest] = halves;
    (picked, rest)
}

/// `mesh` without the triangles most of whose corners are `hidden`.
fn seen(mesh: &MeshPrimitive, hidden: impl Fn(Vector3<f32>) -> bool) -> MeshPrimitive {
    parted(mesh, 2, |at| !hidden(at)).0
}

/// A point of a [`sheet`].
struct SheetPoint {
    /// Where it is.
    at: Vector3<f32>,
    /// Its colour (linear).
    colour: Vector3<f32>,
    /// How open to the light it is, 1 in the open.
    open: f32,
}

/// Half how thick a sheet is, m.
const SHEET_HALF_THICK: f32 = 0.0015;

/// A thin sheet of `rows` by `columns` points laid onto `mesh`, both its
/// faces: `point` gives each one. If it is a `ring`, its last row is joined
/// to its first.
fn sheet(
    mesh: &mut MeshPrimitive,
    (rows, columns): (usize, usize),
    ring: bool,
    point: impl Fn(usize, usize) -> SheetPoint,
) {
    let points: Vec<SheetPoint> = (0..rows * columns)
        .map(|index| point(index / columns, index % columns))
        .collect();
    let at = |row: usize, column: usize| points[row * columns + column].at;
    // The way each point faces: square to the sheet as its neighbours lie.
    let facing = |row: usize, column: usize| {
        let (before, after) = if ring {
            ((row + rows - 1) % rows, (row + 1) % rows)
        } else {
            (row.saturating_sub(1), (row + 1).min(rows - 1))
        };
        let along = at(after, column) - at(before, column);
        let across = at(row, (column + 1).min(columns - 1)) - at(row, column.saturating_sub(1));
        along
            .cross(&across)
            .try_normalize(1e-9)
            .unwrap_or_else(Vector3::y)
    };
    for face in [1.0f32, -1.0] {
        let first = mesh.vertices.len() as u32;
        for row in 0..rows {
            for column in 0..columns {
                let point = &points[row * columns + column];
                let normal = facing(row, column) * face;
                mesh.vertices.push(Vertex {
                    pos: point.at + normal * SHEET_HALF_THICK,
                    color: Vector4::new(point.colour.x, point.colour.y, point.colour.z, 1.0),
                    tex_coords: Vector2::zeros(),
                    normal,
                    ao: point.open,
                });
            }
        }
        let joined = if ring { rows } else { rows - 1 };
        for row in 0..joined {
            let next = (row + 1) % rows;
            for column in 0..columns - 1 {
                let corner = |row: usize, column: usize| first + (row * columns + column) as u32;
                let (a, b) = (corner(row, column), corner(row, column + 1));
                let (c, d) = (corner(next, column + 1), corner(next, column));
                if face > 0.0 {
                    mesh.indices.extend([a, d, b, b, d, c]);
                } else {
                    mesh.indices.extend([a, b, d, b, c, d]);
                }
            }
        }
    }
}

/// A mesh moved by `to`.
fn placed(mut mesh: MeshPrimitive, to: &Isometry3<f32>) -> MeshPrimitive {
    for vertex in &mut mesh.vertices {
        vertex.pos = (to * Point3::from(vertex.pos)).coords;
        vertex.normal = to.rotation * vertex.normal;
    }
    mesh
}

/// A turn about the x axis.
fn pitch(angle: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle)
}

/// A turn about the y axis.
fn yaw(angle: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle)
}

/// Noise of three octaves at `at`.
fn noise(at: Vector3<f32>, seed: u32) -> f32 {
    fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, seed)
}

/// The whole shell as it lies shut, before it is hollowed and parted into
/// its valves.
struct Shell {
    solid: Clay,
}

impl Shell {
    fn new() -> Self {
        let solid = Clay::new(0.2)
            // The body of it, broad toward +X.
            .add(
                Lump::mass((0.56, 0.2, 0.5), (0.17, SEAM, 0.06)),
                PERIOSTRACUM,
            )
            // The straight back the hinge runs along.
            .add(
                Lump::limb((-0.52, SEAM, -0.43), (0.3, SEAM, -0.45), (0.1, 0.13)),
                PERIOSTRACUM,
            )
            // Narrowing to the beak.
            .add(
                Lump::limb((-0.1, SEAM, -0.2), (-0.66, SEAM, -0.43), (0.19, 0.07)),
                PERIOSTRACUM,
            );
        Self { solid }
    }

    /// The point on the shell's outline, where its valves meet, `round`
    /// radians round from ahead (+Z) toward +X as seen from its middle,
    /// and `inset` metres in from the edge.
    fn lip(&self, round: f32, inset: f32) -> Vector3<f32> {
        let out = Vector3::new(round.sin(), 0.0, round.cos());
        let from = Vector3::new(MIDDLE.0, SEAM, MIDDLE.1);
        surface(&self.solid, from, out) - out * inset
    }

    /// The point on a valve's outer face over (or under) `(x, z)`, and the
    /// way out of the shell there.
    fn face(&self, x: f32, z: f32, upper: bool) -> (Vector3<f32>, Vector3<f32>) {
        let toward = if upper { Vector3::y() } else { -Vector3::y() };
        let at = surface(&self.solid, Vector3::new(x, SEAM, z), toward);
        let slope = |axis: Vector3<f32>| {
            self.solid.distance(at + axis * 0.005) - self.solid.distance(at - axis * 0.005)
        };
        let out = Vector3::new(
            slope(Vector3::x()),
            slope(Vector3::y()),
            slope(Vector3::z()),
        );
        (at, out.try_normalize(1e-9).unwrap_or(toward))
    }

    /// How far back the shell reaches: where its hinge runs.
    fn back(&self) -> f32 {
        surface(&self.solid, Vector3::new(0.0, SEAM, -0.3), -Vector3::z()).z
    }

    /// One valve: its weathered outside, and its nacre.
    fn valve(&self, upper: bool) -> [MeshPrimitive; 2] {
        let away = if upper { -1.0 } else { 1.0 };
        // Chips out of its lip: where round it, and how big.
        let chips: &[(f32, f32)] = if upper {
            &[
                (0.2, 0.05),
                (0.9, 0.03),
                (1.5, 0.07),
                (2.2, 0.025),
                (-0.5, 0.04),
                (-1.1, 0.03),
            ]
        } else {
            &[
                (0.5, 0.04),
                (1.2, 0.03),
                (-0.2, 0.06),
                (-0.9, 0.035),
                (1.9, 0.03),
            ]
        };
        // A shell of `clay`, `wall` thick, with the half that is not this
        // valve cut away by a ball so large its side is a plane, and
        // chipped.
        let half = |clay: Clay, wall: f32| {
            let huge = 40.0;
            let mut clay = clay
                .hollow(wall)
                .cut(Lump::ball(huge, (0.0, SEAM + away * huge, 0.0)), 0.004);
            for &(round, size) in chips {
                let at = self.lip(round, 0.0) + Vector3::y() * away * size * 0.35;
                clay = clay.cut(Lump::ball(size, tuple(at)), 0.004);
            }
            clay
        };
        let grain = if upper { UPPER_GRAIN } else { LOWER_GRAIN };
        // The nacre is what faces into the shell; the lip itself, where the
        // valves meet, is the outside's.
        let within = |at: Vector3<f32>| {
            let depth = -self.solid.distance(at);
            let lip = (at.y - SEAM).abs() < 0.005;
            depth > if lip { WALL } else { 0.5 * WALL }
        };
        // The outside carries the rings it grew by; the nacre is laid
        // smooth over the inside of them.
        let ringed = self.solid.clone().relief(Relief::Rings {
            about: vector(UMBO),
            spacing: 0.05,
            depth: 0.007,
            seed: 71,
        });
        // The outside's wall is a little the thicker, so its lip reaches
        // over the nacre's edge, and each keeps only the triangles that
        // are wholly its own: none of one shows as teeth along the other.
        let wall = WALL + NACRE;
        let ringed = half(ringed, wall + 0.003).skin(grain, Paint::Shell.id());
        let (_, mut outside) = parted(&ringed, 3, within);
        // The nacre has its own growth lines: broad, and too shallow to
        // see but for what they do to its shine.
        let lined = self.solid.clone().relief(Relief::Rings {
            about: vector(UMBO),
            spacing: 0.09,
            depth: 0.003,
            seed: 73,
        });
        let fine = if upper { NACRE_GRAIN } else { LOWER_GRAIN };
        let lined = half(lined, wall).skin(fine, Paint::Nacre.id());
        let (mut nacre, _) = parted(&lined, 3, within);
        weather(&mut outside);
        lustre(&mut nacre);
        [outside, nacre]
    }
}

/// The outside of a valve: blue-black in bands as it grew, browner toward
/// its lip, worn to chalk about the beak, and flaked.
fn weather(mesh: &mut MeshPrimitive) {
    let skin = vector(PERIOSTRACUM);
    let violet = Vector3::new(0.05, 0.03, 0.075);
    let horn = Vector3::new(0.085, 0.06, 0.03);
    let chalk = Vector3::new(0.33, 0.34, 0.43);
    let prism = Vector3::new(0.16, 0.2, 0.32);
    for vertex in &mut mesh.vertices {
        let at = vertex.pos;
        let out = (at - vector(UMBO)).norm();
        // Bands as it grew: each year's a little bluer, or more violet, or
        // gone to brown.
        let band = fbm_perlin_3d(out * 26.0, 0.0, 0.0, 3, 0.6, 61);
        let hue = fbm_perlin_3d(out * 9.0, 7.3, 0.0, 2, 0.5, 68);
        let mut colour = skin.lerp(&violet, smooth(0.5 + 1.6 * hue)) * (1.0 + 0.8 * band);
        colour = colour.lerp(&horn, 0.75 * smooth((band - 0.22) / 0.2));
        colour = colour.lerp(&horn, 0.5 * smooth((out - 0.85) / 0.4));
        // Worn through to chalk about the beak, where it is oldest.
        let worn = smooth(1.0 - (out + 0.22 * noise(at * 6.0, 62) - 0.16) / 0.3);
        let pitted = 1.0 - 0.45 * smooth((noise(at * 38.0, 70) - 0.05) / 0.2);
        colour = colour.lerp(&(chalk * pitted), 0.85 * worn);
        // Flaked in small scabs, the layer under it showing.
        let flaked = smooth((noise(at * 30.0, 63) - 0.27) / 0.05)
            * smooth(0.6 + 1.5 * noise(at * 3.5, 69))
            * (1.0 - worn);
        colour = colour.lerp(&prism, 0.6 * flaked);
        vertex.color.x = colour.x;
        vertex.color.y = colour.y;
        vertex.color.z = colour.z;
    }
    Mottle {
        frequency: 9.0,
        value: 0.2,
        warmth: -0.08,
        seed: 64,
    }
    .paint(mesh);
    Grime {
        below: GROUND + 0.02,
        fade: 0.14,
        tint: Vector3::new(0.55, 0.45, 0.3),
    }
    .paint(mesh);
}

/// The inside of a valve: pearl, laid down in bands that shift between
/// rose and green, going to violet at the lip.
fn lustre(mesh: &mut MeshPrimitive) {
    let margin = Vector3::new(0.07, 0.08, 0.2);
    for vertex in &mut mesh.vertices {
        let at = vertex.pos;
        let out = (at - vector(UMBO)).norm();
        let band = fbm_perlin_3d(out * 13.0, 3.1, 0.0, 2, 0.5, 65) + 0.5 * noise(at * 4.0, 66);
        let rose = band.clamp(-1.0, 1.0);
        let green = noise(at * 6.0, 67);
        let pearl = Vector3::new(
            0.82 + 0.15 * rose,
            0.81 - 0.12 * rose + 0.09 * green,
            0.86 - 0.13 * green,
        );
        let deep = smooth(((at.y - SEAM).abs() + 0.02 * noise(at * 10.0, 68)) / 0.085);
        // Stained where the mantle has shrunk back from it.
        let stained = smooth((noise(at * 2.6, 69) - 0.12) / 0.3) * (1.0 - 0.6 * deep);
        let pearl = pearl.component_mul(
            &Vector3::repeat(1.0).lerp(&Vector3::new(0.72, 0.66, 0.5), 0.55 * stained),
        );
        let colour = margin.lerp(&pearl, deep);
        vertex.color.x = colour.x;
        vertex.color.y = colour.y;
        vertex.color.z = colour.z;
    }
}

impl MusselDef {
    /// Where the muscle is rooted in the lower valve, and where it holds
    /// the upper, with the shell shut.
    fn adductor_ends(shell: &Shell) -> (Vector3<f32>, Vector3<f32>) {
        let (x, z) = (0.33, -0.17);
        let (floor, down) = shell.face(x, z, false);
        let (roof, up) = shell.face(x, z, true);
        // Its hold is short of the roof by what its end swells when the
        // shell is shut and it is at its shortest and thickest.
        (floor - down * (WALL + 0.02), roof - up * (WALL + 0.06))
    }

    /// Its hinge: along the back of the shell, where the valves meet.
    fn hinge(shell: &Shell) -> Hinge {
        Hinge {
            pivot: Vector3::new(0.0, SEAM, shell.back()),
            axis: -Vector3::x_axis(),
            gape: GAPE,
        }
    }

    /// The legs of its catch: where along the shell (x) each comes out
    /// between the lips, and how it lies from there.
    fn legs() -> [(FootSide, f32, Lie); 2] {
        [
            (
                FootSide::Left,
                0.36,
                Lie {
                    splay: 0.2,
                    roll: 0.8,
                    knee: 0.22,
                    ankle: 0.55,
                },
            ),
            (
                FootSide::Right,
                0.12,
                Lie {
                    splay: -0.14,
                    roll: 0.2,
                    knee: 0.62,
                    ankle: 0.3,
                },
            ),
        ]
    }

    /// Where the lips have the leg that comes out at `x`: the middle of
    /// its thigh there.
    fn held(shell: &Shell, x: f32) -> Vector3<f32> {
        let lip = surface(&shell.solid, Vector3::new(x, SEAM, 0.0), Vector3::z());
        Vector3::new(x, SEAM + 0.02, lip.z)
    }

    /// How its model moves.
    pub fn rig() -> TrapRig {
        let shell = Shell::new();
        let hinge = Self::hinge(&shell);
        let (root, hold) = Self::adductor_ends(&shell);
        let mut parts = vec![
            TrapPart::Bed,
            TrapPart::Flesh {
                root: Vector3::new(MIDDLE.0, SEAM - 0.06, MIDDLE.1),
                breath: Vector3::new(0.004, 0.1, 0.004),
                lag: 0.0,
                pressed: 0.6,
            },
            TrapPart::Flesh {
                root: Vector3::new(0.08, SEAM - 0.06, -0.05),
                breath: Vector3::new(0.025, 0.07, 0.025),
                lag: 1.1,
                pressed: 0.45,
            },
            TrapPart::Muscle {
                root,
                hold,
                length: Self::adductor_length(&hinge, root, hold),
            },
            TrapPart::Jaw,
        ];
        for (index, (_, x, _)) in Self::legs().into_iter().enumerate() {
            parts.push(TrapPart::Catch {
                held: Self::held(&shell, x),
                kick: 0.16 * (1.0 - 0.45 * index as f32),
                lift: 0.16,
                drawn: Vector3::new(0.0, 0.0, -0.7),
                shrunk: 0.6,
            });
        }
        TrapRig { hinge, parts }
    }

    /// How long the muscle is with the shell wide open: how it is modelled.
    fn adductor_length(hinge: &Hinge, root: Vector3<f32>, hold: Vector3<f32>) -> f32 {
        let open = hinge.pose(1.0);
        (open.translation + open.rotation * hold - root).norm()
    }

    /// The flesh lining the lower valve: plump, and drawn into folds that
    /// all run the one way, from its beak to its broad end.
    fn mantle_clay() -> Clay {
        Clay::new(0.12)
            .add(
                Lump::mass((0.5, 0.125, 0.44), (0.17, SEAM - 0.05, 0.06)),
                MANTLE,
            )
            .add(
                Lump::Ball(
                    Ellipsoid::new((0.36, 0.095, 0.16), (-0.3, SEAM - 0.04, -0.27)).yawed(-0.39),
                ),
                MANTLE,
            )
            .relief(Relief::Folds {
                along: Vector3::new(1.0, 0.0, 0.2),
                frequency: 15.0,
                depth: 0.008,
                seed: 91,
            })
            .relief(Relief::Lumps {
                frequency: 4.0,
                depth: 0.006,
                seed: 72,
            })
    }

    fn mantle(shell: &Shell) -> MeshPrimitive {
        let mesh = Self::mantle_clay().skin(FLESH_GRAIN, Paint::Wet.id());
        // What of it lies in the shell's wall, or under the mound, is never
        // seen.
        let mound = Ellipsoid::new((0.19, 0.1, 0.16), (0.05, SEAM, -0.08));
        let mut mesh = seen(&mesh, |at| {
            (at.y < SEAM && shell.solid.distance(at) > -WALL - 0.004)
                || mound.distance_outside(at) < 0.0
        });
        wet(&mut mesh, 73);
        // Browner toward its edge, and dark in the bottom of each fold.
        let edge = Vector3::new(0.55, 0.36, 0.3);
        for vertex in &mut mesh.vertices {
            let at = vertex.pos;
            let inside = -shell.solid.distance(Vector3::new(at.x, SEAM, at.z));
            let toward_edge = smooth(1.0 - inside / 0.17);
            let folded = smooth((0.93 - vertex.ao) / 0.3);
            let tint = Vector3::repeat(1.0)
                .lerp(&edge, 0.8 * toward_edge)
                .lerp(&Vector3::new(0.5, 0.25, 0.22), 0.7 * folded);
            vertex.color.x *= tint.x;
            vertex.color.y *= tint.y;
            vertex.color.z *= tint.z;
        }
        mesh
    }

    /// The frill along a mantle's edge: a thin sheet thrown into ruffles,
    /// dark at its rim. The lower valve's stands up from the mantle and
    /// leans out over the lip; the upper's lies back along the nacre from
    /// just inside the lip.
    fn frill(shell: &Shell, upper: bool) -> MeshPrimitive {
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: Paint::Wet.id(),
        };
        let (stations, ribs) = (384, 5);
        let (root, rim) = (Vector3::new(0.4, 0.17, 0.09), vector(FRILL));
        let turned = if upper { 1.7 } else { 0.0 };
        sheet(&mut mesh, (stations, ribs), true, |station, rib| {
            let round = station as f32 / stations as f32 * TAU;
            // From its root, 0, to its free edge, 1.
            let t = rib as f32 / (ribs - 1) as f32;
            // Its ruffles grow toward its edge, and no two are alike; nor
            // is it as broad all the way round.
            let ruffle = (round * 47.0 + 3.5 * (round * 6.0 + turned).sin()).sin()
                * (0.6 + 0.4 * (round * 9.0 + turned).sin())
                + 0.6 * (round * 83.0 + 1.3 + turned).sin();
            let broad = 1.0
                + 0.38 * (round * 4.0 + 0.7 + turned).sin()
                + 0.25 * (round * 11.0 + 2.0 * turned).sin();
            let at = if upper {
                shell.lip(round, 0.03 + 0.058 * broad * t)
                    + Vector3::y() * (0.012 + 0.03 * t + 0.012 * ruffle * t * t)
            } else {
                shell.lip(round, 0.064 - 0.03 * broad * t - 0.016 * ruffle * t * t)
                    + Vector3::y() * (-0.02 + 0.064 * broad * t * (1.0 - 0.25 * t))
            };
            let patchy = 1.0 + 0.3 * noise(at * 30.0, 92);
            SheetPoint {
                at,
                colour: root.lerp(&rim, t.powf(0.8)) * patchy,
                open: 0.5 + 0.5 * t,
            }
        });
        mesh
    }

    /// The fringe of feelers along the mantle's edge: each up and out, and
    /// curling over at its tip.
    fn feelers(shell: &Shell) -> MeshPrimitive {
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: Paint::Wet.id(),
        };
        let count = 96;
        let (dark, pale) = (vector(FRILL), Vector3::new(0.55, 0.3, 0.24));
        for index in 0..count {
            let round = (index as f32 + 0.4 * scatter(index, 93)) / count as f32 * TAU;
            let out = Vector3::new(round.sin(), 0.0, round.cos());
            let aside = Vector3::new(out.z, 0.0, -out.x);
            let root = shell.lip(round, 0.07) + Vector3::y() * 0.004;
            let reach = 0.06 + 0.025 * scatter(index, 94);
            let lean = 0.3 + 0.3 * scatter(index, 95);
            let curl = 0.5 * scatter(index, 96);
            let up = Vector3::y();
            let path = vec![
                (root, 0.006),
                (root + (out * lean * 0.4 + up * 0.55) * reach, 0.0045),
                (
                    root + (out * lean * 0.9 + aside * curl * 0.5 + up * 0.95) * reach,
                    0.0028,
                ),
                (
                    root + (out * lean * 1.4 + aside * curl + up * 0.85) * reach,
                    0.0008,
                ),
            ];
            Strand::through(path)
                .sides(5)
                .rings(1)
                .lay(&mut mesh, |along| {
                    (dark.lerp(&pale, along), 0.5 + 0.5 * along)
                });
        }
        mesh
    }

    /// The body: a mound with the pearl seated in it, and the foot: a firm
    /// tongue, orange where it leaves the mound and cream at its tip,
    /// grooved along its top.
    fn body() -> Vec<MeshPrimitive> {
        let pearl_at = vector(PEARL_AT);
        let mound = Clay::new(0.1)
            .add(Lump::mass((0.23, 0.13, 0.2), (0.05, SEAM, -0.08)), MOUND)
            // The flesh that grips the pearl.
            .add(
                Lump::mass((0.17, 0.1, 0.17), tuple(pearl_at - Vector3::y() * 0.07)),
                MOUND,
            )
            .relief(Relief::Folds {
                along: Vector3::new(1.0, 0.0, 0.2),
                frequency: 17.0,
                depth: 0.006,
                seed: 81,
            })
            .relief(Relief::Lumps {
                frequency: 7.0,
                depth: 0.006,
                seed: 74,
            })
            .cut(Lump::ball(PEARL_RADIUS + 0.003, tuple(pearl_at)), 0.014);
        let mut mound = mound.skin(FLESH_GRAIN, Paint::Wet.id());
        wet(&mut mound, 75);
        // Raw where it holds the pearl.
        Blush {
            at: pearl_at,
            radius: PEARL_RADIUS * 1.7,
            tint: Vector3::new(0.75, 0.36, 0.4),
        }
        .paint(&mut mound);

        let (root, tip) = (Vector3::new(-0.04, SEAM + 0.055, 0.0), vector(FOOT_END));
        let knee = Vector3::new(-0.17, SEAM + 0.07, 0.25);
        let middling = (
            0.5 * (FOOT_ROOT.0 + FOOT_TIP.0),
            0.5 * (FOOT_ROOT.1 + FOOT_TIP.1),
            0.5 * (FOOT_ROOT.2 + FOOT_TIP.2),
        );
        let foot = Clay::new(0.03)
            .add(
                Lump::limb(tuple(root), tuple(knee), (0.066, 0.045)),
                FOOT_ROOT,
            )
            .add(
                Lump::limb(tuple(knee), tuple(tip), (0.045, 0.024)),
                middling,
            )
            .add(Lump::ball(0.027, tuple(tip)), FOOT_TIP)
            // Wrinkled across, as a muscle is that is not at full stretch.
            .relief(Relief::Folds {
                along: Vector3::new(1.0, 0.0, 0.45),
                frequency: 26.0,
                depth: 0.003,
                seed: 83,
            })
            // The groove its byssus is laid along.
            .cut(
                Lump::limb(
                    tuple(knee + Vector3::y() * 0.06),
                    tuple(tip + Vector3::y() * 0.034),
                    (0.016, 0.01),
                ),
                0.012,
            );
        let mut foot = foot.skin(0.009, Paint::Wet.id());
        Mottle {
            frequency: 16.0,
            value: 0.16,
            warmth: 0.14,
            seed: 82,
        }
        .paint(&mut foot);
        // Freckled brown along its back.
        Spots {
            frequency: 55.0,
            presence: 0.4,
            radius: (0.2, 0.42),
            tint: Vector3::new(0.55, 0.4, 0.3),
            salt: 84,
        }
        .paint(&mut foot);

        // The pearl: not quite round.
        let pearl = Clay::new(0.04)
            .add(Lump::ball(PEARL_RADIUS, tuple(pearl_at)), PEARL)
            .add(
                Lump::ball(
                    0.62 * PEARL_RADIUS,
                    tuple(pearl_at + Vector3::new(0.04, 0.025, -0.04)),
                ),
                PEARL,
            );
        let mut pearl = pearl.skin(0.011, Paint::Nacre.id());
        Mottle {
            frequency: 14.0,
            value: 0.04,
            warmth: 0.06,
            seed: 78,
        }
        .paint(&mut pearl);
        vec![mound, foot, pearl]
    }

    /// The muscle that shuts it, as it is stretched with the shell wide
    /// open: rooted at its origin and running `length` up +Y. A pale column
    /// of cords drawn taut, a little waisted, and spread where it takes
    /// hold of each valve.
    fn adductor(length: f32) -> MeshPrimitive {
        let cords = 7;
        let waist = 0.5 * length;
        let mut clay = Clay::new(0.014).add(
            Lump::limb((0.0, 0.0, 0.0), (0.0, length, 0.0), (0.04, 0.036)),
            MUSCLE,
        );
        for cord in 0..cords {
            let round = cord as f32 / cords as f32 * TAU;
            let at =
                |spread: f32, height: f32| (spread * round.cos(), height, spread * round.sin());
            let thick = 0.028 + 0.006 * scatter(cord, 110);
            clay = clay
                .add(
                    Lump::limb(at(0.058, 0.0), at(0.03, waist), (thick, 0.72 * thick)),
                    MUSCLE,
                )
                .add(
                    Lump::limb(
                        at(0.03, waist),
                        at(0.05, length),
                        (0.72 * thick, 0.92 * thick),
                    ),
                    MUSCLE,
                );
        }
        let clay = clay.relief(Relief::Folds {
            along: Vector3::y(),
            frequency: 46.0,
            depth: 0.003,
            seed: 79,
        });
        let mut mesh = clay.skin(0.009, Paint::Wet.id());
        Mottle {
            frequency: 12.0,
            value: 0.1,
            warmth: 0.1,
            seed: 80,
        }
        .paint(&mut mesh);
        // Pink between its cords, and flushed where it is rooted.
        for vertex in &mut mesh.vertices {
            let between = smooth((0.95 - vertex.ao) * 2.2);
            let rooted = smooth(((vertex.pos.y / length - 0.5).abs() - 0.3) / 0.2);
            let tint = Vector3::repeat(1.0)
                .lerp(&Vector3::new(0.9, 0.55, 0.5), between)
                .lerp(&Vector3::new(0.95, 0.62, 0.55), 0.7 * rooted);
            vertex.color.x *= tint.x;
            vertex.color.y *= tint.y;
            vertex.color.z *= tint.z;
        }
        mesh
    }

    /// Barnacles crusted on the upper valve.
    fn barnacles(shell: &Shell) -> MeshPrimitive {
        let stone = (0.4, 0.37, 0.31);
        let mut clay = Clay::new(0.006);
        // Where each cluster is, how many are in it and how far it spreads.
        let clusters = [
            (-0.36, -0.28, 13u32, 0.1),
            (0.44, 0.2, 11, 0.09),
            (0.12, -0.42, 8, 0.08),
            (-0.1, 0.3, 5, 0.05),
            (0.3, -0.1, 3, 0.04),
        ];
        let mut key = 0;
        for (x, z, count, spread) in clusters {
            for _ in 0..count {
                key += 1;
                // Crowded toward the cluster's middle.
                let off = scatter(key, 81).abs().sqrt() * scatter(key, 85).signum();
                let (at, out) = shell.face(x + spread * off, z + spread * scatter(key, 82), true);
                let size = 0.02 + 0.013 * scatter(key, 83);
                let lean = Vector3::new(scatter(key, 86), 0.0, scatter(key, 87)) * 0.25;
                let up = (out + lean).normalize();
                let (foot, top) = (at - out * 0.5 * size, at + up * 1.15 * size);
                clay = clay
                    .add(
                        Lump::limb(tuple(foot), tuple(top), (size, 0.52 * size)),
                        stone,
                    )
                    .cut(Lump::ball(0.4 * size, tuple(top + up * 0.18 * size)), 0.002);
            }
        }
        // What of them is under the shell's skin is never seen.
        let mesh = clay.skin(0.008, Paint::Shell.id());
        let mut mesh = seen(&mesh, |at| shell.solid.distance(at) < -0.008);
        Mottle {
            frequency: 30.0,
            value: 0.3,
            warmth: 0.12,
            seed: 84,
        }
        .paint(&mut mesh);
        // Dark in each one's mouth and where it sits on the shell.
        for vertex in &mut mesh.vertices {
            let shut_in = smooth((1.0 - vertex.ao) * 2.2);
            let stain = Vector3::repeat(1.0).lerp(&Vector3::new(0.3, 0.3, 0.26), shut_in);
            vertex.color.x *= stain.x;
            vertex.color.y *= stain.y;
            vertex.color.z *= stain.z;
        }
        mesh
    }

    /// Byssus: the threads it has moored itself to the ground by, spun from
    /// the tip of its foot and out over its lip.
    fn byssus(shell: &Shell) -> MeshPrimitive {
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: Paint::Horn.id(),
        };
        let spun = vector(FOOT_END);
        // The way from the shell's middle out over the foot.
        let toward = (spun.x - MIDDLE.0).atan2(spun.z - MIDDLE.1);
        let threads = 24;
        for thread in 0..threads {
            // A hank of them, fanning as they go, and no two alike.
            let source = toward + 0.14 * scatter(thread, 88);
            let round = source + 0.55 * scatter(thread, 85);
            let out = Vector3::new(round.sin(), 0.0, round.cos());
            let aside = Vector3::new(out.z, 0.0, -out.x);
            let lip = shell.lip(source, 0.0) + Vector3::y() * 0.01;
            let reach = 0.14 + 0.24 * (0.5 + 0.5 * scatter(thread, 86));
            let end = Vector3::new(lip.x, GROUND + 0.002, lip.z) + out * reach;
            let thick = 0.0022 + 0.0009 * scatter(thread, 87);
            let sag = 0.035 * scatter(thread, 89);
            let from = spun + Vector3::new(scatter(thread, 90), 0.0, scatter(thread, 91)) * 0.012
                - Vector3::y() * 0.012;
            let path = vec![
                (from, thick),
                (
                    from + (lip - from) * 0.5 + Vector3::y() * 0.012 * (1.0 + scatter(thread, 97)),
                    thick,
                ),
                (lip, thick),
                (
                    lip + out * 0.3 * reach + aside * sag - Vector3::y() * 0.07,
                    0.9 * thick,
                ),
                (
                    end - out * 0.035 + aside * 0.6 * sag + Vector3::y() * 0.004,
                    0.8 * thick,
                ),
                (end, 3.2 * thick),
            ];
            Strand::through(path)
                .sides(5)
                .rings(4)
                .lay(&mut mesh, |along| {
                    let colour = Vector3::new(0.22, 0.13, 0.04) * (0.6 + 0.6 * along);
                    (colour, 1.0)
                });
        }
        mesh
    }

    /// The ligament in the hinge: a dark band where the valves meet.
    fn ligament(shell: &Shell) -> MeshPrimitive {
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: Paint::Horn.id(),
        };
        let at = |x: f32| Vector3::new(x, SEAM + 0.004, shell.back() + 0.03);
        Strand::through(vec![(at(-0.5), 0.016), (at(-0.1), 0.024), (at(0.34), 0.02)])
            .sides(8)
            .rings(6)
            .lay(&mut mesh, |_| (Vector3::new(0.05, 0.03, 0.02), 0.7));
        mesh
    }

    /// One leg of its catch, limp: out between the shut valves at `held`,
    /// and sloping from there until it rests on the ground.
    fn leg(side: FootSide, held: Vector3<f32>, lie: &Lie) -> Vec<MeshPrimitive> {
        let outward = match side {
            FootSide::Left => 1.0,
            FootSide::Right => -1.0,
        };
        let thigh = anatomy::flesh(&anatomy::thigh(THIGH, LEG_GIRTH), LIMB_GRAIN);
        let shin = anatomy::flesh(&anatomy::shin(SHIN, LEG_GIRTH), 1.2 * LIMB_GRAIN);
        let foot = anatomy::foot(FOOT_LENGTH, side);
        // Where its hip, knee and sole are for a leg sloping `droop` radians
        // down from the lips: turned from standing to lying on its back,
        // out along +Z.
        let frames = |droop: f32| {
            let turn = yaw(lie.splay) * pitch(droop) * pitch(-FRAC_PI_2) * yaw(outward * lie.roll);
            let hip = Isometry3::from_parts(
                Translation3::from(held + turn * Vector3::y() * THIGH_WITHIN),
                turn,
            );
            let knee =
                hip * Isometry3::from_parts(Translation3::new(0.0, -THIGH, 0.0), pitch(lie.knee));
            let sole =
                knee * Isometry3::from_parts(
                    Translation3::new(0.0, ANKLE - SHIN, 0.0),
                    pitch(lie.ankle) * yaw(outward * 0.3),
                ) * Translation3::new(0.0, -ANKLE, 0.0);
            [hip, knee, sole]
        };
        let lowest = |droop: f32| {
            let [_, knee, sole] = frames(droop);
            let low = |mesh: &MeshPrimitive, to: &Isometry3<f32>| {
                let heights = mesh.vertices.iter().map(|v| (to * Point3::from(v.pos)).y);
                heights.fold(f32::MAX, f32::min)
            };
            low(&shin, &knee).min(low(&foot, &sole))
        };
        let (mut clear, mut sunk) = (-0.3, 0.9);
        for _ in 0..16 {
            let droop = 0.5 * (clear + sunk);
            if lowest(droop) > GROUND + 0.003 {
                clear = droop;
            } else {
                sunk = droop;
            }
        }
        let [hip, knee, sole] = frames(clear);

        let thigh = placed(thigh, &hip);
        let mut meshes = vec![
            // What of the thigh is well inside the shell is never seen.
            seen(&thigh, |at| at.z < held.z - 0.04),
            placed(shin, &knee),
            placed(foot, &sole),
        ];
        for mesh in &mut meshes {
            // The blood has gone out of it and pooled where it lies,
            Mottle {
                frequency: 5.0,
                value: 0.1,
                warmth: -0.22,
                seed: 108,
            }
            .paint(mesh);
            Grime {
                below: GROUND + 0.015,
                fade: 0.06,
                tint: Vector3::new(0.62, 0.44, 0.5),
            }
            .paint(mesh);
            for vertex in &mut mesh.vertices {
                let pale = Vector3::new(0.47, 0.42, 0.42);
                let colour = vertex.color.xyz().lerp(&pale, 0.18);
                // and it is bruised where the lips have it.
                let bitten = smooth(1.0 - (vertex.pos.z - held.z).abs() / 0.085);
                let bruise = Vector3::new(0.5, 0.26, 0.4);
                let colour =
                    colour.component_mul(&Vector3::repeat(1.0).lerp(&bruise, 0.85 * bitten));
                vertex.color.x = colour.x;
                vertex.color.y = colour.y;
                vertex.color.z = colour.z;
            }
        }
        meshes
    }

    /// Its look against the palette's slots, lying shut: one part to each
    /// part of [`Self::rig`].
    pub fn sculpt() -> Model {
        let shell = Shell::new();
        let rig = Self::rig();

        // The lower valve, less what is under the ground or under its
        // flesh.
        let [outside, nacre] = shell.valve(false);
        let flesh = Self::mantle_clay();
        let lower = vec![
            seen(&outside, |at| at.y < GROUND - 0.008),
            seen(&nacre, |at| {
                at.y < SEAM - 0.045 && flesh.distance(at + Vector3::y() * 0.02) < 0.0
            }),
            Self::ligament(&shell),
            Self::byssus(&shell),
        ];

        let mantle = vec![
            Self::mantle(&shell),
            Self::frill(&shell, false),
            Self::feelers(&shell),
        ];

        let mut muscle = Vec::new();
        if let TrapPart::Muscle { length, .. } = rig.parts[3] {
            muscle.push(Self::adductor(length));
        }

        let [outside, mut nacre] = shell.valve(true);
        // The scar where the muscle holds the upper valve: the nacre is
        // stained round it, and under it worn dull.
        let (_, hold) = Self::adductor_ends(&shell);
        Blush {
            at: hold,
            radius: 0.24,
            tint: Vector3::new(0.42, 0.4, 0.5),
        }
        .paint(&mut nacre);
        let (mut scar, nacre) = parted(&nacre, 2, |at| {
            let off = Vector3::new(at.x - hold.x, 0.0, at.z - hold.z).norm();
            at.y > SEAM && off < 0.115 + 0.05 * noise(at * 13.0, 109)
        });
        scar.material = Paint::Shell.id();
        for vertex in &mut scar.vertices {
            let chalk = 0.5 + 0.14 * noise(vertex.pos * 24.0, 111);
            vertex.color.x = 0.92 * chalk;
            vertex.color.y = 0.9 * chalk;
            vertex.color.z = 1.08 * chalk;
        }
        let upper = vec![
            outside,
            nacre,
            scar,
            Self::barnacles(&shell),
            Self::frill(&shell, true),
        ];

        let mut parts = vec![
            ModelPart::new(lower),
            ModelPart::new(mantle),
            ModelPart::new(Self::body()),
            ModelPart::new(muscle),
            ModelPart::new(upper),
        ];
        for (side, x, lie) in Self::legs() {
            let held = Self::held(&shell, x);
            parts.push(ModelPart::new(Self::leg(side, held, &lie)));
        }
        Model::flat(parts)
    }

    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        let rig = Self::rig();
        let state = |name, moment: &Moment, reach| Exhibit {
            name,
            model: model.clone(),
            poses: rig.poses(moment),
            lift: -GROUND,
            reach,
        };
        vec![
            state("open", &Moment::OPEN, 0.85),
            state("shut", &Moment::SHUT, 0.95),
        ]
    }
}

/// How a limp leg lies, radians: swung aside, rolled outward about its own
/// length, bent at the knee, and its foot dropped at the ankle.
struct Lie {
    splay: f32,
    roll: f32,
    knee: f32,
    ankle: f32,
}

/// Paint flesh as wet, living meat: uneven, faintly veined, and pored.
fn wet(mesh: &mut MeshPrimitive, seed: u32) {
    Mottle {
        frequency: 8.0,
        value: 0.2,
        warmth: 0.18,
        seed,
    }
    .paint(mesh);
    Veins {
        frequency: 9.0,
        width: 0.03,
        tint: Vector3::new(0.7, 0.42, 0.45),
        seed: seed + 1,
    }
    .paint(mesh);
    Spots {
        frequency: 40.0,
        presence: 0.25,
        radius: (0.15, 0.35),
        tint: Vector3::new(0.8, 0.62, 0.55),
        salt: seed + 2,
    }
    .paint(mesh);
}

impl Spawnable for MusselDef {
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
        let slab = vector(SLAB);
        let at = Point3::new(self.pos.0, ground + slab.y, self.pos.1);
        let body = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(at)
                    .linear_damping(0.3)
                    .angular_damping(0.9),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(slab)
                    .density(DENSITY)
                    .friction(1.5)
                    .restitution(0.0),
            );
            body
        };
        let trap = SnapTrap::new(BITE_RADIUS, BITE, CYCLE, Self::rig());
        let posed = PosedModel {
            model: likeness::model(KIND, materials, Self::sculpt),
            poses: trap.poses(),
            frame: PoseFrame::Entity,
        };
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(posed)
            .with(Renderable)
            .with(all_round(3.0))
            .with(trap)
            .with(Health::new(HEALTH, 4.0))
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_is_tens_of_thousands_of_triangles_with_a_part_for_each_part_of_its_rig() {
        let model = MusselDef::sculpt();
        assert_eq!(model.parts.len(), MusselDef::rig().parts.len());
        let meshes = model.parts.iter().flat_map(|part| &part.primitives);
        let mut triangles = 0;
        for mesh in meshes {
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
                assert!(
                    vertex.pos.norm() < 1.6,
                    "part of it is {} m from its middle",
                    vertex.pos.norm()
                );
            }
        }
        println!("a mussel is {triangles} triangles");
        assert!(
            (30_000..200_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn its_hinge_runs_along_the_back_of_the_shell_where_the_valves_meet() {
        let shell = Shell::new();
        let hinge = MusselDef::hinge(&shell);
        assert!(shell.solid.distance(hinge.pivot).abs() < 1e-3);
        assert!(
            (-SLAB.2 - 0.05..-SLAB.2 + 0.1).contains(&hinge.pivot.z),
            "its hinge is at z {}",
            hinge.pivot.z
        );
    }
}
