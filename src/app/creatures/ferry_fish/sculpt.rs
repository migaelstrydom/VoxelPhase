//! The ferry fish's look, sculpted in clay against the bestiary's slots.
//!
//! ```text
//!   body   its back sand, spotted vermilion, each spot ringed pale and the
//!          whole freckled; its belly mother-of-pearl; the edge of a gill
//!          cover, a ridge of knobs behind its eyes, its lateral line;
//!          a cup each eye sits deep in, lidded; skinned finer over
//!          its eyes, and without its muzzle                         (part 0)
//!   wings  a fan of long rays under a web as pale as pearl           (parts 1, 2)
//!   eyes   gold, each turning in its cup                            (parts 3, 4)
//!   mouth  its muzzle, skinned finer and laid over its body: a
//!          `Jaw` the rig opens (`mouth`)
//!   fins   its fringe down either edge and its tail: `Ripple`s the rig
//!          swings, drawn apart from the model
//! ```
//!
//! Everything is sculpted in the frame `form` sets out and moved into the
//! body's frame (`Deck::placed`) as it is finished. Where its body is too
//! fine for its grid (its eyes, its mouth) a patch skinned finer is laid
//! over it (`patch`), overlapping it in a band all round, so that neither
//! shows a cut edge.

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector3};

use super::super::fish_anatomy::{
    glass_eye, moved, point, surface_along, Crevices, Fin, FinPaint, GlassLook, Ray,
};
use super::super::palette::Paint;
use super::form::{self, Deck, EYES, EYE_RADIUS, SAND, SNOUT, TAIL_ROOT};
use super::mouth;
use crate::app::sculpt::{
    scatter, Clay, Ellipsoid, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::model::{MeshPrimitive, Model, ModelPart, Ripple};
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

/// The parts of its model: the body, a wing each side, and an eye each.
pub const BODY: usize = 0;
pub const RIGHT_WING: usize = 1;
pub const LEFT_WING: usize = 2;
pub const EYE_PARTS: [usize; 2] = [3, 4];

/// Side of the grid its body is skinned from, and the patches laid over
/// it, m.
const BODY_GRAIN: f32 = 0.017;
const PATCH_GRAIN: f32 = 0.01;
/// How wide the band is, all round a patch, in which it lies over the skin
/// of its body, and how far proud of that skin, m: enough that the patch
/// is always the one seen there.
const PATCH_OVERLAP: f32 = 0.03;
const PATCH_PROUD: f32 = 0.0012;

/// Its belly (linear): pale as pearl.
const PEARL: (f32, f32, f32) = (0.8, 0.77, 0.72);

/// Its spots: what a spot's heart and the pale ring round it multiply the
/// sand by.
const SPOT_HEART: (f32, f32, f32) = (2.25, 0.8, 0.34);
const SPOT_RING: (f32, f32, f32) = (1.45, 1.4, 1.25);

/// The cup each eye sits in: its radius, and how far behind the eye's
/// middle its own is, m: deep enough that all but the front of the eye is
/// hidden, and thick enough round the socket that the grid it is skinned
/// from holds it. The socket the eye turns in, and the lid round the cup's
/// rim: how many beads it is puckered into, and their smallest and largest.
const CUP: (f32, f32) = (0.19, 0.09);
const SOCKET: f32 = EYE_RADIUS + 0.01;
const LID_BEADS: usize = 22;
const LID_BEAD: (f32, f32) = (0.026, 0.034);

/// Paint settled in the creases of its skin.
pub(super) const CREVICES: Crevices = Crevices {
    depth: 0.55,
    lift: 0.2,
};

/// A fringe along an edge: where along its rim it starts and ends, how many
/// rays it is stretched on, and how far its longest reaches out, m.
#[derive(Clone, Copy)]
struct Fringe {
    from: f32,
    to: f32,
    rays: usize,
    reach: f32,
}

/// The fringe down each edge: behind the wings it runs to the tail; before
/// them, round the head nearly to the snout, and shorter.
const DORSAL: Fringe = Fringe {
    from: 0.42,
    to: -2.42,
    rays: 70,
    reach: 0.45,
};
const ANAL: Fringe = Fringe {
    from: 0.2,
    to: -2.42,
    rays: 64,
    reach: 0.45,
};
const BROW: Fringe = Fringe {
    from: 1.82,
    to: 1.08,
    rays: 20,
    reach: 0.24,
};
/// Rays in its tail, and in each wing.
const TAIL_RAYS: usize = 17;
const WING_RAYS: usize = 9;

/// The skin of its body over (`x`, `z`), anywhere along it.
fn on_back(x: f32, z: f32) -> Vector3<f32> {
    surface_along(&form::outline_form(), Vector3::new(x, 0.0, z), Vector3::y())
}

/// How far along the wave that runs down its fins a point at `z` is: its
/// distance back from the snout.
pub fn along(z: f32) -> f32 {
    SNOUT - z
}

/// Spots laid over its back as a plaice's are, one or none to each cell of
/// a grid across it (X and Z), each ringed paler: painted on what faces up,
/// fins and all, so that they run on from the body onto them.
struct Speckle {
    /// Side of a cell, m.
    cell: f32,
    /// Share of cells that hold a spot.
    presence: f32,
    /// Smallest and largest spot's radius, m.
    radius: (f32, f32),
    seed: u32,
}

impl Speckle {
    const BACK: Speckle = Speckle {
        cell: 0.46,
        presence: 0.78,
        radius: (0.055, 0.115),
        seed: 211,
    };

    /// How far `at` is from the heart of the nearest spot, as a share of
    /// that spot's radius, and how strongly that spot is coloured, 0 to 1.
    fn nearness(&self, at: Vector3<f32>) -> (f32, f32) {
        let (u, v) = (at.x / self.cell, at.z / self.cell);
        let (cu, cv) = (u.floor() as i32, v.floor() as i32);
        let mut nearest = (f32::MAX, 0.0);
        for du in -1..=1 {
            for dv in -1..=1 {
                let (iu, iv) = (cu + du, cv + dv);
                let key = (iu as u32).wrapping_mul(7919) ^ (iv as u32).wrapping_mul(104_729);
                let roll = |salt: u32| scatter(key, self.seed + salt) * 0.5 + 0.5;
                if roll(0) > self.presence {
                    continue;
                }
                let middle = (
                    iu as f32 + 0.15 + 0.7 * roll(1),
                    iv as f32 + 0.15 + 0.7 * roll(2),
                );
                let radius = self.radius.0 + (self.radius.1 - self.radius.0) * roll(3);
                let off = ((u - middle.0).hypot(v - middle.1) * self.cell) / radius;
                if off < nearest.0 {
                    nearest = (off, 0.55 + 0.45 * roll(4));
                }
            }
        }
        nearest
    }

    /// Paint `mesh`, by as much as `faces_up` gives for each vertex.
    fn paint(&self, mesh: &mut MeshPrimitive, faces_up: impl Fn(&Vertex) -> f32) {
        let (heart, ring) = (point(SPOT_HEART), point(SPOT_RING));
        for vertex in &mut mesh.vertices {
            let shown = faces_up(vertex).clamp(0.0, 1.0);
            if shown <= 0.0 {
                continue;
            }
            let (t, strength) = self.nearness(vertex.pos);
            // A heart that fades out raggedly, as paint soaked into skin.
            let ragged = 0.08
                * fbm_perlin_3d(
                    vertex.pos.x * 30.0,
                    0.0,
                    vertex.pos.z * 30.0,
                    2,
                    0.5,
                    self.seed,
                );
            let within = (1.0 - smooth(0.6, 1.0, t + ragged)) * strength;
            let round = (1.0 - ((t + ragged - 1.22) / 0.3).abs()).max(0.0) * strength;
            let wash = Vector3::repeat(1.0)
                .lerp(&ring, round * shown)
                .lerp(&heart, within * shown);
            vertex.color.x *= wash.x;
            vertex.color.y *= wash.y;
            vertex.color.z *= wash.z;
        }
    }
}

/// 0 below `from`, 1 above `to`, and smooth between.
pub(super) fn smooth(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// How much a vertex is of its back rather than its belly, by the way it
/// faces.
fn of_the_back(vertex: &Vertex) -> f32 {
    smooth(-0.6, -0.15, vertex.normal.y)
}

/// Paint `mark` on `mesh`, by as much as `shown` gives for each vertex.
fn paint_where(mesh: &mut MeshPrimitive, mark: &impl VertexPaint, shown: impl Fn(&Vertex) -> f32) {
    for vertex in &mut mesh.vertices {
        let (tint, cover) = mark.mark(vertex.pos);
        let wash = Vector3::repeat(1.0).lerp(&tint, (cover * shown(vertex)).clamp(0.0, 1.0));
        vertex.color.x *= wash.x;
        vertex.color.y *= wash.y;
        vertex.color.z *= wash.z;
    }
}

/// `mesh` moved from the frame it is sculpted in into its body's.
pub(super) fn placed(mut mesh: MeshPrimitive) -> MeshPrimitive {
    let deck = Deck::get();
    for vertex in &mut mesh.vertices {
        vertex.pos = deck.placed(vertex.pos);
    }
    mesh
}

/// The two halves of `mesh`: the triangles `keep` takes (by their
/// vertices' indices), and the rest in `material`, each with only the
/// vertices it uses.
fn split(
    mesh: &MeshPrimitive,
    keep: impl Fn([usize; 3]) -> bool,
    material: MaterialId,
) -> (MeshPrimitive, MeshPrimitive) {
    let mut halves = [
        (Vec::new(), Vec::new(), vec![u32::MAX; mesh.vertices.len()]),
        (Vec::new(), Vec::new(), vec![u32::MAX; mesh.vertices.len()]),
    ];
    for triangle in mesh.indices.chunks_exact(3) {
        let corners = [0, 1, 2].map(|i| triangle[i] as usize);
        let (vertices, indices, remap) = &mut halves[usize::from(!keep(corners))];
        for &index in triangle {
            if remap[index as usize] == u32::MAX {
                remap[index as usize] = vertices.len() as u32;
                vertices.push(mesh.vertices[index as usize]);
            }
            indices.push(remap[index as usize]);
        }
    }
    let [(kept, kept_indices, _), (rest, rest_indices, _)] = halves;
    (
        MeshPrimitive {
            vertices: kept,
            indices: kept_indices,
            material: mesh.material,
        },
        MeshPrimitive {
            vertices: rest,
            indices: rest_indices,
            material,
        },
    )
}

/// The edge of its gill cover, `t` from −1 (the −X edge) to 1: an arc
/// across the back of its head, bowed toward the tail.
fn gill_edge(t: f32) -> Vector3<f32> {
    on_back(0.1 + 1.08 * t, 0.78 + 0.34 * t * t)
}

/// Its lateral line, from the top of its gill cover down to its tail:
/// arched over where its wing roots, then straight.
const LATERAL_LINE: [(f32, f32); 8] = [
    (0.42, 0.98),
    (0.36, 0.7),
    (0.24, 0.38),
    (0.12, 0.0),
    (0.04, -0.6),
    (0.0, -1.3),
    (0.0, -2.0),
    (0.0, -2.55),
];

/// Its body, carved: its upper lip, the cups its eyes sit in, the ridge
/// behind them, its lateral line, the edge of its gill cover, and the
/// sockets its eyes turn in; without its jaw.
pub(super) fn carved() -> Clay {
    let mut clay = mouth::upper_lip(form::body_form().smoothing(0.05));

    // A cup each eye sits deep in, hiding all but its front.
    clay = clay.smoothing(0.04);
    for eye in 0..EYES.len() {
        let cup = form::eye_centre(eye) - form::eye_rest(eye) * CUP.1;
        clay = clay.add(Lump::ball(CUP.0, (cup.x, cup.y, cup.z)), SAND);
    }
    // A fleshy lid round each cup's rim, puckered, that the eye turns in.
    clay = clay.smoothing(0.02);
    for eye in 0..EYES.len() {
        let (at, gaze) = (form::eye_centre(eye), form::eye_rest(eye));
        let side = gaze
            .cross(&Vector3::y())
            .try_normalize(1e-6)
            .unwrap_or_else(Vector3::x);
        let up = side.cross(&gaze);
        // Where the cup's rim meets the socket, ahead of the eye's middle
        // and out from its line of sight.
        let (cup, back) = CUP;
        let ahead = (cup * cup - SOCKET * SOCKET - back * back) / (2.0 * back);
        let out = (SOCKET * SOCKET - ahead * ahead).sqrt();
        for bead in 0..LID_BEADS {
            let round = bead as f32 / LID_BEADS as f32 * TAU;
            let key = (eye * LID_BEADS + bead) as u32;
            let bead_at = at + gaze * ahead + (side * round.cos() + up * round.sin()) * out;
            let size = LID_BEAD.0 + (LID_BEAD.1 - LID_BEAD.0) * (0.5 + 0.5 * scatter(key, 34));
            clay = clay.add(Lump::ball(size, (bead_at.x, bead_at.y, bead_at.z)), SAND);
        }
    }

    // A ridge of knobs running back from between its eyes.
    clay = clay.smoothing(0.025);
    let (front, back) = (EYES[0].0, EYES[1].0);
    let start = (
        (front.0 + back.0) * 0.5 - 0.02,
        (front.1 + back.1) * 0.5 - 0.04,
    );
    for knob in 0..5 {
        let t = knob as f32 / 4.0;
        let (x, z) = (start.0 + 0.12 * t, start.1 - 0.4 * t);
        let at = on_back(x, z) - Vector3::y() * 0.012;
        let size = 0.042 - 0.012 * t + 0.006 * scatter(knob, 31);
        clay = clay.add(Lump::ball(size, (at.x, at.y, at.z)), SAND);
    }

    // The lateral line: a cord just proud of the skin.
    clay = clay.smoothing(0.012);
    for pair in LATERAL_LINE.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let steps = 4;
        for step in 0..steps {
            let mix = |a: f32, b: f32, t: f32| a + (b - a) * t;
            let at =
                |t: f32| on_back(mix(from.0, to.0, t), mix(from.1, to.1, t)) - Vector3::y() * 0.004;
            let (a, b) = (step as f32 / steps as f32, (step + 1) as f32 / steps as f32);
            clay = clay.add(
                Lump::Limb {
                    from: at(a),
                    to: at(b),
                    radii: (0.011, 0.011),
                },
                SAND,
            );
        }
    }

    // The gill cover's edge, a groove across the back of its head.
    let joints = 14;
    for joint in 0..joints {
        let t = |j: usize| j as f32 / joints as f32 * 2.0 - 1.0;
        let lift = Vector3::y() * 0.01;
        clay = clay.cut(
            Lump::Limb {
                from: gill_edge(t(joint)) + lift,
                to: gill_edge(t(joint + 1)) + lift,
                radii: (0.024, 0.024),
            },
            0.01,
        );
    }

    for eye in 0..EYES.len() {
        let at = form::eye_centre(eye);
        clay = clay.cut(Lump::ball(SOCKET, (at.x, at.y, at.z)), 0.015);
    }
    clay.relief(Relief::Lumps {
        frequency: 2.6,
        depth: 0.014,
        seed: 201,
    })
    .relief(Relief::Lumps {
        frequency: 15.0,
        depth: 0.0028,
        seed: 202,
    })
}

/// Paint its skin on `mesh`: pale as pearl beneath and the sand of its
/// back above, mottled and spotted as a plaice's is; its lips left
/// flushed.
pub(super) fn paint_skin(mesh: &mut MeshPrimitive) {
    let pearl = point(PEARL);
    let lips = mouth::lips();
    for vertex in &mut mesh.vertices {
        let under = 1.0 - of_the_back(vertex);
        let on_lips = lips.distance(vertex.pos) < 0.02;
        if !on_lips {
            let colour = vertex.color.xyz().lerp(&pearl, under);
            vertex.color.x = colour.x;
            vertex.color.y = colour.y;
            vertex.color.z = colour.z;
        }
    }
    Mottle {
        frequency: 1.6,
        value: 0.22,
        warmth: 0.18,
        seed: 203,
    }
    .paint(mesh);
    Mottle {
        frequency: 7.0,
        value: 0.1,
        warmth: 0.1,
        seed: 204,
    }
    .paint(mesh);
    // A flatfish's mottle, the ground its spots lie on: dark blotches,
    // pale flecks between them, and freckles.
    let marks = [
        (5.0, 0.45, (0.25, 0.5), (0.68, 0.66, 0.62), 205),
        (11.0, 0.35, (0.18, 0.42), (1.3, 1.26, 1.15), 206),
        (26.0, 0.4, (0.12, 0.3), (0.55, 0.48, 0.42), 207),
    ];
    for (frequency, presence, radius, tint, salt) in marks {
        let layer = Spots {
            frequency,
            presence,
            radius,
            tint: point(tint),
            salt,
        };
        paint_where(mesh, &layer, of_the_back);
    }
    Speckle::BACK.paint(mesh, |vertex| {
        let on_lips = lips.distance(vertex.pos) < 0.03;
        if on_lips {
            0.0
        } else {
            smooth(0.2, 0.55, vertex.normal.y)
        }
    });
}

/// `mesh` in its materials, moved into its body's frame: its back sand,
/// its belly pearl, and the inside of its mouth (`inside`, by vertex) skin
/// whichever way it faces, kept from the belly's sheen: the sky has no
/// place in there.
pub(super) fn dressed(mesh: &MeshPrimitive, inside: &[bool]) -> Vec<MeshPrimitive> {
    let (skin, mouth) = split(
        mesh,
        |triangle| triangle.iter().filter(|&&i| inside[i]).count() < 2,
        Paint::Skin.id(),
    );
    let (back, belly) = split(
        &skin,
        |triangle| {
            triangle
                .iter()
                .map(|&i| skin.vertices[i].normal.y)
                .sum::<f32>()
                > -0.75
        },
        Paint::Nacre.id(),
    );
    [back, belly, mouth]
        .into_iter()
        .filter(|mesh| !mesh.indices.is_empty())
        .map(placed)
        .collect()
}

/// Where the middle of `triangle` of `mesh` is.
fn middle_of(mesh: &MeshPrimitive, triangle: [usize; 3]) -> Vector3<f32> {
    triangle
        .iter()
        .map(|&i| mesh.vertices[i].pos)
        .sum::<Vector3<f32>>()
        / 3.0
}

/// The skin of `form` over `region`, skinned finer than its body, to lay
/// over the body's skin where `region` is taken out of it: skinned a
/// little past the band it overlaps the body in, so that its cut edge falls
/// outside what is kept; painted by `paint` whole; kept to the band; and
/// stood a hair proud.
pub(super) fn patch(
    form: Clay,
    region: Ellipsoid,
    paint: impl FnOnce(&mut MeshPrimitive),
) -> MeshPrimitive {
    let reach = Ellipsoid {
        radii: region.radii.add_scalar(PATCH_OVERLAP + 2.0 * PATCH_GRAIN),
        ..region
    };
    let mut mesh = form
        .within(Lump::Ball(reach))
        .skin(PATCH_GRAIN, Paint::Skin.id());
    paint(&mut mesh);
    let keep = Clay::new(0.0).add(Lump::Ball(region), SAND);
    let (mut patch, _) = split(
        &mesh,
        |triangle| keep.distance(middle_of(&mesh, triangle)) < PATCH_OVERLAP,
        Paint::Skin.id(),
    );
    for vertex in &mut patch.vertices {
        vertex.pos += vertex.normal * PATCH_PROUD;
    }
    patch
}

/// Round both its eyes: the lids and sockets they turn in, too fine for
/// its body's grid.
fn eyes_region() -> Ellipsoid {
    let (first, second) = (form::eye_centre(0), form::eye_centre(1));
    let between = second - first;
    Ellipsoid {
        radii: Vector3::new(between.norm() * 0.5 + 0.24, 0.28, 0.24),
        centre: (first + second) * 0.5,
        turn: UnitQuaternion::rotation_between(&Vector3::x(), &between)
            .unwrap_or_else(UnitQuaternion::identity),
    }
}

/// Paint its body's skin on `mesh`: its skin, its lateral line paler than
/// what it runs over, and the creases.
fn paint_body(mesh: &mut MeshPrimitive) {
    paint_skin(mesh);
    let line = LATERAL_LINE.windows(2).fold(Clay::new(0.0), |clay, pair| {
        clay.add(
            Lump::Limb {
                from: on_back(pair[0].0, pair[0].1),
                to: on_back(pair[1].0, pair[1].1),
                radii: (0.02, 0.02),
            },
            SAND,
        )
    });
    for vertex in &mut mesh.vertices {
        if line.distance(vertex.pos) < 0.0 {
            vertex.color.x *= 1.3;
            vertex.color.y *= 1.28;
            vertex.color.z *= 1.2;
        }
    }
    CREVICES.paint(mesh);
}

/// Its body: carved, skinned and painted, without its eyes and muzzle; and
/// its eyes skinned finer and laid over it. Its muzzle is the rig's to
/// draw (`mouth::muzzle`).
fn body() -> Vec<MeshPrimitive> {
    let whole = carved();
    let mut mesh = whole.skin(BODY_GRAIN, Paint::Skin.id());
    paint_body(&mut mesh);
    let patched = [mouth::muzzle_region(), eyes_region()]
        .into_iter()
        .fold(Clay::new(0.0), |clay, region| {
            clay.add(Lump::Ball(region), SAND)
        });
    let (rest, _) = split(
        &mesh,
        |triangle| patched.distance(middle_of(&mesh, triangle)) > 0.0,
        Paint::Skin.id(),
    );
    let mut meshes = dressed(&rest, &vec![false; rest.vertices.len()]);
    let eyes = patch(whole, eyes_region(), paint_body);
    meshes.extend(dressed(&eyes, &vec![false; eyes.vertices.len()]));
    meshes
}

/// Its eyes, each about its own middle in its body's frame: gold, clear,
/// and wet.
fn eye(eye: usize) -> MeshPrimitive {
    let mut mesh = glass_eye(
        EYE_RADIUS,
        GlassLook {
            iris: (0.5, 0.36, 0.1),
            iris_angle: 1.6,
            pupil_share: 0.5,
            bloodshot: 0.3,
            film: 0.08,
        },
    );
    mesh.material = Paint::Wet.id();
    let deck = Deck::get();
    moved(
        mesh,
        UnitQuaternion::face_towards(&form::eye_rest(eye), &Vector3::z()),
        deck.placed(form::eye_centre(eye)),
    )
}

/// The rays of the wing on `side`, from its leading edge back.
pub fn wing_rays(side: f32) -> Vec<Ray> {
    let root = form::wing_root(side);
    (0..WING_RAYS)
        .map(|ray| {
            let across = ray as f32 / (WING_RAYS - 1) as f32;
            let at = root + Vector3::z() * (0.22 - 0.44 * across);
            let sweep = -0.12 + 1.3 * across.powf(1.2);
            let long = 2.05 - 1.25 * across.powf(1.3) + 0.04 * (across * 11.0).sin();
            let lift = 0.08 - 0.07 * across;
            let way = Vector3::new(side * sweep.cos(), lift, -sweep.sin()).normalize();
            Ray::new(at, at + way * long).bowed(Vector3::new(
                0.0,
                0.05 * long,
                0.05 * long * (1.0 - across),
            ))
        })
        .collect()
}

/// The wing on `side`: a pale web on long rays, veined and faintly
/// spotted, its root flushed.
fn wing(side: f32) -> MeshPrimitive {
    let paint = FinPaint {
        ray: (0.8, 0.73, 0.6),
        web: (0.68, 0.54, 0.48),
        edge: (0.88, 0.84, 0.78),
    };
    let mut web = Fin::on(wing_rays(side), paint, if side > 0.0 { 221 } else { 222 })
        .thick(0.014, 0.0035)
        .edged(0.1, 0.05)
        .bellied(-side * 0.035)
        .mesh(9, 34, Paint::Nacre.id());
    Mottle {
        frequency: 3.0,
        value: 0.15,
        warmth: 0.2,
        seed: 223,
    }
    .paint(&mut web);
    Veins {
        frequency: 4.0,
        width: 0.03,
        tint: Vector3::new(0.85, 0.62, 0.6),
        seed: 224,
    }
    .paint(&mut web);
    Speckle {
        cell: 0.4,
        presence: 0.45,
        radius: (0.03, 0.06),
        seed: 225,
    }
    .paint(&mut web, |_| 0.35);
    placed(web)
}

/// The rays of `fringe` down the edge on `side`, rooted along its rim:
/// short at either end, longest behind its middle, lying out from the rim
/// and swept back.
fn fringe_rays(side: f32, fringe: Fringe) -> Vec<Ray> {
    let count = fringe.rays;
    (0..count)
        .map(|ray| {
            let t = ray as f32 / (count - 1) as f32;
            let z = fringe.from + (fringe.to - fringe.from) * t;
            let root = form::rim(side, z) - Vector3::x() * (side * 0.012);
            let run = (form::rim(side, z + 0.03) - form::rim(side, z - 0.03)).normalize();
            let mut out = run.cross(&Vector3::y()).normalize();
            if out.x * side < 0.0 {
                out = -out;
            }
            let long = 0.05
                + fringe.reach * (PI * t.powf(0.8)).sin().max(0.0).powf(0.75)
                + 0.012 * scatter(ray as u32, 231);
            let way = (out * 0.94 - run * 0.34 - Vector3::y() * 0.05).normalize();
            Ray::new(root, root + way * long).bowed(-run * 0.03 * long)
        })
        .collect()
}

/// A fringe down the edge on `side`, as the rig ripples it.
fn fringe(side: f32, extent: Fringe, seed: u32) -> Ripple {
    let rays = fringe_rays(side, extent);
    let root: Vec<(Vector3<f32>, f32)> = rays
        .iter()
        .map(|ray| (Deck::get().placed(ray.root), along(ray.root.z)))
        .collect();
    let paint = FinPaint {
        ray: (0.23, 0.19, 0.11),
        web: (0.3, 0.255, 0.16),
        edge: (0.46, 0.41, 0.3),
    };
    let mut mesh = Fin::on(rays, paint, seed)
        .thick(0.006, 0.0022)
        .edged(0.035, 0.05)
        .bellied(0.006)
        .mesh(2, 12, Paint::Skin.id());
    Mottle {
        frequency: 4.0,
        value: 0.2,
        warmth: 0.15,
        seed: seed + 1,
    }
    .paint(&mut mesh);
    for (frequency, presence, radius, tint, salt) in [
        (5.0, 0.45, (0.25, 0.5), (0.72, 0.7, 0.66), 205),
        (11.0, 0.35, (0.18, 0.42), (1.25, 1.22, 1.12), 206),
    ] {
        let layer = Spots {
            frequency,
            presence,
            radius,
            tint: point(tint),
            salt,
        };
        paint_where(&mut mesh, &layer, |vertex| vertex.normal.y.abs());
    }
    Speckle::BACK.paint(&mut mesh, |vertex| vertex.normal.y.max(0.0));
    Ripple::rooted(placed(mesh), &root)
}

/// The fringe down its +X edge.
pub fn dorsal() -> Ripple {
    fringe(1.0, DORSAL, 241)
}

/// The fringe down its −X edge.
pub fn anal() -> Ripple {
    fringe(-1.0, ANAL, 243)
}

/// The fringe round its head on the +X side, before the wing.
pub fn brow_right() -> Ripple {
    fringe(1.0, BROW, 245)
}

/// The fringe round its head on the −X side, before the wing.
pub fn brow_left() -> Ripple {
    fringe(-1.0, BROW, 247)
}

/// Its tail: a round fan off the end of its stalk, flapped up and down
/// whole.
pub fn tail() -> Ripple {
    let rays: Vec<Ray> = (0..TAIL_RAYS)
        .map(|ray| {
            let across = ray as f32 / (TAIL_RAYS - 1) as f32 * 2.0 - 1.0;
            let root = Vector3::new(0.13 * across, 0.0, TAIL_ROOT + 0.04);
            let spread = 1.05 * across;
            let long = 0.7 + 0.16 * (1.0 - across * across) + 0.015 * scatter(ray as u32, 251);
            let way = Vector3::new(spread.sin(), -0.02, -spread.cos());
            Ray::new(root, root + way * long).bowed(Vector3::new(0.0, 0.0, 0.02 * long))
        })
        .collect();
    let paint = FinPaint {
        ray: (0.22, 0.18, 0.1),
        web: (0.3, 0.25, 0.16),
        edge: (0.48, 0.42, 0.31),
    };
    let mut mesh = Fin::on(rays, paint, 252)
        .thick(0.01, 0.003)
        .edged(0.1, 0.05)
        .mesh(4, 16, Paint::Skin.id());
    Mottle {
        frequency: 4.0,
        value: 0.2,
        warmth: 0.15,
        seed: 253,
    }
    .paint(&mut mesh);
    Speckle::BACK.paint(&mut mesh, |vertex| vertex.normal.y.max(0.0));
    let deck = Deck::get();
    let root = [
        (
            deck.placed(Vector3::new(-0.2, 0.0, TAIL_ROOT)),
            along(TAIL_ROOT),
        ),
        (
            deck.placed(Vector3::new(0.2, 0.0, TAIL_ROOT)),
            along(TAIL_ROOT),
        ),
    ];
    Ripple::rooted(placed(mesh), &root)
}

/// The whole model, against the palette's slots: its body, wings and
/// eyes. Its muzzle is apart (`mouth::muzzle`).
pub fn model() -> Model {
    let mut parts = vec![ModelPart::new(Vec::new()); 5];
    parts[BODY] = ModelPart::new(body());
    parts[RIGHT_WING] = ModelPart::new(vec![wing(1.0)]);
    parts[LEFT_WING] = ModelPart::new(vec![wing(-1.0)]);
    for (eye_index, part) in EYE_PARTS.into_iter().enumerate() {
        parts[part] = ModelPart::new(vec![eye(eye_index)]);
    }
    Model::flat(parts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn triangles(meshes: &[MeshPrimitive]) -> usize {
        meshes.iter().map(|mesh| mesh.indices.len() / 3).sum()
    }

    #[test]
    fn it_is_a_hundred_thousand_triangles_or_so_and_sits_about_its_middle() {
        let model = model();
        let all: Vec<MeshPrimitive> = model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter().cloned())
            .chain(mouth::muzzle().rest().iter().cloned())
            .collect();
        let total = triangles(&all);
        println!("a ferry fish is {total} triangles");
        // Its eyes and muzzle are skinned finer, which is most of what it
        // has over the bestiary's 150k to 220k: it is the biggest of them.
        assert!((60_000..320_000).contains(&total), "{total} triangles");
        for mesh in &all {
            let count = mesh.vertices.len() as u32;
            assert!(mesh.indices.iter().all(|index| *index < count));
            for vertex in &mesh.vertices {
                assert!(vertex.pos.iter().all(|x| x.is_finite()));
                assert!(vertex.pos.norm() < 4.5, "{:?} is far out", vertex.pos);
            }
        }
        // Its back is sand and its belly pearl, apart.
        let body = &model.parts[BODY].primitives;
        assert_eq!(body[0].material, Paint::Skin.id());
        assert_eq!(body[1].material, Paint::Nacre.id());
        let facing = |mesh: &MeshPrimitive| {
            mesh.vertices.iter().map(|v| v.normal.y).sum::<f32>() / mesh.vertices.len() as f32
        };
        assert!(
            facing(&body[0]) > 0.4,
            "its back faces {}",
            facing(&body[0])
        );
        assert!(
            facing(&body[1]) < -0.6,
            "its belly faces {}",
            facing(&body[1])
        );
    }

    #[test]
    fn its_fins_root_on_its_body_and_reach_out_from_it() {
        let deck = Deck::get();
        let form = form::outline_form();
        for (name, ripple) in [
            ("dorsal", dorsal()),
            ("anal", anal()),
            ("brow right", brow_right()),
            ("brow left", brow_left()),
            ("tail", tail()),
        ] {
            let farthest = ripple
                .hinges()
                .iter()
                .zip(&ripple.rest().vertices)
                .map(|(hinge, vertex)| (vertex.pos - hinge.at).norm())
                .fold(0.0, f32::max);
            assert!(farthest > 0.2, "the {name} fin reaches {farthest} m");
            for hinge in ripple.hinges() {
                let at = hinge.at + deck.centre;
                let off = form.distance(at);
                assert!(
                    off < 0.02,
                    "the {name} fin roots {off} m off the body at {at:?}"
                );
            }
        }
        // The wave runs from the front of the fringe to the tail.
        let tail = tail();
        let dorsal = dorsal();
        let last = dorsal.hinges().iter().map(|h| h.along).fold(0.0, f32::max);
        assert!(tail.hinges()[0].along > last - 0.5);
    }

    #[test]
    fn its_wing_rays_leave_the_body_by_the_hinge_and_reach_two_metres() {
        for side in [-1.0, 1.0] {
            let root = form::wing_root(side);
            let rays = wing_rays(side);
            assert!(rays.iter().all(|ray| (ray.root - root).norm() < 0.25));
            let span = rays.iter().map(|ray| ray.tip.x * side).fold(0.0, f32::max);
            assert!(span > 3.0, "a wing spans to {span}");
        }
    }
}
