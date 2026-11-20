//! The scroll's shape: the open sheet, the two rolls it is wound on, the
//! gilt knobs on their rods, and the figure laid in gold on the sheet.
//!
//! ```text
//!        knob ●            ● knob
//!          ┌──┐ ╭────────╮ ┌─┐
//!          │  │ │ script │ │ │      the sheet bows back between
//!     roll │  │ │ figure │ │ │ roll the rolls, its front facing +z;
//!          │  │ │ script │ │ │      the rolls stand upright, as a
//!          └──┘ ╰────────╯ └─┘      reader holds them
//!        knob ●            ● knob
//! ```
//!
//! The model is about the sheet's middle, up `+y`.

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3, Vector4};

use super::figure::Figure;
use super::texture::VERSO_SPAN;
use crate::app::sculpt::{Ellipsoid, Strand};
use crate::app::spawnables::shared::gilt::{GOLD, GOLD_BEAD};
use crate::model::MeshPrimitive;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// The open sheet, from one roll to the other and from edge to edge, m.
pub const SHEET_WIDTH: f32 = 0.5;
pub const SHEET_HEIGHT: f32 = 0.36;

/// The square the figure is drawn in, m, in the sheet's middle.
pub const FIGURE_SIZE: f32 = 0.26;

/// How far the sheet's middle stands back from its ends, m: enough that it
/// reads as a sheet held open, not a board.
const BOW: f32 = 0.033;

/// The sheet's thickness, m: far more than papyrus, so its edge is seen.
const THICKNESS: f32 = 0.0012;

/// Columns of the sheet's mesh across it: smooth along its bow.
const SHEET_COLUMNS: usize = 32;

/// The rolls' radii, m: more of the book is wound on the left.
const LEFT_ROLL: f32 = 0.05;
const RIGHT_ROLL: f32 = 0.039;

/// Sides round a roll.
const ROLL_SIDES: usize = 48;

/// One turn of the sheet on a roll, as seen at its end, m: thicker than
/// papyrus, so the spiral is seen, but fine enough not to be a log's rings.
const LAYER: f32 = 0.0028;

/// How dark the gap between two turns is at its deepest, as a share of the
/// papyrus.
const LAYER_GAP: f32 = 0.22;

/// Rings of the roll's end to a turn of the spiral, so the gap between
/// turns is drawn and not lost between two rings.
const RINGS_PER_LAYER: f32 = 4.0;

/// The rod each roll is wound on: its radius, m, where it shows in the end
/// of the roll.
const ROD_RADIUS: f32 = 0.0065;

/// A knob on each end of each rod, turned: its profile, as how far out from
/// the end of the roll, m, and the radius there. A collar over the end of
/// the roll, a neck, a bulb and a point.
const KNOB: [(f32, f32); 9] = [
    (-0.001, 0.013),
    (0.003, 0.013),
    (0.005, 0.0068),
    (0.01, 0.007),
    (0.018, 0.012),
    (0.026, 0.0112),
    (0.033, 0.0064),
    (0.039, 0.0039),
    (0.042, 0.0),
];

/// The gold figure: the radius of its lines, how far their middles stand
/// off the sheet, m, and the dot at each point it names, flattened to the
/// sheet; none reaches through to the verso.
const LINE_RADIUS: f32 = 0.0017;
const LINE_LIFT: f32 = 0.0008;
const DOT_RADIUS: f32 = 0.0045;
const DOT_DEPTH: f32 = 0.0016;

/// The figure seen through the sheet from behind: thinner, and barely
/// standing off the verso, as the gold's light through backlit papyrus.
const THROUGH_RADIUS: f32 = 0.0011;
const THROUGH_LIFT: f32 = 0.0002;

/// The sillybos, the title tag hung from the end of a Greek roll: its
/// size, m, how far below the top of the left roll it hangs, the cord's
/// radius, and its dye (multiplying the papyrus): vermilion, the one spot
/// of the garden's colour on a thing of reason.
const TAG_WIDTH: f32 = 0.034;
const TAG_HEIGHT: f32 = 0.07;
const TAG_DROP: f32 = 0.016;
const TAG_CORD: f32 = 0.0011;
const TAG_DYE: Vector3<f32> = Vector3::new(0.9, 0.16, 0.07);
const CORD_COLOUR: Vector3<f32> = Vector3::new(0.32, 0.1, 0.05);

/// Which way the tag hangs from the roll's axis: out to the left and a
/// little forward, so it is seen from the front.
const TAG_OUT: Vector3<f32> = Vector3::new(-0.94, 0.0, 0.34);

/// A line on the sheet is laid in pieces no longer than this, m, so it
/// follows the sheet's bow.
const LINE_PIECE: f32 = 0.008;

/// Where the sheet is at `x` across it and `y` up it from its middle, m,
/// and which way its front faces.
pub fn sheet_at(x: f32, y: f32) -> (Vector3<f32>, Vector3<f32>) {
    let across = 2.0 * x / SHEET_WIDTH;
    let z = -BOW * (1.0 - across * across);
    let slope = 4.0 * BOW * across / SHEET_WIDTH;
    (
        Vector3::new(x, y, z),
        Vector3::new(-slope, 0.0, 1.0).normalize(),
    )
}

/// The rolls: where each one's axis stands, and its radius. Each meets the
/// sheet's end along the sheet, so the sheet runs on round it unbroken.
pub fn rolls() -> [(Vector3<f32>, f32); 2] {
    let roll = |x: f32, radius: f32| {
        let (end, front) = sheet_at(x, 0.0);
        (end - front * radius, radius)
    };
    [
        roll(-0.5 * SHEET_WIDTH, LEFT_ROLL),
        roll(0.5 * SHEET_WIDTH, RIGHT_ROLL),
    ]
}

/// A vertex in white, open to the light.
fn vertex(pos: Vector3<f32>, normal: Vector3<f32>, uv: Vector2<f32>) -> Vertex {
    Vertex {
        pos,
        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
        tex_coords: uv,
        normal,
        ao: 1.0,
    }
}

/// Add the quad `corners`, in order round it, facing the way its vertices'
/// normals do.
fn quad(mesh: &mut MeshPrimitive, corners: [Vertex; 4]) {
    let facing: Vector3<f32> = corners.iter().map(|v| v.normal).sum();
    let winds = (corners[1].pos - corners[0].pos).cross(&(corners[2].pos - corners[0].pos));
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(corners);
    let order = if winds.dot(&facing) >= 0.0 {
        [0, 1, 2, 0, 2, 3]
    } else {
        [0, 2, 1, 0, 3, 2]
    };
    mesh.indices.extend(order.map(|i| first + i));
}

fn mesh_in(material: MaterialId) -> MeshPrimitive {
    MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    }
}

/// Add `more` to `mesh`.
fn append(mesh: &mut MeshPrimitive, more: MeshPrimitive) {
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(more.vertices);
    mesh.indices
        .extend(more.indices.into_iter().map(|index| index + first));
}

/// Across the sheet, column `i` of [`SHEET_COLUMNS`], m from its middle.
fn column(i: usize) -> f32 {
    (i as f32 / SHEET_COLUMNS as f32 - 0.5) * SHEET_WIDTH
}

/// The sheet's front, the recto, its texture spanning it once.
pub fn recto_mesh(material: MaterialId) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    let half = 0.5 * SHEET_HEIGHT;
    let at = |x: f32, y: f32| {
        let (pos, normal) = sheet_at(x, y);
        let uv = Vector2::new(x / SHEET_WIDTH + 0.5, 0.5 - y / SHEET_HEIGHT);
        vertex(pos, normal, uv)
    };
    for i in 0..SHEET_COLUMNS {
        let (x0, x1) = (column(i), column(i + 1));
        quad(
            &mut mesh,
            [at(x0, -half), at(x1, -half), at(x1, half), at(x0, half)],
        );
    }
    mesh
}

/// The verso and the cut edges of the sheet, the rolls and the spiral in
/// the end of each.
pub fn verso_mesh(material: MaterialId) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    let half = 0.5 * SHEET_HEIGHT;
    let uv = |a: f32, b: f32| Vector2::new(a / VERSO_SPAN, b / VERSO_SPAN);

    // The back of the open sheet.
    let back = |x: f32, y: f32| {
        let (pos, front) = sheet_at(x, y);
        vertex(pos - front * THICKNESS, -front, uv(-x, -y))
    };
    for i in 0..SHEET_COLUMNS {
        let (x0, x1) = (column(i), column(i + 1));
        quad(
            &mut mesh,
            [
                back(x0, -half),
                back(x1, -half),
                back(x1, half),
                back(x0, half),
            ],
        );
    }

    // Its top and bottom edges.
    for (y, up) in [(half, 1.0), (-half, -1.0)] {
        let normal = Vector3::new(0.0, up, 0.0);
        for i in 0..SHEET_COLUMNS {
            let (x0, x1) = (column(i), column(i + 1));
            let (f0, n0) = sheet_at(x0, y);
            let (f1, n1) = sheet_at(x1, y);
            quad(
                &mut mesh,
                [
                    vertex(f0, normal, uv(x0, 0.0)),
                    vertex(f1, normal, uv(x1, 0.0)),
                    vertex(f1 - n1 * THICKNESS, normal, uv(x1, THICKNESS)),
                    vertex(f0 - n0 * THICKNESS, normal, uv(x0, THICKNESS)),
                ],
            );
        }
    }

    for (axis, radius) in rolls() {
        roll_side(&mut mesh, axis, radius);
        for end in [1.0, -1.0] {
            roll_end(&mut mesh, axis, radius, end);
        }
    }
    let [(left, left_radius), _] = rolls();
    sillybos(&mut mesh, left, left_radius);
    mesh
}

/// The sillybos: a tag of papyrus dyed vermilion, hung by a cord from the
/// end of the roll about `axis` of `radius`, down its outside.
fn sillybos(mesh: &mut MeshPrimitive, axis: Vector3<f32>, radius: f32) {
    let out = TAG_OUT.normalize();
    let across = Vector3::y().cross(&out);
    let top =
        axis + out * (radius + 0.002 + THICKNESS) + Vector3::y() * (0.5 * SHEET_HEIGHT - TAG_DROP);
    let dye = Vector4::new(TAG_DYE.x, TAG_DYE.y, TAG_DYE.z, 1.0);
    let corner = |a: f32, b: f32, inset: f32, normal: Vector3<f32>| {
        let pos =
            top + across * (a * 0.5 * TAG_WIDTH) - Vector3::y() * (b * TAG_HEIGHT) - out * inset;
        let mut v = vertex(
            pos,
            normal,
            Vector2::new(a * TAG_WIDTH / VERSO_SPAN, b * TAG_HEIGHT / VERSO_SPAN),
        );
        v.color = dye;
        v
    };
    for (inset, normal) in [(0.0, out), (THICKNESS, -out)] {
        quad(
            mesh,
            [
                corner(-1.0, 0.0, inset, normal),
                corner(1.0, 0.0, inset, normal),
                corner(1.0, 1.0, inset, normal),
                corner(-1.0, 1.0, inset, normal),
            ],
        );
    }

    // The cord: from the tag's top, up over the end of the roll, to the rod.
    let rod = axis + Vector3::y() * (0.5 * SHEET_HEIGHT + 0.003);
    let path = vec![
        (top + out * 0.0008, TAG_CORD),
        (
            axis + out * (radius + 0.002) + Vector3::y() * (0.5 * SHEET_HEIGHT + 0.002),
            TAG_CORD,
        ),
        (rod + out * (radius * 0.5), TAG_CORD),
        (rod + out * 0.012, TAG_CORD),
    ];
    Strand::through(path)
        .sides(5)
        .rings(3)
        .lay(mesh, |_| (CORD_COLOUR, 1.0));
}

/// The side of a roll about `axis` of `radius`: the verso, wound round.
fn roll_side(mesh: &mut MeshPrimitive, axis: Vector3<f32>, radius: f32) {
    let half = 0.5 * SHEET_HEIGHT;
    // The texture wraps round it a whole number of times.
    let wraps = (TAU * radius / VERSO_SPAN).round().max(1.0);
    let at = |side: usize, y: f32| {
        let a = TAU * side as f32 / ROLL_SIDES as f32;
        let out = Vector3::new(a.cos(), 0.0, a.sin());
        let u = wraps * side as f32 / ROLL_SIDES as f32;
        vertex(
            axis + out * radius + Vector3::new(0.0, y, 0.0),
            out,
            Vector2::new(u, 0.5 - y / VERSO_SPAN),
        )
    };
    for side in 0..ROLL_SIDES {
        quad(
            mesh,
            [
                at(side, -half),
                at(side + 1, -half),
                at(side + 1, half),
                at(side, half),
            ],
        );
    }
}

/// The end of a roll, `up` (+1) or down (-1): the sheet's edge wound in a
/// spiral, each turn shadowed where it lies on the next.
fn roll_end(mesh: &mut MeshPrimitive, axis: Vector3<f32>, radius: f32, up: f32) {
    let normal = Vector3::new(0.0, up, 0.0);
    let level = axis + normal * 0.5 * SHEET_HEIGHT;
    let rings = (((radius - ROD_RADIUS) / LAYER) * RINGS_PER_LAYER).ceil() as usize;
    let at = |ring: usize, side: usize| {
        let r = ROD_RADIUS + (radius - ROD_RADIUS) * ring as f32 / rings as f32;
        let a = TAU * side as f32 / ROLL_SIDES as f32;
        let offset = Vector3::new(a.cos(), 0.0, a.sin()) * r;
        let turn = ((radius - r) / LAYER + a / TAU).fract();
        let gap = (1.0 - (turn - 0.5).abs() / 0.25).clamp(0.0, 1.0);
        let shade = 1.0 - LAYER_GAP * gap;
        let mut v = vertex(
            level + offset,
            normal,
            Vector2::new(offset.x / VERSO_SPAN, offset.z / VERSO_SPAN),
        );
        v.color = Vector4::new(shade, shade, shade, 1.0);
        v.ao = 1.0 - 0.5 * gap;
        v
    };
    for ring in 0..rings {
        for side in 0..ROLL_SIDES {
            quad(
                mesh,
                [
                    at(ring, side),
                    at(ring + 1, side),
                    at(ring + 1, side + 1),
                    at(ring, side + 1),
                ],
            );
        }
    }
}

/// The knobs on the rods' ends, gilt.
pub fn knobs_mesh(material: MaterialId) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    for (axis, _) in rolls() {
        for up in [1.0, -1.0] {
            let end = axis + Vector3::new(0.0, up * 0.5 * SHEET_HEIGHT, 0.0);
            let profile = KNOB
                .iter()
                .map(|&(out, radius)| (end + Vector3::new(0.0, up * out, 0.0), radius))
                .collect();
            Strand::through(profile)
                .sides(20)
                .rings(3)
                .lay(&mut mesh, |_| (GOLD, 1.0));
        }
    }
    mesh
}

/// Which face of the sheet a figure is laid on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Face {
    /// The recto, in gold, with a dot at every point the figure names.
    Recto,
    /// The verso: the recto's gold seen through the sheet, so the figure is
    /// never lost as the scroll turns its back.
    Verso,
}

/// `figure` laid on `face` of the sheet, in its square in the sheet's
/// middle.
pub fn figure_mesh(figure: &Figure, face: Face, material: MaterialId) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    let (radius, lift) = match face {
        Face::Recto => (LINE_RADIUS, LINE_LIFT),
        Face::Verso => (THROUGH_RADIUS, THROUGH_LIFT),
    };
    let on_sheet = |p: Vector2<f32>, lift: f32| {
        let (pos, front) = sheet_at(p.x * 0.5 * FIGURE_SIZE, p.y * 0.5 * FIGURE_SIZE);
        match face {
            Face::Recto => pos + front * lift,
            Face::Verso => pos - front * (THICKNESS + lift),
        }
    };
    for stroke in &figure.strokes {
        let mut path = vec![stroke[0]];
        for pair in stroke.windows(2) {
            let length = (pair[1] - pair[0]).norm() * 0.5 * FIGURE_SIZE;
            let pieces = (length / LINE_PIECE).ceil().max(1.0) as usize;
            path.extend((1..=pieces).map(|i| pair[0].lerp(&pair[1], i as f32 / pieces as f32)));
        }
        Strand::through(
            path.into_iter()
                .map(|p| (on_sheet(p, lift), radius))
                .collect(),
        )
        .sides(6)
        .rings(1)
        .lay(&mut mesh, |_| (GOLD, 1.0));
    }
    if face == Face::Verso {
        return mesh;
    }
    let bead = Colour::new(GOLD_BEAD.x, GOLD_BEAD.y, GOLD_BEAD.z, 1.0);
    for &point in &figure.points {
        let at = on_sheet(point, lift);
        append(
            &mut mesh,
            Ellipsoid::new((DOT_RADIUS, DOT_RADIUS, DOT_DEPTH), (at.x, at.y, at.z))
                .mesh(bead, material),
        );
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rolls meet the sheet's ends along it: each end lies on its
    /// roll, and the sheet runs on square to the radius there.
    #[test]
    fn the_sheet_runs_onto_its_rolls() {
        for ((axis, radius), x) in rolls().into_iter().zip([-0.5, 0.5]) {
            let (end, front) = sheet_at(x * SHEET_WIDTH, 0.0);
            assert!(((end - axis).norm() - radius).abs() < 1e-5);
            assert!((end - axis).normalize().dot(&front) > 1.0 - 1e-5);
        }
    }

    /// Each face's gold stays on its face: nothing of the figure reaches
    /// through the sheet to the other.
    #[test]
    fn the_figure_stays_on_its_face() {
        for proposition in crate::app::spawnables::Proposition::ALL {
            let figure = proposition.figure();
            let depths = |face| {
                figure_mesh(&figure, face, MaterialId(0))
                    .vertices
                    .iter()
                    .map(|vertex| {
                        let (on, front) = sheet_at(vertex.pos.x, vertex.pos.y);
                        (vertex.pos - on).dot(&front)
                    })
                    .collect::<Vec<f32>>()
            };
            assert!(
                depths(Face::Recto).iter().all(|&d| d > -THICKNESS),
                "{proposition:?}"
            );
            assert!(
                depths(Face::Verso).iter().all(|&d| d < 0.0),
                "{proposition:?}"
            );
        }
    }

    /// The figure lies within the space the script leaves it.
    #[test]
    fn the_figure_square_fits_between_the_rolls() {
        let [(left, left_radius), (right, right_radius)] = rolls();
        assert!(left.x + left_radius < -0.5 * FIGURE_SIZE);
        assert!(right.x - right_radius > 0.5 * FIGURE_SIZE);
        assert!(FIGURE_SIZE < SHEET_HEIGHT);
    }

    /// Every face of the paper faces the way its normals say.
    #[test]
    fn the_quads_wind_outward() {
        for mesh in [recto_mesh(MaterialId(0)), verso_mesh(MaterialId(1))] {
            for tri in mesh.indices.chunks(3) {
                let [a, b, c] = [0, 1, 2].map(|i| &mesh.vertices[tri[i] as usize]);
                let winds = (b.pos - a.pos).cross(&(c.pos - a.pos));
                assert!(winds.dot(&(a.normal + b.normal + c.normal)) > 0.0);
            }
        }
    }
}
