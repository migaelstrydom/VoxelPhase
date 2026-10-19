//! The sled's look and proportions: a low, rounded plastic hull with a
//! rubber deck, a chrome bumper round its waist and four chrome pods under
//! its corners.
//!
//! ```text
//!            deck pad (rubber)
//!        ╭──────────────────────╮  ← nose kicks up
//!   ═════╡     hull (plastic)    ╞═══  chrome bumper at the waist
//!        ╰─┬──┬────────────┬──┬─╯
//!          ╰◯─╯            ╰◯─╯    chrome pods; their dark lens is where
//!                                  the field leaves (anchor)
//! ```
//!
//! The hull is a superquadric: a rounded rectangle from above, a soft slab
//! from the side. The frame is the body's: `+x` forward, `+y` up, and the
//! origin the hull's middle, which is its centre of mass.

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::substance::{Substance, CHROME, PLASTIC, RUBBER};
use crate::rendering::vertex::Vertex;

/// Half the hull's length (`x`), width (`z`) and depth (`y`).
pub const HULL_HALF_EXTENTS: Vector3<f32> = Vector3::new(0.78, 0.115, 0.5);
/// How square the hull is from above: 2 is an ellipse, higher is boxier.
const PLAN_EXPONENT: f32 = 3.6;
/// How square it is from the side.
const PROFILE_EXPONENT: f32 = 2.6;
/// How far the nose kicks up at its tip, m, and from how far back.
const NOSE_KICK: f32 = 0.07;
const NOSE_KICK_FROM: f32 = 0.35;
/// The deck pad: its share of the hull's plan, and how far it stands proud.
const PAD_SCALE: f32 = 0.7;
const PAD_LIFT: f32 = 0.012;
/// The bumper: its tube radius, how far its centre stands out from the hull,
/// and the latitude it runs round at. Below the waist, so the red shell shows
/// from the side; under it the hull is the pale belly the field lights.
const TRIM_RADIUS: f32 = 0.024;
const TRIM_STAND_OFF: f32 = 0.012;
const TRIM_LATITUDE: f32 = -0.3;
/// Where the pods hang, in the hull's frame: their axes' `x` and `z`.
pub const POD_SPOTS: [Vector2<f32>; 4] = [
    Vector2::new(0.6, 0.33),
    Vector2::new(0.6, -0.33),
    Vector2::new(-0.6, 0.33),
    Vector2::new(-0.6, -0.33),
];
/// Radius of a pod's housing, and of the dark lens in its face.
pub const POD_RADIUS: f32 = 0.11;
const LENS_RADIUS: f32 = 0.075;
/// Height of each pod's face (where its field leaves) and of the top of its
/// housing, which is buried in the hull.
pub const POD_FACE_HEIGHT: f32 = -0.13;
const POD_TOP_HEIGHT: f32 = -0.04;

/// Vertices around the hull, and from its bottom to its top.
const AROUND: usize = 72;
const UP: usize = 26;
/// Rings across the deck pad.
const PAD_RINGS: usize = 6;
/// Vertices around the bumper's tube, and around a pod.
const TUBE_AROUND: usize = 10;
const POD_AROUND: usize = 32;

/// The hull's shell: a saturated toy red, the complement of the field. Its
/// belly is a pale warm white, which the field below lights cyan.
const HULL_RED: Colour = Colour::rgb(0.86, 0.1, 0.08);
const BELLY_WHITE: Colour = Colour::rgb(0.86, 0.85, 0.82);
const PAD_GREY: Colour = Colour::rgb(0.16, 0.16, 0.17);
const CHROME_WHITE: Colour = Colour::rgb(0.95, 0.95, 0.97);
const LENS_BLACK: Colour = Colour::rgb(0.03, 0.035, 0.045);

/// One piece of the sled's look: its mesh and what it is made of.
pub struct SledPart {
    pub substance: Substance,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

/// Every piece of the sled, in the hull's frame.
pub fn sled_parts() -> Vec<SledPart> {
    vec![
        part(PLASTIC, hull()),
        part(RUBBER, deck_pad()),
        part(CHROME, bumper()),
        part(CHROME, pod_housings()),
        part(PLASTIC, pod_lenses()),
    ]
}

/// The pods' anchors in the hull's frame: the middle of each pod's face.
pub fn pod_anchors() -> [Vector3<f32>; 4] {
    POD_SPOTS.map(|spot| Vector3::new(spot.x, POD_FACE_HEIGHT, spot.y))
}

fn part(substance: Substance, (vertices, indices): (Vec<Vertex>, Vec<u32>)) -> SledPart {
    SledPart {
        substance,
        vertices,
        indices,
    }
}

/// `|x|^e`, keeping the sign of `x`.
fn signed_pow(x: f32, e: f32) -> f32 {
    x.signum() * x.abs().powf(e)
}

/// How far the nose kicks up at `x`.
fn kick(x: f32) -> f32 {
    let a = HULL_HALF_EXTENTS.x;
    let t = ((x / a - NOSE_KICK_FROM) / (1.0 - NOSE_KICK_FROM)).clamp(0.0, 1.0);
    NOSE_KICK * t * t
}

/// The point on the hull at plan angle `around` and latitude `up`
/// (−π/2 at the bottom to π/2 at the top).
fn hull_point(around: f32, up: f32) -> Vector3<f32> {
    let h = HULL_HALF_EXTENTS;
    let plan = 2.0 / PLAN_EXPONENT;
    let profile = 2.0 / PROFILE_EXPONENT;
    let width = up.cos().abs().powf(profile);
    let x = h.x * signed_pow(around.cos(), plan) * width;
    let z = h.z * signed_pow(around.sin(), plan) * width;
    let y = h.y * signed_pow(up.sin(), profile);
    Vector3::new(x, y + kick(x), z)
}

/// Height of the hull's top surface over `(x, z)`, inside its outline.
fn hull_top(x: f32, z: f32) -> f32 {
    let h = HULL_HALF_EXTENTS;
    let plan = ((x / h.x).abs().powf(PLAN_EXPONENT) + (z / h.z).abs().powf(PLAN_EXPONENT))
        .powf(1.0 / PLAN_EXPONENT);
    let r = plan.min(1.0).powf(PROFILE_EXPONENT);
    h.y * (1.0 - r).max(0.0).powf(1.0 / PROFILE_EXPONENT) + kick(x)
}

/// The hull: rings of latitude between a pole at the bottom and one at the
/// top, smooth-shaded.
fn hull() -> (Vec<Vertex>, Vec<u32>) {
    let shell = |up: f32| {
        if up > TRIM_LATITUDE {
            HULL_RED
        } else {
            BELLY_WHITE
        }
    };
    let mut positions = vec![hull_point(0.0, -std::f32::consts::FRAC_PI_2)];
    let mut colours = vec![BELLY_WHITE];
    for j in 1..UP {
        let up = -std::f32::consts::FRAC_PI_2 + std::f32::consts::PI * j as f32 / UP as f32;
        for i in 0..AROUND {
            let around = std::f32::consts::TAU * i as f32 / AROUND as f32;
            positions.push(hull_point(around, up));
            colours.push(shell(up));
        }
    }
    positions.push(hull_point(0.0, std::f32::consts::FRAC_PI_2));
    colours.push(HULL_RED);
    let top = positions.len() as u32 - 1;

    let ring = |j: usize, i: usize| (1 + (j - 1) * AROUND + i % AROUND) as u32;
    let mut indices = Vec::new();
    for i in 0..AROUND {
        indices.extend_from_slice(&[0, ring(1, i), ring(1, i + 1)]);
        indices.extend_from_slice(&[top, ring(UP - 1, i), ring(UP - 1, i + 1)]);
    }
    for j in 1..UP - 1 {
        for i in 0..AROUND {
            let (a, b, c, d) = (
                ring(j, i),
                ring(j, i + 1),
                ring(j + 1, i),
                ring(j + 1, i + 1),
            );
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }
    smooth(positions, colours, indices, |p| *p)
}

/// The rubber deck: a slab following the top of the hull inside the outline
/// at `PAD_SCALE`, with a short skirt down into the hull at its edge.
fn deck_pad() -> (Vec<Vertex>, Vec<u32>) {
    let h = HULL_HALF_EXTENTS;
    let plan = 2.0 / PLAN_EXPONENT;
    let outline = |around: f32, scale: f32| {
        Vector2::new(
            h.x * PAD_SCALE * scale * signed_pow(around.cos(), plan),
            h.z * PAD_SCALE * scale * signed_pow(around.sin(), plan),
        )
    };
    let on_top = |p: Vector2<f32>, lift: f32| Vector3::new(p.x, hull_top(p.x, p.y) + lift, p.y);

    let mut positions = vec![on_top(Vector2::zeros(), PAD_LIFT)];
    for k in 1..=PAD_RINGS {
        let scale = k as f32 / PAD_RINGS as f32;
        for i in 0..AROUND {
            let around = std::f32::consts::TAU * i as f32 / AROUND as f32;
            positions.push(on_top(outline(around, scale), PAD_LIFT));
        }
    }
    for i in 0..AROUND {
        let around = std::f32::consts::TAU * i as f32 / AROUND as f32;
        positions.push(on_top(outline(around, 1.0), -PAD_LIFT));
    }

    let ring = |k: usize, i: usize| (1 + (k - 1) * AROUND + i % AROUND) as u32;
    let mut indices = Vec::new();
    for i in 0..AROUND {
        indices.extend_from_slice(&[0, ring(1, i), ring(1, i + 1)]);
    }
    for k in 1..=PAD_RINGS {
        for i in 0..AROUND {
            let (a, b, c, d) = (
                ring(k, i),
                ring(k, i + 1),
                ring(k + 1, i),
                ring(k + 1, i + 1),
            );
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }
    // The pad's top faces up and its skirt faces out from its middle.
    let colours = vec![PAD_GREY; positions.len()];
    smooth(positions, colours, indices, |p| {
        Vector3::new(p.x * 0.2, 1.0, p.z * 0.2)
    })
}

/// The chrome bumper: a tube round the hull's waist, following the nose up.
fn bumper() -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::with_capacity(AROUND * TUBE_AROUND);
    for i in 0..AROUND {
        let around = std::f32::consts::TAU * i as f32 / AROUND as f32;
        let waist = hull_point(around, TRIM_LATITUDE);
        let ahead = hull_point(around + 1e-3, TRIM_LATITUDE);
        let tangent = (ahead - waist).normalize();
        let outward = tangent.cross(&Vector3::y()).normalize();
        let outward = if outward.dot(&Vector3::new(waist.x, 0.0, waist.z)) < 0.0 {
            -outward
        } else {
            outward
        };
        let centre = waist + outward * TRIM_STAND_OFF;
        for j in 0..TUBE_AROUND {
            let angle = std::f32::consts::TAU * j as f32 / TUBE_AROUND as f32;
            let normal = outward * angle.cos() + Vector3::y() * angle.sin();
            vertices.push(vertex(centre + normal * TRIM_RADIUS, normal, CHROME_WHITE));
        }
    }
    let at = |i: usize, j: usize| ((i % AROUND) * TUBE_AROUND + j % TUBE_AROUND) as u32;
    let mut indices = Vec::new();
    for i in 0..AROUND {
        for j in 0..TUBE_AROUND {
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i, j + 1), at(i + 1, j + 1));
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }
    wind_outward(&vertices, &mut indices);
    (vertices, indices)
}

/// The four chrome housings: a short cylinder from inside the hull down to
/// the pod's face, and a chrome rim round the lens.
fn pod_housings() -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for spot in POD_SPOTS {
        let first = vertices.len() as u32;
        for i in 0..POD_AROUND {
            let angle = std::f32::consts::TAU * i as f32 / POD_AROUND as f32;
            let radial = Vector3::new(angle.cos(), 0.0, angle.sin());
            let at = |r: f32, y: f32| Vector3::new(spot.x, y, spot.y) + radial * r;
            // Side, top to bottom, then the face's rim, flat.
            vertices.push(vertex(at(POD_RADIUS, POD_TOP_HEIGHT), radial, CHROME_WHITE));
            vertices.push(vertex(
                at(POD_RADIUS, POD_FACE_HEIGHT),
                radial,
                CHROME_WHITE,
            ));
            vertices.push(vertex(
                at(POD_RADIUS, POD_FACE_HEIGHT),
                -Vector3::y(),
                CHROME_WHITE,
            ));
            vertices.push(vertex(
                at(LENS_RADIUS, POD_FACE_HEIGHT),
                -Vector3::y(),
                CHROME_WHITE,
            ));
        }
        let at = |i: usize, k: u32| first + (i % POD_AROUND) as u32 * 4 + k;
        for i in 0..POD_AROUND {
            indices.extend_from_slice(&[at(i, 0), at(i + 1, 0), at(i, 1)]);
            indices.extend_from_slice(&[at(i + 1, 0), at(i + 1, 1), at(i, 1)]);
            indices.extend_from_slice(&[at(i, 2), at(i + 1, 2), at(i, 3)]);
            indices.extend_from_slice(&[at(i + 1, 2), at(i + 1, 3), at(i, 3)]);
        }
    }
    wind_outward(&vertices, &mut indices);
    (vertices, indices)
}

/// The dark glossy lens in each pod's face, a hair recessed.
fn pod_lenses() -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for spot in POD_SPOTS {
        let centre = vertices.len() as u32;
        let middle = Vector3::new(spot.x, POD_FACE_HEIGHT + 0.004, spot.y);
        vertices.push(vertex(middle, -Vector3::y(), LENS_BLACK));
        for i in 0..POD_AROUND {
            let angle = std::f32::consts::TAU * i as f32 / POD_AROUND as f32;
            let at = middle + Vector3::new(angle.cos(), 0.0, angle.sin()) * LENS_RADIUS;
            vertices.push(vertex(at, -Vector3::y(), LENS_BLACK));
        }
        for i in 0..POD_AROUND as u32 {
            let next = (i + 1) % POD_AROUND as u32;
            indices.extend_from_slice(&[centre, centre + 1 + i, centre + 1 + next]);
        }
    }
    wind_outward(&vertices, &mut indices);
    (vertices, indices)
}

fn vertex(pos: Vector3<f32>, normal: Vector3<f32>, colour: Colour) -> Vertex {
    Vertex {
        pos,
        color: Vector4::new(colour.r, colour.g, colour.b, 1.0),
        tex_coords: Vector2::zeros(),
        normal,
        ao: 1.0,
    }
}

/// A smooth-shaded mesh from bare positions and their colours: each triangle
/// is turned to face the way `outward` says at its middle, and each vertex
/// takes the area weighted normal of the triangles round it.
fn smooth(
    positions: Vec<Vector3<f32>>,
    colours: Vec<Colour>,
    mut indices: Vec<u32>,
    outward: impl Fn(&Vector3<f32>) -> Vector3<f32>,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut normals = vec![Vector3::zeros(); positions.len()];
    for triangle in indices.chunks_mut(3) {
        let [a, b, c] = [0, 1, 2].map(|k| positions[triangle[k] as usize]);
        // A front face's right-hand normal points out, as a cube's does.
        if (b - a).cross(&(c - a)).dot(&outward(&((a + b + c) / 3.0))) < 0.0 {
            triangle.swap(1, 2);
        }
        let [a, b, c] = [0, 1, 2].map(|k| positions[triangle[k] as usize]);
        let face = (b - a).cross(&(c - a));
        for &index in triangle.iter() {
            normals[index as usize] += face;
        }
    }
    let vertices = positions
        .into_iter()
        .zip(normals)
        .zip(colours)
        .map(|((pos, normal), colour)| {
            vertex(
                pos,
                normal.try_normalize(1e-12).unwrap_or(Vector3::y()),
                colour,
            )
        })
        .collect();
    (vertices, indices)
}

/// Turn every triangle to face the way its vertices' normals do: a front
/// face's right-hand normal points out.
fn wind_outward(vertices: &[Vertex], indices: &mut [u32]) {
    for triangle in indices.chunks_mut(3) {
        let [a, b, c] = [0, 1, 2].map(|k| vertices[triangle[k] as usize]);
        let face = (b.pos - a.pos).cross(&(c.pos - a.pos));
        if face.dot(&(a.normal + b.normal + c.normal)) < 0.0 {
            triangle.swap(1, 2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every triangle of every part faces the way its normals do: the
    /// pipeline culls back faces, so one wound inward is a hole.
    #[test]
    fn every_part_is_wound_outward() {
        for part in sled_parts() {
            for triangle in part.indices.chunks(3) {
                let [a, b, c] = [0, 1, 2].map(|k| part.vertices[triangle[k] as usize]);
                let face = (b.pos - a.pos).cross(&(c.pos - a.pos));
                assert!(face.dot(&(a.normal + b.normal + c.normal)) >= -1e-9);
            }
        }
    }

    /// The hull fits the box its collider is bevelled from, but for the nose.
    #[test]
    fn the_hull_fits_its_collider() {
        let (vertices, _) = hull();
        for v in vertices {
            assert!(v.pos.x.abs() <= HULL_HALF_EXTENTS.x + 1e-4);
            assert!(v.pos.z.abs() <= HULL_HALF_EXTENTS.z + 1e-4);
            assert!(v.pos.y <= HULL_HALF_EXTENTS.y + NOSE_KICK + 1e-4);
            assert!(v.pos.y >= -HULL_HALF_EXTENTS.y - 1e-4);
        }
    }

    /// A pod's face hangs below the hull, so its field leaves from open air.
    #[test]
    fn the_pods_hang_below_the_hull() {
        for anchor in pod_anchors() {
            let bottom = 2.0 * kick(anchor.x) - hull_top(anchor.x, anchor.z);
            assert!(anchor.y < bottom - 0.03, "{} vs {}", anchor.y, bottom);
        }
    }
}
