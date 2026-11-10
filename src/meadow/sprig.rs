//! How each thing in the meadow is shaped and coloured.
//!
//! ```text
//!   tuft     three blades leaning apart, dark at the root, pale at the tip
//!   flower   a stem, a star of petals tipped towards the sky, a yellow eye
//!   berry    a red drop on the ground, green at its crown
//! ```
//!
//! Every face is drawn from both sides: a blade has no back. Colours ride on
//! the vertices, so the whole meadow wears one material.

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3, Vector4};

use super::sow::{share, Sprig, SprigKind};
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// A tuft's blades, their height, how far they lean out as a share of it,
/// and half their width at the root, m.
const BLADES: usize = 5;
const BLADE_HEIGHT: (f32, f32) = (0.14, 0.32);
const BLADE_LEAN: (f32, f32) = (0.3, 0.65);
const BLADE_HALF_WIDTH: f32 = 0.022;

/// A blade at its root, and at its tip where the lawn is lush and where it
/// is dry.
const ROOT: Vector3<f32> = Vector3::new(0.09, 0.15, 0.035);
const LUSH_TIP: Vector3<f32> = Vector3::new(0.2, 0.37, 0.08);
const DRY_TIP: Vector3<f32> = Vector3::new(0.33, 0.42, 0.11);

/// A flower's height, the reach of its petals, and how far its head tips
/// off the vertical.
const STEM_HEIGHT: (f32, f32) = (0.12, 0.24);
const PETAL_REACH: (f32, f32) = (0.06, 0.09);
const HEAD_TIP: f32 = 0.3;
/// Petals to a flower, and how far in the notch between two is cut, as a
/// share of their reach.
const PETALS: usize = 6;
const PETAL_NOTCH: f32 = 0.45;

/// The colours flowers come in, commonest first, and how much of all
/// flowers each of the first two is.
const PETAL_COLOURS: [Vector3<f32>; 3] = [
    Vector3::new(1.0, 0.97, 0.88),
    Vector3::new(0.78, 0.26, 0.3),
    Vector3::new(0.3, 0.4, 0.72),
];
const PETAL_SHARES: [f32; 2] = [0.6, 0.22];
const EYE: Vector3<f32> = Vector3::new(0.85, 0.62, 0.1);

/// A berry's half-width and half-height, m, and its colours.
const BERRY_HALF: (f32, f32) = (0.05, 0.06);
const BERRY: Vector3<f32> = Vector3::new(0.6, 0.06, 0.05);
const BERRY_CROWN: Vector3<f32> = Vector3::new(0.25, 0.4, 0.1);

/// A run of shares, each drawn from a sprig's chance.
struct Chances(u32);

impl Chances {
    /// The next share, 0 to 1.
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        share(self.0)
    }

    /// The next share, laid over `range`.
    fn within(&mut self, range: (f32, f32)) -> f32 {
        range.0 + (range.1 - range.0) * self.next()
    }

    /// A level direction.
    fn heading(&mut self) -> Vector3<f32> {
        let angle = self.next() * TAU;
        Vector3::new(angle.cos(), 0.0, angle.sin())
    }
}

/// A mesh being gathered.
struct Gathered {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

impl Gathered {
    fn corner(&mut self, pos: Vector3<f32>, normal: Vector3<f32>, colour: Vector3<f32>) -> u32 {
        self.vertices.push(Vertex {
            pos,
            color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
            tex_coords: Vector2::zeros(),
            normal,
            ao: 1.0,
        });
        (self.vertices.len() - 1) as u32
    }

    /// A triangle seen from both sides.
    fn leaf(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend([a, b, c, a, c, b]);
    }

    /// A blade from `root` to `tip`, `half_width` either side of the root
    /// across the way it leans.
    fn blade(
        &mut self,
        root: Vector3<f32>,
        tip: Vector3<f32>,
        half_width: f32,
        colours: (Vector3<f32>, Vector3<f32>),
    ) {
        let reach = tip - root;
        let across = reach
            .cross(&Vector3::y())
            .try_normalize(1.0e-6)
            .unwrap_or_else(Vector3::x)
            * half_width;
        // Lit as ground that leans the blade's way: soft, and never black
        // on the side turned from the sun.
        let normal = (Vector3::y() + reach.normalize() * 0.5).normalize();
        let left = self.corner(root - across, normal, colours.0);
        let right = self.corner(root + across, normal, colours.0);
        let top = self.corner(tip, normal, colours.1);
        self.leaf(left, right, top);
    }

    fn tuft(&mut self, at: Vector3<f32>, chances: &mut Chances) {
        let tip = DRY_TIP.lerp(&LUSH_TIP, chances.next());
        for _ in 0..BLADES {
            let height = chances.within(BLADE_HEIGHT);
            let lean = chances.heading() * height * chances.within(BLADE_LEAN);
            // Rooted a little apart, so a tuft is a clump and not a spray.
            let root = at + chances.heading() * 0.05 * chances.next();
            self.blade(
                root,
                root + lean + Vector3::y() * height,
                BLADE_HALF_WIDTH,
                (ROOT, tip),
            );
        }
    }

    fn flower(&mut self, at: Vector3<f32>, chances: &mut Chances) {
        let height = chances.within(STEM_HEIGHT);
        let facing = (Vector3::y() + chances.heading() * HEAD_TIP).normalize();
        let head = at + facing * height;
        self.blade(at, head, 0.008, (ROOT, LUSH_TIP));

        let drawn = chances.next();
        let petal = if drawn < PETAL_SHARES[0] {
            PETAL_COLOURS[0]
        } else if drawn < PETAL_SHARES[0] + PETAL_SHARES[1] {
            PETAL_COLOURS[1]
        } else {
            PETAL_COLOURS[2]
        };
        let reach = chances.within(PETAL_REACH);
        let side = facing.cross(&Vector3::x()).normalize();
        let other = facing.cross(&side);
        let rim = |k: usize, radius: f32| {
            let angle = k as f32 / (2 * PETALS) as f32 * TAU;
            head + (side * angle.cos() + other * angle.sin()) * radius
        };

        // The petals: a star about the head.
        let hub = self.corner(head, facing, petal);
        let star: Vec<u32> = (0..2 * PETALS)
            .map(|k| {
                let radius = if k % 2 == 0 {
                    reach
                } else {
                    reach * PETAL_NOTCH
                };
                self.corner(rim(k, radius), facing, petal)
            })
            .collect();
        for k in 0..star.len() {
            self.leaf(hub, star[k], star[(k + 1) % star.len()]);
        }
        // The eye, a hair above them.
        let lift = facing * 0.004;
        let centre = self.corner(head + lift, facing, EYE);
        let eye: Vec<u32> = (0..PETALS)
            .map(|k| self.corner(rim(2 * k, reach * 0.3) + lift, facing, EYE))
            .collect();
        for k in 0..eye.len() {
            self.indices
                .extend([centre, eye[(k + 1) % eye.len()], eye[k]]);
        }
    }

    fn berry(&mut self, at: Vector3<f32>, chances: &mut Chances) {
        let size = 0.8 + 0.5 * chances.next();
        let (wide, tall) = (BERRY_HALF.0 * size, BERRY_HALF.1 * size);
        let middle = at + Vector3::y() * tall;
        let turn = chances.next() * TAU;
        let foot = self.corner(middle - Vector3::y() * tall, -Vector3::y(), BERRY);
        let crown = self.corner(middle + Vector3::y() * tall, Vector3::y(), BERRY_CROWN);
        let girth: Vec<u32> = (0..4)
            .map(|k| {
                let angle = turn + k as f32 * TAU / 4.0;
                let out = Vector3::new(angle.cos(), 0.0, angle.sin());
                self.corner(middle + out * wide + Vector3::y() * tall * 0.2, out, BERRY)
            })
            .collect();
        for k in 0..4 {
            let (here, next) = (girth[k], girth[(k + 1) % 4]);
            self.indices.extend([crown, next, here, foot, here, next]);
        }
    }
}

/// The mesh of `sprigs`, in the world's frame, drawn in `material`.
pub fn mesh(sprigs: &[Sprig], material: MaterialId) -> MeshPrimitive {
    let mut gathered = Gathered {
        vertices: Vec::new(),
        indices: Vec::new(),
    };
    for sprig in sprigs {
        let mut chances = Chances(sprig.chance | 1);
        match sprig.kind {
            SprigKind::Tuft => gathered.tuft(sprig.at, &mut chances),
            SprigKind::Flower => gathered.flower(sprig.at, &mut chances),
            SprigKind::Berry => gathered.berry(sprig.at, &mut chances),
        }
    }
    MeshPrimitive {
        vertices: gathered.vertices,
        indices: gathered.indices,
        material,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one(kind: SprigKind) -> MeshPrimitive {
        let sprig = Sprig {
            cell: (0, 0),
            at: Vector3::new(4.0, 2.0, -3.0),
            kind,
            chance: 0x1234_5678,
        };
        mesh(&[sprig], MaterialId(0))
    }

    #[test]
    fn every_kind_stands_on_its_spot_and_within_reach_of_it() {
        for kind in [SprigKind::Tuft, SprigKind::Flower, SprigKind::Berry] {
            let mesh = one(kind);
            assert!(!mesh.indices.is_empty());
            assert!(mesh
                .indices
                .iter()
                .all(|i| (*i as usize) < mesh.vertices.len()));
            for vertex in &mesh.vertices {
                let off = vertex.pos - Vector3::new(4.0, 2.0, -3.0);
                assert!(off.y > -1.0e-4, "{kind:?} reaches under the ground");
                assert!(off.norm() < 0.4, "{kind:?} reaches {} m", off.norm());
                assert!((vertex.normal.norm() - 1.0).abs() < 1.0e-4);
            }
        }
    }

    #[test]
    fn a_blade_is_seen_from_both_sides() {
        let mesh = one(SprigKind::Tuft);
        assert_eq!(mesh.indices.len(), BLADES * 6);
    }
}
