//! Cloth the legs push out: the back of a robe, sewn to the hips at its
//! top and hanging free below, bulged aside each frame wherever a leg would
//! come through it.
//!
//! ```text
//!   sewn ══════╗         cloth above a leg point: drawn in a line from
//!              ║╲        the seam down to it
//!              ║ ●knee   at the leg point: kept a clearance behind it
//!              ║ │       below it: hanging straight down from it
//!   rest cloth ║ │hem
//! ```
//!
//! No bones and no weights: the push is worked out from the leg's own
//! joints, along the hips' −Z, and fades to nothing where the cloth is sewn
//! to what does not move (the seam at its top, and the rigid robe either
//! side), so the two stay joined. A leg pushes the cloth at once, since
//! cloth cannot be walked through; it falls back at its own pace.

use nalgebra::{Point3, Vector3};

use crate::model::{MeshPrimitive, Transform};
use crate::rendering::material::MaterialId;

/// Points along each bone of a leg the cloth is kept clear of.
const SAMPLES_PER_BONE: usize = 8;

/// How fast pushed cloth falls back once the leg lets it, 1/s.
const FALL_RATE: f32 = 6.0;

/// Side of the cells the cloth's inside face is mapped on, m.
const CELL: f32 = 0.025;

/// Cells either side, across and up, the inside face is taken over: wider
/// than a pleat, so that crown and valley, and the cloth's inside and
/// outside skins, are pushed alike and keep their shape.
const FACE_REACH: (i32, i32) = (3, 1);

/// How the cloth hangs: what it is at rest and where it is held.
pub struct Drape {
    /// The cloth first, then anything hanging from it (threads at a hem),
    /// at rest in the hips' frame.
    rest: Vec<MeshPrimitive>,
    /// For each vertex of each, how free it hangs: 0 where it is sewn to
    /// what does not move, 1 where it hangs free.
    freedom: Vec<Vec<f32>>,
    /// For each vertex of each, the depth (z) of the cloth's inside face
    /// where it is: what a leg is kept behind.
    inside: Vec<Vec<f32>>,
    /// The inside face mapped over the cloth's width and height.
    face: InsideFace,
    /// Height of the seam along its top, in the hips' frame.
    sewn: f32,
    /// How far behind a leg the inside of the cloth is kept, m: the leg's
    /// own thickness, and a little air.
    clearance: f32,
    /// How wide the bulge a leg makes spreads either side of it, m.
    spread: f32,
}

impl Drape {
    /// Cloth `rest` (in the hips' frame, the cloth itself first), sewn
    /// along the height `sewn`, hanging as free at each point as `freedom`
    /// says, kept `clearance` behind a leg and bulging `spread` either
    /// side of it.
    pub fn new(
        rest: Vec<MeshPrimitive>,
        sewn: f32,
        freedom: impl Fn(Vector3<f32>) -> f32,
        clearance: f32,
        spread: f32,
    ) -> Self {
        let face = InsideFace::of(&rest[0]);
        let freedom = rest
            .iter()
            .map(|mesh| mesh.vertices.iter().map(|v| freedom(v.pos)).collect())
            .collect();
        let inside = rest
            .iter()
            .map(|mesh| {
                mesh.vertices
                    .iter()
                    .map(|v| face.at(v.pos.x, v.pos.y).unwrap_or(v.pos.z).max(v.pos.z))
                    .collect()
            })
            .collect();
        Self {
            rest,
            freedom,
            inside,
            face,
            sewn,
            clearance,
            spread,
        }
    }

    /// The cloth at rest, in the hips' frame.
    pub fn rest(&self) -> &[MeshPrimitive] {
        &self.rest
    }

    /// The same cloth in other materials.
    pub fn recoloured(&self, material: impl Fn(MaterialId) -> MaterialId) -> Self {
        let mut rest = self.rest.clone();
        for mesh in &mut rest {
            mesh.material = material(mesh.material);
        }
        Self {
            rest,
            freedom: self.freedom.clone(),
            inside: self.inside.clone(),
            face: self.face.clone(),
            sewn: self.sewn,
            clearance: self.clearance,
            spread: self.spread,
        }
    }

    /// The points along `legs` (hip, knee, ankle each, in the hips' frame)
    /// that are behind the cloth's inside face: the ones it must give to.
    fn pressing(&self, legs: &[[Point3<f32>; 3]]) -> Vec<Point3<f32>> {
        let mut pressing = Vec::new();
        for leg in legs {
            for bone in leg.windows(2) {
                for sample in 0..=SAMPLES_PER_BONE {
                    let t = sample as f32 / SAMPLES_PER_BONE as f32;
                    let at = bone[0] + (bone[1] - bone[0]) * t;
                    let Some(face) = self.face.at(at.x, at.y) else {
                        continue;
                    };
                    if at.y < self.sewn && at.z - self.clearance < face {
                        pressing.push(at);
                    }
                }
            }
        }
        pressing
    }

    /// How far back the inside of the cloth at `at`, whose inside face is
    /// at depth `inside`, must be pushed to give to a leg at `leg`.
    fn give(&self, at: Vector3<f32>, inside: f32, leg: &Point3<f32>) -> f32 {
        let aside = (at.x - leg.x).abs() - self.clearance;
        let spread = if aside <= 0.0 {
            1.0
        } else if aside > 3.0 * self.spread {
            return 0.0;
        } else {
            (-0.5 * (aside / self.spread).powi(2)).exp()
        };
        let behind = inside - (leg.z - self.clearance);
        // Below the leg it hangs straight down from it; above, it is drawn
        // in a line from the seam down to it.
        let drawn = if at.y <= leg.y {
            1.0
        } else {
            ((self.sewn - at.y) / (self.sewn - leg.y).max(1e-3)).clamp(0.0, 1.0)
        };
        spread * drawn * behind.max(0.0)
    }
}

/// The depth of the cloth's inside face over its width and height: the
/// foremost cloth in each cell.
#[derive(Clone)]
struct InsideFace {
    /// Lowest x and y mapped.
    origin: (f32, f32),
    /// Cells across and up.
    size: (usize, usize),
    /// Depth of the foremost cloth in each cell, row by row; NaN where
    /// there is none.
    depth: Vec<f32>,
}

impl InsideFace {
    fn of(cloth: &MeshPrimitive) -> Self {
        let (mut low, mut high) = ((f32::MAX, f32::MAX), (f32::MIN, f32::MIN));
        for v in &cloth.vertices {
            low = (low.0.min(v.pos.x), low.1.min(v.pos.y));
            high = (high.0.max(v.pos.x), high.1.max(v.pos.y));
        }
        let size = (
            ((high.0 - low.0) / CELL) as usize + 1,
            ((high.1 - low.1) / CELL) as usize + 1,
        );
        let mut face = Self {
            origin: low,
            size,
            depth: vec![f32::NAN; size.0 * size.1],
        };
        for v in &cloth.vertices {
            if let Some(cell) = face.cell(v.pos.x, v.pos.y) {
                let depth = &mut face.depth[cell];
                *depth = if depth.is_nan() {
                    v.pos.z
                } else {
                    depth.max(v.pos.z)
                };
            }
        }
        face
    }

    fn cell(&self, x: f32, y: f32) -> Option<usize> {
        let column = ((x - self.origin.0) / CELL).floor();
        let row = ((y - self.origin.1) / CELL).floor();
        if column < 0.0 || row < 0.0 {
            return None;
        }
        let (column, row) = (column as usize, row as usize);
        (column < self.size.0 && row < self.size.1).then_some(row * self.size.0 + column)
    }

    /// The inside face's depth at (`x`, `y`): the foremost cloth over the
    /// cells around it, smoothing over its pleats and any bite out of it.
    fn at(&self, x: f32, y: f32) -> Option<f32> {
        let (reach_x, reach_y) = FACE_REACH;
        (-reach_y..=reach_y)
            .flat_map(|up| (-reach_x..=reach_x).map(move |across| (across, up)))
            .filter_map(|(across, up)| self.cell(x + across as f32 * CELL, y + up as f32 * CELL))
            .map(|cell| self.depth[cell])
            .filter(|depth| !depth.is_nan())
            .reduce(f32::max)
    }
}

/// A drape moving.
#[derive(Clone, Debug, Default)]
pub struct DrapeMotion {
    /// How far back each vertex of each mesh is pushed, m.
    push: Vec<Vec<f32>>,
}

impl DrapeMotion {
    /// Advance by `dt` seconds for `drape` on hips posed `hips`, over
    /// `legs` (hip, knee, ankle each, in the world).
    pub fn step(&mut self, drape: &Drape, hips: &Transform, legs: &[[Point3<f32>; 3]], dt: f32) {
        if self.push.len() != drape.rest.len() {
            self.push = drape
                .rest
                .iter()
                .map(|m| vec![0.0; m.vertices.len()])
                .collect();
        }
        let into_hips = hips.rotation.inverse();
        let local: Vec<[Point3<f32>; 3]> = legs
            .iter()
            .map(|leg| leg.map(|at| Point3::from(into_hips * (at.coords - hips.translation))))
            .collect();
        let pressing = drape.pressing(&local);
        let fall = (-FALL_RATE * dt).exp();
        for (mesh, ((push, freedom), inside)) in drape
            .rest
            .iter()
            .zip(self.push.iter_mut().zip(&drape.freedom).zip(&drape.inside))
        {
            for (index, vertex) in mesh.vertices.iter().enumerate() {
                let wanted = if freedom[index] > 0.0 {
                    let most = pressing
                        .iter()
                        .map(|leg| drape.give(vertex.pos, inside[index], leg))
                        .fold(0.0f32, f32::max);
                    freedom[index] * most
                } else {
                    0.0
                };
                push[index] = wanted.max(push[index] * fall);
            }
        }
    }

    /// How far back the vertex `index` of mesh `mesh` is pushed, m.
    pub fn push(&self, mesh: usize, index: usize) -> f32 {
        self.push
            .get(mesh)
            .and_then(|push| push.get(index))
            .copied()
            .unwrap_or(0.0)
    }

    /// The cloth as it hangs now, placed by `place` from the hips' frame:
    /// into `out`, reusing what it holds.
    pub fn shape(&self, drape: &Drape, place: &Transform, out: &mut Vec<MeshPrimitive>) {
        if out.len() != drape.rest.len() {
            *out = drape.rest.clone();
        }
        let matrix = place.to_matrix();
        for (which, (rest, mesh)) in drape.rest.iter().zip(out.iter_mut()).enumerate() {
            mesh.material = rest.material;
            mesh.vertices.clone_from(&rest.vertices);
            // Each vertex keeps the normal it was sculpted with: the pleats
            // shade the cloth, and a push is too gentle a bend to need more.
            for (index, vertex) in mesh.vertices.iter_mut().enumerate() {
                vertex.pos.z -= self.push(which, index);
                vertex.pos = matrix.transform_point(&Point3::from(vertex.pos)).coords;
                vertex.normal = place.rotation * vertex.normal;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector2, Vector4};

    use super::*;
    use crate::rendering::vertex::Vertex;

    const DT: f32 = 1.0 / 60.0;
    const SEWN: f32 = 0.2;
    const CLEARANCE: f32 = 0.05;

    /// A back of a robe as two skins of a hanging sheet, the inside one at
    /// z = −0.3 and the outside 4 cm behind it, from the seam at y = 0.2
    /// down to a hem at y = −0.6, 0.6 m across.
    fn sheet() -> Drape {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let (across, down) = (25, 33);
        for (skin, z) in [(0, -0.3), (1, -0.34)] {
            let base = (skin * across * down) as u32;
            for row in 0..down {
                for column in 0..across {
                    vertices.push(Vertex {
                        pos: Vector3::new(
                            -0.3 + 0.6 * column as f32 / (across - 1) as f32,
                            SEWN - 0.8 * row as f32 / (down - 1) as f32,
                            z,
                        ),
                        color: Vector4::repeat(1.0),
                        tex_coords: Vector2::zeros(),
                        normal: Vector3::z() * if skin == 0 { 1.0 } else { -1.0 },
                        ao: 1.0,
                    });
                }
            }
            for row in 0..down - 1 {
                for column in 0..across - 1 {
                    let at = base + (row * across + column) as u32;
                    let (right, below) = (at + 1, at + across as u32);
                    indices.extend([at, below, right, right, below, below + 1]);
                }
            }
        }
        let cloth = MeshPrimitive {
            vertices,
            indices,
            material: MaterialId(0),
        };
        let freedom = |at: Vector3<f32>| ((SEWN - at.y) / 0.15).clamp(0.0, 1.0);
        Drape::new(vec![cloth], SEWN, freedom, CLEARANCE, 0.05)
    }

    fn hips() -> Transform {
        Transform {
            translation: Vector3::new(0.0, 1.0, 0.0),
            rotation: UnitQuaternion::identity(),
            scale: Vector3::repeat(1.0),
        }
    }

    /// A leg from the hip at x = 0.1 to `knee` (in the hips' frame), then
    /// straight down: in the world, for hips at y = 1.
    fn leg(knee: Vector3<f32>) -> [[Point3<f32>; 3]; 1] {
        let lift = Vector3::new(0.0, 1.0, 0.0);
        let hip = Point3::from(Vector3::new(0.1, 0.0, 0.0) + lift);
        let knee = Point3::from(knee + lift);
        [[hip, knee, knee - Vector3::y() * 0.7]]
    }

    fn cloth(drape: &Drape, motion: &DrapeMotion) -> Vec<Vector3<f32>> {
        let mut out = Vec::new();
        let place = Transform::default();
        motion.shape(drape, &place, &mut out);
        out[0].vertices.iter().map(|v| v.pos).collect()
    }

    #[test]
    fn a_leg_under_the_cloth_leaves_it_hanging() {
        let drape = sheet();
        let mut motion = DrapeMotion::default();
        motion.step(&drape, &hips(), &leg(Vector3::new(0.1, -0.7, -0.1)), DT);
        let rest = &drape.rest()[0].vertices;
        for (pushed, rest) in cloth(&drape, &motion).iter().zip(rest) {
            assert_eq!(*pushed, rest.pos);
        }
    }

    #[test]
    fn a_knee_behind_it_is_kept_inside_and_the_seam_holds() {
        let drape = sheet();
        let mut motion = DrapeMotion::default();
        let knee = Vector3::new(0.1, -0.25, -0.5);
        motion.step(&drape, &hips(), &leg(knee), DT);
        let shape = cloth(&drape, &motion);
        let rest = &drape.rest()[0].vertices;
        for (at, rest) in shape.iter().zip(rest) {
            let inside = rest.pos.z > -0.32;
            // Over the leg, at the knee and below it, the inside of the
            // cloth is a clearance behind the knee: hanging from it.
            if inside && (rest.pos.x - knee.x).abs() < 0.02 && rest.pos.y <= knee.y {
                assert!(
                    at.z <= knee.z - CLEARANCE + 1e-4,
                    "the cloth at {} is at z {}",
                    rest.pos,
                    at.z
                );
            }
            // Its seam does not move, nor does the cloth far aside.
            if rest.pos.y >= SEWN - 1e-4 || (rest.pos.x - knee.x).abs() > 0.25 {
                assert!((at - rest.pos).norm() < 1e-3, "{} moved to {at}", rest.pos);
            }
        }
        // Both skins move together: the cloth keeps its thickness.
        let skin = shape.len() / 2;
        for index in 0..skin {
            let thick = shape[index].z - shape[index + skin].z;
            assert!(
                (thick - 0.04).abs() < 1e-4,
                "{thick} thick at {}",
                rest[index].pos
            );
        }
    }

    #[test]
    fn it_gives_at_once_and_falls_back_slowly() {
        let drape = sheet();
        let mut motion = DrapeMotion::default();
        motion.step(&drape, &hips(), &leg(Vector3::new(0.1, -0.25, -0.5)), DT);
        // The inside skin's hem, under the leg at x = 0.1.
        let lowest = 32 * 25 + 16;
        let pushed = motion.push(0, lowest);
        assert!(pushed > 0.2, "{pushed}");
        let under = leg(Vector3::new(0.1, -0.7, -0.1));
        motion.step(&drape, &hips(), &under, DT);
        let falling = motion.push(0, lowest);
        assert!(falling > 0.8 * pushed && falling < pushed, "{falling}");
        for _ in 0..90 {
            motion.step(&drape, &hips(), &under, DT);
        }
        assert!(motion.push(0, lowest) < 0.002);
    }
}
