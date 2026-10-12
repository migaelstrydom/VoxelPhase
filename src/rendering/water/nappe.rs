//! A falling sheet's body: the tongue of water from its lip down to its
//! foot, curling over as it leaves the lip and thinning as it falls.
//!
//! ```text
//!   lip ●━━━━━━╮           top face: from the lip's surface along its
//!   bed ●──╮    ╲          ballistic curve to the foot
//!           ╲    ╲         under face: the top less its thickness, the
//!            ╲    ╲        flow over the lip by its speed there
//!             ╲   ┃
//!   foot ~~~~~~●~~●~~
//! ```
//!
//! Each face is a strip of rows down the sheet, two vertices to a row, one
//! at each end of its edge. The top face is wound to face out and up, the
//! under face in and down towards the rock, and the sheets' pipeline culls
//! back faces: from in front only the top is seen, from behind only the
//! under face. Where it lands, a patch of foam lies on the water out from
//! its foot (`plunge`).

use nalgebra::{Vector2, Vector3, Vector4};

use super::river_mesh::RiverMesh;
use super::vertex::RiverVertex;

/// Rows of vertices down a sheet, after its lip. Fixed, so a sheet keeps
/// its vertices as its water rises and falls.
pub(super) const NAPPE_ROWS: u32 = 8;

/// Gravity, m/s².
const GRAVITY: f32 = 9.81;

/// How far out from its foot the foam where a sheet lands reaches, m: so
/// far for any fall, and this much further per metre it falls.
const PLUNGE_REACH: f32 = 0.4;
const PLUNGE_REACH_PER_FALL: f32 = 0.12;
const MAX_PLUNGE_REACH: f32 = 1.5;
/// How far above the water its foam is drawn, m, clear of the surface's
/// own depth.
pub(super) const PLUNGE_LIFT: f32 = 0.02;
/// The least share of its reach a foam vertex is drawn at, so its whole
/// patch reads as past the foot (`RiverVertex::sheet`).
pub(super) const PLUNGE_START: f32 = 1.0e-3;

/// Thinnest and thickest a falling body of water is drawn, m.
const MIN_THICKNESS: f32 = 0.02;
const MAX_THICKNESS: f32 = 0.8;

/// Which face of a falling body of water.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Face {
    /// Its outer surface, facing out and up.
    Top,
    /// The surface under it, facing in and down.
    Under,
}

impl Face {
    pub const BOTH: [Face; 2] = [Face::Top, Face::Under];
}

/// A sheet's tongue between its lip and its foot, both on its top face.
pub(super) struct Nappe {
    lip: RiverVertex,
    /// How far out the foot lands from the lip, m.
    throw: Vector2<f32>,
    /// How far it falls, m.
    fall: f32,
    /// How long its water falls, s.
    duration: f32,
}

impl Nappe {
    /// The tongue from `lip` thrown out by `throw` and falling `fall`.
    pub fn new(lip: RiverVertex, throw: Vector2<f32>, fall: f32) -> Self {
        let fall = fall.max(0.0);
        Self {
            lip,
            throw,
            fall,
            duration: (2.0 * fall / GRAVITY).sqrt(),
        }
    }

    /// The tongue whose top face runs from `lip` to `foot`.
    pub fn between(lip: RiverVertex, foot: &RiverVertex) -> Self {
        Self::new(lip, foot.xz - lip.xz, lip.surface - foot.surface)
    }

    /// The vertex of `face` on row `row`, at the end of the edge its lip is
    /// at: row 0 is the lip, row `NAPPE_ROWS` the foot. Its depth stays the
    /// lip's, how deep the sheet pours, which the draw cuts it by.
    pub fn vertex(&self, row: u32, face: Face) -> RiverVertex {
        let share = row as f32 / NAPPE_ROWS as f32;
        let drop = self.fall * share * share;
        let level = if self.duration > 0.0 {
            self.throw / self.duration
        } else {
            Vector2::zeros()
        };
        let velocity = Vector3::new(level.x, -GRAVITY * self.duration * share, level.y);
        let lip = self.lip;
        // The flow over the lip keeps its rate as it speeds up: it thins.
        let lip_speed = level.norm();
        let thickness = if lip_speed > 0.0 {
            lip.depth * lip_speed / velocity.norm()
        } else {
            lip.depth
        };
        let top = RiverVertex {
            xz: lip.xz + self.throw * share,
            surface: lip.surface - drop,
            sheet: Vector4::new(drop, self.fall, lip.sheet.z, lip.sheet.w),
            ..lip
        };
        layered(top, velocity, thickness, face)
    }
}

/// The vertex `top` on a body of water's top face, moving at `velocity`,
/// on `face` of the body `thickness` thick there: the top's normal is
/// square to the velocity, out and up; the under face lies that thickness
/// below the top along it, facing the other way.
fn layered(top: RiverVertex, velocity: Vector3<f32>, thickness: f32, face: Face) -> RiverVertex {
    let level = Vector2::new(velocity.x, velocity.z);
    let speed = level.norm();
    let out = if speed > 1.0e-4 {
        level / speed
    } else {
        top.face
    };
    let normal = Vector3::new(out.x * -velocity.y, speed, out.y * -velocity.y)
        .try_normalize(1.0e-6)
        .unwrap_or_else(Vector3::y);
    let thickness = thickness.clamp(MIN_THICKNESS, MAX_THICKNESS);
    match face {
        Face::Top => RiverVertex { normal, ..top },
        Face::Under => {
            let under = Vector3::new(top.xz.x, top.surface, top.xz.y) - normal * thickness;
            RiverVertex {
                xz: Vector2::new(under.x, under.z),
                surface: under.y,
                normal: -normal,
                ..top
            }
        }
    }
}

/// A vertex of the foam where a sheet whose top face runs from `lip` to
/// `foot` lands: on the water at the foot, `share` of its reach out the way
/// the sheet was thrown. Its drop is its fall plus that share, past its
/// foot: what the draw takes for foam.
pub(super) fn plunge(lip: RiverVertex, foot: &RiverVertex, share: f32) -> RiverVertex {
    let fall = lip.surface - foot.surface;
    let out = (foot.xz - lip.xz).try_normalize(1.0e-4).unwrap_or(lip.face);
    let reach = (PLUNGE_REACH + PLUNGE_REACH_PER_FALL * fall).min(MAX_PLUNGE_REACH);
    let share = share.max(PLUNGE_START);
    RiverVertex {
        xz: foot.xz + out * (reach * share),
        surface: foot.surface + PLUNGE_LIFT,
        sheet: Vector4::new(fall + share, fall, lip.sheet.z, lip.sheet.w),
        normal: Vector3::y(),
        ..lip
    }
}

/// The triangles of a patch of foam: `centre` joined to each pair of
/// neighbours round `ring`, wound to face up.
pub(super) fn fan(mesh: &mut RiverMesh, centre: u32, ring: &[u32]) {
    for pair in ring.windows(2) {
        let triangle = wound(mesh, [centre, pair[0], pair[1]]);
        mesh.sheet_indices.extend(triangle);
    }
}

/// The triangles of a body of water's surface: `rows` of vertices across
/// it, each joined to the next along the row and to the row after, each
/// triangle wound to face the way its vertices' normals do, so the pipeline
/// culls the faces turned away. A face of a sheet is rows of its two ends;
/// a jet, rows of a ring closed by its first vertex again.
pub(super) fn strip<R: AsRef<[u32]>>(mesh: &mut RiverMesh, rows: &[R]) {
    for pair in rows.windows(2) {
        let (near, far) = (pair[0].as_ref(), pair[1].as_ref());
        for i in 0..near.len().min(far.len()).saturating_sub(1) {
            let (a, b, c, d) = (near[i], near[i + 1], far[i + 1], far[i]);
            for [p, q, r] in [[a, b, c], [a, c, d]] {
                mesh.sheet_indices.extend(wound(mesh, [p, q, r]));
            }
        }
    }
}

/// The triangle `corners` ordered to face the mean of their normals.
fn wound(mesh: &RiverMesh, corners: [u32; 3]) -> [u32; 3] {
    let at = |i: u32| {
        let v = &mesh.vertices[i as usize];
        (Vector3::new(v.xz.x, v.surface, v.xz.y), v.normal)
    };
    let [(p, np), (q, nq), (r, nr)] = corners.map(at);
    let [a, b, c] = corners;
    if (q - p).cross(&(r - p)).dot(&(np + nq + nr)) >= 0.0 {
        [a, b, c]
    } else {
        [a, c, b]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A lip at height 10, 0.4 m deep, pouring towards +x.
    fn lip() -> RiverVertex {
        RiverVertex {
            xz: Vector2::new(1.0, 2.0),
            surface: 10.0,
            depth: 0.4,
            flow: Vector2::new(1.0, 0.0),
            face: Vector2::new(1.0, 0.0),
            sheet: Vector4::new(0.0, 4.0, 1.0, 2.0),
            normal: Vector3::y(),
            fray: 1.0,
            reach: 0.0,
        }
    }

    #[test]
    fn the_top_face_runs_from_the_lip_to_the_foot() {
        let nappe = Nappe::new(lip(), Vector2::new(0.9, 0.0), 4.0);
        let top = nappe.vertex(0, Face::Top);
        assert_eq!((top.xz, top.surface), (lip().xz, 10.0));
        assert!(
            (top.normal - Vector3::y()).norm() < 1e-6,
            "level at the lip"
        );
        let foot = nappe.vertex(NAPPE_ROWS, Face::Top);
        assert!((foot.xz - Vector2::new(1.9, 2.0)).norm() < 1e-6);
        assert!((foot.surface - 6.0).abs() < 1e-6);
        assert!(foot.normal.x > 0.9, "facing out once it falls fast");
        let between = Nappe::between(lip(), &foot).vertex(3, Face::Top);
        assert_eq!(between, nappe.vertex(3, Face::Top));
    }

    #[test]
    fn the_under_face_leaves_the_lip_its_depth_down_and_closes_in_as_it_falls() {
        let nappe = Nappe::new(lip(), Vector2::new(0.9, 0.0), 4.0);
        let gap = |row| {
            let (top, under) = (nappe.vertex(row, Face::Top), nappe.vertex(row, Face::Under));
            let at = |v: &RiverVertex| Vector3::new(v.xz.x, v.surface, v.xz.y);
            assert!((top.normal + under.normal).norm() < 1e-6, "faces opposite");
            (at(&top) - at(&under)).norm()
        };
        assert!((gap(0) - 0.4).abs() < 1e-5);
        assert!(gap(NAPPE_ROWS) < 0.1 && gap(NAPPE_ROWS) >= MIN_THICKNESS - 1e-6);
        assert!((1..=NAPPE_ROWS).all(|row| gap(row) < gap(row - 1)));
    }

    #[test]
    fn a_strip_is_wound_to_face_its_normals() {
        let nappe = Nappe::new(lip(), Vector2::new(0.9, 0.0), 4.0);
        let mut mesh = RiverMesh::default();
        for face in Face::BOTH {
            let mut rows = Vec::new();
            for row in 0..=NAPPE_ROWS {
                rows.push([0.0, 1.0].map(|dz| {
                    let mut v = nappe.vertex(row, face);
                    v.xz.y += dz;
                    mesh.vertices.push(v);
                    mesh.vertices.len() as u32 - 1
                }));
            }
            strip(&mut mesh, &rows);
        }
        for triangle in mesh.sheet_indices.chunks(3) {
            let [p, q, r] = [0, 1, 2].map(|i| {
                let v = &mesh.vertices[triangle[i] as usize];
                (Vector3::new(v.xz.x, v.surface, v.xz.y), v.normal)
            });
            let facing = (q.0 - p.0).cross(&(r.0 - p.0));
            assert!(facing.dot(&(p.1 + q.1 + r.1)) > 0.0);
        }
    }
}
