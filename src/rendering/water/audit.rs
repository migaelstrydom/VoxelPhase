//! A drawn river mesh checked against the water it was drawn from: is any
//! water drawn where none is?
//!
//! The mesh is built from the field by rules of its own (`field_mesher`,
//! `curtain`), and a rule gone wrong draws water the field does not have: a
//! curtain flung up a dam's face to the ground above its hole, a surface
//! ramped up to a roof. This reads the mesh's vertices alone and asks only
//! what holds of any drawing of the field's water:
//!
//! - every triangle is of vertices the mesh has;
//! - every vertex is finite;
//! - a surface vertex under water stands no higher than the water near it,
//!   or the head of water run full in a hole near it;
//! - a curtain's vertex stands no higher than the lip its water left, or
//!   a jet's than the top of its arc.

use nalgebra::{Point3, Vector2};

use crate::water::falling::GRAVITY;
use crate::water::geometry::{Column, COLUMN_SIZE};
use crate::water::WaterWorld;

use super::river_mesh::RiverMesh;
use super::vertex::RiverVertex;

/// How far over the water near it a surface vertex may stand, m: the
/// surface between two cells is drawn on a slope.
const SLACK: f32 = 0.5;

/// How far over its fall's lip a curtain's vertex may stand, m. A fall's
/// lip is the middle of its edge, and its curtain hangs from the corners
/// at the edge's ends, which stand higher where the surface slopes steeply
/// along it: a metre or so, where a surge runs along a brink.
const LIP_SLACK: f32 = 1.5;

/// Depth below which a surface vertex is not asked about, m: the draw cuts
/// the surface off at its shore.
const SHORE: f32 = 0.05;

/// Columns out from a surface vertex that its water is looked for in, and
/// that the head of water run full in a hole is. Such water is drawn at
/// its head, over its own roof and inside the ground wherever the roof is
/// really there, and where several holes meet in a pocket each corner
/// takes the mean of the heads round it, which reach a cell further.
const NEAR: i32 = 2;
const HEAD_NEAR: i32 = 3;

/// How far from where a curtain's vertex says its water left the lip of
/// the fall it is drawn for may lie, m.
const SAME_LIP: f32 = 0.8;

/// What a stray vertex is part of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drawn {
    Surface,
    Curtain,
}

/// A vertex drawn where the field has no water.
#[derive(Debug, Clone, Copy)]
pub struct Stray {
    pub drawn: Drawn,
    pub at: Point3<f32>,
    /// How far over the water near it, or the lip it left, it stands, m:
    /// not finite for a vertex that is not.
    pub over: f32,
}

/// Whether every index of `mesh` is of a vertex it has.
pub fn indexed_within(mesh: &RiverMesh) -> bool {
    let count = mesh.vertices.len();
    (mesh.indices.iter().chain(&mesh.sheet_indices)).all(|&index| (index as usize) < count)
}

/// The vertex of `mesh` that strays furthest from `water`'s field, if any
/// does. Indices past the mesh's vertices (`indexed_within`) are passed
/// over, and so are triangles folded away to nothing.
pub fn worst_stray(mesh: &RiverMesh, water: &WaterWorld) -> Option<Stray> {
    let surface = strays(mesh, &mesh.indices, Drawn::Surface, water);
    let curtain = strays(mesh, &mesh.sheet_indices, Drawn::Curtain, water);
    surface
        .chain(curtain)
        .max_by(|a, b| key(a.over).total_cmp(&key(b.over)))
}

/// A stray's distance as a key to sort by: a vertex that is not finite is
/// the furthest.
fn key(over: f32) -> f32 {
    if over.is_finite() {
        over
    } else {
        f32::INFINITY
    }
}

fn strays<'a>(
    mesh: &'a RiverMesh,
    indices: &'a [u32],
    drawn: Drawn,
    water: &'a WaterWorld,
) -> impl Iterator<Item = Stray> + 'a {
    let mut asked = vec![false; mesh.vertices.len()];
    let place = |index: u32| {
        let vertex = mesh.vertices.get(index as usize)?;
        Some((vertex.xz, vertex.surface))
    };
    // A triangle folded to a line or a point draws nothing: a spent fall's
    // curtain, put away where its top was.
    let shown = indices.chunks_exact(3).filter(move |triangle| {
        let [a, b, c] = [triangle[0], triangle[1], triangle[2]].map(place);
        a != b && b != c && c != a
    });
    shown.flatten().filter_map(move |&index| {
        let vertex = mesh.vertices.get(index as usize)?;
        if std::mem::replace(&mut asked[index as usize], true) {
            return None;
        }
        let at = Point3::new(vertex.xz.x, vertex.surface, vertex.xz.y);
        if !finite(vertex) {
            return Some(Stray {
                drawn,
                at,
                over: f32::NAN,
            });
        }
        // Falling water is drawn as a body round its arc, half as thick
        // again over it as the water is there.
        let (highest, slack) = match drawn {
            Drawn::Surface if vertex.depth > SHORE => {
                let near = water_near(water, vertex.xz);
                (
                    near.stands.into_iter().chain(near.head).reduce(f32::max)?,
                    SLACK,
                )
            }
            Drawn::Surface => return None,
            Drawn::Curtain => {
                // It hangs from the surface's corners, which stand at the
                // head of any water run full beside its lip.
                let lip = lip_of(water, vertex.flow)? + 0.5 * vertex.sheet.w;
                let head = water_near(water, vertex.flow).head;
                (head.map_or(lip, |head| head.max(lip)), LIP_SLACK)
            }
        };
        let over = vertex.surface - highest;
        (over > slack).then_some(Stray { drawn, at, over })
    })
}

fn finite(vertex: &RiverVertex) -> bool {
    let RiverVertex {
        xz,
        surface,
        depth,
        flow,
        face,
        sheet,
        normal,
        fray,
        air,
        life: _,
    } = vertex;
    xz.iter()
        .chain([surface, depth, fray, air])
        .chain(flow.iter())
        .chain(face.iter())
        .chain(sheet.iter())
        .chain(normal.iter())
        .all(|v| v.is_finite())
}

/// The water near a point.
#[derive(Default)]
struct Near {
    /// The highest it stands within `NEAR` columns, in the field or as it
    /// was last drawn.
    stands: Option<f32>,
    /// The highest head of water run full in a hole within `HEAD_NEAR`.
    head: Option<f32>,
}

/// The water near a point: how high it stands, and the head of any of it
/// run full in a hole.
fn water_near(water: &WaterWorld, xz: Vector2<f32>) -> Near {
    let field = water.field();
    let (graph, cells) = (field.graph(), field.cells());
    let drawn = water.drawn_depths();
    let middle = Column::containing(xz.x - 0.5 * COLUMN_SIZE, xz.y - 0.5 * COLUMN_SIZE);
    let mut found = Near::default();
    let higher = |a: Option<f32>, b: f32| Some(a.map_or(b, |a| a.max(b)));
    for di in -HEAD_NEAR + 1..=HEAD_NEAR {
        for dk in -HEAD_NEAR + 1..=HEAD_NEAR {
            let column = middle.offset(di, dk);
            if !graph.contains_column(column) {
                continue;
            }
            let near = (-NEAR + 1..=NEAR).contains(&di) && (-NEAR + 1..=NEAR).contains(&dk);
            for ordinal in 0..graph.spans(column).len() {
                let Some(cell) = cells.index(graph, column, ordinal as u8) else {
                    continue;
                };
                let was = (drawn.depths.get(cell.slot as usize))
                    .and_then(|depths| depths.get(cell.flat as usize))
                    .copied()
                    .unwrap_or(0.0);
                let depth = cells.depth(cell).max(was);
                if depth <= 0.0 {
                    continue;
                }
                if near {
                    found.stands = higher(found.stands, cells.bed(cell) + depth as f32);
                }
                if let Some(head) = cells.head(cell, |end| cells.surface(end) as f32) {
                    found.head = higher(found.head, head);
                }
            }
        }
    }
    found
}

/// The highest that falling water which left within `SAME_LIP` of `left`,
/// level, begins or rises: the lip of a fall, or the top of a spring's or
/// pump's jet, which may be thrown upward. None where nothing leaves from
/// there: a curtain whose fall is gone.
fn lip_of(water: &WaterWorld, left: Vector2<f32>) -> Option<f32> {
    let field = water.field();
    let near = |x: f32, z: f32| (Vector2::new(x, z) - left).norm() <= SAME_LIP;
    let falls = (field.falls().iter())
        .filter(|fall| near(fall.lip.x, fall.lip.z))
        .map(|fall| fall.lip.y);
    let jets = (field.jets())
        .filter(|jet| near(jet.origin.x, jet.origin.z))
        .map(|jet| jet.origin.y + jet.velocity.y.max(0.0).powi(2) / (2.0 * GRAVITY));
    falls.chain(jets).reduce(f32::max)
}
