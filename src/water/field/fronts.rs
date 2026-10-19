//! Fronts: where the field's water runs out over ground it has not covered.
//!
//! ```text
//!   awake chunk ──▶ each face carrying water from a wet cell into a dry
//!                   one (`WET_DEPTH`, the depth the surface is drawn from)
//!               ──▶ a Front at the face: the water's top and the floor it
//!                   runs out over, and how fast the water behind it runs
//! ```
//!
//! A front is where the surface ends against nothing: no bank rises to meet
//! it, so whatever draws the water has only its cells' edges to end it on.
//! It is also where a surge breaks, so it is where whitewater belongs.

use nalgebra::{Point3, Vector2, Vector3};

use crate::water::geometry::{SpanChunkCoord, SpanIndex, CHUNK_COLUMNS, COLUMN_SIZE};

use super::activity::CALM_DISCHARGE;
use super::cells::FieldCells;
use super::faces::{Axis, FaceRef};
use super::field::WET_DEPTH;

/// Least depth a front's discharge is divided by for its speed, m, so a
/// film's front is not taken for a torrent.
const MEASURED_DEPTH: f64 = 0.05;

/// Water running out through one face on to ground it has not covered.
#[derive(Debug, Clone, Copy)]
pub struct Front {
    /// The middle of the face, at the surface of the water running out
    /// through it.
    pub top: Point3<f32>,
    /// The surface of the surge behind it: the higher of the cell the water
    /// runs out of and the one behind that, since what runs out ahead of a
    /// surge is a film, drawn tilting up to the surge.
    pub crest: f32,
    /// The floor the water runs out over.
    pub foot: f32,
    /// The water's velocity through the face, m/s: horizontal, outwards.
    pub velocity: Vector3<f32>,
    /// Water through the face, m³/s.
    pub discharge: f32,
    /// Along the face, unit.
    pub across: Vector3<f32>,
}

impl Front {
    /// Half the face's width, m.
    pub const HALF_WIDTH: f32 = COLUMN_SIZE * 0.5;
}

/// Every front in the field's awake chunks, or in those that come within
/// `near`'s range of its point.
pub fn fronts(
    cells: &FieldCells,
    near: Option<(Point3<f32>, f32)>,
) -> impl Iterator<Item = Front> + '_ {
    cells
        .chunks
        .iter()
        .enumerate()
        .filter(|(_, chunk)| chunk.awake)
        .filter(move |(_, chunk)| near.is_none_or(|(at, range)| within(chunk.coord, at, range)))
        .flat_map(move |(slot, chunk)| {
            chunk.faces.iter().filter_map(move |face| {
                let a = SpanIndex {
                    slot: slot as u32,
                    flat: face.a,
                };
                let (from, to, outward, upstream) = if face.q >= 0.0 {
                    (a, face.b, 1.0, face.behind.map(|f| owner(cells, f)))
                } else {
                    (face.b, a, -1.0, face.ahead.map(|f| neighbour(cells, f)))
                };
                let q = face.q.abs();
                let (behind, ahead) = (cells.depth(from), cells.depth(to));
                if q < CALM_DISCHARGE || behind <= WET_DEPTH || ahead > WET_DEPTH {
                    return None;
                }
                let (x, z) = centre(cells, a);
                let (normal, across) = match face.axis {
                    Axis::X => (Vector2::x(), Vector3::z()),
                    Axis::Z => (Vector2::y(), Vector3::x()),
                };
                let at = Vector2::new(x, z) + normal * COLUMN_SIZE * 0.5;
                let crest = upstream.map_or(cells.surface(from), |c| {
                    cells.surface(from).max(cells.surface(c))
                });
                let speed = q / behind.max(MEASURED_DEPTH) as f32;
                let out = normal * outward * speed;
                Some(Front {
                    top: Point3::new(at.x, cells.surface(from) as f32, at.y),
                    crest: crest as f32,
                    foot: cells.bed(to),
                    velocity: Vector3::new(out.x, 0.0, out.y),
                    discharge: q * COLUMN_SIZE,
                    across,
                })
            })
        })
}

/// Whether any of a chunk's square comes within `range` of `at`, in plan.
fn within(coord: SpanChunkCoord, at: Point3<f32>, range: f32) -> bool {
    let size = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
    let (x0, z0) = (coord.x as f32 * size, coord.z as f32 * size);
    let dx = (x0 - at.x).max(at.x - (x0 + size)).max(0.0);
    let dz = (z0 - at.z).max(at.z - (z0 + size)).max(0.0);
    dx * dx + dz * dz <= range * range
}

/// The cell on the − side of a face: its owner.
fn owner(cells: &FieldCells, face: FaceRef) -> SpanIndex {
    let a = cells.chunks[face.slot as usize].faces[face.index as usize].a;
    SpanIndex {
        slot: face.slot,
        flat: a,
    }
}

/// The cell on the + side of a face.
fn neighbour(cells: &FieldCells, face: FaceRef) -> SpanIndex {
    cells.chunks[face.slot as usize].faces[face.index as usize].b
}

/// The centre of a cell's column, x and z.
fn centre(cells: &FieldCells, at: SpanIndex) -> (f32, f32) {
    let chunk = &cells.chunks[at.slot as usize];
    let (local, _) = chunk.column_of(at.flat as usize);
    chunk.coord.column(local).centre()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::field::faces::{link_faces, Face};
    use crate::water::geometry::{Span, SpanChunk, SpanGraph, COLUMNS_PER_CHUNK};

    /// One awake chunk of open columns on a level floor, with a face from
    /// the first column to the next along x carrying `q`.
    fn two_cells(behind: f64, ahead: f64, q: f32) -> FieldCells {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let open = Span {
            floor_c: 0.0,
            bed: 0.0,
            quarters: [0.0; 4],
            floor_min: 0.0,
            floor_max: 0.0,
            ceiling: f32::INFINITY,
        };
        let mut graph = SpanGraph::new(coord, coord);
        graph.replace_chunk(
            coord,
            SpanChunk::from_columns(&vec![vec![open]; COLUMNS_PER_CHUNK], 1),
        );
        let mut cells = FieldCells::build(&graph);
        let chunk = &mut cells.chunks[0];
        chunk.awake = true;
        chunk.depth[0] = behind;
        chunk.depth[1] = ahead;
        chunk.faces.push(Face {
            a: 0,
            b: SpanIndex { slot: 0, flat: 1 },
            axis: Axis::X,
            roof: f32::INFINITY,
            q,
            behind: None,
            ahead: None,
        });
        cells
    }

    #[test]
    fn water_running_into_a_dry_cell_is_a_front_at_their_face() {
        let cells = two_cells(0.5, 0.0, 1.0);
        let fronts: Vec<Front> = fronts(&cells, None).collect();
        assert_eq!(fronts.len(), 1);
        let front = fronts[0];
        assert!((front.top.x - COLUMN_SIZE).abs() < 1e-6, "{:?}", front.top);
        assert!((front.top.y - 0.5).abs() < 1e-6);
        assert!(
            (front.velocity.x - 2.0).abs() < 1e-6,
            "{:?}",
            front.velocity
        );
        assert!(front.velocity.z.abs() < 1e-6);
    }

    #[test]
    fn a_front_running_the_other_way_points_the_other_way() {
        let cells = two_cells(0.0, 0.5, -1.0);
        let front = fronts(&cells, None).next().expect("a front");
        assert!(front.velocity.x < 0.0);
        assert!((front.top.y - 0.5).abs() < 1e-6);
    }

    #[test]
    fn water_running_into_water_is_no_front() {
        assert_eq!(fronts(&two_cells(0.5, 0.3, 1.0), None).count(), 0);
    }

    #[test]
    fn a_film_ahead_of_a_surge_fronts_at_the_surge_s_top() {
        let mut cells = two_cells(1.0, 0.01, 1.0);
        let chunk = &mut cells.chunks[0];
        chunk.depth[2] = 0.0;
        chunk.depth[1] = 0.01;
        chunk.faces.push(Face {
            a: 1,
            b: SpanIndex { slot: 0, flat: 2 },
            axis: Axis::X,
            roof: f32::INFINITY,
            q: 0.1,
            behind: None,
            ahead: None,
        });
        link_faces(&mut cells, &[0]);
        let front = fronts(&cells, None).next().expect("a front");
        assert!(
            (front.top.x - 2.0 * COLUMN_SIZE).abs() < 1e-6,
            "{:?}",
            front.top
        );
        assert!((front.top.y - 0.01).abs() < 1e-6, "{:?}", front.top);
        assert!((front.crest - 1.0).abs() < 1e-6, "{}", front.crest);
    }

    #[test]
    fn a_chunk_out_of_range_is_not_searched() {
        let cells = two_cells(0.5, 0.0, 1.0);
        let near = |x: f32| Some((Point3::new(x, 0.0, 4.0), 5.0));
        assert_eq!(fronts(&cells, near(12.0)).count(), 1);
        assert_eq!(fronts(&cells, near(14.0)).count(), 0);
    }

    #[test]
    fn a_sleeping_chunk_has_no_fronts() {
        let mut cells = two_cells(0.5, 0.0, 1.0);
        cells.chunks[0].awake = false;
        assert_eq!(fronts(&cells, None).count(), 0);
    }
}
