//! Static meshes for reaches: built on route or re-route, never per frame.
//!
//! ```text
//!   centreline ──▶ sections every ALONG, square to the centreline's
//!                  direction over TANGENT_WINDOW, out to the design top
//!                  width but no further than a bend's radius on its inside
//!              ──▶ a row of quads between each pair; each vertex carries
//!                  the section's bed, the design depth there, its distance
//!                  down the reach, and the flow direction
//! ```
//!
//! The centreline's own points are an eighth of a metre apart and its
//! tangent wobbles with the D8 steps under it. Sections cut there, metres
//! wide, cross one another; so does any section on the inside of a bend
//! tighter than its half width.
//!
//! Each draw pushes the reach's depth scale `(Q/Q_design)^0.6`, its wetted
//! range `[x_t, x_f]`, and how far its ends ease to meet its ports
//! (`Reach::surface_at`, §7.9); the shader discards outside the range, and
//! the depth test trims the width at lower flow, where the banks stand above
//! the water.

use nalgebra::{Point3, Vector2};

use crate::water::ids::StoreId;
use crate::water::network::{Centreline, Reach, ReachEnds};

use super::vertex::RiverVertex;

/// Spacing of vertices across a section, m.
const ACROSS: f32 = 0.5;

/// Width beyond the design top width each side, m, so the bank cuts the
/// water rather than the mesh edge.
const BANK_MARGIN: f32 = 0.5;

/// Spacing of sections down a reach, m.
const ALONG: f32 = 0.5;

/// Distance each side of a section over which its direction is taken, m:
/// longer than the wobble of the D8 steps, so a straight run's sections are
/// parallel.
const TANGENT_WINDOW: f32 = 1.5;

/// How far across the inside of a bend a section reaches, as a share of
/// the bend's radius. Short of the centre of the bend, neighbouring sections
/// do not cross there.
const INSIDE_REACH: f32 = 0.8;

/// One reach's draw.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverDraw {
    pub reach: StoreId,
    pub first_index: u32,
    pub index_count: u32,
}

/// Every reach's surface, in one vertex and index buffer.
#[derive(Debug, Clone, Default)]
pub struct RiverMesh {
    pub vertices: Vec<RiverVertex>,
    pub indices: Vec<u32>,
    pub draws: Vec<RiverDraw>,
}

/// A reach's state as a draw needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiverState {
    /// Depth now over depth at the design discharge.
    pub depth_scale: f32,
    /// Speed now over speed at the design discharge.
    pub speed_scale: f32,
    /// The wetted range, m from the top of the reach.
    pub tail: f32,
    pub front: f32,
    /// How far its ends are eased to meet its ports (§7.9).
    pub ends: ReachEnds,
    /// Its length, m, which its ends ease back from.
    pub length: f32,
}

impl RiverState {
    pub fn of(reach: &Reach, ends: ReachEnds) -> Self {
        let design = reach.rating.at(reach.rating.design());
        let running = reach.running();
        Self {
            depth_scale: if design.depth > 0.0 {
                running.depth / design.depth
            } else {
                0.0
            },
            speed_scale: if design.velocity > 0.0 {
                running.velocity / design.velocity
            } else {
                0.0
            },
            tail: reach.tail,
            front: reach.front,
            ends,
            length: reach.length,
        }
    }
}

/// Build the surface of every reach.
pub fn build<'a>(reaches: impl Iterator<Item = (StoreId, &'a Reach)>) -> RiverMesh {
    let mut mesh = RiverMesh::default();
    for (id, reach) in reaches {
        append_reach(&mut mesh, id, reach);
    }
    mesh
}

fn append_reach(mesh: &mut RiverMesh, id: StoreId, reach: &Reach) {
    let design = reach.rating.at(reach.rating.design());
    let half = design.top_width * 0.5 + BANK_MARGIN;
    let steps = (half / ACROSS).ceil() as i32;
    let row = (2 * steps + 1) as u32;
    let first_index = mesh.indices.len() as u32;
    let base = mesh.vertices.len() as u32;
    let mut sections = sections(&reach.centreline);
    if let Some(lip) = fall_lip(reach) {
        extend_to(&mut sections, lip);
    }
    for section in &sections {
        let across = section.across();
        for s in -steps..=steps {
            let reach_out = if s > 0 { section.left } else { section.right };
            let offset = across * (s as f32 * ACROSS).clamp(-reach_out, reach_out);
            mesh.vertices.push(RiverVertex {
                xz: Vector2::new(section.centre.x + offset.x, section.centre.z + offset.y),
                bed: section.centre.y,
                depth: design.depth,
                along: section.along,
                flow: section.tangent * design.velocity,
            });
        }
    }
    for i in 0..sections.len().saturating_sub(1) as u32 {
        for s in 0..row - 1 {
            let a = base + i * row + s;
            let b = a + 1;
            let c = a + row + 1;
            let d = a + row;
            mesh.indices.extend_from_slice(&[a, d, c, a, c, b]);
        }
    }
    mesh.draws.push(RiverDraw {
        reach: id,
        first_index,
        index_count: mesh.indices.len() as u32 - first_index,
    });
}

/// A cut across a reach's surface.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Section {
    /// On the centreline, at the bed.
    centre: Point3<f32>,
    /// Unit direction of flow in plan.
    tangent: Vector2<f32>,
    /// Distance down the reach, m.
    along: f32,
    /// How far the section may reach to the left of the flow, m.
    left: f32,
    /// How far it may reach to the right.
    right: f32,
}

impl Section {
    /// Unit direction to the left of the flow, in plan.
    fn across(&self) -> Vector2<f32> {
        Vector2::new(-self.tangent.y, self.tangent.x)
    }
}

/// Sections every `ALONG` down a centreline, the last at its end.
fn sections(line: &Centreline) -> Vec<Section> {
    let length = line.length();
    let count = (length / ALONG).ceil().max(1.0) as usize;
    let mut sections: Vec<Section> = (0..=count)
        .map(|k| {
            let along = (k as f32 * ALONG).min(length);
            Section {
                centre: line.point_at(along),
                tangent: direction_at(line, along),
                along,
                left: f32::INFINITY,
                right: f32::INFINITY,
            }
        })
        .collect();
    for k in 1..sections.len().saturating_sub(1) {
        let (before, after) = (sections[k - 1], sections[k + 1]);
        let turn = before
            .tangent
            .perp(&after.tangent)
            .atan2(before.tangent.dot(&after.tangent));
        let run = after.along - before.along;
        if turn.abs() < 1e-4 || run <= 0.0 {
            continue;
        }
        let inside = INSIDE_REACH * run / turn.abs();
        if turn > 0.0 {
            sections[k].left = inside;
        } else {
            sections[k].right = inside;
        }
    }
    sections
}

/// Where a reach's water leaves its bed over a fall: the fall's first
/// point. The centreline ends at the middle of its last column, up to half
/// a column short of it.
fn fall_lip(reach: &Reach) -> Option<Vector2<f32>> {
    let fall = reach.outlet.as_ref()?.fall.as_ref()?;
    let first = fall.points.first()?;
    Some(Vector2::new(first.x, first.z))
}

/// Carry the surface on from its last section to `lip`, when the lip lies
/// ahead of it.
fn extend_to(sections: &mut Vec<Section>, lip: Vector2<f32>) {
    let Some(&last) = sections.last() else {
        return;
    };
    let ahead = lip - Vector2::new(last.centre.x, last.centre.z);
    if ahead.dot(&last.tangent) <= 1e-3 {
        return;
    }
    sections.push(Section {
        centre: Point3::new(lip.x, last.centre.y, lip.y),
        ..last
    });
}

/// The centreline's direction in plan at `along`, over `TANGENT_WINDOW`
/// each side.
fn direction_at(line: &Centreline, along: f32) -> Vector2<f32> {
    let (a, b) = (
        line.point_at(along - TANGENT_WINDOW),
        line.point_at(along + TANGENT_WINDOW),
    );
    let d = Vector2::new(b.x - a.x, b.z - a.z);
    if d.norm() > 1e-6 {
        d.normalize()
    } else {
        Vector2::x()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A D8 path in 0.5 m steps: east for `east` steps, then north.
    fn bend(east: usize, north: usize) -> Centreline {
        let mut path: Vec<Point3<f32>> = (0..=east)
            .map(|i| Point3::new(i as f32 * 0.5, 0.0, 0.0))
            .collect();
        let corner = east as f32 * 0.5;
        path.extend((1..=north).map(|j| Point3::new(corner, 0.0, j as f32 * 0.5)));
        Centreline::from_path(&path)
    }

    /// Each edge of the surface, left and right, in order down the reach.
    fn edges(line: &Centreline, half: f32) -> [Vec<Vector2<f32>>; 2] {
        let sections = sections(line);
        let edge = |sign: f32| {
            sections
                .iter()
                .map(|s| {
                    let out = if sign > 0.0 { s.left } else { s.right };
                    Vector2::new(s.centre.x, s.centre.z) + s.across() * (sign * half.min(out))
                })
                .collect()
        };
        [edge(1.0), edge(-1.0)]
    }

    #[test]
    fn the_surface_does_not_fold_on_the_inside_of_a_bend() {
        let line = bend(20, 20);
        let sections = sections(&line);
        for side in edges(&line, 3.0) {
            for (k, pair) in side.windows(2).enumerate() {
                let step = pair[1] - pair[0];
                let tangent = sections[k].tangent + sections[k + 1].tangent;
                assert!(
                    step.dot(&tangent) >= -1e-4,
                    "the edge runs back up the reach at section {k}: {step:?}"
                );
            }
        }
    }

    #[test]
    fn a_straight_run_s_sections_are_parallel_over_its_steps() {
        // A D8 path zig-zagging along a 30° line.
        let path: Vec<Point3<f32>> = (0..40)
            .map(|i| Point3::new(i as f32 * 0.5, 0.0, (i / 2) as f32 * 0.5))
            .collect();
        let line = Centreline::from_path(&path);
        let sections = sections(&line);
        let n = sections.len();
        let angles: Vec<f32> = sections[n / 4..3 * n / 4]
            .iter()
            .map(|s| s.tangent.y.atan2(s.tangent.x).to_degrees())
            .collect();
        let spread = angles.iter().copied().fold(f32::MIN, f32::max)
            - angles.iter().copied().fold(f32::MAX, f32::min);
        // Within 5°, a section's edge 4 m out swings less than the spacing
        // of sections, so they cannot cross.
        assert!(spread < 5.0, "sections turn through {spread}°");
    }

    #[test]
    fn sections_run_the_whole_reach() {
        let line = bend(7, 5);
        let sections = sections(&line);
        assert_eq!(sections.first().unwrap().along, 0.0);
        assert_eq!(sections.last().unwrap().along, line.length());
        assert_eq!(
            sections.last().unwrap().centre,
            *line.points.last().unwrap()
        );
    }

    #[test]
    fn a_surface_runs_on_to_a_lip_ahead_of_it_only() {
        let line = bend(10, 0);
        let end = *line.points.last().unwrap();
        let mut ahead = sections(&line);
        let n = ahead.len();
        extend_to(&mut ahead, Vector2::new(end.x + 0.25, end.z));
        assert_eq!(ahead.len(), n + 1);
        assert_eq!(ahead[n].centre, Point3::new(end.x + 0.25, end.y, end.z));
        let mut behind = sections(&line);
        extend_to(&mut behind, Vector2::new(end.x - 0.25, end.z));
        assert_eq!(behind.len(), n);
    }
}
