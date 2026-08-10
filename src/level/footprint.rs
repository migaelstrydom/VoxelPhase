//! The ground an object actually stands on, rather than the one point it was
//! authored at.
//!
//! A level object is a single position in the file and often a great deal of
//! geometry in the world: a twelve-block domino row is eight metres long, a
//! `BoxWall` five columns wide, an arch two abutments apart. Validating any of
//! those at their authored point asks about one corner and says nothing about
//! the rest, so a row that starts on one bench and finishes on the next reads
//! as correct while two thirds of it hangs in the air.
//!
//! A [`Footprint`] is the horizontal ground the object covers. It is
//! deliberately coarse — a disc or a turned rectangle — because its job is to
//! catch an object straddling a drop, not to reproduce the collider.
//!
//! ```text
//!            authored point
//!                  │
//!                  ▼
//!     ┌────────────────────────────┐   footprint
//!     └────────────────────────────┘
//!     ────────────┐
//!       bench B3  │                     ← the surface the row was authored to
//!                 └──────────────────
//!                       bench B4        ← what two thirds of it is over
//! ```

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Widest span a footprint is sampled at, per axis.
///
/// Sampling is otherwise driven by the local voxel size, since a feature
/// narrower than a voxel cannot be in the terrain anyway. The cap exists so a
/// long object over fine terrain cannot cost hundreds of ray queries; it trades
/// resolution on the longest objects for a bounded check, which is the right way
/// round for something that reports drops of a metre and more.
const MAX_SAMPLES_PER_AXIS: usize = 9;

/// The horizontal ground an object covers, in world coordinates once the
/// object has been placed.
///
/// Offsets and extents are in the object's own axes, before `yaw` — the same
/// convention every spawnable builds its parts in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Footprint {
    /// Small enough that its authored point is an honest sample on its own.
    ///
    /// Not "has no size": a beach ball has a radius. It means the object is
    /// within a voxel or so of a point, so spreading samples across it would
    /// only ever re-sample the same terrain cell.
    Point,

    /// Radially symmetric about the authored point.
    Disc { radius: f32 },

    /// A rectangle in the object's own axes.
    ///
    /// `offset` is the rectangle's centre relative to the authored point, which
    /// is not always the middle of the object — a domino row is authored at its
    /// *first* block and extends one way from there.
    Rect {
        offset: (f32, f32),
        half: (f32, f32),
        yaw: f32,
    },
}

/// What the ground under an object is expected to look like.
///
/// Almost everything is [`Support::Bedded`]. The exception matters because it
/// is otherwise indistinguishable from the bug this whole module exists to
/// catch: a bridge and a mis-placed wall both have nothing under their middle,
/// and only the author knows which one was intended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Support {
    /// Rests on the ground along its whole footprint. Ground that steps or
    /// falls away under part of it is a mistake.
    Bedded,
    /// Authored to cross a gap, so the ground under it is not expected to be
    /// continuous and asking whether it is level is asking the wrong question.
    Spanning,
}

impl Footprint {
    /// World `(x, z)` points to probe the terrain at.
    ///
    /// `at` is the object's authored horizontal position and `step` the voxel
    /// size where it stands. Always includes the extremes, because the whole
    /// point is the edge that falls away.
    pub fn samples(&self, at: (f32, f32), step: f32) -> Vec<(f32, f32)> {
        match *self {
            Footprint::Point => vec![at],
            Footprint::Disc { radius } => disc_samples(at, radius, step),
            Footprint::Rect { offset, half, yaw } => rect_samples(at, offset, half, yaw, step),
        }
    }

    /// Whether this footprint is wide enough to be worth probing across.
    ///
    /// Lets a caller skip the sampling and the terrain queries entirely for the
    /// many objects that are effectively a point.
    pub fn spans_more_than(&self, step: f32) -> bool {
        match *self {
            Footprint::Point => false,
            Footprint::Disc { radius } => radius * 2.0 > step,
            Footprint::Rect { half, .. } => half.0.max(half.1) * 2.0 > step,
        }
    }
}

/// Centre, plus one or two rings out to the rim.
///
/// A disc is radially symmetric, so a grid would spend most of its samples
/// re-asking the same question. The inner ring only appears once the disc is
/// wide enough for it to land in a different terrain cell.
fn disc_samples(at: (f32, f32), radius: f32, step: f32) -> Vec<(f32, f32)> {
    const SPOKES: usize = 8;

    let mut out = vec![at];
    let mut rings = vec![radius];
    if radius > step {
        rings.push(radius * 0.5);
    }

    for r in rings {
        for spoke in 0..SPOKES {
            let angle = spoke as f32 / SPOKES as f32 * std::f32::consts::TAU;
            out.push((at.0 + r * angle.cos(), at.1 + r * angle.sin()));
        }
    }
    out
}

/// A grid over the rectangle, turned by `yaw` about the authored point.
fn rect_samples(
    at: (f32, f32),
    offset: (f32, f32),
    half: (f32, f32),
    yaw: f32,
    step: f32,
) -> Vec<(f32, f32)> {
    let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw.to_radians());
    let origin = Point3::new(at.0, 0.0, at.1);

    let nx = samples_across(half.0, step);
    let nz = samples_across(half.1, step);

    let mut out = Vec::with_capacity(nx * nz);
    for ix in 0..nx {
        let lx = offset.0 + lerp_extent(half.0, ix, nx);
        for iz in 0..nz {
            let lz = offset.1 + lerp_extent(half.1, iz, nz);
            let world = origin + turn * Vector3::new(lx, 0.0, lz);
            out.push((world.x, world.z));
        }
    }
    out
}

/// How many samples an axis of half-extent `half` needs at resolution `step`.
fn samples_across(half: f32, step: f32) -> usize {
    if half * 2.0 <= step {
        return 1;
    }
    let wanted = (half * 2.0 / step).ceil() as usize + 1;
    wanted.clamp(2, MAX_SAMPLES_PER_AXIS)
}

/// The `i`th of `n` positions spread evenly across `[-half, half]`.
fn lerp_extent(half: f32, i: usize, n: usize) -> f32 {
    if n <= 1 {
        return 0.0;
    }
    -half + 2.0 * half * i as f32 / (n - 1) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The extremes are the whole point: an object straddling a bench edge is
    /// only visible if the ends are probed.
    #[test]
    fn a_rectangle_is_sampled_out_to_its_corners() {
        let f = Footprint::Rect {
            offset: (0.0, 0.0),
            half: (4.0, 1.0),
            yaw: 0.0,
        };
        let samples = f.samples((10.0, 20.0), 1.0);

        let min_x = samples.iter().map(|s| s.0).fold(f32::MAX, f32::min);
        let max_x = samples.iter().map(|s| s.0).fold(f32::MIN, f32::max);
        assert!((min_x - 6.0).abs() < 1e-4, "got {min_x}");
        assert!((max_x - 14.0).abs() < 1e-4, "got {max_x}");
    }

    /// A domino row is authored at its first block, so its footprint reaches
    /// one way only. Sampling it symmetrically would probe ground the row never
    /// touches and miss the ground it does.
    #[test]
    fn an_offset_rectangle_reaches_one_way() {
        let f = Footprint::Rect {
            offset: (4.0, 0.0),
            half: (4.0, 0.5),
            yaw: 0.0,
        };
        let samples = f.samples((0.0, 0.0), 1.0);

        let min_x = samples.iter().map(|s| s.0).fold(f32::MAX, f32::min);
        let max_x = samples.iter().map(|s| s.0).fold(f32::MIN, f32::max);
        assert!(min_x.abs() < 1e-4, "got {min_x}");
        assert!((max_x - 8.0).abs() < 1e-4, "got {max_x}");
    }

    /// Same convention as `Yaw`: a quarter turn maps local `+X` onto world
    /// `−Z`. If these disagree, a turned wall is checked against the ground
    /// beside it.
    #[test]
    fn a_quarter_turn_swings_the_long_axis_onto_minus_z() {
        let f = Footprint::Rect {
            offset: (0.0, 0.0),
            half: (4.0, 0.5),
            yaw: 90.0,
        };
        let samples = f.samples((0.0, 0.0), 1.0);

        let min_z = samples.iter().map(|s| s.1).fold(f32::MAX, f32::min);
        let max_x = samples.iter().map(|s| s.0).fold(f32::MIN, f32::max);
        assert!((min_z + 4.0).abs() < 1e-4, "got {min_z}");
        assert!(max_x < 1.0, "long axis stayed on X: {max_x}");
    }

    /// An object narrower than a voxel cannot straddle anything the terrain is
    /// able to represent, so it should not cost a grid of ray queries.
    #[test]
    fn something_smaller_than_a_voxel_is_one_sample() {
        let f = Footprint::Rect {
            offset: (0.0, 0.0),
            half: (0.2, 0.2),
            yaw: 0.0,
        };
        assert_eq!(f.samples((3.0, 4.0), 1.0), vec![(3.0, 4.0)]);
        assert!(!f.spans_more_than(1.0));
    }

    #[test]
    fn sampling_stays_bounded_on_a_long_object_over_fine_terrain() {
        let f = Footprint::Rect {
            offset: (0.0, 0.0),
            half: (60.0, 30.0),
            yaw: 0.0,
        };
        assert_eq!(
            f.samples((0.0, 0.0), 0.25).len(),
            MAX_SAMPLES_PER_AXIS * MAX_SAMPLES_PER_AXIS
        );
    }

    #[test]
    fn a_disc_gains_an_inner_ring_only_once_it_is_wider_than_a_voxel() {
        let tight = Footprint::Disc { radius: 0.5 };
        let wide = Footprint::Disc { radius: 4.0 };
        assert_eq!(tight.samples((0.0, 0.0), 1.0).len(), 9);
        assert_eq!(wide.samples((0.0, 0.0), 1.0).len(), 17);
    }
}
