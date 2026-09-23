use nalgebra::Point3;

use crate::perf::Ground;

/// Where to set off the charges: a regular grid of surface points across the
/// level.
///
/// A crater's cost depends on how many chunks it touches, and that depends on
/// where it lands relative to the chunk lattice — one chunk in the middle of a
/// chunk, up to eight at a corner. A spacing that is not a multiple of the
/// chunk size walks the sites across every alignment, so the sweep samples all
/// of them without having to know the lattice.
#[derive(Debug, Clone, Copy)]
pub struct BlastSweep {
    /// Distance between neighbouring sites, in metres. Should be more than
    /// twice the blast's reach so that no crater disturbs another's chunks.
    pub spacing: f32,
    /// Distance kept from the level's edge, in metres, so every crater lands
    /// wholly inside the terrain.
    pub margin: f32,
    /// Stop after this many sites.
    pub limit: usize,
}

impl Default for BlastSweep {
    fn default() -> Self {
        Self {
            spacing: 21.0,
            margin: 8.0,
            limit: 24,
        }
    }
}

impl BlastSweep {
    /// Surface points in row order (x fastest). Columns with no meshed surface
    /// are skipped rather than counted towards the limit.
    pub fn sites(&self, ground: &Ground) -> Vec<Point3<f32>> {
        let bounds = ground.terrain().bounds();
        let (min_x, max_x) = (bounds.min.x + self.margin, bounds.max.x - self.margin);
        let (min_z, max_z) = (bounds.min.z + self.margin, bounds.max.z - self.margin);
        let columns = axis_steps(min_x, max_x, self.spacing);
        let rows = axis_steps(min_z, max_z, self.spacing);

        rows.iter()
            .flat_map(|&z| columns.iter().map(move |&x| (x, z)))
            .filter_map(|(x, z)| ground.surface_at(x, z))
            .take(self.limit)
            .collect()
    }
}

/// Positions from `min` to `max` inclusive, `step` apart.
fn axis_steps(min: f32, max: f32, step: f32) -> Vec<f32> {
    if max < min || step <= 0.0 {
        return Vec::new();
    }
    let count = ((max - min) / step).floor() as usize + 1;
    (0..count).map(|i| min + i as f32 * step).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_cover_the_range_and_stop_inside_it() {
        assert_eq!(axis_steps(0.0, 10.0, 5.0), vec![0.0, 5.0, 10.0]);
        assert_eq!(axis_steps(0.0, 9.0, 5.0), vec![0.0, 5.0]);
        assert!(axis_steps(1.0, 0.0, 5.0).is_empty());
    }
}
