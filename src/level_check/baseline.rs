//! Committed mesh-integrity baselines.
//!
//! Terrain is not perfectly watertight: ambiguous marching-cubes configurations
//! in cave noise leave a small number of open edges, measured at stage 1 and
//! unrelated to chunk seams. An absolute "zero open edges" rule would therefore
//! fail every shipped level, and no rule at all would let a real crack through.
//!
//! So the check is relative: a committed per-level count, and an error only when
//! the measured count rises meaningfully above it. The baseline file lives
//! beside the levels it describes, so it is found from the level path rather
//! than from the working directory.

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// File name of the baseline document, expected in the same directory as the
/// level files.
pub const BASELINE_FILE: &str = "mesh_baselines.ron";

/// Fractional rise in open edges tolerated before it counts as an error.
const RISE_TOLERANCE: f32 = 0.25;

/// Absolute rise always tolerated, so tiny baselines are not hair-triggered.
const RISE_FLOOR: usize = 8;

/// Fractional fall after which the baseline is reported as stale.
const FALL_TOLERANCE: f32 = 0.25;

/// One level's committed mesh figures.
#[derive(Debug, Clone, Deserialize)]
pub struct Baseline {
    /// Level file name, e.g. `"test_arena.level.ron"`.
    pub level: String,
    /// Triangles the level meshed to when the baseline was taken. Recorded for
    /// context — the open-edge count means little without knowing the mesh size.
    pub triangles: usize,
    /// Open edges measured at that time.
    pub open_edges: usize,
}

/// The committed baselines for a directory of levels.
#[derive(Debug, Clone, Deserialize, Default)]
pub struct Baselines {
    pub entries: Vec<Baseline>,
}

impl Baselines {
    /// Load the baselines that apply to `level_path`, from its own directory.
    ///
    /// A missing file is not an error: it yields an empty set, and every level
    /// then reports as unbaselined.
    pub fn for_level(level_path: &Path) -> Result<Self, String> {
        let path = Self::path_for(level_path);
        if !path.exists() {
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| format!("{} could not be read: {e}", path.display()))?;
        ron::from_str(&text).map_err(|e| format!("{} could not be parsed: {e}", path.display()))
    }

    /// Where the baseline file for a given level would live.
    pub fn path_for(level_path: &Path) -> PathBuf {
        level_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(BASELINE_FILE)
    }

    fn get(&self, level_file: &str) -> Option<&Baseline> {
        self.entries.iter().find(|e| e.level == level_file)
    }

    /// Compare a measured open-edge count against the committed one.
    pub fn compare(&self, level_path: &Path, open_edges: usize) -> BaselineVerdict {
        let file = level_path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();

        let Some(baseline) = self.get(&file) else {
            return BaselineVerdict::Unbaselined;
        };

        let limit = baseline.open_edges + (baseline.open_edges as f32 * RISE_TOLERANCE) as usize;
        let limit = limit.max(baseline.open_edges + RISE_FLOOR);
        if open_edges > limit {
            return BaselineVerdict::Risen {
                baseline: baseline.open_edges,
                limit,
            };
        }

        let fall_threshold =
            baseline.open_edges as f32 * (1.0 - FALL_TOLERANCE) - RISE_FLOOR as f32;
        if (open_edges as f32) < fall_threshold {
            return BaselineVerdict::Stale {
                baseline: baseline.open_edges,
            };
        }

        BaselineVerdict::Within {
            baseline: baseline.open_edges,
        }
    }
}

/// Outcome of comparing a measured open-edge count against the baseline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaselineVerdict {
    /// No committed figure for this level.
    Unbaselined,
    /// At or near the committed figure.
    Within { baseline: usize },
    /// Meaningfully above it — new cracks. An error.
    Risen { baseline: usize, limit: usize },
    /// Meaningfully below it — the mesh improved and the baseline should be
    /// re-committed, or it stops catching regressions.
    Stale { baseline: usize },
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baselines() -> Baselines {
        Baselines {
            entries: vec![Baseline {
                level: "a.level.ron".into(),
                triangles: 1000,
                open_edges: 200,
            }],
        }
    }

    fn verdict(open_edges: usize) -> BaselineVerdict {
        baselines().compare(Path::new("levels/a.level.ron"), open_edges)
    }

    #[test]
    fn equal_and_slightly_higher_counts_are_within() {
        assert!(matches!(verdict(200), BaselineVerdict::Within { .. }));
        assert!(matches!(verdict(240), BaselineVerdict::Within { .. }));
    }

    #[test]
    fn a_large_rise_is_flagged() {
        assert!(matches!(verdict(400), BaselineVerdict::Risen { .. }));
    }

    #[test]
    fn a_large_fall_marks_the_baseline_stale() {
        assert!(matches!(verdict(20), BaselineVerdict::Stale { .. }));
    }

    /// A level with no committed figure must be reported, not silently passed.
    #[test]
    fn unknown_levels_are_unbaselined() {
        let v = baselines().compare(Path::new("levels/other.level.ron"), 0);
        assert_eq!(v, BaselineVerdict::Unbaselined);
    }

    /// A small baseline must not be tripped by a one-edge wobble.
    #[test]
    fn small_baselines_have_an_absolute_floor() {
        let b = Baselines {
            entries: vec![Baseline {
                level: "a.level.ron".into(),
                triangles: 10,
                open_edges: 2,
            }],
        };
        let path = Path::new("levels/a.level.ron");
        assert!(matches!(b.compare(path, 9), BaselineVerdict::Within { .. }));
        assert!(matches!(b.compare(path, 20), BaselineVerdict::Risen { .. }));
    }
}
