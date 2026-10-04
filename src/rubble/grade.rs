//! What a fragment is worth simulating as.

use crate::terrain::Fragment;

/// What becomes of a fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grade {
    /// Too small or too thin to show: a puff of dust where it broke.
    Dust,
    /// A chip: falls on its own simple flight, no physics body, and crumbles
    /// where it lands.
    Scree,
    /// Big and solid enough to tumble and come to rest as a rigid body.
    Boulder,
}

/// The facts about a fragment that grading reads, so the rules can be judged
/// without building one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measure {
    /// Solid sample count × voxel volume, in m³.
    pub volume: f32,
    /// Solid samples.
    pub samples: usize,
    /// Samples that bear load.
    pub bearing: usize,
}

impl Measure {
    pub fn of(fragment: &Fragment) -> Self {
        Self {
            volume: fragment.volume(),
            samples: fragment.sample_count(),
            bearing: fragment.bearing_samples(),
        }
    }
}

/// Where the grades divide. Starting values, to be tuned in play.
#[derive(Debug, Clone, Copy)]
pub struct GradeRules {
    /// Below this volume, in m³, a fragment is dust.
    pub dust_volume: f32,
    /// Below this volume, in m³, a fragment is scree.
    pub scree_volume: f32,
    /// With fewer samples than this, a fragment is scree whatever its
    /// volume. It is a few marching-cubes cells, every face of it sub-voxel
    /// detail that bricks only guess at, and as a body it started inside the
    /// crater wall beside it (E23).
    pub min_boulder_samples: usize,
}

impl Default for GradeRules {
    fn default() -> Self {
        Self {
            dust_volume: 0.02,
            scree_volume: 0.25,
            min_boulder_samples: 4,
        }
    }
}

impl GradeRules {
    /// The grade of a fragment with `measure`.
    ///
    /// A fragment with no bearing sample is dust whatever its volume: every
    /// sample is a skin around nothing, and marching cubes draws it a few
    /// hundredths of a voxel thick. A thin one with bearing samples is a
    /// boulder like any other: a column one or two samples across is
    /// drawn solid, and its bricks are fitted to what is drawn.
    pub fn grade(&self, measure: Measure) -> Grade {
        if measure.volume < self.dust_volume || measure.bearing == 0 {
            Grade::Dust
        } else if measure.volume < self.scree_volume || measure.samples < self.min_boulder_samples {
            Grade::Scree
        } else {
            Grade::Boulder
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn measure(volume: f32, samples: usize, bearing: usize) -> Measure {
        Measure {
            volume,
            samples,
            bearing,
        }
    }

    #[test]
    fn each_grade_starts_where_the_rules_say() {
        let rules = GradeRules::default();
        let cases = [
            (measure(0.019, 8, 1), Grade::Dust),
            (measure(0.02, 8, 1), Grade::Scree),
            (measure(4.0, 32, 0), Grade::Dust),
            (measure(0.249, 30, 30), Grade::Scree),
            (measure(0.25, 30, 30), Grade::Boulder),
            (measure(3.0, 3, 3), Grade::Scree),
            (measure(0.5, 4, 4), Grade::Boulder),
            (measure(8.5, 68, 64), Grade::Boulder),
        ];
        for (m, expected) in cases {
            assert_eq!(rules.grade(m), expected, "{m:?}");
        }
    }
}
