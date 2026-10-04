//! What a fragment is worth simulating as.

use crate::terrain::Fragment;

/// What becomes of a fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grade {
    /// Too small or too thin to show: a puff of dust where it broke.
    Dust,
    /// A sliver or a chip: falls on its own simple flight, no physics body,
    /// and crumbles where it lands.
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
    /// Samples that bear load.
    pub bearing: usize,
    /// Interior samples: bearing, with all six neighbours solid.
    pub core: usize,
}

impl Measure {
    pub fn of(fragment: &Fragment) -> Self {
        Self {
            volume: fragment.volume(),
            bearing: fragment.bearing_samples(),
            core: fragment.core_samples(),
        }
    }
}

/// Where the grades divide. Starting values, to be tuned in play.
#[derive(Debug, Clone, Copy)]
pub struct GradeRules {
    /// Below this volume, in m³, a fragment is dust.
    pub dust_volume: f32,
    /// Below this volume, in m³, a fragment with a core is still scree.
    pub scree_volume: f32,
}

impl Default for GradeRules {
    fn default() -> Self {
        Self {
            dust_volume: 0.02,
            scree_volume: 0.25,
        }
    }
}

impl GradeRules {
    /// The grade of a fragment with `measure`.
    ///
    /// A fragment with no bearing sample is dust whatever its volume: every
    /// sample is a skin around nothing, and marching cubes draws it a few
    /// hundredths of a voxel thick. One with no core is scree: a shell, a
    /// sheet or a strip, which should fall away rather than land and slide
    /// around as a body with no thickness.
    pub fn grade(&self, measure: Measure) -> Grade {
        if measure.volume < self.dust_volume || measure.bearing == 0 {
            Grade::Dust
        } else if measure.core == 0 || measure.volume < self.scree_volume {
            Grade::Scree
        } else {
            Grade::Boulder
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn measure(volume: f32, bearing: usize, core: usize) -> Measure {
        Measure {
            volume,
            bearing,
            core,
        }
    }

    #[test]
    fn each_grade_starts_where_the_rules_say() {
        let rules = GradeRules::default();
        let cases = [
            (measure(0.019, 1, 1), Grade::Dust),
            (measure(0.02, 1, 0), Grade::Scree),
            (measure(4.0, 0, 0), Grade::Dust),
            (measure(4.0, 30, 0), Grade::Scree),
            (measure(0.249, 30, 1), Grade::Scree),
            (measure(0.25, 30, 1), Grade::Boulder),
            (measure(50.0, 400, 120), Grade::Boulder),
        ];
        for (m, expected) in cases {
            assert_eq!(rules.grade(m), expected, "{m:?}");
        }
    }
}
