//! How much terrain a charge removes.
//!
//! A blast carries a **budget** and spends it outward from the detonation
//! point, paying [`VoxelMaterial::toughness`] for each cubic metre it takes. Soft
//! ground lets the budget reach far and the crater is wide; hard rock stalls it
//! close in and the same charge leaves a dent. The charge does not care what it
//! is digging through — only how much it can afford.
//!
//! ```text
//!   Blast ──▶ budget = yield x charge confinement   (how enclosed the charge is)
//!         └─▶ spend outward, cost = toughness x voxel volume x voxel confinement
//!         └─▶ the radius where the budget runs out ──▶ Chunk::carve_sphere
//! ```
//!
//! # Independent of voxel size
//!
//! Everything is measured in metres: a sample costs its volume, and each
//! enclosure is judged over a fixed radius in metres. So a charge cuts the same
//! crater at any resolution, only drawn finer. The figures are tuned at 1 m
//! voxels.
//!
//! # Why a radius and not a set of voxels
//!
//! Spending outward from a point means the affordable voxels are exactly those
//! within some distance of it, so the whole budget calculation collapses to one
//! number: the radius where the money runs out. That keeps the carve itself
//! untouched — it stays the smooth signed-distance cut that `csg::carve_density`
//! performs, which is what stops craters putting a step in the density field
//! that marching cubes would read as a slope. A budget that picked out
//! individual voxels could not carve smoothly.
//!
//! # Confinement
//!
//! Two distinct effects, easily conflated, that pull in opposite directions:
//!
//! - **Charge confinement** multiplies the *budget*. A grenade lying in the
//!   open vents most of its energy to the air; the same grenade at the bottom of
//!   a shaft couples it into the rock. This is the dominant term, and it is what
//!   makes digging deep stay viable.
//! - **Voxel confinement** multiplies a voxel's *cost*. A voxel with air on five
//!   sides — a ledge, a cave roof — shatters more easily than one buried in a
//!   mass. Deliberately given a narrow range: every voxel at a crater's rim has
//!   just been un-confined, so a wide range makes craters cheaper to widen than
//!   to deepen, and the player ends up scooping shallow bowls instead of
//!   tunnelling. It shades the geometry; it does not set the strategy.

use nalgebra::Point3;

use super::chunk_grid::ChunkGrid;

/// The voxel size the figures in [`BlastConfig::default`] are tuned at.
const TUNED_VOXEL_SIZE: f32 = 1.0;

/// A multiplier interpolated from how enclosed something is.
///
/// `at(0.0)` is fully exposed, `at(1.0)` fully buried.
#[derive(Debug, Clone, Copy)]
pub struct ConfinementRange {
    /// Multiplier when nothing solid surrounds the point.
    pub exposed: f32,
    /// Multiplier when solid surrounds it completely.
    pub buried: f32,
}

impl ConfinementRange {
    pub fn at(&self, enclosure: f32) -> f32 {
        self.exposed + (self.buried - self.exposed) * enclosure.clamp(0.0, 1.0)
    }
}

/// Tuning for how a charge converts into removed terrain.
#[derive(Debug, Clone, Copy)]
pub struct BlastConfig {
    /// Budget an unconfined charge carries, in toughness-units. One unit buys
    /// one cubic metre of `toughness` 1.0 — grass or sand.
    pub charge_yield: f32,

    /// Hard cap on how far the cut can reach, whatever the budget affords.
    /// Keeps a charge in loose sand from excavating half a level.
    pub max_radius: f32,

    /// How much the budget grows as the detonation point becomes enclosed.
    pub charge_confinement: ConfinementRange,

    /// How much a voxel's cost grows as the voxel becomes enclosed.
    pub voxel_confinement: ConfinementRange,
    /// Radius around a voxel sampled to judge how enclosed it is, in metres:
    /// a voxel and a half at the [`TUNED_VOXEL_SIZE`] the figures are tuned at.
    pub voxel_probe_radius: f32,

    /// Radius around the detonation point sampled to judge how enclosed the
    /// charge is. Roughly the standoff over which venting is decided.
    pub charge_probe_radius: f32,
}

impl Default for BlastConfig {
    fn default() -> Self {
        Self {
            charge_yield: 12.0,
            max_radius: 8.0,
            charge_confinement: ConfinementRange {
                exposed: 1.0,
                buried: 3.0,
            },
            voxel_confinement: ConfinementRange {
                exposed: 0.7,
                buried: 2.3,
            },
            charge_probe_radius: 3.0,
            voxel_probe_radius: 1.5,
        }
    }
}

impl BlastConfig {
    /// A charge that cuts exactly `radius` regardless of what it meets.
    ///
    /// For benches and tests that need one fixed, reproducible crater: the
    /// budget is unbounded and confinement is neutral, so the only thing left
    /// deciding the cut is the reach. Not for gameplay — a charge that ignores
    /// the material is the thing this module exists to replace.
    pub fn fixed_radius(radius: f32) -> Self {
        let neutral = ConfinementRange {
            exposed: 1.0,
            buried: 1.0,
        };
        Self {
            charge_yield: f32::MAX,
            max_radius: radius,
            charge_confinement: neutral,
            voxel_confinement: neutral,
            charge_probe_radius: radius,
            voxel_probe_radius: radius,
        }
    }
}

/// One voxel the blast could pay for.
struct Candidate {
    sample: Point3<f32>,
    distance: f32,
    toughness: f32,
}

/// Radius the charge can actually afford to cut, in grid-local units.
///
/// Returns `None` when there is nothing to remove — no solid within reach, or
/// nothing but indestructible material.
///
/// Indestructible voxels are skipped rather than charged for: bedrock stops the
/// cut reaching *it*, but does not shield the destructible rock beside it, which
/// would make an invisible property of one voxel decide the fate of its
/// neighbours.
pub fn effective_radius(
    grid: &ChunkGrid,
    centre: Point3<f32>,
    config: &BlastConfig,
) -> Option<f32> {
    let step = grid.voxel_size();
    let budget = config.charge_yield
        * config.charge_confinement.at(enclosure(
            grid,
            lattice_points(centre, config.charge_probe_radius, step),
        ));

    let volume = step.powi(3);
    let mut candidates: Vec<Candidate> = lattice_points(centre, config.max_radius, step)
        .filter_map(|sample| {
            let voxel = grid.get(sample);
            if !voxel.is_solid() {
                return None;
            }
            Some(Candidate {
                sample,
                distance: nalgebra::distance(&sample, &centre),
                toughness: voxel.material.toughness()?,
            })
        })
        .collect();

    candidates.sort_by(|a, b| a.distance.total_cmp(&b.distance));
    // A voxel's enclosure is the expensive part, and only the voxels reached
    // before the budget runs out need it.
    let cost = |candidate: &Candidate| {
        candidate.toughness
            * volume
            * config.voxel_confinement.at(enclosure(
                grid,
                probe_points(candidate.sample, config.voxel_probe_radius, step),
            ))
    };

    // Spend outward. The cut lands between the last voxel paid for and the
    // first that could not be, so the carve sphere contains exactly the set the
    // budget bought.
    let mut spent = 0.0;
    let mut last_paid: Option<f32> = None;
    for candidate in &candidates {
        // The nearest destructible voxel always goes, however little the charge
        // carries. No material is ever a wall — a small charge repeated has to
        // make progress, or the player is left staring at terrain that looks
        // destructible and never is.
        let cost = cost(candidate);
        let affordable = last_paid.is_none() || spent + cost <= budget;
        if !affordable {
            return Some(midpoint(last_paid.unwrap(), candidate.distance, step));
        }
        spent += cost;
        last_paid = Some(candidate.distance);
    }

    last_paid.map(|d| (d + step * 0.5).min(config.max_radius))
}

/// A cut radius that separates `paid` from `unpaid`, biased to sit clear of both
/// so floating-point noise cannot include or drop a voxel at the boundary.
fn midpoint(paid: f32, unpaid: f32, step: f32) -> f32 {
    if unpaid - paid > 1e-4 {
        (paid + unpaid) * 0.5
    } else {
        paid + step * 0.01
    }
}

/// Fraction of `samples` that are solid.
///
/// 0.0 is open air, 1.0 is fully buried. Sampling the lattice rather than
/// casting rays keeps this independent of voxel size: it reads the same
/// neighbourhood shape whether a voxel is 2 m or 0.5 m.
fn enclosure(grid: &ChunkGrid, samples: impl Iterator<Item = Point3<f32>>) -> f32 {
    let mut total = 0u32;
    let mut solid = 0u32;
    for sample in samples {
        total += 1;
        if grid.get(sample).is_solid() {
            solid += 1;
        }
    }
    if total == 0 {
        return 0.0;
    }
    solid as f32 / total as f32
}

/// Samples within `radius` of the lattice sample `centre`, spaced as on the
/// lattice the figures are tuned at ([`TUNED_VOXEL_SIZE`]), or on the grid's
/// own when that is coarser.
///
/// A voxel's enclosure is the expensive part of a blast, read once for every
/// voxel it pays for. Sampled on the grid's own lattice, a probe of fixed size
/// costs 8× more with each halving of the voxel size: a second a grenade at
/// 0.125 m. At the tuned spacing it reads the same ~19 samples at every
/// resolution, the ones the 1 m lattice reads, and they still land on the
/// grid's lattice, since the spacing is a whole number of its voxels.
fn probe_points(centre: Point3<f32>, radius: f32, step: f32) -> impl Iterator<Item = Point3<f32>> {
    let spacing = step * (TUNED_VOXEL_SIZE / step).round().max(1.0);
    lattice_points(Point3::origin(), radius, spacing).map(move |offset| centre + offset.coords)
}

/// Lattice sample positions within `radius` of `centre`.
///
/// Samples land on the voxel lattice (multiples of `step`), because that is
/// where a chunk actually stores a voxel — reading between them would evaluate
/// a position no voxel occupies.
fn lattice_points(
    centre: Point3<f32>,
    radius: f32,
    step: f32,
) -> impl Iterator<Item = Point3<f32>> {
    let index = |v: f32| (v / step).round() as i32;
    let span = (radius / step).ceil() as i32;
    let (cx, cy, cz) = (index(centre.x), index(centre.y), index(centre.z));
    let radius_sq = radius * radius;

    (-span..=span).flat_map(move |dx| {
        (-span..=span).flat_map(move |dy| {
            (-span..=span).filter_map(move |dz| {
                let p = Point3::new(
                    (cx + dx) as f32 * step,
                    (cy + dy) as f32 * step,
                    (cz + dz) as f32 * step,
                );
                ((p - centre).magnitude_squared() <= radius_sq).then_some(p)
            })
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::voxel::{Voxel, VoxelMaterial};

    const VOXEL: f32 = 1.0;

    /// A solid block of one material, centred on the origin.
    fn block(material: VoxelMaterial, half_extent: f32) -> ChunkGrid {
        let mut grid = ChunkGrid::new(VOXEL);
        let n = (half_extent / VOXEL) as i32;
        for i in -n..=n {
            for j in -n..=n {
                for k in -n..=n {
                    let p = Point3::new(i as f32 * VOXEL, j as f32 * VOXEL, k as f32 * VOXEL);
                    grid.set(p, Voxel::solid(material));
                }
            }
        }
        grid
    }

    /// Solid ground filling everything at or below y = 0.
    fn ground(material: VoxelMaterial, half_extent: f32) -> ChunkGrid {
        let mut grid = ChunkGrid::new(VOXEL);
        let n = (half_extent / VOXEL) as i32;
        for i in -n..=n {
            for j in -n..=0 {
                for k in -n..=n {
                    let p = Point3::new(i as f32 * VOXEL, j as f32 * VOXEL, k as f32 * VOXEL);
                    grid.set(p, Voxel::solid(material));
                }
            }
        }
        grid
    }

    fn voxels_removed(grid: &ChunkGrid, centre: Point3<f32>, radius: f32) -> usize {
        lattice_points(centre, radius, VOXEL)
            .filter(|p| grid.get(*p).is_solid())
            .count()
    }

    /// The point of the whole module: one charge, different ground, different
    /// hole. Sand should give way considerably further than rock.
    #[test]
    fn the_same_charge_cuts_further_through_soft_ground() {
        let config = BlastConfig::default();
        let origin = Point3::origin();

        let sand = effective_radius(&block(VoxelMaterial::Sand, 12.0), origin, &config).unwrap();
        let rock = effective_radius(&block(VoxelMaterial::Rock, 12.0), origin, &config).unwrap();
        let slate = effective_radius(&block(VoxelMaterial::Slate, 12.0), origin, &config).unwrap();

        assert!(
            sand > rock && rock > slate,
            "sand {sand:.2} > rock {rock:.2} > slate {slate:.2}"
        );
    }

    /// No material is a wall. A charge too small to afford even one voxel of
    /// slate must still take the voxel it is touching, or the player is left
    /// hitting terrain that looks destructible and never yields.
    #[test]
    fn an_underpowered_charge_still_makes_progress() {
        let grid = block(VoxelMaterial::Slate, 12.0);
        let config = BlastConfig {
            charge_yield: 0.1,
            ..BlastConfig::default()
        };

        let radius = effective_radius(&grid, Point3::origin(), &config)
            .expect("a charge in solid ground always has something to remove");
        assert!(radius > 0.0, "radius {radius}");
        assert!(
            radius < VOXEL,
            "but only the one voxel it is touching: {radius}"
        );
    }

    /// Bedrock is the floor of the world, and it is the only thing a budget
    /// cannot buy.
    #[test]
    fn no_charge_reaches_into_bedrock() {
        let grid = block(VoxelMaterial::Bedrock, 12.0);
        let config = BlastConfig {
            charge_yield: 1.0e6,
            ..BlastConfig::default()
        };

        assert!(effective_radius(&grid, Point3::origin(), &config).is_none());
    }

    /// Bedrock stops the cut reaching *it*, but must not shield the ordinary
    /// rock beside it — an invisible property of one voxel deciding its
    /// neighbours' fate is exactly the failure this model exists to avoid.
    #[test]
    fn bedrock_does_not_shield_its_neighbours() {
        let mut grid = block(VoxelMaterial::Rock, 12.0);
        // A bedrock slab through the middle, one voxel from the charge.
        for i in -6..=6 {
            for k in -6..=6 {
                grid.set(
                    Point3::new(i as f32 * VOXEL, -VOXEL, k as f32 * VOXEL),
                    Voxel::solid(VoxelMaterial::Bedrock),
                );
            }
        }
        let config = BlastConfig::default();
        let radius = effective_radius(&grid, Point3::origin(), &config).unwrap();
        let plain =
            effective_radius(&block(VoxelMaterial::Rock, 12.0), Point3::origin(), &config).unwrap();

        assert!(
            radius >= plain,
            "bedrock nearby must not shorten the cut through rock: {radius:.2} vs {plain:.2}"
        );
    }

    /// Charge confinement, the term that keeps digging deep viable: the same
    /// charge enclosed in rock couples more of itself into it than one lying on
    /// the surface, which vents most of its energy to the air.
    ///
    /// Measured in voxels removed rather than radius — the two charges sit at
    /// different depths, so their radii are not measuring the same thing.
    #[test]
    fn a_confined_charge_removes_more_than_one_in_the_open() {
        let config = BlastConfig::default();
        let grid = ground(VoxelMaterial::Rock, 16.0);

        let on_surface = Point3::new(0.0, 0.0, 0.0);
        let down_a_shaft = Point3::new(0.0, -8.0, 0.0);

        let vented = voxels_removed(
            &grid,
            on_surface,
            effective_radius(&grid, on_surface, &config).unwrap(),
        );
        let coupled = voxels_removed(
            &grid,
            down_a_shaft,
            effective_radius(&grid, down_a_shaft, &config).unwrap(),
        );

        assert!(
            coupled > vented,
            "a charge down a shaft ({coupled} voxels) should out-cut one on the surface ({vented})"
        );
    }

    /// Loose ground must not let one charge excavate a level.
    #[test]
    fn the_reach_cap_bounds_a_charge_in_soft_ground() {
        let config = BlastConfig {
            charge_yield: 1.0e6,
            max_radius: 5.0,
            ..BlastConfig::default()
        };
        let radius =
            effective_radius(&block(VoxelMaterial::Sand, 20.0), Point3::origin(), &config).unwrap();
        assert!(radius <= config.max_radius, "radius {radius}");
    }

    /// Solid ground of `material` at or below y = 0 on a lattice of `step`.
    fn ground_at(step: f32, material: VoxelMaterial, half_extent: f32) -> ChunkGrid {
        let mut grid = ChunkGrid::new(step);
        let n = (half_extent / step) as i32;
        for i in -n..=n {
            for j in -n..=0 {
                for k in -n..=n {
                    let p = Point3::new(i as f32 * step, j as f32 * step, k as f32 * step);
                    grid.set(p, Voxel::solid(material));
                }
            }
        }
        grid
    }

    /// A grenade cuts the crater it cuts at 1 m voxels at any resolution, only
    /// drawn finer: in the open and buried, in sand and in rock. The radii
    /// agree to within half a 1 m voxel, the step by which a crater on that
    /// lattice grows; at 0.5 and 0.25 m they agree to a tenth of a metre.
    #[test]
    fn a_charge_cuts_the_same_crater_at_any_voxel_size() {
        let config = BlastConfig::default();
        for material in [VoxelMaterial::Sand, VoxelMaterial::Rock] {
            for at in [Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, -4.0, 0.0)] {
                let radii: Vec<f32> = [1.0, 0.5, 0.25]
                    .iter()
                    .map(|&step| {
                        effective_radius(&ground_at(step, material, 12.0), at, &config).unwrap()
                    })
                    .collect();
                let spread = radii.iter().cloned().fold(f32::MIN, f32::max)
                    - radii.iter().cloned().fold(f32::MAX, f32::min);
                assert!(
                    spread <= 0.5,
                    "{material:?} at {at:?}: radii at 1, 0.5, 0.25 m are {radii:?}"
                );
            }
        }
    }

    #[test]
    fn a_charge_in_empty_air_has_nothing_to_remove() {
        let grid = ChunkGrid::new(VOXEL);
        assert!(effective_radius(&grid, Point3::origin(), &BlastConfig::default()).is_none());
    }
}
