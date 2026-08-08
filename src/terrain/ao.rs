//! Baked ambient occlusion from a blurred occupancy field.
//!
//! See `docs/BAKED_AO_DESIGN.md`. In short: build a coarse scalar field holding
//! *how solid the neighbourhood is*, read it slightly off the surface along the
//! normal, and calibrate so open flat ground reads as unoccluded.
//!
//! ```text
//!   VoxelSource ──fill_block──▶ VoxelBlock (own lattice, coarser, wider halo)
//!                                    │
//!                                    │  occupancy ramp across the surface
//!                                    ▼
//!                              separable blur (x, then y, then z)
//!                                    │
//!                                    ▼
//!            occlusion(p, n) ──▶ trilinear read at p + n·offset, calibrated
//! ```
//!
//! Three properties this leans on, each of which is load-bearing:
//!
//! - **Occupancy is a ramp across the surface, not a binary test.** See
//!   [`occupancy`]. A binary test quantises the surface to the occlusion
//!   lattice, which is coarser than the mesh, and flat ground then ripples by
//!   more than a third of the field's whole range depending on where it happens
//!   to fall between samples. Reading the density as the clamped signed
//!   distance it already is takes that ripple to ~4%.
//! - **The lattice is anchored to global indices**, exactly as `SampleLattice`
//!   already anchors the marching-cubes grid. Two blocks evaluating occlusion
//!   for the same world position therefore read the same lattice points and get
//!   bit-identical answers, so seams cannot exist rather than being blended out.
//! - **No read is ever clamped.** The halo is sized from the parameters that
//!   determine reach, and an out-of-range read is a `debug_assert`, not a
//!   `clamp`. Clamping is what produced the visible seams on the abandoned
//!   `sdf-ao` branch, and it did so silently.

use nalgebra::{Point3, Vector3};

use super::voxel_block::{SampleLattice, VoxelBlock, VoxelSource};

/// How solid a lattice sample's neighbourhood is, from its density.
///
/// `csg::union_solid` writes `density = clamp(-sdf / voxel_size, -1, 1)`, so
/// density is the signed distance to the surface *in voxels*, saturating one
/// voxel out. Reading it as such gives the surface's position to sub-voxel
/// precision, which is what keeps flat ground from rippling — see the module
/// docs and `the_field_barely_ripples_as_a_plane_moves_between_samples`.
///
/// This deliberately trusts density's magnitude, which
/// `docs/BAKED_AO_DESIGN.md` originally argued against. The argument was
/// against assuming an *arbitrary* scale; the scale here is neither arbitrary
/// nor assumed — it is written down in `csg.rs`, and marching cubes already
/// trusts exactly the same linear model when it places a vertex along an edge.
/// AO agreeing with the mesh about where the surface is matters more than
/// either agreeing with the ideal surface.
///
/// The destruction path is the weak case: `apply_damage` returns a whole air
/// voxel at `-1.0` rather than a partial distance, so a fresh crater's surface
/// carries no sub-voxel offset. The mesh has the same limitation from the same
/// cause, so AO and geometry still agree — which is the property that matters.
fn occupancy(density: f32) -> f32 {
    (0.5 + 0.5 * density).clamp(0.0, 1.0)
}

/// Sub-cell positions of the flat plane averaged over when deriving the
/// baseline. The blurred field is discrete, so the value above flat ground
/// depends slightly on where the ground falls between lattice samples; the
/// baseline is the mean over that phase, and the residual is the ripple.
const BASELINE_PHASE_STEPS: usize = 32;

/// Sub-cell positions averaged over on each of the two axes when deriving the
/// fully-occluded reference. Fewer than [`BASELINE_PHASE_STEPS`] because the
/// cost is the square of it and the reference is one number, not a field.
const CORNER_PHASE_STEPS: usize = 8;

/// How the occlusion field is built and read.
///
/// **Reach is in metres, deliberately.** It was in grid cells, and that made
/// the effect's size a property of the level's voxel size rather than of the
/// world: the visual bench meshes at 0.25 m voxels and `test_arena` declares
/// 1.0, so identical settings produced a 1 m kernel in the bench and a 4 m one
/// in the game. The bench showed almost nothing while the game bled darkness
/// several metres across open ground and read as broad streaks. How far one
/// surface shades another is a fact about the world, not about how finely it
/// happens to be sampled.
///
/// Everything resolution-dependent is derived from `reach` and the voxel size
/// in [`OcclusionSettings::resolve`], so there is one place where the two can
/// disagree and it has the voxel size in hand.
#[derive(Debug, Clone, Copy)]
pub struct OcclusionSettings {
    /// Radius of the occlusion kernel, **in metres**. Together with `strength`,
    /// the aesthetic dial: how far away a surface can be and still shade this
    /// one.
    pub reach: f32,
    /// How far off the surface the field is read, in **voxels**.
    ///
    /// In voxels rather than metres because, unlike `reach`, this exists to
    /// escape the surface's own discretisation: the occupancy ramp spans a
    /// voxel either side of the surface, and reading inside that band reports
    /// the half-solid value no matter what the geometry is. Its job is to get
    /// clear of the band, so the band's units are the right ones.
    pub normal_offset: f32,
    /// How dark a floor-meets-wall corner goes. See [`fully_occluded_excess`].
    pub strength: f32,
}

impl Default for OcclusionSettings {
    fn default() -> Self {
        Self {
            reach: 1.0,
            normal_offset: 1.0,
            strength: 0.7,
        }
    }
}

/// Grid spacing to prefer, as a multiple of the voxel size, when the reach is
/// wide enough to still resolve on it.
///
/// Occlusion is low-frequency, so it does not need the mesh's resolution, and
/// the cube of the saving is real. Not more than 2: density saturates one voxel
/// out, so a cell wider than two voxels loses the sub-voxel surface position
/// that keeps flat ground from mottling — measured, at divisor 4 the phase
/// ripple is four times worse than at 2.
const PREFERRED_DIVISOR: usize = 2;

/// Fewest kernel cells the blur is allowed, before dropping to a finer grid.
///
/// A one-cell kernel barely smooths the surface's quantisation: the phase
/// ripple goes from 0.040 to 0.170. When the reach is too small to buy two
/// cells at the preferred spacing, the answer is a finer grid, not a narrower
/// kernel.
const MIN_BLUR_CELLS: usize = 2;

/// [`OcclusionSettings`] resolved against a particular voxel size.
///
/// Nothing outside this struct converts between metres, voxels and cells.
#[derive(Debug, Clone, Copy)]
struct Resolution {
    /// Grid spacing as a multiple of the voxel size.
    divisor: usize,
    /// Distance between adjacent grid samples, in world units.
    spacing: f32,
    /// Blur kernel reach, in grid cells.
    blur_radius: usize,
    /// Read offset along the normal, in grid cells.
    offset_cells: f32,
}

impl OcclusionSettings {
    fn resolve(&self, voxel_size: f32) -> Resolution {
        // Take the coarse grid only if the reach still spans enough of its
        // cells to be a kernel rather than a step function.
        let coarse_would_resolve =
            self.reach >= MIN_BLUR_CELLS as f32 * PREFERRED_DIVISOR as f32 * voxel_size;
        let divisor = if coarse_would_resolve {
            PREFERRED_DIVISOR
        } else {
            1
        };

        let spacing = divisor as f32 * voxel_size;
        let blur_radius = (self.reach / spacing).round().max(1.0) as usize;

        // Capped at half the kernel, so the read cannot step so far off the
        // surface that it lands outside the neighbourhood being measured.
        let offset_cells = (self.normal_offset / divisor as f32).min(blur_radius as f32 / 2.0);

        Resolution {
            divisor,
            spacing,
            blur_radius,
            offset_cells,
        }
    }
}

impl Resolution {
    /// Samples of halo needed on each side of the owned region.
    ///
    /// Three things stack: the blur consumes `blur_radius` samples at every
    /// boundary, a read may land `offset_cells` outside the owned region, and
    /// trilinear interpolation needs the sample beyond the one it lands in.
    fn halo(&self) -> usize {
        self.blur_radius + self.offset_cells.ceil() as usize + 1
    }
}

/// A blurred occupancy field covering one block, plus the calibration that
/// turns a reading into an occlusion factor.
pub struct OcclusionGrid {
    lattice: SampleLattice,
    /// Blurred occupancy, indexed `(x * dims[1] + y) * dims[2] + z` to match
    /// [`VoxelBlock`]'s layout.
    field: Vec<f32>,
    /// Samples at each boundary whose blurred value read outside the grid and
    /// is therefore meaningless. Reads must stay inside this inset.
    invalid_margin: usize,
    /// Read offset along the normal, in world units.
    normal_offset: f32,
    strength: f32,
    /// Blurred occupancy above an infinite flat half-space. Subtracting it is
    /// what makes open ground read as unoccluded instead of as uniform grey.
    baseline: f32,
    /// How far above [`Self::baseline`] a reading has to sit to count as fully
    /// occluded. See [`fully_occluded_excess`].
    full_excess: f32,
}

impl OcclusionGrid {
    /// Bake the field for a block of `cells` marching-cubes cells whose first
    /// cell corner is at sample index `first_sample`, matching
    /// `MeshOctree::generate_block`'s framing of the same region.
    ///
    /// The coarse lattice is anchored by dividing the global sample index, so
    /// both must be divisible by `resolution_divisor` for neighbouring blocks
    /// to land on one shared lattice.
    pub fn build<S: VoxelSource>(
        lattice_origin: Point3<f32>,
        first_sample: [i32; 3],
        cells: usize,
        voxel_size: f32,
        source: &S,
        settings: &OcclusionSettings,
    ) -> Self {
        let resolved = settings.resolve(voxel_size);
        let divisor = resolved.divisor;
        assert!(
            cells.is_multiple_of(divisor),
            "block of {cells} cells does not divide by the occlusion divisor {divisor}"
        );
        assert!(
            first_sample.iter().all(|i| i % divisor as i32 == 0),
            "first sample {first_sample:?} is off the occlusion lattice (divisor {divisor})"
        );

        let halo = resolved.halo();
        let spacing = resolved.spacing;
        // The owned cells span `cells` voxels, which is `cells / divisor`
        // coarse cells, and therefore one more sample than that.
        let owned = cells / divisor + 1;
        let dims = owned + 2 * halo;
        let base = first_sample.map(|i| i / divisor as i32 - halo as i32);

        let lattice = SampleLattice::new(lattice_origin, base, spacing, [dims; 3]);
        let mut block = VoxelBlock::air(lattice);
        source.fill_block(&mut block);

        let kernel = gaussian_kernel(resolved.blur_radius);
        let field = blurred_occupancy(&block, &kernel);
        let baseline = flat_baseline(&kernel, resolved.offset_cells, divisor);

        Self {
            lattice,
            field,
            invalid_margin: resolved.blur_radius,
            normal_offset: resolved.offset_cells * resolved.spacing,
            strength: settings.strength,
            baseline,
            full_excess: fully_occluded_excess(&kernel, resolved.offset_cells, divisor) - baseline,
        }
    }

    /// Occlusion factor for a surface point: `1.0` is fully open, and a fully
    /// enclosed point reaches `1.0 - strength`.
    pub fn occlusion(&self, position: Point3<f32>, normal: Vector3<f32>) -> f32 {
        let occupancy = self.sample(position + normal * self.normal_offset);
        let excess = (occupancy - self.baseline) / self.full_excess;
        1.0 - self.strength * excess.clamp(0.0, 1.0)
    }

    /// Blurred occupancy at a world position, trilinearly interpolated.
    fn sample(&self, position: Point3<f32>) -> f32 {
        let dims = self.lattice.dims();
        let mut cell = [0usize; 3];
        let mut frac = [0f32; 3];

        for axis in 0..3 {
            let local =
                (position[axis] - self.lattice.axis_position(axis, 0)) / self.lattice.spacing();
            let floor = local.floor();
            debug_assert!(
                floor >= self.invalid_margin as f32
                    && floor + 1.0 < (dims[axis] - self.invalid_margin) as f32,
                "occlusion read at {position:?} falls outside the valid grid on axis {axis}; \
                 the halo is too small for the settings that produced it"
            );
            let clamped = (floor as isize).clamp(0, dims[axis] as isize - 2) as usize;
            cell[axis] = clamped;
            frac[axis] = (local - clamped as f32).clamp(0.0, 1.0);
        }

        let corner = |dx: usize, dy: usize, dz: usize| {
            self.field[((cell[0] + dx) * dims[1] + cell[1] + dy) * dims[2] + cell[2] + dz]
        };
        let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;

        let y0 = lerp(
            lerp(corner(0, 0, 0), corner(0, 0, 1), frac[2]),
            lerp(corner(0, 1, 0), corner(0, 1, 1), frac[2]),
            frac[1],
        );
        let y1 = lerp(
            lerp(corner(1, 0, 0), corner(1, 0, 1), frac[2]),
            lerp(corner(1, 1, 0), corner(1, 1, 1), frac[2]),
            frac[1],
        );
        lerp(y0, y1, frac[0])
    }

    /// Blurred occupancy above flat ground, for tests and diagnostics.
    pub fn baseline(&self) -> f32 {
        self.baseline
    }
}

/// A normalised Gaussian kernel of `2 * radius + 1` taps.
///
/// Sigma is half the radius, which puts the outermost tap two standard
/// deviations out — far enough that truncating it costs little, near enough
/// that the kernel is not mostly zeros.
fn gaussian_kernel(radius: usize) -> Vec<f32> {
    if radius == 0 {
        return vec![1.0];
    }
    let sigma = radius as f32 / 2.0;
    let mut weights: Vec<f32> = (0..=2 * radius)
        .map(|i| {
            let d = i as f32 - radius as f32;
            (-(d * d) / (2.0 * sigma * sigma)).exp()
        })
        .collect();
    let total: f32 = weights.iter().sum();
    for w in &mut weights {
        *w /= total;
    }
    weights
}

/// Occupancy over the block, convolved with `kernel` on each axis in turn.
///
/// Taps that fall outside the block are clamped to its edge. That only affects
/// samples within the kernel radius of a boundary, which is exactly the region
/// [`OcclusionGrid::invalid_margin`] forbids reading.
fn blurred_occupancy(block: &VoxelBlock, kernel: &[f32]) -> Vec<f32> {
    let dims = block.dims();
    let mut front = Vec::with_capacity(dims[0] * dims[1] * dims[2]);
    for x in 0..dims[0] {
        for y in 0..dims[1] {
            for z in 0..dims[2] {
                front.push(occupancy(block.get(x, y, z).density));
            }
        }
    }

    let mut back = vec![0.0; front.len()];
    for axis in 0..3 {
        blur_axis(&front, &mut back, dims, axis, kernel);
        std::mem::swap(&mut front, &mut back);
    }
    front
}

/// One separable blur pass along `axis`.
fn blur_axis(src: &[f32], dst: &mut [f32], dims: [usize; 3], axis: usize, kernel: &[f32]) {
    let stride = match axis {
        0 => (dims[1] * dims[2]) as isize,
        1 => dims[2] as isize,
        _ => 1,
    };
    let extent = dims[axis] as isize;
    let radius = (kernel.len() / 2) as isize;

    for x in 0..dims[0] {
        for y in 0..dims[1] {
            for z in 0..dims[2] {
                let index = ((x * dims[1] + y) * dims[2] + z) as isize;
                let along = [x, y, z][axis] as isize;
                let mut total = 0.0;
                for (tap, weight) in kernel.iter().enumerate() {
                    let at = (along + tap as isize - radius).clamp(0, extent - 1);
                    total += weight * src[(index + (at - along) * stride) as usize];
                }
                dst[index as usize] = total;
            }
        }
    }
}

/// Blurred occupancy `offset_cells` above an infinite flat half-space.
///
/// Derived from the kernel rather than tuned, so changing the blur radius
/// cannot silently re-tint the world. A separable blur of a field that varies
/// on one axis only is a 1D blur on that axis, so this is a 1D problem.
///
/// The plane's position between lattice samples changes the answer slightly, so
/// the result is the mean over that phase. What is left over is a small ripple
/// across flat ground, which the tests bound.
fn flat_baseline(kernel: &[f32], offset_cells: f32, divisor: usize) -> f32 {
    let radius = (kernel.len() / 2) as isize;
    // Enough samples either side that every tap of every read is a genuine
    // half-space value rather than an edge effect.
    let reach = radius + offset_cells.ceil() as isize + 2;

    let mut total = 0.0;
    for step in 0..BASELINE_PHASE_STEPS {
        let surface = step as f32 / BASELINE_PHASE_STEPS as f32;
        // Positions are in cells, and density measures voxels, so the signed
        // distance from a sample to the plane scales by the divisor.
        let at = |i: isize| occupancy((surface - i as f32) * divisor as f32);
        let blurred = |i: isize| {
            kernel
                .iter()
                .enumerate()
                .map(|(tap, w)| w * at(i + tap as isize - radius))
                .sum::<f32>()
        };

        let read = surface + offset_cells;
        let floor = read.floor();
        debug_assert!(floor + 1.0 <= reach as f32);
        total += blurred(floor as isize)
            + (blurred(floor as isize + 1) - blurred(floor as isize)) * (read - floor);
    }
    total / BASELINE_PHASE_STEPS as f32
}

/// Blurred occupancy at the reference *fully occluded* surface point: the base
/// of a wall meeting a floor, a 90° inside corner.
///
/// This is the upper calibration point, and leaving it out was the reason the
/// first working version of this bake was almost invisible. The obvious
/// normaliser is `1 - baseline` — the range up to a completely solid
/// neighbourhood — but a *surface* point never approaches that. A point on open
/// ground reads about 0.48 here and a hard inside corner about 0.81, so the
/// reachable range is a third of the range being divided by, and every crease
/// in the game came out about 1.6× too faint.
///
/// A 90° corner is a choice, not a derivation, but it is a concrete and common
/// one: it is a floor meeting a wall, and it means `strength` says what it
/// claims — how dark that corner goes. Anything more enclosed clamps to it.
///
/// The configuration is invariant along the corner's own axis, so the taps on
/// that axis sum to one and drop out, leaving a 2D sum. Both in-plane phases are
/// averaged over for the same reason [`flat_baseline`] averages over one.
fn fully_occluded_excess(kernel: &[f32], offset_cells: f32, divisor: usize) -> f32 {
    let radius = (kernel.len() / 2) as isize;
    let phases = CORNER_PHASE_STEPS as f32;
    let mut total = 0.0;

    for across in 0..CORNER_PHASE_STEPS {
        for along in 0..CORNER_PHASE_STEPS {
            // The floor is at height `up` and the wall face at `side`, each
            // offset within the cell it falls in.
            let up = across as f32 / phases;
            let side = along as f32 / phases;
            let read_height = up + offset_cells;

            // Solid below the floor or beyond the wall, so depth is whichever
            // of the two the point is further inside — the union of the pair.
            let depth = |x: f32, y: f32| (up - y).max(side - x);

            let mut sum = 0.0;
            for (tap_x, wx) in kernel.iter().enumerate() {
                for (tap_y, wy) in kernel.iter().enumerate() {
                    let x = side + (tap_x as isize - radius) as f32;
                    let y = read_height.floor() + (tap_y as isize - radius) as f32;
                    let lower = occupancy(depth(x, y) * divisor as f32);
                    let upper = occupancy(depth(x, y + 1.0) * divisor as f32);
                    let t = read_height - read_height.floor();
                    sum += wx * wy * (lower + (upper - lower) * t);
                }
            }
            total += sum;
        }
    }

    total / (phases * phases)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::voxel::{Voxel, VoxelMaterial};

    const VOXEL_SIZE: f32 = 0.25;
    const CELLS: usize = 32;
    /// World-space extent of the block the tests bake, `[0, SPAN]` on each axis.
    const SPAN: f32 = CELLS as f32 * VOXEL_SIZE;

    /// A voxel field defined by depth below the surface, in world units:
    /// positive inside the solid, negative in the air.
    ///
    /// Encoded into density exactly as `csg::union_solid` does, so the tests
    /// exercise the sub-voxel information the real terrain carries rather than
    /// a binary field that would hide the very ripple they exist to bound.
    struct Depth2<F>(F, f32);

    impl<F: Fn(Point3<f32>) -> f32> VoxelSource for Depth2<F> {
        fn fill_block(&self, block: &mut VoxelBlock) {
            let dims = block.dims();
            for x in 0..dims[0] {
                for y in 0..dims[1] {
                    for z in 0..dims[2] {
                        let depth = (self.0)(block.lattice().position(x, y, z));
                        let density = (depth / self.1).clamp(-1.0, 1.0);
                        if density > 0.0 {
                            let mut voxel = Voxel::solid(VoxelMaterial::Rock, 10);
                            voxel.density = density;
                            block.set(x, y, z, voxel);
                        } else {
                            let mut voxel = Voxel::air();
                            voxel.density = density;
                            block.set(x, y, z, voxel);
                        }
                    }
                }
            }
        }
    }

    fn bake<F: Fn(Point3<f32>) -> f32>(depth: F) -> OcclusionGrid {
        bake_with(depth, OcclusionSettings::default())
    }

    fn bake_with<F: Fn(Point3<f32>) -> f32>(
        depth: F,
        settings: OcclusionSettings,
    ) -> OcclusionGrid {
        bake_at(depth, VOXEL_SIZE, CELLS, settings)
    }

    fn bake_at<F: Fn(Point3<f32>) -> f32>(
        depth: F,
        voxel_size: f32,
        cells: usize,
        settings: OcclusionSettings,
    ) -> OcclusionGrid {
        OcclusionGrid::build(
            Point3::origin(),
            [0, 0, 0],
            cells,
            voxel_size,
            &Depth2(depth, voxel_size),
            &settings,
        )
    }

    /// Ground at `height`, as a depth field.
    fn ground(height: f32) -> impl Fn(Point3<f32>) -> f32 {
        move |p| height - p.y
    }

    /// The calibration test. Its failure mode — everything a uniform grey — is
    /// otherwise easy to mistake for working output.
    #[test]
    fn flat_ground_is_unoccluded() {
        // Several heights, including ones that fall between coarse lattice
        // samples, because the baseline is a mean over that phase and the
        // ripple around it is what this bounds.
        for height in [3.5, 3.6, 3.75, 3.9, 4.0] {
            let grid = bake(ground(height));
            for (x, z) in [(2.0, 2.0), (4.0, 5.5), (6.0, 3.0)] {
                let ao = grid.occlusion(Point3::new(x, height, z), Vector3::y());
                assert!(
                    (ao - 1.0).abs() < 0.05,
                    "flat ground at y={height} read {ao}, expected ~1.0"
                );
            }
        }
    }

    /// The bound on the artefact that binary occupancy produced: the occlusion
    /// lattice is coarser than the mesh, so a plane's position between its
    /// samples must not change how bright the plane is. Failure here reads as
    /// soft mottling across open ground at the grid's own spacing.
    #[test]
    fn the_field_barely_ripples_as_a_plane_moves_between_samples() {
        let spacing = OcclusionSettings::default().resolve(VOXEL_SIZE).spacing;
        let readings: Vec<f32> = (0..8)
            .map(|step| {
                let height = 4.0 + step as f32 * spacing / 8.0;
                bake(ground(height)).occlusion(Point3::new(4.0, height, 4.0), Vector3::y())
            })
            .collect();

        let low = readings.iter().cloned().fold(f32::MAX, f32::min);
        let high = readings.iter().cloned().fold(f32::MIN, f32::max);
        assert!(
            high - low < 0.06,
            "flat ground varies by {:.3} over one grid cell of phase: {readings:?}",
            high - low
        );
    }

    #[test]
    fn enclosed_point_reaches_full_strength() {
        let settings = OcclusionSettings::default();
        let grid = bake_with(|_| 10.0, settings);
        let ao = grid.occlusion(Point3::new(4.0, 4.0, 4.0), Vector3::y());
        assert!(
            (ao - (1.0 - settings.strength)).abs() < 1e-4,
            "fully enclosed read {ao}, expected {}",
            1.0 - settings.strength
        );
    }

    /// The reachable end of the scale, and the one that decides whether any of
    /// this is visible. A solid neighbourhood is not something a *surface* point
    /// can have; the base of a wall is, and `strength` is defined against it.
    /// Calibrating against the unreachable end instead left every crease in the
    /// game about 1.6x too faint to see.
    #[test]
    fn a_wall_meeting_a_floor_reaches_most_of_the_strength() {
        let settings = OcclusionSettings::default();
        let grid = bake_with(|p| (4.0 - p.y).max(4.0 - p.x), settings);
        let ao = grid.occlusion(Point3::new(4.02, 4.0, 4.0), Vector3::y());

        let full = 1.0 - settings.strength;
        assert!(
            ao < full + 0.15 * settings.strength,
            "the base of a wall read {ao}, nowhere near the {full} that full strength means"
        );
    }

    #[test]
    fn inside_corner_darkens_toward_the_crease() {
        // Floor at y=4 meeting a wall at x=4, solid in -x. The crease runs
        // along z at (4, 4).
        let grid = bake(|p| (4.0 - p.y).max(4.0 - p.x));

        let floor_ao = |x: f32| grid.occlusion(Point3::new(x, 4.0, 4.0), Vector3::y());
        let readings: Vec<f32> = [7.0, 6.0, 5.0, 4.6, 4.3]
            .iter()
            .map(|&x| floor_ao(x))
            .collect();

        for pair in readings.windows(2) {
            assert!(
                pair[1] < pair[0] + 1e-4,
                "occlusion should not lighten approaching the crease: {readings:?}"
            );
        }
        assert!(
            readings[0] - readings[readings.len() - 1] > 0.1,
            "the crease barely darkened at all: {readings:?}"
        );
    }

    /// Guards the sign. Inverted, this whole feature brightens creases and
    /// darkens the open ground between them.
    #[test]
    fn convex_edge_is_not_darkened() {
        // The outside of the same corner: solid only where both hold.
        let grid = bake(|p| (4.0 - p.y).min(4.0 - p.x));

        for x in [3.9, 3.7, 3.4, 3.0] {
            let ao = grid.occlusion(Point3::new(x, 4.0, 4.0), Vector3::y());
            assert!(
                ao > 0.95,
                "convex ground near an outside edge read {ao} at x={x}"
            );
        }
    }

    /// The halo is sized so no legal read leaves the grid. With debug
    /// assertions armed, a surface hard against the block boundary is where
    /// that would first fail.
    #[test]
    fn no_read_falls_outside_the_grid() {
        let grid = bake(ground(4.0));
        for corner in [0.0, SPAN] {
            for other in [0.0, SPAN] {
                grid.occlusion(Point3::new(corner, 4.0, other), Vector3::y());
                grid.occlusion(Point3::new(corner, 0.0, other), -Vector3::y());
                grid.occlusion(Point3::new(corner, SPAN, other), Vector3::y());
            }
        }
    }

    /// The bug that made the game and the bench disagree: reach used to be
    /// expressed in grid cells, so a level meshed at 1.0 m voxels got a kernel
    /// four times wider than one meshed at 0.25 m. In the bench the effect was
    /// nearly invisible; in the game it bled darkness metres across open ground.
    /// The same world geometry has to shade the same way however finely it is
    /// sampled.
    #[test]
    fn the_bake_does_not_follow_the_voxel_size() {
        let settings = OcclusionSettings::default();
        let corner = |p: Point3<f32>| (4.0 - p.y).max(4.0 - p.x);

        // Same world span, so the two grids cover the same cube of world.
        let fine = bake_at(corner, 0.25, 32, settings);
        let coarse = bake_at(corner, 0.5, 16, settings);

        for distance in [0.3, 0.8, 1.5, 2.5] {
            let at = Point3::new(4.0 + distance, 4.0, 4.0);
            let a = fine.occlusion(at, Vector3::y());
            let b = coarse.occlusion(at, Vector3::y());
            assert!(
                (a - b).abs() < 0.12,
                "{distance} m from the wall: {a} at 0.25 m voxels, {b} at 0.5 m"
            );
        }
    }

    #[test]
    fn rebaking_is_deterministic() {
        let point = Point3::new(4.2, 4.0, 5.1);
        let corner = |p: Point3<f32>| (4.0 - p.y).max(4.0 - p.x);
        let first = bake(corner).occlusion(point, Vector3::y());
        let second = bake(corner).occlusion(point, Vector3::y());
        assert_eq!(first, second);
    }

    /// The baseline exists to be derived, not typed in. If it stops tracking
    /// the kernel, flat ground stops reading as open.
    #[test]
    fn the_baseline_tracks_the_reach() {
        let baseline_of = |reach| {
            let settings = OcclusionSettings {
                reach,
                ..OcclusionSettings::default()
            };
            bake_with(ground(4.0), settings).baseline()
        };
        assert!(baseline_of(0.5) != baseline_of(2.0));
        for reach in [0.5, 1.0, 1.5, 2.0] {
            let b = baseline_of(reach);
            assert!((0.0..1.0).contains(&b), "baseline {b} is not a fraction");
        }
    }

    /// The resolution is derived from the reach and the voxel size, and the
    /// derivation is what keeps the two ends of the trade honest: a coarse grid
    /// where the reach can afford it, a fine one where it cannot.
    #[test]
    fn the_grid_coarsens_only_when_the_reach_can_carry_it() {
        let settings = OcclusionSettings::default();

        // Bench scale: 1.5 m of reach is six 0.25 m voxels, plenty for a
        // two-cell kernel on the coarse grid.
        let fine_world = settings.resolve(0.25);
        assert_eq!(fine_world.divisor, PREFERRED_DIVISOR);
        assert!(fine_world.blur_radius >= MIN_BLUR_CELLS);

        // Game scale: at 1.0 m voxels the coarse grid's cells are 2 m, and 1.5 m
        // of reach would not even fill one. Drop to the voxel grid instead.
        let coarse_world = settings.resolve(1.0);
        assert_eq!(coarse_world.divisor, 1);
        assert!(coarse_world.blur_radius >= 1);

        // Either way the kernel is about as wide as it was asked to be.
        for voxel_size in [0.125, 0.25, 0.5, 1.0, 2.0] {
            let r = settings.resolve(voxel_size);
            let metres = r.blur_radius as f32 * r.spacing;
            assert!(
                (metres - settings.reach).abs() <= voxel_size * r.divisor as f32,
                "{voxel_size} m voxels gave a {metres} m kernel for a {} m reach",
                settings.reach
            );
        }
    }
}
