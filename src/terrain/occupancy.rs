//! Which samples a fragment is made of, for shaping it as a body.
//!
//! The fragment's own block, read as a box of flags in the lattice it was cut
//! from, with the frame that takes a (fractional) sample index to the world
//! as it was at the blast. A shape built on the lattice's axes is turned into
//! the world by [`Occupancy::rotation`] alone.

use std::collections::HashSet;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::fragment::Fragment;

/// What a sample of a fragment's block is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sample {
    /// The fragment's own.
    Solid,
    /// Solid but not the fragment's: the ground it broke from, or another
    /// piece. Its shape must not reach into one.
    Obstacle,
    Air,
}

/// A fragment's samples as solid-or-not flags on its block's lattice.
pub struct Occupancy {
    /// Samples per axis.
    dims: [usize; 3],
    /// What each sample is, x-major as the block stores it.
    samples: Vec<Sample>,
    /// Distance between adjacent samples, in metres.
    spacing: f32,
    /// World position of sample `[0, 0, 0]` at the blast.
    origin: Point3<f32>,
    /// Turns the lattice's axes into the world's.
    rotation: UnitQuaternion<f32>,
}

impl Occupancy {
    /// A `dims` block whose samples are `sample`, `spacing` apart, sample
    /// `[0, 0, 0]` at `origin` and the block's axes turned by `rotation`.
    pub fn new(
        dims: [usize; 3],
        spacing: f32,
        origin: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        sample: impl Fn([usize; 3]) -> Sample,
    ) -> Self {
        let mut samples = Vec::with_capacity(dims.iter().product());
        for x in 0..dims[0] {
            for y in 0..dims[1] {
                for z in 0..dims[2] {
                    samples.push(sample([x, y, z]));
                }
            }
        }
        Self {
            dims,
            samples,
            spacing,
            origin,
            rotation,
        }
    }

    /// Samples per axis.
    pub fn dims(&self) -> [usize; 3] {
        self.dims
    }

    /// What the sample at `index` is.
    pub fn sample(&self, [x, y, z]: [usize; 3]) -> Sample {
        self.samples[(x * self.dims[1] + y) * self.dims[2] + z]
    }

    /// Whether the sample at `index` is the fragment's.
    pub fn is_solid(&self, index: [usize; 3]) -> bool {
        self.sample(index) == Sample::Solid
    }

    /// Every air sample with the fragment on both sides of it along some
    /// line of the lattice (an axis, or a face or body diagonal): in a hole,
    /// a gap or a hollow. Air outside a convex surface never is.
    pub fn enclosed(&self) -> Vec<[usize; 3]> {
        let mut enclosed = vec![false; self.samples.len()];
        let lines = (-1..=1i64)
            .flat_map(|x| (-1..=1i64).flat_map(move |y| (-1..=1i64).map(move |z| [x, y, z])))
            .filter(|d| *d > [0, 0, 0]);
        for d in lines {
            let behind = self.solid_behind(d);
            let ahead = self.solid_behind(d.map(|c| -c));
            for (i, flag) in enclosed.iter_mut().enumerate() {
                *flag |= behind[i] && ahead[i];
            }
        }
        let [_, ny, nz] = self.dims;
        enclosed
            .into_iter()
            .enumerate()
            .filter(|&(i, flag)| flag && self.samples[i] == Sample::Air)
            .map(|(i, _)| [i / (ny * nz), i / nz % ny, i % nz])
            .collect()
    }

    /// For every sample, whether one of the fragment's lies somewhere behind
    /// it along `step`: one sweep down each line of the lattice, each sample
    /// taking its answer from the one before it.
    fn solid_behind(&self, step: [i64; 3]) -> Vec<bool> {
        let [nx, ny, nz] = self.dims;
        let flat = |[x, y, z]: [usize; 3]| (x * ny + y) * nz + z;
        // Each axis walked the way the step goes, so the sample behind is
        // always answered first.
        let order = |n: usize, step: i64| -> Vec<usize> {
            if step < 0 {
                (0..n).rev().collect()
            } else {
                (0..n).collect()
            }
        };
        let back_of = |s: [usize; 3]| -> Option<[usize; 3]> {
            let mut back = [0; 3];
            for a in 0..3 {
                let b = s[a] as i64 - step[a];
                if b < 0 || b >= self.dims[a] as i64 {
                    return None;
                }
                back[a] = b as usize;
            }
            Some(back)
        };
        let mut behind = vec![false; self.samples.len()];
        for &x in &order(nx, step[0]) {
            for &y in &order(ny, step[1]) {
                for &z in &order(nz, step[2]) {
                    if let Some(back) = back_of([x, y, z]) {
                        behind[flat([x, y, z])] = self.is_solid(back) || behind[flat(back)];
                    }
                }
            }
        }
        behind
    }

    /// Distance between adjacent samples, in metres.
    pub fn spacing(&self) -> f32 {
        self.spacing
    }

    /// Turns the lattice's axes into the world's.
    pub fn rotation(&self) -> UnitQuaternion<f32> {
        self.rotation
    }

    /// World position at the blast of a point given in sample indices, which
    /// may be fractional.
    pub fn world_position(&self, index: Vector3<f32>) -> Point3<f32> {
        self.origin + self.rotation * (index * self.spacing)
    }

    /// Where a world position at the blast falls, in sample indices.
    pub fn index_position(&self, world: Point3<f32>) -> Vector3<f32> {
        self.rotation
            .inverse_transform_vector(&(world - self.origin))
            / self.spacing
    }
}

impl Fragment {
    /// Which samples of its block it is made of.
    pub fn occupancy(&self) -> Occupancy {
        let voxels = self.voxels();
        let pose = self.pose();
        let obstacles: HashSet<[usize; 3]> = self.obstacles().iter().copied().collect();
        Occupancy::new(
            voxels.dims(),
            voxels.lattice().spacing(),
            pose * voxels.lattice().position(0, 0, 0),
            pose.rotation,
            |index| {
                let [x, y, z] = index;
                if voxels.get(x, y, z).is_solid() {
                    Sample::Solid
                } else if obstacles.contains(&index) {
                    Sample::Obstacle
                } else {
                    Sample::Air
                }
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// Air with the fragment on both sides along some lattice line, found by
    /// walking every line from every sample.
    fn enclosed_by_walking(occupancy: &Occupancy) -> Vec<[usize; 3]> {
        let dims = occupancy.dims();
        let solid_along = |s: [usize; 3], d: [i64; 3]| {
            (1..)
                .map_while(|k| {
                    let at = [0, 1, 2].map(|a| s[a] as i64 + d[a] * k);
                    let inside = (0..3).all(|a| (0..dims[a] as i64).contains(&at[a]));
                    inside.then(|| occupancy.is_solid(at.map(|i| i as usize)))
                })
                .any(|solid| solid)
        };
        (0..dims[0])
            .flat_map(|x| (0..dims[1]).flat_map(move |y| (0..dims[2]).map(move |z| [x, y, z])))
            .filter(|&s| occupancy.sample(s) == Sample::Air)
            .filter(|&s| {
                (-1..=1i64)
                    .flat_map(|x| {
                        (-1..=1i64).flat_map(move |y| (-1..=1i64).map(move |z| [x, y, z]))
                    })
                    .filter(|d| *d > [0, 0, 0])
                    .any(|d| solid_along(s, d) && solid_along(s, d.map(|c| -c)))
            })
            .collect()
    }

    #[test]
    fn enclosed_air_is_air_between_the_fragments_samples() {
        let mut rng = StdRng::seed_from_u64(7);
        for _ in 0..50 {
            let dims = [0; 3].map(|_| rng.gen_range(1..9));
            let picks: Vec<Sample> = (0..dims.iter().product::<usize>())
                .map(|_| match rng.gen_range(0..10) {
                    0..3 => Sample::Solid,
                    3 => Sample::Obstacle,
                    _ => Sample::Air,
                })
                .collect();
            let occupancy = Occupancy::new(
                dims,
                0.5,
                Point3::origin(),
                UnitQuaternion::identity(),
                |[x, y, z]| picks[(x * dims[1] + y) * dims[2] + z],
            );
            assert_eq!(occupancy.enclosed(), enclosed_by_walking(&occupancy));
        }
    }
}
