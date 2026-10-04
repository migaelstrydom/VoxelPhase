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
