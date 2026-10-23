//! Path coordinates: the frame every section, plan and scene is written in.

use std::ops::{Add, Mul, Neg, Sub};

use serde::{Deserialize, Serialize};

/// A point or direction in path coordinates.
///
/// `s` runs along the predicted path from the body, `d` across it (positive to
/// the body's left), and `z` up. The body is always on the path, at `d = 0`;
/// its feet and the ground they find are not.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PathVec {
    /// Distance along the path, in metres.
    pub s: f32,
    /// Signed offset across the path, positive to the left, in metres.
    pub d: f32,
    /// Height, in metres.
    pub z: f32,
}

impl PathVec {
    pub const UP: Self = Self::new(0.0, 0.0, 1.0);

    pub const fn new(s: f32, d: f32, z: f32) -> Self {
        Self { s, d, z }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.s * other.s + self.d * other.d + self.z * other.z
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    /// This direction at unit length; `UP` for a zero vector.
    pub fn normalized(self) -> Self {
        let length = self.length();
        if length > f32::EPSILON {
            self * (1.0 / length)
        } else {
            Self::UP
        }
    }
}

impl Add for PathVec {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.s + other.s, self.d + other.d, self.z + other.z)
    }
}

impl Sub for PathVec {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.s - other.s, self.d - other.d, self.z - other.z)
    }
}

impl Mul<f32> for PathVec {
    type Output = Self;
    fn mul(self, factor: f32) -> Self {
        Self::new(self.s * factor, self.d * factor, self.z * factor)
    }
}

impl Neg for PathVec {
    type Output = Self;
    fn neg(self) -> Self {
        self * -1.0
    }
}
