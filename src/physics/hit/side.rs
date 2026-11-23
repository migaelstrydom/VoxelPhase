//! A hit as one of its sides met it: which way it was pushed, what struck
//! it, and whether it was backed.

use nalgebra::Vector3;

use super::event::Hit;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};

/// One side of a hit: its `body_a` or its `body_b`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    A,
    B,
}

impl Side {
    /// The side across the contact.
    pub fn other(self) -> Self {
        match self {
            Side::A => Side::B,
            Side::B => Side::A,
        }
    }

    /// Its place in a per-side `[a, b]` array ([`Hit::side_masses`],
    /// [`Hit::backed`]).
    pub fn index(self) -> usize {
        match self {
            Side::A => 0,
            Side::B => 1,
        }
    }

    /// Which way a contact whose normal points towards `body_b` pushes this
    /// side.
    pub fn push(self, normal: &Vector3<f32>) -> Vector3<f32> {
        match self {
            Side::A => -normal,
            Side::B => *normal,
        }
    }
}

/// A hit as one of its sides met it.
#[derive(Debug, Clone, Copy)]
pub struct HitSide<'a> {
    /// The hit.
    pub(super) hit: &'a Hit,
    /// Which side of it this is.
    pub(super) side: Side,
}

impl Hit {
    /// Which side of the hit `body` was, `None` if neither.
    fn side_of(&self, body: RigidBodyHandle) -> Option<Side> {
        if self.body_b == body {
            Some(Side::B)
        } else if self.body_a == Some(body) {
            Some(Side::A)
        } else {
            None
        }
    }

    /// The hit as `body` met it, `None` if it was not in it.
    pub fn side(&self, body: RigidBodyHandle) -> Option<HitSide<'_>> {
        self.side_of(body).map(|side| HitSide { hit: self, side })
    }

    /// Whether `body` is either side of the hit.
    pub fn involves(&self, body: RigidBodyHandle) -> bool {
        self.side_of(body).is_some()
    }
}

impl<'a> HitSide<'a> {
    /// The hit itself.
    pub fn hit(&self) -> &'a Hit {
        self.hit
    }

    /// Its collider; `None` for static geometry, or where the contact came
    /// from a sweep that keeps no collider.
    pub fn collider(&self) -> Option<ColliderHandle> {
        match self.side {
            Side::A => self.hit.collider_a,
            Side::B => self.hit.collider_b,
        }
    }

    /// What struck it: the other side's collider, if it has one.
    pub fn struck_by(&self) -> Option<ColliderHandle> {
        HitSide {
            side: self.side.other(),
            ..*self
        }
        .collider()
    }

    /// Which way the hit pushed it, unit length.
    pub fn push(&self) -> Vector3<f32> {
        self.side.push(&self.hit.normal)
    }

    /// Whether it was backed against the blow ([`Hit::backed`]).
    pub fn backed(&self) -> bool {
        self.hit.backed[self.side.index()]
    }
}
