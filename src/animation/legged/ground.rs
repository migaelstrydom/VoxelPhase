//! What the foot probes found this frame.

use nalgebra::{Point3, Vector3};

use crate::sensing::ContactCandidate;

use super::super::foot_placer::FootSide;

/// Probe tags used by legged locomotion. One per foot, so a contact
/// candidate can be attributed to the leg that asked for it.
pub mod probe_tags {
    pub const FOOT_LEFT: u32 = 0;
    pub const FOOT_RIGHT: u32 = 1;
}

/// The tag a given foot's probe carries.
pub fn probe_tag(side: FootSide) -> u32 {
    match side {
        FootSide::Left => probe_tags::FOOT_LEFT,
        FootSide::Right => probe_tags::FOOT_RIGHT,
    }
}

/// The latest ground reading under one foot.
///
/// `None` is a real answer, not a missing one: a foot swinging out over a
/// ledge has no ground under its landing target, and the placer treats
/// that target as illegal rather than guessing a height for it.
#[derive(Clone, Copy, Debug, Default)]
pub struct FootGround {
    /// Where the probe hit, if it hit.
    pub contact: Option<Point3<f32>>,
    /// The surface normal there, if it hit.
    pub normal: Option<Vector3<f32>>,
}

impl FootGround {
    /// The surface normal, defaulting to world up when there is no hit.
    pub fn normal_or_up(&self) -> Vector3<f32> {
        self.normal.unwrap_or_else(Vector3::y)
    }
}

/// Pick the nearest contact carrying each foot's tag.
pub fn resolve(contacts: &[ContactCandidate]) -> (FootGround, FootGround) {
    let mut left: Option<&ContactCandidate> = None;
    let mut right: Option<&ContactCandidate> = None;

    for contact in contacts {
        let best = match contact.tag {
            probe_tags::FOOT_LEFT => &mut left,
            probe_tags::FOOT_RIGHT => &mut right,
            _ => continue,
        };
        if best.map_or(true, |b| contact.distance < b.distance) {
            *best = Some(contact);
        }
    }

    (from_contact(left), from_contact(right))
}

fn from_contact(contact: Option<&ContactCandidate>) -> FootGround {
    match contact {
        Some(contact) => FootGround {
            contact: Some(contact.point),
            normal: Some(contact.normal),
        },
        None => FootGround::default(),
    }
}
