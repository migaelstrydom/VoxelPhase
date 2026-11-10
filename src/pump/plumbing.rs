//! Plumbing: a pump whose pipe runs up through the parts of a body that can
//! break, so that what is left after a break pumps as its pipe now stands.
//!
//! ```text
//!   sections[2] ── spouts          whole: water leaves by the last
//!   sections[1] ── spouts          section's spouts
//!   sections[0] ── spouts, intake  the top broken off: by the spouts of the
//!                                  last section still joined to the intake
//! ```
//!
//! Everything is kept in a collider's frame, since a collider keeps its
//! handle whichever body it ends up on and a body's origin moves when it
//! breaks.

use nalgebra::Point3;
use specs::{Component, DenseVecStorage};

use super::component::{Spout, WaterPump};
use crate::physics::{ColliderHandle, PhysicsWorld, RigidBodyHandle};

/// One length of the pipe, in one part of the body.
#[derive(Debug, Clone)]
pub struct PipeSection {
    /// The collider the section runs through.
    pub collider: ColliderHandle,
    /// The openings the water leaves by when this is the last section
    /// still joined, in the collider's frame.
    pub spouts: Vec<Spout>,
}

/// A pump's pipework through a breakable body.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Plumbing {
    /// The collider the intake is on, and where in that collider's frame.
    pub intake: (ColliderHandle, Point3<f32>),
    /// The pipe's sections, from the one the intake is on upwards.
    pub sections: Vec<PipeSection>,
    /// The most it moves, m³/s.
    pub rate: f32,
}

impl Plumbing {
    /// The pump `body` carries: none unless the intake is on it, else one
    /// throwing from the last section of pipe still joined to the intake
    /// there.
    pub fn pump_on(&self, world: &PhysicsWorld, body: RigidBodyHandle) -> Option<WaterPump> {
        let on = world.body(body)?.colliders();
        let intake = world.collider(self.intake.0)?;
        if !on.contains(&self.intake.0) {
            return None;
        }
        let last = self
            .sections
            .iter()
            .take_while(|section| on.contains(&section.collider))
            .last()?;
        let frame = world.collider(last.collider)?.offset();
        Some(WaterPump {
            intake: intake.offset() * self.intake.1,
            spouts: last
                .spouts
                .iter()
                .map(|spout| Spout {
                    outlet: frame * spout.outlet,
                    velocity: frame.rotation * spout.velocity,
                })
                .collect(),
            rate: self.rate,
        })
    }
}
