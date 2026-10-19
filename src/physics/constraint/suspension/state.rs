//! A suspension's persistent state: its declaration, this frame's hit, and
//! what it reads out.

use nalgebra::Point3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::declaration::SuspensionDesc;
use super::hit::SuspensionHit;
use super::rows::SPRING_ROW;

/// A suspension's state, as gameplay and visuals read it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SuspensionReadout {
    /// What the cast hit this frame, if anything.
    pub hit: Option<SuspensionHit>,
    /// Whether the hit is within reach (rest length plus slack): a support.
    pub grounded: bool,
    /// The spring's current length, from the anchor to the cast sphere's
    /// centre, in metres. The reach when nothing was hit.
    pub length: f32,
    /// How far the spring is pressed in from its rest length, in metres;
    /// negative in the slack band, zero with nothing hit.
    pub compression: f32,
    /// The force the spring pushed with on the last substep, in newtons.
    pub load: f32,
    /// `load` as a fraction of `max_force`.
    pub load_fraction: f32,
    /// Energy stored in the spring, `½·k·x²`, in joules: the spring's own
    /// line in an energy account, not energy created.
    pub stored_energy: f32,
}

/// A suspension: its declaration and what the world last found under it.
#[derive(Debug, Clone)]
pub struct Suspension {
    /// What the owner declared.
    desc: SuspensionDesc,
    /// What the cast hit this frame.
    hit: Option<SuspensionHit>,
    /// Substep length of the frame the hit was cast for, in seconds: what the
    /// spring's impulse per substep is divided by to give its force.
    substep_dt: f32,
}

impl Suspension {
    pub fn new(desc: SuspensionDesc) -> Self {
        Self {
            desc,
            hit: None,
            substep_dt: 0.0,
        }
    }

    pub fn desc(&self) -> &SuspensionDesc {
        &self.desc
    }

    pub fn body(&self) -> RigidBodyHandle {
        self.desc.body
    }

    pub fn hit(&self) -> Option<&SuspensionHit> {
        self.hit.as_ref()
    }

    /// The body the suspension pushes against, if it is one that moves.
    pub fn partner(&self) -> Option<RigidBodyHandle> {
        self.hit.and_then(|hit| hit.partner)
    }

    /// The farthest the spring reaches and still holds the body as a support.
    pub fn reach(&self) -> f32 {
        self.desc.rest_length + self.desc.slack
    }

    /// Whether this frame's hit is within reach.
    pub fn is_grounded(&self) -> bool {
        self.hit.is_some_and(|hit| hit.length <= self.reach())
    }

    /// The anchor in world space for the body at its current pose.
    pub fn anchor_in_world(&self, body: &RigidBody) -> Point3<f32> {
        body.position() + body.rotation() * self.desc.anchor
    }

    /// Store this frame's cast. Returns whether the warm impulses must be
    /// zeroed: nothing was hit, or what was hit is not what was hit before.
    pub(crate) fn land(&mut self, hit: Option<SuspensionHit>, substep_dt: f32) -> bool {
        let reset = match (&self.hit, &hit) {
            (Some(before), Some(now)) => before.partner != now.partner,
            _ => true,
        };
        self.hit = hit;
        self.substep_dt = substep_dt;
        reset
    }

    /// Drop the hit if it is on `partner`, which is gone.
    pub(crate) fn forget_partner(&mut self, partner: RigidBodyHandle) {
        if self.partner() == Some(partner) {
            self.hit = None;
        }
    }

    /// The spring's impulse over the last substep, from the constraint's
    /// row impulses.
    pub fn spring_impulse(impulses: &[f32]) -> f32 {
        impulses.get(SPRING_ROW).copied().unwrap_or(0.0)
    }

    /// The suspension as it stands, for the body at its current pose and
    /// the spring's last impulse.
    pub fn readout(&self, body: &RigidBody, spring_impulse: f32) -> SuspensionReadout {
        let Some(hit) = self.hit else {
            return SuspensionReadout {
                hit: None,
                grounded: false,
                length: self.reach(),
                compression: 0.0,
                load: 0.0,
                load_fraction: 0.0,
                stored_energy: 0.0,
            };
        };
        let moved = (self.anchor_in_world(body) - hit.anchor).dot(&hit.axis);
        let length = hit.length + moved;
        let compression = self.desc.rest_length - length;
        let load = if self.substep_dt > 0.0 {
            spring_impulse.max(0.0) / self.substep_dt
        } else {
            0.0
        };
        let stiffness = self.desc.spring.stiffness(body.mass());
        SuspensionReadout {
            hit: Some(hit),
            grounded: self.is_grounded(),
            length,
            compression,
            load,
            load_fraction: if self.desc.max_force > 0.0 {
                load / self.desc.max_force
            } else {
                0.0
            },
            stored_energy: 0.5 * stiffness * compression.max(0.0).powi(2),
        }
    }
}
