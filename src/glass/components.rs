//! The component that makes a compound body a sheet of something brittle.

use specs::{Component, VecStorage};

use super::crazing::CrazeRule;
use super::fatigue::{FatigueRule, FatigueTracker};
use super::jolt::JoltTracker;
use super::pane::SheetFrame;
use crate::fracture::{ContactLoadTracker, Deadband};
use crate::rendering::material::MaterialId;

/// A flat compound body whose children crack into webs where they are hit.
///
/// Sits beside a [`CompoundFracture`](crate::fracture::CompoundFracture):
/// this component decides how the sheet *divides*, that one decides when the
/// divisions *let go*. A sheet starts life as one box child and no joints;
/// every hit that clears the craze threshold replaces the child under it
/// with a web of convex shards, joined to each other and to their old
/// neighbours, and the fracture system takes it from there.
#[derive(Component)]
#[storage(VecStorage)]
pub struct BrittleSheet {
    /// The sheet's axes in the body's frame, and its thickness.
    pub frame: SheetFrame,
    /// When a cell cracks and into what.
    pub crazing: CrazeRule,
    /// Blast impulse, in N·s, that breaks a joint between two shards.
    /// Copied onto every joint the crazing creates.
    pub joint_threshold: f32,
    /// Whether standing on the sheet wears it out, and how fast.
    pub fatigue: Option<FatigueRule>,
    /// Radius, in metres, of the hole a fatigued cell opens around the load
    /// that broke it. Sized to what was standing there: a person's foot
    /// needs a hole wider than the player's capsule to fall through.
    pub fatigue_hole_radius: f32,
    /// What every shard is drawn with, and what marks a child as glass: a
    /// child of the body wearing any other material is frame. The frame is
    /// never crazed, never counted as remnant, and holds the shards that
    /// touch it at `frame_grip` rather than `joint_threshold`.
    pub material: MaterialId,
    /// Impulse, in N·s, that pulls a shard out of the frame it touches.
    pub frame_grip: f32,
    /// Area, in m², below which what is left of the sheet stops being a
    /// sheet and comes free as one more shard. Without it a fixed pane
    /// chipped away at ends as a crumb hanging where the pane was.
    pub min_remnant_area: f32,
    /// Contact spikes per child, read on this component's own baseline so
    /// crazing can run before the fracture system judges the same frame.
    pub contact_load: ContactLoadTracker,
    /// Held-load damage per child.
    pub damage: FatigueTracker,
    /// The body's motion across frames, so a jarred frame loads its glass.
    pub jolt: JoltTracker,
}

impl BrittleSheet {
    /// A sheet that is one whole pane, not yet cracked anywhere.
    pub fn whole(
        frame: SheetFrame,
        crazing: CrazeRule,
        joint_threshold: f32,
        material: MaterialId,
    ) -> Self {
        Self {
            frame,
            crazing,
            joint_threshold,
            fatigue: None,
            fatigue_hole_radius: 0.35,
            material,
            frame_grip: joint_threshold,
            min_remnant_area: 0.03,
            contact_load: ContactLoadTracker::new(1),
            damage: FatigueTracker::new(1),
            jolt: JoltTracker::default(),
        }
    }

    /// A sheet set in a frame of some other material, which holds each shard
    /// touching it at `grip` N·s.
    pub fn held_by_frame(mut self, grip: f32) -> Self {
        self.frame_grip = grip;
        self
    }

    /// Judge the glass's contact spikes against each child's own weight, as
    /// a sheet with a heavy frame must.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.contact_load = std::mem::take(&mut self.contact_load).with_deadband(deadband);
        self
    }

    /// Whether the child wearing `material` is glass rather than frame.
    pub fn is_glass(&self, material: MaterialId) -> bool {
        material == self.material
    }

    /// A sheet whose remnant lets go once it is smaller than `area` m².
    pub fn letting_go_below(mut self, area: f32) -> Self {
        self.min_remnant_area = area;
        self
    }

    /// A sheet that gives way under a held load.
    pub fn wearing_out_under(mut self, fatigue: FatigueRule, hole_radius: f32) -> Self {
        self.fatigue = Some(fatigue);
        self.fatigue_hole_radius = hole_radius;
        self
    }
}
