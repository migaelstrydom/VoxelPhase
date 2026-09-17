//! The component that makes a compound body a sheet of something brittle.

use specs::{Component, VecStorage};

use super::crazing::CrazeRule;
use super::depth::CrazeDepths;
use super::fatigue::{FatigueRule, FatigueTracker};
use super::jolt::JoltTracker;
use super::pane::SheetFrame;
use super::remnant::RemnantRule;
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
    /// How many times over a cell may crack before a hit takes it out whole
    /// instead of breaking it down further. One means the shard sizes of the
    /// first break are the sizes that fall; each extra level multiplies the
    /// children one sheet can end up carrying.
    pub max_craze_depth: u32,
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
    /// When what is left of the sheet stops being a sheet and comes free as
    /// shards. Without it a fixed pane chipped away at ends as a shard
    /// hanging where the pane was, immovable because the body behind it is
    /// static.
    pub remnant: RemnantRule,
    /// Glass area the sheet was spawned with, m², filled in on the first
    /// frame it is seen whole. The denominator of
    /// [`RemnantRule::min_fraction`].
    pub whole_glass_area: Option<f32>,
    /// Collider and joint counts at the last time the remnant was judged, so
    /// it is judged again exactly when the glass has changed — by this
    /// system's crazing, by a shard severed under a hit, or by a joint the
    /// fracture system broke on its own.
    pub last_census: (usize, usize),
    /// Contact spikes per child, read on this component's own baseline so
    /// crazing can run before the fracture system judges the same frame.
    pub contact_load: ContactLoadTracker,
    /// Held-load damage per child.
    pub damage: FatigueTracker,
    /// How deep in the crazing each child was born.
    pub depths: CrazeDepths,
    /// The body's motion across frames, so a jarred frame loads its glass.
    pub jolt: JoltTracker,
}

/// Cells crack once, by default: a whole pane breaks into a web, and a hit
/// on one of those shards takes the shard out rather than crazing it again.
const DEFAULT_MAX_CRAZE_DEPTH: u32 = 1;

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
            max_craze_depth: DEFAULT_MAX_CRAZE_DEPTH,
            joint_threshold,
            fatigue: None,
            fatigue_hole_radius: 0.35,
            material,
            frame_grip: joint_threshold,
            remnant: RemnantRule::default(),
            whole_glass_area: None,
            last_census: (0, 0),
            contact_load: ContactLoadTracker::new(1),
            damage: FatigueTracker::new(1),
            depths: CrazeDepths::new(1),
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

    /// A sheet whose shards may themselves crack, `depth` levels deep.
    /// Every level costs children, and children are the whole expense of a
    /// broken sheet; two is already a lot of glass.
    pub fn crazing_at_most(mut self, depth: u32) -> Self {
        self.max_craze_depth = depth.max(1);
        self
    }

    /// Whether a hit on `child` cracks it into a web, or takes it out whole.
    pub fn may_craze(&self, child: usize) -> bool {
        self.depths.of(child) < self.max_craze_depth
    }

    /// A sheet whose remnant lets go once it is smaller than `area` m²,
    /// whatever share of the pane that is.
    pub fn letting_go_below(mut self, area: f32) -> Self {
        self.remnant.min_area = area;
        self
    }

    /// A sheet that gives the rest up once less than `fraction` of its glass
    /// is left.
    pub fn letting_go_under(mut self, fraction: f32) -> Self {
        self.remnant.min_fraction = fraction;
        self
    }

    /// A sheet that gives way under a held load.
    pub fn wearing_out_under(mut self, fatigue: FatigueRule, hole_radius: f32) -> Self {
        self.fatigue = Some(fatigue);
        self.fatigue_hole_radius = hole_radius;
        self
    }
}
