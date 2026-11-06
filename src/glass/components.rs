//! The component that makes a compound body a sheet of something brittle.

use specs::{Component, VecStorage};

use super::crazing::CrazeRule;
use super::fatigue::{FatigueRule, FatigueTracker};
use super::held::HeldEdges;
use super::jolt::JoltTracker;
use super::pane::SheetFrame;
use super::remnant::RemnantRule;
use crate::fracture::{BreakDepths, ContactLoadTracker, Cuttable, Deadband};
use crate::physics::{ColliderHandle, ConstraintHandle};

/// A flat compound body whose children crack into webs where they are hit.
///
/// Sits beside a [`CompoundFracture`](crate::fracture::CompoundFracture):
/// this component decides how the sheet *divides*, that one decides when the
/// divisions *let go*. A sheet starts life as one pane part and no bonds;
/// every hit that clears the craze threshold replaces the part under it
/// with a web of convex shards, bonded to each other and inheriting the old
/// part's bonds and welds, and the fracture system takes it from there.
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
    /// Blast loading, in N·s/m², that breaks a bond between two shards.
    /// Copied onto every bond the crazing creates; contact between shards
    /// is judged on the compound's default toughness.
    pub joint_threshold: f32,
    /// Whether standing on the sheet wears it out, and how fast.
    pub fatigue: Option<FatigueRule>,
    /// Radius, in metres, of the hole a fatigued cell opens around the load
    /// that broke it. Sized to what was standing there: a person's foot
    /// needs a hole wider than the player's capsule to fall through.
    pub fatigue_hole_radius: f32,
    /// The glass colliders and how a shard of them is made: a collider of
    /// the body it does not cover is frame. The frame is never crazed, never
    /// counted as remnant, and holds the shards that touch it at the grip it
    /// was bonded with.
    pub glass: Cuttable,
    /// Which stretch of edge each of the sheet's welds to the world holds,
    /// for a sheet held along its edges.
    pub held: HeldEdges,
    /// When what is left of the sheet stops being a sheet and comes free as
    /// shards. Without it a held pane chipped away at ends as a shard
    /// hanging where the pane was, held up by the welds behind it.
    pub remnant: RemnantRule,
    /// Glass area the sheet was spawned with, m², filled in on the first
    /// frame it is seen whole. The denominator of
    /// [`RemnantRule::min_fraction`].
    pub whole_glass_area: Option<f32>,
    /// Collider and bond counts at the last time the remnant was judged, so
    /// it is judged again exactly when the glass has changed — by this
    /// system's crazing, by a shard severed under a hit, or by a bond the
    /// fracture system broke on its own.
    pub last_census: (usize, usize),
    /// Contact spikes per child, read on this component's own baseline so
    /// crazing can run before the fracture system judges the same frame.
    pub contact_load: ContactLoadTracker,
    /// Held-load damage per child.
    pub damage: FatigueTracker,
    /// How deep in the crazing each child was born.
    pub depths: BreakDepths,
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
        glass: Cuttable,
    ) -> Self {
        Self {
            frame,
            crazing,
            max_craze_depth: DEFAULT_MAX_CRAZE_DEPTH,
            joint_threshold,
            fatigue: None,
            fatigue_hole_radius: 0.35,
            glass,
            held: HeldEdges::default(),
            remnant: RemnantRule::default(),
            whole_glass_area: None,
            last_census: (0, 0),
            contact_load: ContactLoadTracker::default(),
            damage: FatigueTracker::default(),
            depths: BreakDepths::default(),
            jolt: JoltTracker::default(),
        }
    }

    /// Judge the glass's contact spikes against each child's own weight, as
    /// a sheet with a heavy frame must.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.contact_load = std::mem::take(&mut self.contact_load).with_deadband(deadband);
        self
    }

    /// Whether `child` is glass rather than frame.
    pub fn is_glass(&self, child: ColliderHandle) -> bool {
        self.glass.covers(child)
    }

    /// A sheet whose shards may themselves crack, `depth` levels deep.
    /// Every level costs children, and children are the whole expense of a
    /// broken sheet; two is already a lot of glass.
    pub fn crazing_at_most(mut self, depth: u32) -> Self {
        self.max_craze_depth = depth.max(1);
        self
    }

    /// Whether a hit on `child` cracks it into a web, or takes it out whole.
    pub fn may_craze(&self, child: ColliderHandle) -> bool {
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

    /// Whether a piece split off with `children` is still sheet enough to
    /// need this: more than one cell, or a cell that may craze.
    pub fn worth_sharing(&self, children: &[ColliderHandle]) -> bool {
        let glass = children.iter().filter(|c| self.is_glass(**c)).count();
        children.len() > 1 || (glass == 1 && children.iter().any(|c| self.may_craze(*c)))
    }

    /// The sheet of a piece split off with `children`, held by `welds`: the
    /// same rules, the records of its own cells and welds, and its census
    /// as it now stands, so it is judged again only once it changes.
    pub fn share(
        &self,
        children: &[ColliderHandle],
        welds: &[ConstraintHandle],
        census: (usize, usize),
    ) -> Self {
        Self {
            frame: self.frame,
            crazing: self.crazing,
            max_craze_depth: self.max_craze_depth,
            joint_threshold: self.joint_threshold,
            fatigue: self.fatigue,
            fatigue_hole_radius: self.fatigue_hole_radius,
            glass: self.glass.share(children),
            held: self.held.share(welds),
            remnant: self.remnant,
            whole_glass_area: self.whole_glass_area,
            last_census: census,
            contact_load: self.contact_load.share(children),
            damage: self.damage.share(children),
            depths: self.depths.share(children),
            jolt: JoltTracker::default(),
        }
    }

    /// Forget the records of every cell but `children` and every weld but
    /// `welds`: what is left once pieces have gone.
    pub fn retain(&mut self, children: &[ColliderHandle], welds: &[ConstraintHandle]) {
        self.glass.retain(children);
        self.held.retain(welds);
        self.contact_load = self.contact_load.share(children);
        self.damage = self.damage.share(children);
        self.depths.retain(children);
    }
}
