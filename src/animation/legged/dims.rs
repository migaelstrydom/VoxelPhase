//! The measurements a gait is planned against.

/// The rig geometry the foot placer needs, and nothing else.
///
/// A gait is planned from four numbers. A humanoid derives them from its
/// full skeleton config (which also knows about shoulders and a head); a
/// four-limbed critter with no arms derives them from its own. Neither
/// config is the placer's business, and the placer should not have to
/// carry one to find a hip width.
///
/// This is a *view* over whichever rig config owns the real values, not a
/// second home for them: build it with `CharacterRigConfig::leg_dims` or
/// the equivalent on another rig, never by copying numbers in by hand.
#[derive(Clone, Copy, Debug)]
pub struct LegRigDims {
    /// Lateral hip separation. Half of it either side of the pelvis is
    /// the neutral stance the placer pulls idle feet back to.
    pub hip_width: f32,
    /// Hip-to-foot distance with the leg straight. The reach budget every
    /// stride is planned and every overstretch release is measured
    /// against.
    pub leg_length: f32,
    /// Pelvis-to-foot-centre distance at rest, which is less than
    /// `leg_length` because a standing leg keeps some bend. Doubles as
    /// the fallback ground height when a foot has no probe hit.
    pub standing_height: f32,
    /// How far a foot probe reaches before it is extended to outreach
    /// whatever it was aimed at. See `LeggedLocomotion::foot_probe`.
    pub probe_length: f32,
}
