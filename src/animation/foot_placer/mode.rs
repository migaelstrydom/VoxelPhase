/// What the legs do: the one thing that tells them whether they have ground
/// to answer to. The caller maps its own state to it (a character's
/// locomotion state, a creature's grounding); the legs never decide it.
///
/// There is no mode for a flight. A foot holds a place in the world only
/// while the body is over ground it can stand on; carried through a flight,
/// a place left behind or one not yet reached is a leg drawn pointing at it.
/// A flight short enough to stride through stays `Step`, where a foot with
/// nowhere to land gathers under its hip; any other is `Off`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegMode {
    /// The gait: clock, swings, capture point, a plan's footfalls if it has
    /// any.
    Step,
    /// Nothing under the body the legs answer to: the pose draws the legs,
    /// and may hand the feet back as it draws them
    /// ([`FootPlacer::hold_feet`](super::FootPlacer::hold_feet)).
    Off,
}
