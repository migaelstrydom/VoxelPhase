/// What the hands count as an edge, and how they are held to one. Feel
/// numbers, tuned in play; `docs/LEDGE_GRABS.md` says what each is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LedgeConfig {
    /// Lowest lip the hands catch, in metres above the capsule's centre: the
    /// chest. Lower, the body lands on it instead.
    pub catch_low: f32,
    /// Highest lip the hands catch, in metres above the capsule's centre:
    /// the hands' overhead reach and the stretch of a hand that just snags it.
    pub catch_high: f32,
    /// Lips at or below this, in metres above the capsule's centre, are
    /// under the shoulders: caught with the stick pushing in, a mantle.
    pub shoulder: f32,
    /// How far ahead of the capsule's surface the hands look for a lip, in
    /// metres.
    pub reach_ahead: f32,
    /// Half the spacing of the hands along an edge, in metres.
    pub hand_offset: f32,
    /// Steepest top the fingers hold, in degrees from level.
    pub grip_slope_degrees: f32,
    /// Shallowest top a hand holds, in metres behind the lip: a finger hold.
    pub grip_depth: f32,
    /// Open air above the lip a hand needs, in metres.
    pub hand_clearance: f32,
    /// Most the two hands' lips may differ in height, in metres.
    pub hand_step: f32,
    /// Steepest face under a lip that still counts as ground rather than a
    /// face, in degrees from level: a face must be steeper.
    pub face_slope_degrees: f32,
    /// Steepest top the body stands on after a pull-up, in degrees.
    pub stand_slope_degrees: f32,
    /// How far behind the lip the body stands after a pull-up, in metres:
    /// its radius and a margin.
    pub stand_depth: f32,
}

impl Default for LedgeConfig {
    fn default() -> Self {
        Self {
            catch_low: 0.2,
            catch_high: 0.65,
            shoulder: 0.25,
            reach_ahead: 0.25,
            hand_offset: 0.15,
            grip_slope_degrees: 45.0,
            grip_depth: 0.04,
            hand_clearance: 0.05,
            hand_step: 0.04,
            face_slope_degrees: 60.0,
            stand_slope_degrees: 45.0,
            stand_depth: 0.35,
        }
    }
}
