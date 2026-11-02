/// What the hands count as an edge, when they catch one, and how a body
/// hangs, climbs and leaves. Feel numbers, tuned in play;
/// `docs/LEDGE_GRABS.md` says what each is for.
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
    /// Lowest lip a mantle climbs, in metres above the capsule's centre:
    /// below the chest, where a grab from the air would not catch. Lower
    /// still is a step the traversal plan hops.
    pub mantle_low: f32,
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

    /// Fastest a body may rise past a lip and still catch it, in m/s,
    /// relative to the lip.
    pub catch_rise: f32,
    /// Fastest a body may move relative to a lip and still catch it, in m/s.
    pub catch_speed: f32,
    /// Most the heading may turn from facing into the face at a catch, in
    /// degrees.
    pub catch_facing_degrees: f32,
    /// Seconds after letting go of a ledge before the hands catch again.
    pub regrab_lock: f32,

    /// Seconds after a catch that the stick is ignored.
    pub settle: f32,
    /// Seconds the stick must be held towards or away from the face before
    /// a pull-up or a drop commits.
    pub commit_hold: f32,
    /// Half the cone of stick directions that count as towards or away from
    /// the face, in degrees; anything between shimmies.
    pub lean_cone_degrees: f32,
    /// Shortest stick push that counts at all, as a fraction of a full push.
    pub stick_dead_zone: f32,
    /// Speed along the edge while shimmying, in m/s.
    pub shimmy_speed: f32,
    /// How far along the edge the hands look for more of it before a shimmy
    /// moves, in metres.
    pub shimmy_probe: f32,

    /// How far above the capsule's centre the hands hold, hanging, in metres:
    /// the shoulders and the arms near straight.
    pub hang_reach: f32,
    /// Gap between the hanging capsule and the face, in metres.
    pub face_gap: f32,
    /// How many times the hanger's mass an edge's body must weigh for the
    /// grip's joint to anchor to it; a lighter one takes the hanger's weight
    /// instead (`Grip`).
    pub anchor_mass_ratio: f32,
    /// Seconds a pull-up from a hang takes.
    pub pull_up_time: f32,
    /// Seconds a mantle takes.
    pub mantle_time: f32,
    /// How far over the lip the capsule's bottom clears it as it climbs, in
    /// metres.
    pub lip_clearance: f32,

    /// How far ahead along the arc, in seconds, the hands look for an edge
    /// to reach for.
    pub reach_ahead_time: f32,
    /// Spacing of the looks along the arc, in seconds.
    pub reach_ahead_step: f32,
    /// How far below the capsule's centre the feet look for a face to brace
    /// on while hanging, in metres.
    pub brace_drop: f32,

    /// Speed out from the wall a back-jump leaves at, in m/s.
    pub back_jump_speed: f32,
    /// Seconds a back-jump taken with the stick neutral or towards the wall
    /// keeps straight out, unsteered.
    pub back_jump_lock: f32,
}

impl Default for LedgeConfig {
    fn default() -> Self {
        Self {
            catch_low: 0.2,
            catch_high: 0.65,
            shoulder: 0.25,
            mantle_low: 0.05,
            reach_ahead: 0.25,
            hand_offset: 0.15,
            grip_slope_degrees: 40.0,
            grip_depth: 0.04,
            hand_clearance: 0.05,
            hand_step: 0.04,
            face_slope_degrees: 60.0,
            stand_slope_degrees: 45.0,
            stand_depth: 0.35,
            catch_rise: 1.0,
            catch_speed: 12.0,
            catch_facing_degrees: 60.0,
            regrab_lock: 0.3,
            settle: 0.3,
            commit_hold: 0.2,
            lean_cone_degrees: 45.0,
            stick_dead_zone: 0.3,
            shimmy_speed: 1.0,
            shimmy_probe: 0.1,
            hang_reach: 0.53,
            face_gap: 0.01,
            anchor_mass_ratio: 3.0,
            pull_up_time: 0.6,
            mantle_time: 0.3,
            lip_clearance: 0.03,
            reach_ahead_time: 0.3,
            reach_ahead_step: 0.05,
            brace_drop: 0.35,
            back_jump_speed: 4.0,
            back_jump_lock: 0.6,
        }
    }
}
