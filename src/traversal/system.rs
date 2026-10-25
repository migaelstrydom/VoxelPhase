use std::path::Path;

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};
use traversal_planner::plan::{Limb, Motion};
use traversal_planner::section::{SectionQuery, StripLayout, StripQuery};
use traversal_planner::{GaitParams, PathVec, PlannerSettings};
use winit::keyboard::KeyCode;

use crate::character::{CharacterIntent, CharacterState, LocomotionConfig};
use crate::components::RigidBodyComponent;
use crate::debug::{DebugLines, DebugLog, DebugOverlays};
use crate::input::InputState;
use crate::rendering::Colour;
use crate::systems::{resolve_ground_speed, PhysicsResource};
use crate::terrain::TerrainWorld;
use crate::time::Time;

use super::gait::player_gait;
use super::marker::PlansTraversal;
use super::plan_capture::{scene_of, write_capture};
use super::planning::{PlanFrame, PlanningInput, TraversalPlanning};
use super::tracking::{floor_grade, track, Pace};

/// Key that shows and hides the plan in the world.
const OVERLAY_KEY: KeyCode = KeyCode::F6;
/// Key that writes the current strip, body and plan to a scene file.
const CAPTURE_KEY: KeyCode = KeyCode::F7;
/// Key that turns the plan driving the body on and off.
const TRACK_KEY: KeyCode = KeyCode::F8;
/// The on-screen line saying, while tracking, what sets the body's pace.
const TRACK_LINE: &str = "Traversal track";
/// Where captures are written, relative to the working directory.
const CAPTURE_DIR: &str = "scratch/captures";
/// Spacing along the path at which the body's planned path is drawn, in
/// metres.
const PATH_STEP: f32 = 0.1;
/// Radius footfalls are drawn at, in metres.
const FOOTFALL_RADIUS: f32 = 0.06;
const LEFT: Colour = Colour::new(0.19, 0.51, 0.81, 1.0);
const RIGHT: Colour = Colour::new(0.87, 0.42, 0.13, 1.0);
const PATH: Colour = Colour::new(0.1, 0.13, 0.16, 1.0);
const REFUSED: Colour = Colour::new(0.77, 0.19, 0.19, 1.0);
/// Opacity of what the plan has not committed to.
const TENTATIVE_ALPHA: f32 = 0.45;

/// The player's traversal planning, kept frame to frame. An ECS resource.
#[derive(Default)]
pub struct TraversalPlanState {
    planning: Option<TraversalPlanning>,
    /// This frame's plan, when the player is moving.
    pub last: Option<PlanFrame>,
    /// Whether the plan is drawn in the world.
    pub overlay: bool,
    /// Whether the plan drives the body: off, the plan is only drawn.
    pub tracking: bool,
}

/// Plans the way across the ground ahead of the character marked
/// [`PlansTraversal`] every frame and, on
/// [`OVERLAY_KEY`], draws it; on [`CAPTURE_KEY`], writes it to a scene file
/// the planner's tools load.
///
/// With tracking on ([`TRACK_KEY`]) the plan drives the body: it runs between
/// the player's input and `CharacterControlSystem`, plans from the player's
/// intent, and rewrites that intent to follow the plan ([`track`]): at the
/// plan's speed, steered by the stick (along the path where it slides on a
/// wall), and taking off where it hops. The stick sets a refused plan's
/// pace. A jump the player presses still jumps.
pub struct TraversalPlanSystem;

impl<'a> System<'a> for TraversalPlanSystem {
    type SystemData = (
        ReadStorage<'a, PlansTraversal>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, CharacterIntent>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, LocomotionConfig>,
        Read<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, InputState>,
        Read<'a, Time>,
        Write<'a, TraversalPlanState>,
        Write<'a, DebugOverlays>,
        Write<'a, DebugLog>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            planned,
            bodies,
            mut intents,
            states,
            configs,
            physics,
            terrain,
            input,
            time,
            mut state,
            mut overlays,
            mut log,
            mut lines,
        ): Self::SystemData,
    ) {
        if input.is_key_just_pressed(OVERLAY_KEY) {
            state.overlay = !state.overlay;
        }
        if input.is_key_just_pressed(TRACK_KEY) {
            state.tracking = !state.tracking;
            log::info!(
                "traversal tracking {}",
                if state.tracking { "on" } else { "off" }
            );
        }
        let (overlay, tracking) = (state.overlay, state.tracking);
        let Some(terrain) = terrain else {
            return;
        };
        let Some((body, intent, character, config)) =
            (&planned, &bodies, &mut intents, &states, &configs)
                .join()
                .next()
                .map(|(_, body, intent, character, config)| (body.0, intent, character, config))
        else {
            return;
        };
        let Some(rigid) = physics.world.body(body) else {
            return;
        };

        let planning = state.planning.get_or_insert_with(|| {
            TraversalPlanning::new(
                player_gait(config),
                PlannerSettings::default(),
                StripLayout::default(),
            )
        });
        let spine = planning.params().body.spine();
        let up = rigid.rotation() * Vector3::y();
        let position = rigid.position() - up * (0.5 * spine);
        let velocity = rigid.linear_velocity();
        let push = Vector3::new(intent.direction.x, 0.0, intent.direction.z);
        let heading = push
            .try_normalize(1.0e-3)
            .or_else(|| Vector3::new(velocity.x, 0.0, velocity.z).try_normalize(0.1))
            .unwrap_or_else(Vector3::z);
        let target_speed =
            resolve_ground_speed(intent, config, &character.crouch_lockout) * push.norm().min(1.0);
        let changes: Vec<(Point3<f32>, Point3<f32>)> = terrain
            .changed_regions()
            .iter()
            .map(|region| (region.min, region.max))
            .collect();
        let exclude = [body];
        let frame = planning.update(&PlanningInput {
            world: &physics.world,
            scenery: &*terrain,
            exclude: &exclude,
            terrain_changes: &changes,
            position,
            velocity,
            heading,
            target_speed,
            dt: time.delta_seconds().max(1.0e-3),
        });

        if let Some(frame) = &frame {
            let cost = frame.cost;
            log.add(
                "Traversal/Plan",
                format!(
                    "{} | strip {:.2} ms, {} casts{} | plan {:.2} ms",
                    if frame.outcome.is_ok() {
                        "crosses"
                    } else {
                        "refuses"
                    },
                    cost.section_seconds * 1.0e3,
                    cost.casts,
                    if cost.laid_afresh { ", path laid" } else { "" },
                    cost.plan_seconds * 1.0e3
                ),
            );
            if overlay {
                draw(frame, planning.params(), &mut overlays);
            }
            if tracking && target_speed > 0.0 {
                // The drive keeps only `ground_speed` of what it is asked
                // for up or down a grade: ask for that much more.
                let body_dims = &planning.params().body;
                let grade = floor_grade(&frame.section.body_floor, frame.body.s);
                let gait_speed = resolve_ground_speed(intent, config, &character.crouch_lockout)
                    * body_dims.ground_speed(1.0, grade);
                let tracked = track(
                    frame.plan(),
                    frame.frame.forward(),
                    time.delta_seconds().max(1.0e-3),
                    frame.outcome.is_err(),
                );
                // The plan sets the pace; the stick steers, as the path is
                // relaid only after it turns some way, but along a wall.
                let steer = if frame.slides {
                    frame.frame.forward()
                } else {
                    heading
                };
                let share = match tracked.pace {
                    Pace::Asked => push.norm().min(1.0),
                    Pace::Planned(speed) => (speed / gait_speed).min(1.0),
                };
                intent.direction = steer * share;
                intent.takeoff = tracked.takeoff;
                lines.add(
                    TRACK_LINE,
                    format!(
                        "{}{}",
                        match tracked.pace {
                            Pace::Asked => format!("as asked, {target_speed:.1} m/s"),
                            Pace::Planned(speed) =>
                                format!("plan {speed:.1} of {target_speed:.1} m/s"),
                        },
                        if tracked.takeoff.is_some() {
                            ", hop"
                        } else {
                            ""
                        }
                    ),
                );
            }
            if input.is_key_just_pressed(CAPTURE_KEY) {
                let scene = scene_of(frame, planning.params(), target_speed);
                let note = format!("Captured in game with {CAPTURE_KEY:?}.");
                match write_capture(&scene, Path::new(CAPTURE_DIR), &note) {
                    Ok(path) => log::info!("traversal capture written to {}", path.display()),
                    Err(error) => log::warn!("traversal capture failed: {error}"),
                }
            }
        }
        state.last = frame;
    }
}

/// The plan in the world: the body's path (dashed in flight by its gaps) and
/// every footfall, faded where tentative, red for a refusal.
fn draw(frame: &PlanFrame, params: &GaitParams, overlays: &mut DebugOverlays) {
    let query = StripQuery::new(&frame.section, params);
    let plan = frame.plan();
    let base = if frame.outcome.is_ok() { PATH } else { REFUSED };
    let tentative_from = plan.phases.get(plan.committed).map(|phase| phase.start);
    let fade = |time: f32, colour: Colour| {
        if tentative_from.is_some_and(|from| time > from) {
            Colour::new(colour.r, colour.g, colour.b, TENTATIVE_ALPHA)
        } else {
            colour
        }
    };
    for phase in &plan.phases {
        let (from, to, flying) = match phase.motion {
            Motion::Grounded { from, to, .. } => (from, to, false),
            Motion::Flight { takeoff, .. } => (takeoff.s, phase.end_s(), true),
        };
        let steps = ((to - from) / PATH_STEP).ceil().max(1.0) as usize;
        let point = |step: usize| {
            let s = from + (to - from) * step as f32 / steps as f32;
            let z = phase
                .flight_height(s)
                .or_else(|| query.floor(s).map(|floor| floor.z))?;
            Some(frame.frame.world_point(PathVec::new(s, 0.0, z)))
        };
        let colour = fade(phase.start, base);
        for step in 0..steps {
            if flying && step % 2 == 1 {
                continue;
            }
            if let (Some(a), Some(b)) = (point(step), point(step + 1)) {
                overlays.add_line(a, b, colour);
            }
        }
    }
    for (time, contact) in plan.footfalls() {
        let colour = match contact.limb {
            Limb::LeftFoot => LEFT,
            Limb::RightFoot => RIGHT,
        };
        overlays.add_sphere(
            frame.frame.world_point(contact.at),
            FOOTFALL_RADIUS,
            fade(time, colour),
        );
    }
}
