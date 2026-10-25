use std::time::Instant;

use nalgebra::{Point3, Vector3};
use traversal_planner::plan::{BodyState, Contact, Limb, Plan};
use traversal_planner::section::{Section, StripLayout, StripQuery};
use traversal_planner::{
    GaitParams, PathVec, PlanRequest, Planned, Planner, PlannerSettings, Refusal,
};

use crate::collision::AABB;

use crate::physics::{PhysicsWorld, RigidBodyHandle, StaticGeometry};

use super::path_frame::PathFrame;
use super::physics_caster::{strip_reach, PhysicsCaster};
use super::section_cache::SectionCache;
use super::strip_scenery::{SceneryView, StripScenery};
use super::wall_slide::WallSlide;

/// How far the heading may turn from the laid path before it is laid
/// afresh, in degrees.
const TURN_LIMIT_DEGREES: f32 = 10.0;
/// The same while the path slides along a wall, in degrees: the wall, not
/// the stick, sets its way, and it must not run into the wall.
const SLIDE_TURN_LIMIT_DEGREES: f32 = 1.0;
/// How far the body may leave the laid path's line before it is laid
/// afresh, in metres.
const OFF_LINE_LIMIT: f32 = 0.2;
/// How far the body may climb or drop from where the path was laid before
/// it is laid afresh, in metres: the casts start and end relative to there.
const HEIGHT_LIMIT: f32 = 1.5;
/// How far ahead the strip is sampled on the frame a path is laid, in
/// metres: the next footfall or two.
const LAID_REACH: f32 = 2.0;
/// How much further it is sampled each frame after, in metres: about 300
/// casts.
const REACH_GROWTH: f32 = 1.0;
/// How far past the strip's reach ahead the fetched geometry goes, in
/// metres, so it is fetched again only every few metres.
const SCENERY_AHEAD: f32 = 5.0;
/// Margin around the strip the fetched geometry covers, in metres.
const SCENERY_MARGIN: f32 = 0.5;
/// Slowest speed along the heading a plan is made for, in m/s: slower, the
/// body is standing.
const STANDING_SPEED: f32 = 0.3;

/// What a frame's planning is given.
pub struct PlanningInput<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    /// The player's own body, which the casts pass through.
    pub exclude: &'a [RigidBodyHandle],
    /// World boxes of terrain that changed since last frame.
    pub terrain_changes: &'a [(Point3<f32>, Point3<f32>)],
    /// The capsule's bottom-sphere centre.
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    /// Where the player is heading: horizontal, unit length.
    pub heading: Vector3<f32>,
    /// The speed the player asks for, in m/s.
    pub target_speed: f32,
    /// Seconds to the next frame.
    pub dt: f32,
}

/// What a frame's planning cost.
#[derive(Clone, Copy, Debug, Default)]
pub struct PlanningCost {
    /// Casts made this frame; the rest were kept from earlier frames.
    pub casts: usize,
    /// Laying the path, if it was, and sampling the strip.
    pub section_seconds: f64,
    pub plan_seconds: f64,
    /// Whether the path was laid afresh this frame, every cast with it.
    pub laid_afresh: bool,
}

/// A frame's plan, the strip it was made on, and what it cost.
pub struct PlanFrame {
    pub frame: PathFrame,
    /// Whether the path was laid along a wall the heading meets
    /// ([`WallSlide`]) rather than along the heading.
    pub slides: bool,
    pub section: Section,
    pub body: BodyState,
    pub outcome: Result<Planned, Refusal>,
    pub cost: PlanningCost,
}

impl PlanFrame {
    /// The plan, or a refusal's plan as far as it got.
    pub fn plan(&self) -> &Plan {
        match &self.outcome {
            Ok(planned) => &planned.plan,
            Err(refusal) => &refusal.partial,
        }
    }
}

/// Plans the body's way frame after frame along a path laid ahead of it:
/// keeping the strip's casts and the last plan from one frame to the next,
/// and the feet the last plan had down.
pub struct TraversalPlanning {
    planner: Planner,
    layout: StripLayout,
    cache: SectionCache,
    /// The static geometry around the strip, fetched once for many frames.
    scenery: Option<StripScenery>,
    /// How far along the path the fetched geometry reaches.
    scenery_until: f32,
    /// The height the path was laid at, which the casts reach around.
    laid_at: f32,
    /// How far above and below there they reach ([`strip_reach`]).
    reach: (f32, f32),
    /// Whether the path was laid along a wall ([`WallSlide`]).
    slides: bool,
    /// How far ahead of the body the strip is sampled: short when a path is
    /// laid, growing frame by frame to the layout's reach, so laying a path
    /// costs a few frames a little each rather than one frame a lot.
    sampled_ahead: f32,
    previous: Option<Plan>,
    planted: Option<Contact>,
    lead: Limb,
}

impl TraversalPlanning {
    pub fn new(params: GaitParams, settings: PlannerSettings, layout: StripLayout) -> Self {
        let reach = strip_reach(&layout, &params);
        Self {
            planner: Planner::new(params, settings),
            layout,
            cache: SectionCache::default(),
            scenery: None,
            scenery_until: f32::MIN,
            laid_at: 0.0,
            reach,
            slides: false,
            previous: None,
            sampled_ahead: 0.0,
            planted: None,
            lead: Limb::LeftFoot,
        }
    }

    pub fn params(&self) -> &GaitParams {
        &self.planner.params
    }

    /// The caster the strip is sampled with along the laid path, reaching
    /// [`strip_reach`] above and below where it was laid.
    pub fn caster<'a>(
        &self,
        world: &'a PhysicsWorld,
        scenery: &'a dyn StaticGeometry,
        exclude: &'a [RigidBodyHandle],
    ) -> PhysicsCaster<'a> {
        PhysicsCaster {
            world,
            scenery,
            frame: *self.cache.frame().expect("a laid path"),
            exclude,
            top: self.laid_at + self.reach.0,
            bottom: self.laid_at - self.reach.1,
        }
    }

    /// Plans this frame; `None` while the body stands still.
    pub fn update(&mut self, input: &PlanningInput) -> Option<PlanFrame> {
        let speed = input.velocity.dot(&input.heading);
        if input.target_speed < STANDING_SPEED && speed < STANDING_SPEED {
            self.previous = None;
            return None;
        }
        let started = Instant::now();
        let laid_afresh = self.lay_path_if_needed(input);
        let frame = *self.cache.frame().expect("laid");
        let at = frame.path_point(input.position);
        let body = BodyState {
            s: at.s,
            z: at.z,
            speed: input.velocity.dot(&frame.forward()).max(STANDING_SPEED),
            climb: input.velocity.y,
        };

        self.cache
            .forget_what_moved(input.world, input.exclude, input.terrain_changes);
        self.fetch_scenery_if_needed(input, body.s);
        let view = SceneryView {
            strip: self.scenery.as_ref().expect("fetched"),
            source: input.scenery,
        };
        let caster = self.caster(input.world, &view, input.exclude);
        self.sampled_ahead = if laid_afresh {
            LAID_REACH
        } else {
            (self.sampled_ahead + REACH_GROWTH).min(self.layout.ahead)
        };
        let layout = StripLayout {
            ahead: self.sampled_ahead,
            ..self.layout.clone()
        };
        let (section, casts) =
            self.cache
                .sample(&caster, &layout, &self.planner.params, body.s, body.z);
        let sampled = Instant::now();

        let query = StripQuery::new(&section, &self.planner.params);
        let request = PlanRequest {
            body,
            lead: self.lead,
            target_speed: input.target_speed.max(STANDING_SPEED),
            planted: self.planted,
            previous: self.previous.as_ref(),
            // A step short of the sampled reach, so the last footfall has
            // ground, but at least half of it.
            reach_limit: (self.sampled_ahead < self.layout.ahead).then(|| {
                let step = self.planner.params.stride.max_length;
                body.s + (self.sampled_ahead - step).max(0.5 * self.sampled_ahead)
            }),
        };
        let outcome = self.planner.plan(&query, &request);
        let planned = Instant::now();

        let plan = match &outcome {
            Ok(planned) => &planned.plan,
            Err(refusal) => &refusal.partial,
        };
        let later = plan.after(input.dt);
        self.planted = carried_stance(&outcome, input.dt);
        self.lead = match self.planted {
            Some(stance) => stance.limb.other(),
            None => later
                .footfalls()
                .next()
                .map_or(self.lead, |(_, next)| next.limb),
        };
        self.previous = Some(later);

        Some(PlanFrame {
            frame,
            slides: self.slides,
            section,
            body,
            outcome,
            cost: PlanningCost {
                casts,
                section_seconds: (sampled - started).as_secs_f64(),
                plan_seconds: (planned - sampled).as_secs_f64(),
                laid_afresh,
            },
        })
    }

    /// Fetches the static geometry around the strip when there is none for
    /// this path, the strip is nearing the end of it, or the terrain changed.
    fn fetch_scenery_if_needed(&mut self, input: &PlanningInput, s: f32) {
        let ahead = s + self.layout.ahead + self.params().body.radius;
        if self.scenery.is_some() && ahead <= self.scenery_until && input.terrain_changes.is_empty()
        {
            return;
        }
        let frame = *self.cache.frame().expect("laid");
        let from = s - self.layout.behind - SCENERY_MARGIN;
        let until = ahead + SCENERY_AHEAD;
        let half_width = self.layout.lanes.iter().fold(0.0f32, |w, d| w.max(d.abs()))
            + self.params().body.radius
            + SCENERY_MARGIN;
        let (low, high) = (
            self.laid_at - self.reach.1 - SCENERY_MARGIN,
            self.laid_at + self.reach.0 + SCENERY_MARGIN,
        );
        let corners = [
            (from, -half_width),
            (from, half_width),
            (until, -half_width),
            (until, half_width),
        ]
        .map(|(s, d)| frame.world_point(PathVec::new(s, d, 0.0)));
        let min = corners
            .iter()
            .fold(Point3::new(f32::MAX, low, f32::MAX), |m, c| {
                Point3::new(m.x.min(c.x), low, m.z.min(c.z))
            });
        let max = corners
            .iter()
            .fold(Point3::new(f32::MIN, high, f32::MIN), |m, c| {
                Point3::new(m.x.max(c.x), high, m.z.max(c.z))
            });
        self.scenery = Some(StripScenery::fetch(input.scenery, AABB::new(min, max)));
        self.scenery_until = until;
    }

    /// Lays the path the way the body will move ([`WallSlide`]) when there is
    /// none, that way has turned, or the body has left its line; the last
    /// plan goes with the old path. Whether it did.
    fn lay_path_if_needed(&mut self, input: &PlanningInput) -> bool {
        let line = WallSlide {
            world: input.world,
            scenery: input.scenery,
            exclude: input.exclude,
            body: &self.planner.params.body,
        }
        .path(input.position, input.heading);
        let fresh = match self.cache.frame() {
            None => true,
            Some(frame) => {
                let at = frame.path_point(input.position);
                let limit = if line.slides {
                    SLIDE_TURN_LIMIT_DEGREES
                } else {
                    TURN_LIMIT_DEGREES
                };
                let turned = frame.forward().angle(&line.frame.forward()).to_degrees() > limit;
                turned
                    || at.d.abs() > OFF_LINE_LIMIT
                    || (input.position.y - self.laid_at).abs() > HEIGHT_LIMIT
            }
        };
        if fresh {
            self.cache.lay_path(line.frame);
            self.scenery = None;
            self.laid_at = input.position.y;
            self.slides = line.slides;
            self.previous = None;
            self.planted = None;
        }
        fresh
    }
}

/// The foot `outcome`'s plan stands on `dt` seconds in, for the next frame's
/// plan to start from; none under a refusal, whose partial plan holds its
/// first foot to where it runs out however far the body has gone past it,
/// and would be refused again from it. The next plan then stands on a foot
/// under the body.
fn carried_stance(outcome: &Result<Planned, Refusal>, dt: f32) -> Option<Contact> {
    let planned = outcome.as_ref().ok()?;
    planned.plan.stance_at(dt).copied()
}

#[cfg(test)]
mod tests {
    use smallvec::smallvec;
    use traversal_planner::plan::Phase;
    use traversal_planner::section::Footing;
    use traversal_planner::Motion;

    use super::*;

    /// A plan standing on a foot through the next frame carries it on; the
    /// same plan refused carries none.
    #[test]
    fn only_a_plan_that_gets_through_carries_its_stance_on() {
        let foot = Contact {
            limb: Limb::LeftFoot,
            at: PathVec::new(0.0, 0.1, 0.0),
            normal: PathVec::UP,
            footing: Footing::Fixed,
        };
        let plan = Plan {
            phases: vec![Phase {
                start: 0.0,
                duration: 1.0,
                contacts: smallvec![foot],
                motion: Motion::Grounded {
                    from: 0.0,
                    to: 5.0,
                    start_speed: 5.0,
                    end_speed: 5.0,
                },
            }],
            ..Plan::default()
        };
        let planned = Ok(Planned {
            plan: plan.clone(),
            cost: 0.0,
            diverged: false,
        });
        assert_eq!(carried_stance(&planned, 0.1), Some(foot));
        let refused = Err(Refusal {
            partial: plan,
            diverged: false,
        });
        assert_eq!(carried_stance(&refused, 0.1), None);
    }
}
