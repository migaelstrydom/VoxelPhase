use std::time::Instant;

use nalgebra::{Point3, Vector3};
use traversal_planner::plan::{BodyState, Contact, Limb, Plan};
use traversal_planner::section::{Footing, Section, SectionQuery, StripLayout, StripQuery};
use traversal_planner::{
    GaitParams, PathVec, PlanRequest, Planned, Planner, PlannerSettings, Refusal,
};

use crate::collision::AABB;

use crate::physics::{PhysicsWorld, RigidBodyHandle, StaticGeometry};

use super::legs::{footfall_id, Legs, Planted};
use super::path_frame::PathFrame;
use super::physics_caster::{strip_reach, PhysicsCaster};
use super::section_cache::SectionCache;
use super::strip_scenery::{SceneryView, StripScenery};
use super::wall_slide::WallSlide;

/// How far the heading may turn from the laid path before it is laid
/// afresh, in degrees: the footfalls lie along the laid path, so this is how
/// far they may point from where the body runs.
const TURN_LIMIT_DEGREES: f32 = 2.0;
/// How far a path laid afresh may turn from the last one and the last plan
/// still be kept, carried onto it, in degrees; past it the plan is made anew.
const CARRY_LIMIT_DEGREES: f32 = 10.0;
/// The same while the path slides along a wall, in degrees: the wall, not
/// the stick, sets its way, and it must not run into the wall.
const SLIDE_TURN_LIMIT_DEGREES: f32 = 1.0;
/// How far the body may be shoved off the laid path's line, the heading
/// still along it, before the path is laid afresh, in metres: rough ground
/// and rubble jostle the body this much and back, and each relay moves the
/// plan's kept footfalls off the footholds it chose between the stones.
const OFF_LINE_LIMIT: f32 = 0.2;
/// How far the body may drift off the laid path's line along a heading
/// turned from it, however little, before the path is laid afresh, in
/// metres: the footfalls are planned about the line, so this is how far
/// beside the body they land. A heading nudged by less than
/// [`TURN_LIMIT_DEGREES`] drifts the body off the line by 1.7 cm a metre for
/// each degree.
const DRIFT_LIMIT: f32 = 0.02;
/// Least turn of the heading from the laid path that drifts the body off
/// it, in degrees: less is the same heading, read back.
const STEERED_DEGREES: f32 = 0.01;
/// How far the body may climb or drop from where the path was laid before
/// it is laid afresh, in metres: the casts start and end relative to there.
const HEIGHT_LIMIT: f32 = 1.5;
/// How far ahead the strip is sampled on the frame a path is laid with no
/// plan to carry onto it, in metres: the next footfall or two. The reach
/// then grows by [`REACH_GROWTH`] a frame, so laying a path costs a few
/// frames a little each rather than one frame a lot.
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
/// How far from a planted foot the strip's foothold it stands on may be, in
/// metres.
const PLANTED_TOLERANCE: f32 = 0.05;

/// What a frame's planning is given.
pub struct PlanningInput<'a> {
    pub world: &'a PhysicsWorld,
    pub scenery: &'a dyn StaticGeometry,
    /// The player's own body, which the casts pass through.
    pub exclude: &'a [RigidBodyHandle],
    /// Bodies that carry what stands on them, read as fixed ground where
    /// they stand ([`super::carriers`]).
    pub carriers: &'a [RigidBodyHandle],
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
    /// The feet the legs have down, which the plan starts from.
    pub legs: Legs,
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
/// and starting each plan from the feet the legs have down.
pub struct TraversalPlanning {
    planner: Planner,
    layout: StripLayout,
    cache: SectionCache,
    /// The static geometry around the strip, fetched once for many frames
    /// and paths.
    scenery: Option<StripScenery>,
    /// The height the path was laid at, which the casts reach around.
    laid_at: f32,
    /// How far above and below there they reach ([`strip_reach`]).
    reach: (f32, f32),
    /// Whether the path was laid along a wall ([`WallSlide`]).
    slides: bool,
    /// How far ahead of the body the strip is sampled: short when a path is
    /// laid with no plan to carry, growing frame by frame to the layout's
    /// reach.
    sampled_ahead: f32,
    previous: Option<Plan>,
    /// The foot that lands next when the legs do not say: the last plan's.
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
            laid_at: 0.0,
            reach,
            slides: false,
            previous: None,
            sampled_ahead: 0.0,
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
        carriers: &'a [RigidBodyHandle],
    ) -> PhysicsCaster<'a> {
        PhysicsCaster {
            world,
            scenery,
            frame: *self.cache.frame().expect("a laid path"),
            exclude,
            carriers,
            top: self.laid_at + self.reach.0,
            bottom: self.laid_at - self.reach.1,
        }
    }

    /// Forgets the last plan, the path it was laid along and the casts and
    /// scenery kept along it: the next plan lays its path and starts afresh
    /// from the feet. What changed in the world meanwhile was not watched,
    /// so nothing kept is trusted.
    pub fn forget(&mut self) {
        self.previous = None;
        self.cache = SectionCache::default();
        self.scenery = None;
    }

    /// Plans this frame; `None`, and the planning forgotten, while the body
    /// stands still.
    pub fn update(&mut self, input: &PlanningInput) -> Option<PlanFrame> {
        let speed = input.velocity.dot(&input.heading);
        if input.target_speed < STANDING_SPEED && speed < STANDING_SPEED {
            self.forget();
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

        self.cache.forget_what_moved(
            input.world,
            input.exclude,
            input.carriers,
            input.terrain_changes,
        );
        self.fetch_scenery_if_needed(input, body.s);
        let view = SceneryView {
            strip: self.scenery.as_ref().expect("fetched"),
            source: input.scenery,
        };
        let caster = self.caster(input.world, &view, input.exclude, input.carriers);
        self.sampled_ahead = match (laid_afresh, &self.previous) {
            // A path laid afresh carrying the last plan is sampled to its
            // full reach at once: the plan goes on along it, and planned on
            // with less in sight it may refuse what it crossed a frame ago.
            (true, Some(_)) => self.layout.ahead,
            (true, None) => LAID_REACH,
            (false, _) => (self.sampled_ahead + REACH_GROWTH).min(self.layout.ahead),
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
        let legs = input.legs;
        let request = PlanRequest {
            body,
            lead: legs.lead.unwrap_or(self.lead),
            target_speed: input.target_speed.max(STANDING_SPEED),
            planted: legs
                .planted
                .map(|planted| planted_contact(&query, &frame, planted)),
            aim: legs.planner_aim(),
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
        self.lead = later
            .footfalls()
            .next()
            .map_or(request.lead, |(_, next)| next.limb);
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

    /// Fetches the static geometry around the strip when what was fetched
    /// does not cover it or the terrain changed: some way further ahead than
    /// needed, so it is fetched again only every few metres.
    fn fetch_scenery_if_needed(&mut self, input: &PlanningInput, s: f32) {
        let from = s - self.layout.behind;
        let ahead = s + self.layout.ahead + self.params().body.radius;
        let covered = self
            .scenery
            .as_ref()
            .is_some_and(|scenery| scenery.covers(&self.strip_region(from, ahead)));
        if covered && input.terrain_changes.is_empty() {
            return;
        }
        let region = self.strip_region(from, ahead + SCENERY_AHEAD);
        self.scenery = Some(StripScenery::fetch(input.scenery, region));
    }

    /// The world box around the strip from `from` to `until` along the laid
    /// path, as high and low as the casts reach, with [`SCENERY_MARGIN`]
    /// about it.
    fn strip_region(&self, from: f32, until: f32) -> AABB {
        let frame = self.cache.frame().expect("laid");
        let half_width = self.layout.lanes.iter().fold(0.0f32, |w, d| w.max(d.abs()))
            + self.params().body.radius
            + SCENERY_MARGIN;
        let (low, high) = (
            self.laid_at - self.reach.1 - SCENERY_MARGIN,
            self.laid_at + self.reach.0 + SCENERY_MARGIN,
        );
        let corners = [
            (from - SCENERY_MARGIN, -half_width),
            (from - SCENERY_MARGIN, half_width),
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
        AABB::new(min, max)
    }

    /// Lays the path the way the body will move ([`WallSlide`]) when there is
    /// none, that way has turned, or the body has left its line or its
    /// height; the last plan is carried onto the new path if it turned
    /// within [`CARRY_LIMIT_DEGREES`], and dropped if more. Whether it did.
    fn lay_path_if_needed(&mut self, input: &PlanningInput) -> bool {
        let line = WallSlide {
            world: input.world,
            scenery: input.scenery,
            exclude: input.exclude,
            body: &self.planner.params.body,
        }
        .path(input.position, input.heading);
        let laid = self.cache.frame().copied();
        let fresh = laid.is_none_or(|laid| {
            strayed(
                &laid,
                self.laid_at,
                &line.frame,
                line.slides,
                input.position,
            )
        });
        if fresh {
            self.previous = laid.zip(self.previous.take()).and_then(|(old, plan)| {
                let turned = old.forward().angle(&line.frame.forward()).to_degrees();
                (turned <= CARRY_LIMIT_DEGREES).then(|| old.carry(&plan, &line.frame))
            });
            self.cache.lay_path(line.frame);
            self.laid_at = input.position.y;
            self.slides = line.slides;
        }
        fresh
    }
}

/// Whether the body at `position` has left the path `laid` at height
/// `laid_at` for the way `heading` it would be laid now, sliding along a wall
/// if `slides`: turned past the turn limit, shoved past [`OFF_LINE_LIMIT`]
/// off its line, drifted past [`DRIFT_LIMIT`] along a heading turned from it,
/// or climbed or dropped past [`HEIGHT_LIMIT`].
fn strayed(
    laid: &PathFrame,
    laid_at: f32,
    heading: &PathFrame,
    slides: bool,
    position: Point3<f32>,
) -> bool {
    let off = laid.path_point(position).d.abs();
    let limit = if slides {
        SLIDE_TURN_LIMIT_DEGREES
    } else {
        TURN_LIMIT_DEGREES
    };
    let turned = laid.forward().angle(&heading.forward()).to_degrees();
    turned > limit
        || off > OFF_LINE_LIMIT
        || (off > DRIFT_LIMIT && turned > STEERED_DEGREES)
        || (position.y - laid_at).abs() > HEIGHT_LIMIT
}

/// The contact a `planted` foot makes: where the legs have it, standing on
/// the strip's foothold there, or on fixed level ground where the strip has
/// none, and on the plan's footfall the legs say it stands on.
fn planted_contact(query: &StripQuery, frame: &PathFrame, planted: Planted) -> Contact {
    let (limb, footfall) = (planted.limb, planted.footfall.map(footfall_id));
    let at = frame.path_point(planted.at);
    match query.foothold_at(&at, PLANTED_TOLERANCE) {
        Some(foothold) => Contact {
            limb,
            footfall,
            at: PathVec::new(at.s, at.d, foothold.at.z),
            normal: foothold.normal,
            footing: foothold.footing,
        },
        None => Contact {
            limb,
            footfall,
            at,
            normal: PathVec::UP,
            footing: Footing::Fixed,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A path from the origin along +x, and the body `ahead` along it and
    /// `off` to its left, heading `degrees` left of it.
    fn strays(ahead: f32, off: f32, degrees: f32) -> bool {
        let laid = PathFrame::new(Point3::origin(), 90.0);
        let position = laid.world_point(PathVec::new(ahead, off, 0.0));
        let heading = PathFrame::new(position, 90.0 - degrees);
        strayed(&laid, 0.0, &heading, false, position)
    }

    #[test]
    fn a_nudged_heading_relays_once_the_body_drifts_a_few_centimetres() {
        assert!(!strays(5.0, 0.0, 1.0));
        assert!(!strays(1.0, 0.015, 1.0));
        assert!(strays(1.5, 0.025, 1.0));
        assert!(strays(0.0, 0.0, 2.5));
    }

    #[test]
    fn a_body_shoved_off_its_line_along_the_same_heading_keeps_it_longer() {
        assert!(!strays(1.0, 0.1, 0.0));
        assert!(strays(1.0, 0.25, 0.0));
    }
}
