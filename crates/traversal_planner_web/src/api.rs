//! The operations the page calls, each a JSON request and a JSON answer;
//! a failure answers `{"error": "..."}`.

use serde::{Deserialize, Serialize};
use serde_json::json;
use traversal_planner::plan::{BodyState, Limb};
use traversal_planner::scene::TraversalScene;
use traversal_planner::section::{SectionQuery, StripQuery};
use traversal_planner::svg::{render_scene_with_layout, SceneLayout};
use traversal_planner::{Planner, PlannerSettings};

use crate::builtins::SCENES;
use crate::scrub::{scrub, Scrubbed};

/// The operations, by the number the page passes.
#[derive(Clone, Copy)]
pub enum Op {
    /// `[name, ...]` of the built-in scenes.
    Builtins = 0,
    /// The built-in scene named in the request, as JSON.
    Builtin = 1,
    /// The RON scene in the request, as JSON.
    FromRon = 2,
    /// The JSON scene in the request, as RON.
    ToRon = 3,
    /// The planner's default settings, as JSON.
    DefaultSettings = 4,
    /// A [`RenderRequest`] planned and drawn: a [`RenderAnswer`].
    Render = 5,
    /// The JSON scene with its body set down on the floor under it.
    SettleBody = 6,
}

/// What the page asks to see.
#[derive(Deserialize)]
pub struct RenderRequest {
    pub scene: TraversalScene,
    pub settings: PlannerSettings,
    /// Seconds to run the body along its plans first, replanning every
    /// frame; zero plans the scene as it is.
    #[serde(default)]
    pub time: f32,
    /// Whether loose footing gives way under each foot that lands on it
    /// while running.
    #[serde(default)]
    pub perturb: bool,
}

/// A footfall, flattened for the page.
#[derive(Serialize)]
pub struct FootfallView {
    pub limb: Limb,
    pub s: f32,
    pub d: f32,
    pub z: f32,
    /// Seconds into the plan.
    pub time: f32,
    pub committed: bool,
}

/// What the page shows.
#[derive(Serialize)]
pub struct RenderAnswer {
    pub svg: String,
    pub layout: SceneLayout,
    pub crosses: bool,
    /// The plan's cost; `None` for a refusal.
    pub cost: Option<f32>,
    pub footfalls: Vec<FootfallView>,
    /// The body when drawn, after running.
    pub body: BodyState,
    /// What happened while running: commitments dropped, footfalls moved,
    /// stones given way.
    pub events: Vec<String>,
    /// The scene as it is when drawn: the body moved on, and any loose
    /// footing that gave way moved.
    pub scene: TraversalScene,
}

impl Op {
    const ALL: [Op; 7] = [
        Op::Builtins,
        Op::Builtin,
        Op::FromRon,
        Op::ToRon,
        Op::DefaultSettings,
        Op::Render,
        Op::SettleBody,
    ];

    fn from_number(number: u32) -> Option<Op> {
        Self::ALL.into_iter().find(|op| *op as u32 == number)
    }
}

pub fn handle(op: u32, request: &str) -> String {
    let answer = match Op::from_number(op) {
        Some(Op::Builtins) => to_json(&SCENES.iter().map(|(name, _)| *name).collect::<Vec<_>>()),
        Some(Op::Builtin) => builtin(request),
        Some(Op::FromRon) => TraversalScene::from_ron(request)
            .map_err(|error| error.to_string())
            .and_then(|scene| to_json(&scene)),
        Some(Op::ToRon) => serde_json::from_str::<TraversalScene>(request)
            .map_err(|error| error.to_string())
            .and_then(|scene| scene.to_ron().map_err(|error| error.to_string())),
        Some(Op::DefaultSettings) => to_json(&PlannerSettings::default()),
        Some(Op::Render) => serde_json::from_str::<RenderRequest>(request)
            .map_err(|error| error.to_string())
            .and_then(|request| to_json(&render(&request))),
        Some(Op::SettleBody) => serde_json::from_str::<TraversalScene>(request)
            .map_err(|error| error.to_string())
            .and_then(|scene| to_json(&settle_body(scene))),
        None => Err(format!("no operation {op}")),
    };
    answer.unwrap_or_else(|error| json!({ "error": error }).to_string())
}

fn to_json(value: &impl Serialize) -> Result<String, String> {
    serde_json::to_string(value).map_err(|error| error.to_string())
}

fn builtin(name: &str) -> Result<String, String> {
    let (_, ron) = SCENES
        .iter()
        .find(|(builtin, _)| *builtin == name.trim())
        .ok_or_else(|| format!("no built-in scene {name}"))?;
    let scene = TraversalScene::from_ron(ron).map_err(|error| error.to_string())?;
    to_json(&scene)
}

/// The scene with its body resting on the body floor under it, if there is
/// one.
fn settle_body(mut scene: TraversalScene) -> TraversalScene {
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    if let Some(floor) = query.floor(scene.body.s) {
        scene.body.z = floor.z;
    }
    scene
}

fn render(request: &RenderRequest) -> RenderAnswer {
    let planner = Planner::new(request.scene.params.clone(), request.settings.clone());
    let Scrubbed {
        scene,
        outcome,
        events,
    } = scrub(&request.scene, &planner, request.time, request.perturb);
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let outcome = outcome.unwrap_or_else(|| planner.plan(&query, &scene.request()));
    let (plan, cost) = match &outcome {
        Ok(planned) => (&planned.plan, Some(planned.cost)),
        Err(refusal) => (&refusal.partial, None),
    };
    let first_tentative = plan.phases.get(plan.committed).map(|phase| phase.start);
    let footfalls = plan
        .footfalls()
        .map(|(time, contact)| FootfallView {
            limb: contact.limb,
            s: contact.at.s,
            d: contact.at.d,
            z: contact.at.z,
            time,
            committed: first_tentative.map_or(true, |tentative| time <= tentative),
        })
        .collect();
    let title = match (&outcome, cost) {
        (Ok(_), Some(cost)) => format!("crosses, cost {cost:.2}"),
        _ => "refuses".to_string(),
    };
    let (svg, layout) = render_scene_with_layout(&title, &scene, &section, Some(&outcome));
    RenderAnswer {
        svg,
        layout,
        crosses: outcome.is_ok(),
        cost,
        footfalls,
        body: scene.body,
        events,
        scene,
    }
}
