use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use traversal_planner::plan::Limb;
use traversal_planner::scene::{Ground, Intent, TraversalScene};
use traversal_planner::GaitParams;

use super::planning::PlanFrame;

/// A frame's planning as a scene: its strip, the body, the foot its plan
/// leads with and the speed asked for, under `params`. Planned afresh, it
/// plans as the frame did with nothing kept from the frames before.
pub fn scene_of(frame: &PlanFrame, params: &GaitParams, target_speed: f32) -> TraversalScene {
    let lead = frame
        .plan()
        .footfalls()
        .next()
        .map_or(Limb::LeftFoot, |(_, contact)| contact.limb);
    TraversalScene {
        ground: Ground::Captured(frame.section.clone()),
        body: frame.body,
        lead,
        intent: Intent {
            speed: target_speed,
        },
        params: params.clone(),
        layout: frame.section.layout.clone(),
        expect: None,
    }
}

/// Writes `scene` to a file of its own in `dir`, headed by `note`; where.
pub fn write_capture(scene: &TraversalScene, dir: &Path, note: &str) -> Result<PathBuf, String> {
    let ron = scene.to_ron().map_err(|error| error.to_string())?;
    fs::create_dir_all(dir).map_err(|error| error.to_string())?;
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_millis());
    let path = dir.join(format!("traversal_{stamp}.ron"));
    fs::write(&path, format!("// {note}\n{ron}")).map_err(|error| error.to_string())?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use traversal_planner::section::StripQuery;
    use traversal_planner::{Planner, PlannerSettings};

    use super::*;
    use crate::traversal::{PathFrame, PlanningCost};

    /// A frame planned over the 25 cm lip, as the game would have it.
    fn lip_frame() -> (PlanFrame, TraversalScene) {
        let scene = TraversalScene::from_ron(include_str!(
            "../../crates/traversal_planner/fixtures/lip_25cm.ron"
        ))
        .expect("the fixture reads");
        let section = scene.section();
        let planner = Planner::new(scene.params.clone(), PlannerSettings::default());
        let outcome = planner.plan(&StripQuery::new(&section, &scene.params), &scene.request());
        let frame = PlanFrame {
            frame: PathFrame::new(Point3::origin(), 90.0),
            section,
            body: scene.body,
            outcome,
            cost: PlanningCost::default(),
        };
        (frame, scene)
    }

    #[test]
    fn a_capture_replans_as_its_frame_planned() {
        let (frame, scene) = lip_frame();
        let captured = scene_of(&frame, &scene.params, scene.intent.speed);
        let read =
            TraversalScene::from_ron(&captured.to_ron().expect("writes")).expect("reads back");

        assert_eq!(read, captured, "survives the file");
        assert_eq!(read.body, frame.body);
        assert_eq!(read.section(), frame.section);
        let replanned = Planner::new(read.params.clone(), PlannerSettings::default())
            .plan(
                &StripQuery::new(&read.section(), &read.params),
                &read.request(),
            )
            .expect("the lip is crossed");
        assert_eq!(&replanned.plan, frame.plan());
    }

    #[test]
    fn a_capture_leads_with_the_foot_its_plan_puts_down_first() {
        let (frame, scene) = lip_frame();
        let first = frame.plan().footfalls().next().expect("a footfall").1.limb;
        assert_eq!(scene_of(&frame, &scene.params, 5.0).lead, first);
    }
}
