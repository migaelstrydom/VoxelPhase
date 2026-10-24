use crate::properties::Rule;
use crate::scene::{Ground, TraversalScene};

use super::sweep::try_scene;

/// The fewest shapes of `scene` that still break `rule`: shapes are dropped
/// one at a time while the rule stays broken, until none can be.
pub fn shrink(scene: &TraversalScene, rule: Rule) -> TraversalScene {
    let breaks = |scene: &TraversalScene| {
        try_scene(scene)
            .violations
            .iter()
            .any(|violation| violation.rule == rule)
    };
    let mut smallest = scene.clone();
    loop {
        let Ground::Authored(ground) = &smallest.ground else {
            return smallest;
        };
        let fewer = (0..ground.shapes.len()).find_map(|index| {
            let mut candidate = smallest.clone();
            if let Ground::Authored(ground) = &mut candidate.ground {
                ground.shapes.remove(index);
            }
            breaks(&candidate).then_some(candidate)
        });
        match fewer {
            Some(candidate) => smallest = candidate,
            None => return smallest,
        }
    }
}
