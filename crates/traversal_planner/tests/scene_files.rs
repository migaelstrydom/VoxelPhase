//! Every fixture parses, and a scene survives a round trip through RON.

use std::fs;
use std::path::Path;

use traversal_planner::TraversalScene;

fn fixtures() -> Vec<(String, TraversalScene)> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures");
    let mut scenes: Vec<_> = fs::read_dir(&dir)
        .expect("fixtures directory")
        .map(|entry| entry.expect("fixture entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "ron"))
        .map(|path| {
            let text = fs::read_to_string(&path).expect("fixture readable");
            let scene = TraversalScene::from_ron(&text)
                .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
            (path.display().to_string(), scene)
        })
        .collect();
    scenes.sort_by(|a, b| a.0.cmp(&b.0));
    scenes
}

#[test]
fn fixtures_parse() {
    assert!(!fixtures().is_empty());
}

#[test]
fn every_fixture_states_its_outcome() {
    for (name, scene) in fixtures() {
        assert!(scene.expect.is_some(), "{name} states no expected outcome");
    }
}

#[test]
fn scenes_round_trip() {
    for (name, scene) in fixtures() {
        let text = scene.to_ron().expect("serialises");
        let back = TraversalScene::from_ron(&text).expect("parses back");
        assert_eq!(back, scene, "{name}");
    }
}
