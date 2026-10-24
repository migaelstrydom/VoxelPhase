//! The page's operations, natively: every built-in scene renders, a scene
//! runs forward with loose footing giving way, and scenes survive RON.

use serde_json::{json, Value};
use traversal_planner_web::{handle, Op};

fn call(op: Op, request: &str) -> Value {
    let answer = handle(op as u32, request);
    let value: Value = serde_json::from_str(&answer).unwrap_or(Value::String(answer));
    assert!(value.get("error").is_none(), "{}: {value}", op as u32);
    value
}

fn builtins() -> Vec<String> {
    serde_json::from_value(call(Op::Builtins, "")).expect("names")
}

#[test]
fn every_builtin_renders() {
    let settings = call(Op::DefaultSettings, "");
    for name in builtins() {
        let scene = call(Op::Builtin, &name);
        let answer = call(
            Op::Render,
            &json!({ "scene": scene, "settings": settings }).to_string(),
        );
        assert!(
            answer["svg"]
                .as_str()
                .is_some_and(|svg| svg.starts_with("<svg")),
            "{name}"
        );
        assert!(
            answer["layout"]["side"]["scale"].as_f64().is_some(),
            "{name}"
        );
    }
}

#[test]
fn a_scene_runs_forward_with_stones_giving_way() {
    let settings = call(Op::DefaultSettings, "");
    let scene = call(Op::Builtin, "plank_bridge_middle");
    let start = scene["body"]["s"].as_f64().expect("s");
    let answer = call(
        Op::Render,
        &json!({ "scene": scene, "settings": settings, "time": 1.5, "perturb": true }).to_string(),
    );
    let end = answer["body"]["s"].as_f64().expect("s");
    assert!(end > start + 3.0, "ran from {start} to {end}");
    let events = answer["events"].as_array().expect("events");
    assert!(
        events
            .iter()
            .any(|event| event.as_str().is_some_and(|text| text.contains("gave way"))),
        "{events:?}"
    );
}

#[test]
fn a_scene_survives_ron() {
    let scene = call(Op::Builtin, "zigzag_stones");
    let ron = handle(Op::ToRon as u32, &scene.to_string());
    assert_eq!(call(Op::FromRon, &ron), scene);
}
