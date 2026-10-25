//! Scenes the page offers to start from: the planner's fixtures.

/// `(name, RON)` of each built-in scene.
pub const SCENES: &[(&str, &str)] = &[
    (
        "plank_bridge_approach",
        include_str!("../../traversal_planner/fixtures/plank_bridge_approach.ron"),
    ),
    (
        "plank_bridge_middle",
        include_str!("../../traversal_planner/fixtures/plank_bridge_middle.ron"),
    ),
    (
        "zigzag_stones",
        include_str!("../../traversal_planner/fixtures/zigzag_stones.ron"),
    ),
    (
        "stepping_stones_in_line",
        include_str!("../../traversal_planner/fixtures/stepping_stones_in_line.ron"),
    ),
    (
        "tilted_stone",
        include_str!("../../traversal_planner/fixtures/tilted_stone.ron"),
    ),
    (
        "lip_03cm",
        include_str!("../../traversal_planner/fixtures/lip_03cm.ron"),
    ),
    (
        "lip_25cm",
        include_str!("../../traversal_planner/fixtures/lip_25cm.ron"),
    ),
    (
        "lip_40cm",
        include_str!("../../traversal_planner/fixtures/lip_40cm.ron"),
    ),
    (
        "lip_60cm",
        include_str!("../../traversal_planner/fixtures/lip_60cm.ron"),
    ),
    (
        "vertical_wall",
        include_str!("../../traversal_planner/fixtures/vertical_wall.ron"),
    ),
    (
        "garden_bridge_approach (captured)",
        include_str!("../../traversal_planner/fixtures/garden_bridge_approach.ron"),
    ),
    (
        "stack_yard_crate_pile_east (captured)",
        include_str!("../../traversal_planner/fixtures/stack_yard_crate_pile_east.ron"),
    ),
    (
        "stack_yard_voussoir_arch (captured)",
        include_str!("../../traversal_planner/fixtures/stack_yard_voussoir_arch.ron"),
    ),
];
