use crate::section::Footing;

/// Authored shapes behind everything else.
pub const SHAPE_FILL: &str = "#e4e4e4";
/// Authored shapes that do not cross the path itself, in the side view.
pub const SHAPE_OFF_PATH_FILL: &str = "#f1f1f1";
pub const SHAPE_EDGE: &str = "#b0b0b0";
pub const BODY_FLOOR: &str = "#2b6cb0";
pub const CEILING: &str = "#805ad5";
pub const BODY: &str = "#2b6cb0";
/// The planned path of the bottom-sphere centre.
pub const BODY_PATH: &str = "#1a202c";
/// The partial plan of a refusal: where the way ran out.
pub const REFUSED: &str = "#c53030";
pub const LEFT_FOOT: &str = "#3182ce";
pub const RIGHT_FOOT: &str = "#dd6b20";
pub const GRID: &str = "#eeeeee";
pub const LABEL: &str = "#555555";

/// A foothold's colour by what holds it up: firm green, resting rubble
/// amber, moving rubble red.
pub fn footing(footing: Footing) -> &'static str {
    match footing {
        Footing::Fixed => "#2f855a",
        Footing::Loose { resting: true, .. } => "#d69e2e",
        Footing::Loose { resting: false, .. } => "#e53e3e",
    }
}
