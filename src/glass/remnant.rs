//! When what is left of a pane stops being a pane.
//!
//! Glass is broken away a piece at a time, and there is always a last piece.
//! Left to the joints alone that piece stays exactly where it was — a shard
//! hanging in an empty window, immovable because the body behind it is
//! static. So a sheet also judges *what is left of it*, and once that is no
//! longer a sheet in any useful sense it gives the rest up as shards and the
//! body retires.
//!
//! Three ways to stop being a sheet, because one is not enough:
//!
//! ```text
//!   too few cells  ──┐
//!   too little area ─┼──▶ the remnant lets go
//!   too little left ─┘
//! ```
//!
//! The cell count catches the lone shard whatever its size. The absolute
//! area catches a crumb. The fraction catches the case the other two miss:
//! a pane broken into coarse shards, most of them gone, the rest still a
//! respectable few hundred square centimetres of glass clinging to the
//! frame. That last one is what makes a window empty out as you work on it
//! rather than asymptotically approach empty.

/// What is left of a sheet's glass: the largest group of cells that still
/// hold together, measured over its glass alone. A framed pane's group
/// includes the frame that grips it; the frame is not glass and is not
/// counted here.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Remnant {
    /// Glass area of the group, in m².
    pub area: f32,
    /// How many glass cells are in it.
    pub cells: usize,
}

/// The size below which a remnant is no longer a sheet.
#[derive(Debug, Clone, Copy)]
pub struct RemnantRule {
    /// Area, in m², below which the remnant lets go whatever else is true.
    pub min_area: f32,
    /// Share of the pane's original glass area below which it lets go. The
    /// rule that survives coarse shards: a remnant can be far too big to be
    /// a crumb and still be the last quarter of a window.
    pub min_fraction: f32,
    /// Cell count at or below which it lets go. One shard alone is not a
    /// sheet, however large it is.
    pub min_cells: usize,
}

impl Default for RemnantRule {
    fn default() -> Self {
        Self {
            min_area: 0.03,
            min_fraction: 0.25,
            min_cells: 1,
        }
    }
}

impl RemnantRule {
    /// Whether `remnant` is still a sheet, given the `whole` glass area the
    /// pane started with. A `whole` of zero means the original size is not
    /// known, and only the absolute rules apply.
    pub fn holds(&self, remnant: Remnant, whole: f32) -> bool {
        remnant.cells > self.min_cells
            && remnant.area >= self.min_area
            && (whole <= 0.0 || remnant.area >= whole * self.min_fraction)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WHOLE: f32 = 3.0;

    fn remnant(area: f32, cells: usize) -> Remnant {
        Remnant { area, cells }
    }

    #[test]
    fn most_of_a_pane_is_still_a_pane() {
        assert!(RemnantRule::default().holds(remnant(2.0, 18), WHOLE));
    }

    #[test]
    fn a_lone_shard_is_not_a_sheet_however_large() {
        assert!(!RemnantRule::default().holds(remnant(1.5, 1), WHOLE));
    }

    /// The case coarse shards brought back: far too big to be a crumb, and
    /// still only the last corner of a window.
    #[test]
    fn the_last_quarter_of_a_pane_lets_go_though_it_is_no_crumb() {
        let rule = RemnantRule::default();
        assert!(rule.holds(remnant(0.8, 7), WHOLE));
        assert!(!rule.holds(remnant(0.7, 6), WHOLE));
    }

    #[test]
    fn a_crumb_lets_go_even_of_a_pane_whose_size_is_unknown() {
        assert!(!RemnantRule::default().holds(remnant(0.02, 4), 0.0));
        assert!(RemnantRule::default().holds(remnant(0.5, 4), 0.0));
    }
}
