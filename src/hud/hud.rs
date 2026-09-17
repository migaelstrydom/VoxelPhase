//! The element list.

use super::element::{HudContext, HudElement};
use super::painter::HudPainter;
use super::reticle::AimReticle;
use crate::rendering::overlay::OverlayGeometry;

/// Every HUD element, updated and drawn in order.
///
/// Order is paint order: later elements draw over earlier ones.
pub struct Hud {
    elements: Vec<Box<dyn HudElement>>,
}

impl Hud {
    pub fn new(elements: Vec<Box<dyn HudElement>>) -> Self {
        Self { elements }
    }

    /// The HUD the game runs with.
    pub fn game() -> Self {
        Self::new(vec![Box::<AimReticle>::default()])
    }

    /// Advance every element, then paint them into one batch of geometry.
    pub fn render(
        &mut self,
        ctx: &HudContext,
        solid_uv: nalgebra::Vector2<f32>,
    ) -> OverlayGeometry {
        for element in &mut self.elements {
            element.update(ctx);
        }

        let mut painter = HudPainter::new(solid_uv);
        for element in &self.elements {
            element.draw(&mut painter);
        }
        painter.into_geometry()
    }
}

impl Default for Hud {
    fn default() -> Self {
        Self::game()
    }
}
