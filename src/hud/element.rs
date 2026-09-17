//! What the HUD tells its elements, and what they must be able to do.

use nalgebra::{Matrix4, Point3, Vector2};

use crate::aim::AimState;

use super::painter::HudPainter;

/// Screen height the HUD's pixel sizes are authored against. A taller display
/// scales everything up in proportion, so the cursor covers the same fraction
/// of the screen on a retina panel as on a small window.
const REFERENCE_HEIGHT: f32 = 900.0;

/// The frame, as an element sees it.
pub struct HudContext<'a> {
    /// Overlay size in pixels, (0,0) at the top-left.
    pub screen: Vector2<f32>,
    /// Multiplier from authored pixels to this display's pixels.
    pub scale: f32,
    /// Seconds since the last frame, for animation.
    pub dt: f32,
    /// Where the player's throw would land this frame.
    pub aim: &'a AimState,
    /// World to clip space, for anchoring elements to things in the scene.
    view_projection: Matrix4<f32>,
}

impl<'a> HudContext<'a> {
    pub fn new(
        screen: Vector2<f32>,
        dt: f32,
        view_projection: Matrix4<f32>,
        aim: &'a AimState,
    ) -> Self {
        Self {
            screen,
            scale: (screen.y / REFERENCE_HEIGHT).clamp(0.75, 2.5),
            dt,
            aim,
            view_projection,
        }
    }

    /// Where a world-space point lands on screen, in overlay pixels.
    ///
    /// `None` when the point is behind the eye or on the near plane, where
    /// the projection has no answer that means anything on screen.
    pub fn project(&self, world: Point3<f32>) -> Option<Vector2<f32>> {
        let clip = self.view_projection * world.to_homogeneous();
        if clip.w <= 1e-4 {
            return None;
        }

        let ndc = clip.xy() / clip.w;
        Some(Vector2::new(
            (ndc.x * 0.5 + 0.5) * self.screen.x,
            (ndc.y * 0.5 + 0.5) * self.screen.y,
        ))
    }
}

/// One piece of the HUD.
///
/// Update and draw are separate because animation state belongs to the element
/// and geometry does not: `update` is where an element decides what it looks
/// like this frame, `draw` only says so.
pub trait HudElement {
    /// Advance this element's animation by `ctx.dt`.
    fn update(&mut self, ctx: &HudContext);

    /// Emit this frame's geometry. Called after [`Self::update`], and must not
    /// depend on anything the update did not already settle.
    fn draw(&self, painter: &mut HudPainter);
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    /// The engine's convention: a right-handed view, and a projection with Y
    /// flipped for Vulkan's downward clip space.
    fn context(aim: &AimState) -> HudContext<'_> {
        let view = Matrix4::look_at_rh(
            &Point3::new(0.0, 0.0, 0.0),
            &Point3::new(0.0, 0.0, -1.0),
            &Vector3::y(),
        );
        let mut projection = Matrix4::new_perspective(16.0 / 9.0, 1.0, 0.1, 100.0);
        projection[(1, 1)] *= -1.0;

        HudContext::new(
            Vector2::new(1600.0, 900.0),
            1.0 / 60.0,
            projection * view,
            aim,
        )
    }

    #[test]
    fn a_point_straight_ahead_lands_in_the_middle_of_the_screen() {
        let aim = AimState::default();
        let screen = context(&aim)
            .project(Point3::new(0.0, 0.0, -10.0))
            .expect("a point in front of the eye projects");

        assert!((screen.x - 800.0).abs() < 1.0, "x was {}", screen.x);
        assert!((screen.y - 450.0).abs() < 1.0, "y was {}", screen.y);
    }

    /// Overlay pixels count downward from the top, so higher in the world must
    /// mean a smaller Y. Getting this backwards puts the cursor in the mirror
    /// image of where the throw lands.
    fn projects_higher_up_the_screen() -> bool {
        let aim = AimState::default();
        let ctx = context(&aim);
        let high = ctx.project(Point3::new(0.0, 2.0, -10.0)).unwrap();
        let low = ctx.project(Point3::new(0.0, -2.0, -10.0)).unwrap();
        high.y < low.y
    }

    #[test]
    fn higher_in_the_world_is_higher_on_the_screen() {
        assert!(projects_higher_up_the_screen());
    }

    #[test]
    fn a_point_behind_the_eye_has_no_place_on_screen() {
        let aim = AimState::default();
        assert!(context(&aim).project(Point3::new(0.0, 0.0, 10.0)).is_none());
    }
}
