//! The aiming cursor: four brackets that fly in around where a throw lands.

use nalgebra::Vector2;

use crate::rendering::colour::Colour;

use super::element::{HudContext, HudElement};
use super::painter::HudPainter;

/// The four corners a bracket can sit at, as screen-space signs.
const CORNERS: [(f32, f32); 4] = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];

/// Look and timing of the aiming cursor.
///
/// Sizes are in pixels at the HUD's reference height and are scaled by
/// [`HudContext::scale`] when drawn, so one set of numbers covers every
/// display.
#[derive(Debug, Clone, Copy)]
pub struct ReticleStyle {
    /// Distance from the aim point to each bracket's outer corner.
    pub radius: f32,
    /// Length of each of a bracket's two arms.
    pub arm_length: f32,
    /// Stroke thickness.
    pub thickness: f32,
    /// Half-size of the dot at the exact aim point.
    pub dot_half_size: f32,
    /// How far out a bracket starts before it flies in.
    pub fly_in_distance: f32,
    /// Fraction of the fade a bracket's arrival is delayed by, per corner.
    /// Small: the four arrivals should read as one gesture, not a queue.
    pub stagger: f32,
    /// How far the whole cursor is turned when it starts, unwinding to square
    /// as it settles (radians).
    pub entry_spin: f32,
    /// Seconds to fade fully in.
    pub rise_time: f32,
    /// Seconds to fade fully out. Shorter than the rise: appearing is an
    /// event worth watching, leaving is not.
    pub fall_time: f32,
    /// Seconds for the cursor to catch up with a moving aim point. Enough to
    /// take the jitter off a surface-to-surface jump, too little to lie about
    /// where the throw goes.
    pub follow_time: f32,
    /// A jump bigger than this fraction of the screen is a different target,
    /// not a moving one, so the cursor cuts to it instead of sliding.
    pub snap_fraction: f32,
    /// Cursor colour. Pale and half-transparent: it has to be readable over
    /// sky, stone and ice without becoming the brightest thing on screen.
    pub colour: Colour,
    /// Dark underlay drawn one pixel behind every stroke, which is what keeps
    /// a pale cursor legible against a bright sky.
    pub shadow: Colour,
}

impl Default for ReticleStyle {
    fn default() -> Self {
        Self {
            radius: 15.0,
            arm_length: 8.0,
            thickness: 2.0,
            dot_half_size: 1.5,
            fly_in_distance: 44.0,
            stagger: 0.12,
            entry_spin: 0.45,
            rise_time: 0.20,
            fall_time: 0.11,
            follow_time: 0.05,
            snap_fraction: 0.25,
            colour: Colour::new(0.74, 0.89, 1.0, 0.7),
            shadow: Colour::new(0.03, 0.05, 0.09, 0.3),
        }
    }
}

/// A cursor pinned to the point the player's throw would reach.
///
/// It owns two pieces of state and no opinions about what is being aimed: how
/// far into its entrance it is, and where on screen it currently sits. The
/// decision about *whether* there is something to aim belongs upstream, in
/// [`crate::aim::AimState`] — here, a solution means show, no solution means
/// go.
pub struct AimReticle {
    style: ReticleStyle,
    /// 0 gone, 1 fully arrived. Drives the entrance and the exit alike.
    presence: f32,
    /// Where the cursor is drawn, which trails the projected aim point by
    /// [`ReticleStyle::follow_time`]. `None` until the first frame it is shown.
    anchor: Option<Vector2<f32>>,
    /// Pixels-per-authored-pixel, taken from the frame's context.
    scale: f32,
}

impl Default for AimReticle {
    fn default() -> Self {
        Self::new(ReticleStyle::default())
    }
}

impl AimReticle {
    pub fn new(style: ReticleStyle) -> Self {
        Self {
            style,
            presence: 0.0,
            anchor: None,
            scale: 1.0,
        }
    }

    /// Draw one stroke: a dark underlay, then the stroke itself.
    fn stroke(&self, painter: &mut HudPainter, from: Vector2<f32>, to: Vector2<f32>, alpha: f32) {
        let thickness = self.style.thickness * self.scale;
        let offset = Vector2::new(self.scale, self.scale);

        painter.bar(
            from + offset,
            to + offset,
            thickness,
            self.style.shadow.with_alpha(self.style.shadow.a * alpha),
        );
        painter.bar(
            from,
            to,
            thickness,
            self.style.colour.with_alpha(self.style.colour.a * alpha),
        );
    }

    /// How far along its entrance the bracket at `index` is.
    fn bracket_progress(&self, index: usize) -> f32 {
        let stagger = self.style.stagger;
        let span = 1.0 - stagger * (CORNERS.len() - 1) as f32;
        ((self.presence - stagger * index as f32) / span.max(1e-3)).clamp(0.0, 1.0)
    }
}

impl HudElement for AimReticle {
    fn update(&mut self, ctx: &HudContext) {
        self.scale = ctx.scale;

        let target = ctx
            .aim
            .solution
            .and_then(|solution| ctx.project(solution.impact.point));

        let rate = if target.is_some() {
            1.0 / self.style.rise_time.max(1e-3)
        } else {
            -1.0 / self.style.fall_time.max(1e-3)
        };
        self.presence = (self.presence + rate * ctx.dt).clamp(0.0, 1.0);

        // The cursor keeps its last anchor while it fades out, so it leaves
        // from where it was rather than from wherever the camera is pointing.
        if let Some(target) = target {
            self.anchor = Some(match self.anchor {
                Some(current) => {
                    let snap = self.style.snap_fraction * ctx.screen.magnitude();
                    if (target - current).magnitude() > snap {
                        target
                    } else {
                        let catch_up = 1.0 - (-ctx.dt / self.style.follow_time.max(1e-3)).exp();
                        current + (target - current) * catch_up
                    }
                }
                None => target,
            });
        } else if self.presence <= 0.0 {
            self.anchor = None;
        }
    }

    fn draw(&self, painter: &mut HudPainter) {
        let (Some(anchor), true) = (self.anchor, self.presence > 0.0) else {
            return;
        };

        let settle = ease_out_cubic(self.presence);
        let spin = self.style.entry_spin * (1.0 - settle);
        let radius = self.style.radius * self.scale;
        let arm = self.style.arm_length * self.scale;

        for (index, (sx, sy)) in CORNERS.iter().enumerate() {
            let progress = self.bracket_progress(index);
            if progress <= 0.0 {
                continue;
            }

            // Each bracket starts out along its own diagonal and eases home,
            // overshooting a little so it arrives with some weight.
            let outward = Vector2::new(*sx, *sy).normalize();
            let flight = self.style.fly_in_distance * self.scale * (1.0 - ease_out_back(progress));

            let corner = rotate(Vector2::new(sx * radius, sy * radius), spin) + outward * flight;
            let along_x = rotate(Vector2::new(-sx * arm, 0.0), spin);
            let along_y = rotate(Vector2::new(0.0, -sy * arm), spin);

            self.stroke(
                painter,
                anchor + corner,
                anchor + corner + along_x,
                progress,
            );
            self.stroke(
                painter,
                anchor + corner,
                anchor + corner + along_y,
                progress,
            );
        }

        // The dot lands last, once the brackets have somewhere to point at.
        let dot = ((self.presence - 0.55) / 0.45).clamp(0.0, 1.0);
        if dot > 0.0 {
            let half = self.style.dot_half_size * self.scale * ease_out_back(dot);
            painter.rotated_rect(
                anchor,
                Vector2::new(half, half),
                std::f32::consts::FRAC_PI_4,
                self.style.colour.with_alpha(self.style.colour.a * dot),
            );
        }
    }
}

/// Rotate a screen-space vector clockwise (screen Y points down).
fn rotate(v: Vector2<f32>, angle: f32) -> Vector2<f32> {
    let (sin, cos) = angle.sin_cos();
    Vector2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos)
}

/// Fast at first, settling at the end.
fn ease_out_cubic(t: f32) -> f32 {
    let inverted = 1.0 - t.clamp(0.0, 1.0);
    1.0 - inverted * inverted * inverted
}

/// Like [`ease_out_cubic`] but overshoots past 1 before settling back, which
/// is what makes an arrival read as a movement rather than a fade.
fn ease_out_back(t: f32) -> f32 {
    const OVERSHOOT: f32 = 1.7;
    let inverted = t.clamp(0.0, 1.0) - 1.0;
    1.0 + (OVERSHOOT + 1.0) * inverted.powi(3) + OVERSHOOT * inverted.powi(2)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix4, Point3};

    use crate::aim::{AimKind, AimSolution, AimState, Impact};

    use super::*;

    fn aiming_at(point: Point3<f32>) -> AimState {
        AimState {
            solution: Some(AimSolution {
                kind: AimKind::Grenade,
                impact: Impact {
                    point,
                    normal: nalgebra::Vector3::y(),
                    time: 1.0,
                },
            }),
        }
    }

    /// Looking down -Z from the origin, which puts a point at -Z on screen
    /// centre.
    fn context<'a>(aim: &'a AimState, dt: f32) -> HudContext<'a> {
        let view = Matrix4::look_at_rh(
            &Point3::origin(),
            &Point3::new(0.0, 0.0, -1.0),
            &nalgebra::Vector3::y(),
        );
        // Flipped in Y, as the engine's camera builds it for Vulkan.
        let mut projection = Matrix4::new_perspective(16.0 / 9.0, 1.0, 0.1, 100.0);
        projection[(1, 1)] *= -1.0;
        HudContext::new(Vector2::new(1600.0, 900.0), dt, projection * view, aim)
    }

    fn run(reticle: &mut AimReticle, aim: &AimState, seconds: f32) {
        let dt = 1.0 / 60.0;
        let mut elapsed = 0.0;
        while elapsed < seconds {
            reticle.update(&context(aim, dt));
            elapsed += dt;
        }
    }

    fn quads(reticle: &AimReticle) -> usize {
        let mut painter = HudPainter::new(Vector2::zeros());
        reticle.draw(&mut painter);
        painter.geometry().indices().len() / 6
    }

    #[test]
    fn nothing_is_drawn_until_there_is_something_to_aim_at() {
        let reticle = AimReticle::default();
        assert_eq!(quads(&reticle), 0);
    }

    #[test]
    fn a_settled_cursor_draws_every_bracket_and_the_dot() {
        let aim = aiming_at(Point3::new(0.0, 0.0, -10.0));
        let mut reticle = AimReticle::default();
        run(&mut reticle, &aim, 0.5);

        // Four brackets of two strokes, each stroke drawn over a shadow, plus
        // the dot.
        assert_eq!(quads(&reticle), 4 * 2 * 2 + 1);
    }

    #[test]
    fn the_cursor_sits_where_the_throw_lands() {
        let aim = aiming_at(Point3::new(0.0, 0.0, -10.0));
        let mut reticle = AimReticle::default();
        run(&mut reticle, &aim, 0.5);

        let anchor = reticle.anchor.expect("a settled cursor has an anchor");
        assert!((anchor.x - 800.0).abs() < 1.0, "anchor x was {}", anchor.x);
        assert!((anchor.y - 450.0).abs() < 1.0, "anchor y was {}", anchor.y);
    }

    #[test]
    fn losing_the_target_fades_the_cursor_out_completely() {
        let aim = aiming_at(Point3::new(0.0, 0.0, -10.0));
        let mut reticle = AimReticle::default();
        run(&mut reticle, &aim, 0.5);

        run(&mut reticle, &AimState::default(), 0.5);

        assert_eq!(reticle.presence, 0.0);
        assert_eq!(quads(&reticle), 0);
    }

    #[test]
    fn a_point_behind_the_camera_is_not_aimed_at() {
        let aim = aiming_at(Point3::new(0.0, 0.0, 10.0));
        let mut reticle = AimReticle::default();
        run(&mut reticle, &aim, 0.5);

        assert_eq!(quads(&reticle), 0);
    }

    #[test]
    fn the_brackets_arrive_one_after_another() {
        let mut reticle = AimReticle::default();
        reticle.presence = 0.5;

        let progress: Vec<f32> = (0..CORNERS.len())
            .map(|index| reticle.bracket_progress(index))
            .collect();

        assert!(
            progress.windows(2).all(|pair| pair[0] > pair[1]),
            "brackets should trail each other: {:?}",
            progress
        );
    }
}
