use crate::plan::{Limb, Motion, Plan};
use crate::section::SectionQuery;

use super::canvas::{Canvas, Viewport};
use super::palette;

/// Spacing along `s` at which the body's path is drawn, in metres.
const PATH_STEP: f32 = 0.02;
/// Radius footfalls are drawn at, in metres.
const FOOTFALL_RADIUS: f32 = 0.035;
/// Opacity of tentative footfalls; committed ones are opaque.
const TENTATIVE_OPACITY: f32 = 0.5;

/// A plan over both views: the bottom-sphere centre's path in the side view
/// (flights dashed), and every footfall in both.
pub fn draw_plan(
    canvas: &mut Canvas,
    side: &Viewport,
    top: &Viewport,
    query: &dyn SectionQuery,
    plan: &Plan,
    refused: bool,
) {
    let colour = if refused {
        palette::REFUSED
    } else {
        palette::BODY_PATH
    };
    for phase in &plan.phases {
        let (from, to, dash) = match phase.motion {
            Motion::Grounded { from, to, .. } => (from, to, ""),
            Motion::Flight { takeoff, .. } => {
                (takeoff.s, phase.end_s(), r#" stroke-dasharray="5 3""#)
            }
        };
        let steps = ((to - from) / PATH_STEP).ceil().max(1.0) as usize;
        let points = (0..=steps).filter_map(|step| {
            let s = from + (to - from) * step as f32 / steps as f32;
            let z = phase
                .flight_height(s)
                .or_else(|| query.floor(s).map(|floor| floor.z))?;
            Some((s, z))
        });
        canvas.polyline(
            side,
            points,
            &format!(r#"stroke="{colour}" stroke-width="2"{dash}"#),
        );
    }

    let first_tentative = plan.phases.get(plan.committed).map(|phase| phase.start);
    for (time, contact) in plan.footfalls() {
        let fill = match contact.limb {
            Limb::LeftFoot => palette::LEFT_FOOT,
            Limb::RightFoot => palette::RIGHT_FOOT,
        };
        let opacity = if first_tentative.is_some_and(|tentative| time > tentative) {
            TENTATIVE_OPACITY
        } else {
            1.0
        };
        let style =
            format!(r#"fill="{fill}" fill-opacity="{opacity}" stroke="{colour}" stroke-width="1""#);
        canvas.circle(side, (contact.at.s, contact.at.z), FOOTFALL_RADIUS, &style);
        canvas.circle(top, (contact.at.s, contact.at.d), FOOTFALL_RADIUS, &style);
    }
}
