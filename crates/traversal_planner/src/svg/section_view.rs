use crate::planner::{Planned, Refusal};
use crate::scene::{AuthoredGround, Ground, Shape, TraversalScene};
use crate::section::{Profile, Section, StripQuery, SurfacePoint};

use super::canvas::{Canvas, Viewport};
use super::palette;
use super::plan_view::draw_plan;

/// Pixels per metre, on both axes of both views.
const SCALE: f32 = 300.0;
/// Pixels around and between the views.
const MARGIN: f32 = 40.0;
/// Metres of headroom drawn above and below what the section holds.
const PADDING: f32 = 0.15;
/// Metres between grid lines.
const GRID_STEP: f32 = 0.5;

/// The scene's section in a side view over a top-down strip, with the
/// planner's answer over it when there is one.
pub fn render_scene(
    title: &str,
    scene: &TraversalScene,
    section: &Section,
    outcome: Option<&Result<Planned, Refusal>>,
) -> String {
    let s_min = scene.body.s - section.layout.behind;
    let s_max = scene.body.s + section.layout.ahead;
    let (z_min, z_max) = height_range(scene, section);
    let d_half = section
        .layout
        .lanes
        .iter()
        .fold(scene.params.body.radius, |widest, d| {
            widest.max(d.abs() + section.foot_radius)
        })
        + PADDING;

    let width = 2.0 * MARGIN + (s_max - s_min) * SCALE;
    let side = Viewport {
        left: MARGIN,
        top: MARGIN,
        scale: SCALE,
        s_min,
        up_max: z_max,
    };
    let top_view_top = side.top + (z_max - z_min) * SCALE + MARGIN;
    let top = Viewport {
        left: MARGIN,
        top: top_view_top,
        scale: SCALE,
        s_min,
        up_max: d_half,
    };
    let height = top_view_top + 2.0 * d_half * SCALE + MARGIN;

    let mut canvas = Canvas::new(width, height);
    canvas.text(
        MARGIN,
        24.0,
        title,
        &format!(r#"font-size="16" fill="{}""#, palette::LABEL),
    );
    grid(&mut canvas, &side, (s_min, s_max), (z_min, z_max), "z");
    grid(&mut canvas, &top, (s_min, s_max), (-d_half, d_half), "d");

    if let Ground::Authored(ground) = &scene.ground {
        side_shapes(&mut canvas, &side, ground, (s_min, s_max), (z_min, z_max));
        top_shapes(&mut canvas, &top, ground, (s_min, s_max), d_half);
    }

    side_footholds(&mut canvas, &side, section);
    canvas.polyline(
        &side,
        centres(&section.body_floor),
        &format!(
            r#"stroke="{}" stroke-width="1.5" stroke-dasharray="6 3""#,
            palette::BODY_FLOOR
        ),
    );
    canvas.polyline(
        &side,
        centres(&section.ceiling),
        &format!(
            r#"stroke="{}" stroke-width="1.5" stroke-dasharray="6 3""#,
            palette::CEILING
        ),
    );
    capsule(&mut canvas, &side, scene);

    top_footholds(&mut canvas, &top, section);
    canvas.circle(
        &top,
        (scene.body.s, 0.0),
        scene.params.body.radius,
        &format!(
            r#"fill="none" stroke="{}" stroke-width="1.5""#,
            palette::BODY
        ),
    );

    if let Some(outcome) = outcome {
        let (plan, refused) = match outcome {
            Ok(planned) => (&planned.plan, false),
            Err(refusal) => (&refusal.partial, true),
        };
        draw_plan(
            &mut canvas,
            &side,
            &top,
            &StripQuery::new(section, &scene.params),
            plan,
            refused,
        );
    }

    legend(&mut canvas, width, top_view_top - 10.0);
    canvas.finish()
}

/// The heights the side view must show: the ground under every foothold,
/// the body, the ceiling.
fn height_range(scene: &TraversalScene, section: &Section) -> (f32, f32) {
    let mut low = scene.body.z - scene.params.body.radius;
    let mut high = scene.body.z + scene.params.body.spine() + scene.params.body.radius;
    let lanes = section.foothold_lanes.iter().map(|lane| &lane.profile);
    for profile in lanes.chain([&section.body_floor, &section.ceiling]) {
        for point in profile.surfaces.iter().flat_map(|surface| &surface.points) {
            low = low.min(point.z - section.body_radius);
            high = high.max(point.z + section.body_radius);
        }
    }
    (low - PADDING, high + PADDING)
}

fn centres(profile: &Profile) -> impl Iterator<Item = (f32, f32)> + '_ {
    profile
        .surfaces
        .iter()
        .flat_map(|surface| surface.points.iter().map(|point| (point.s, point.z)))
}

/// Where a foot-sized sphere at `point` touches the ground.
fn contact(point: &SurfacePoint, radius: f32) -> (f32, f32) {
    (
        point.s - radius * point.normal.s,
        point.z - radius * point.normal.z,
    )
}

fn grid(canvas: &mut Canvas, view: &Viewport, s: (f32, f32), up: (f32, f32), up_name: &str) {
    let style = format!(r#"stroke="{}" stroke-width="1""#, palette::GRID);
    let label = format!(r#"font-size="10" fill="{}""#, palette::LABEL);
    let mut line = (s.0 / GRID_STEP).ceil() * GRID_STEP;
    while line <= s.1 {
        let from = view.at(line, up.1);
        let to = view.at(line, up.0);
        canvas.pixel_line(from, to, &style);
        canvas.text(to.0 - 8.0, to.1 + 12.0, &format!("{line:.1}"), &label);
        line += GRID_STEP;
    }
    let mut line = (up.0 / GRID_STEP).ceil() * GRID_STEP;
    while line <= up.1 {
        let from = view.at(s.0, line);
        let to = view.at(s.1, line);
        canvas.pixel_line(from, to, &style);
        canvas.text(
            from.0 - 34.0,
            from.1 + 3.0,
            &format!("{up_name} {line:.1}"),
            &label,
        );
        line += GRID_STEP;
    }
}

/// Where a shape's footprint crosses the path line `d = 0`, as a range of
/// `s`; `None` when it lies wholly to one side.
fn cut_on_path(shape: &Shape) -> Option<(f32, f32)> {
    let footprint = shape.footprint();
    let mut cut: Option<(f32, f32)> = None;
    for i in 0..footprint.len() {
        let (start, end) = (footprint[i], footprint[(i + 1) % footprint.len()]);
        if (start.1 > 0.0) == (end.1 > 0.0) && start.1 != 0.0 {
            continue;
        }
        let t = if end.1 == start.1 {
            0.0
        } else {
            start.1 / (start.1 - end.1)
        };
        let s = start.0 + t * (end.0 - start.0);
        cut = Some(cut.map_or((s, s), |(from, to)| (from.min(s), to.max(s))));
    }
    cut
}

/// The shape's reach along the path, over the whole of its footprint.
fn extent_along(shape: &Shape) -> (f32, f32) {
    shape
        .footprint()
        .iter()
        .fold((f32::MAX, f32::MIN), |(from, to), &(s, _)| {
            (from.min(s), to.max(s))
        })
}

fn side_shapes(
    canvas: &mut Canvas,
    view: &Viewport,
    ground: &AuthoredGround,
    s: (f32, f32),
    z: (f32, f32),
) {
    let edge = format!(r#"stroke="{}" stroke-width="1""#, palette::SHAPE_EDGE);
    canvas.begin_clip("side", view.at(s.0, z.1), view.at(s.1, z.0));
    if let Some(base) = ground.base {
        canvas.polygon(
            view,
            [(s.0, base), (s.1, base), (s.1, z.0), (s.0, z.0)],
            &format!(r#"fill="{}" {edge}"#, palette::SHAPE_FILL),
        );
    }
    // Shapes off the path behind, drawn over their whole reach; shapes on it
    // in front, drawn where the path line cuts them.
    let (crossing, off_path): (Vec<_>, Vec<_>) = ground
        .shapes
        .iter()
        .map(|shape| (shape, cut_on_path(shape)))
        .partition(|(_, cut)| cut.is_some());
    for (shape, cut) in off_path.into_iter().chain(crossing) {
        let fill = if cut.is_some() {
            palette::SHAPE_FILL
        } else {
            palette::SHAPE_OFF_PATH_FILL
        };
        let style = format!(r#"fill="{fill}" {edge}"#);
        let (from, to) = cut.unwrap_or_else(|| extent_along(shape));
        let face = shape.face();
        let (start, end) = (face.at(from, 0.0), face.at(to, 0.0));
        let solid = if shape.is_roof() { z.1 } else { z.0 };
        canvas.polygon(
            view,
            [(from, start), (to, end), (to, solid), (from, solid)],
            &style,
        );
    }
    canvas.end_clip();
}

/// Every floor's footprint, cut to the strip.
fn top_shapes(
    canvas: &mut Canvas,
    view: &Viewport,
    ground: &AuthoredGround,
    s: (f32, f32),
    d_half: f32,
) {
    let style = format!(
        r#"fill="{}" stroke="{}" stroke-width="1""#,
        palette::SHAPE_FILL,
        palette::SHAPE_EDGE
    );
    canvas.begin_clip("strip", view.at(s.0, d_half), view.at(s.1, -d_half));
    for shape in ground.shapes.iter().filter(|shape| !shape.is_roof()) {
        canvas.polygon(view, shape.footprint(), &style);
    }
    canvas.end_clip();
}

/// Every lane's footholds where a foot touches them, the path's own lane
/// drawn strongest.
fn side_footholds(canvas: &mut Canvas, view: &Viewport, section: &Section) {
    let mut lanes: Vec<_> = section.foothold_lanes.iter().collect();
    lanes.sort_by(|a, b| b.d.abs().total_cmp(&a.d.abs()));
    for lane in lanes {
        let (width, opacity) = if lane.d == 0.0 {
            (2.5, 1.0)
        } else {
            (1.0, 0.45)
        };
        for surface in &lane.profile.surfaces {
            canvas.polyline(
                view,
                surface
                    .points
                    .iter()
                    .map(|point| contact(point, section.foot_radius)),
                &format!(
                    r#"stroke="{}" stroke-width="{width}" stroke-opacity="{opacity}""#,
                    palette::footing(surface.footing)
                ),
            );
        }
    }
}

fn top_footholds(canvas: &mut Canvas, view: &Viewport, section: &Section) {
    let lane_style = format!(r#"stroke="{}" stroke-width="1""#, palette::GRID);
    for lane in &section.foothold_lanes {
        let from = view.at(view.s_min, lane.d);
        let to = (view.left + 1.0e4, from.1);
        canvas.pixel_line(from, to, &lane_style);
        for surface in &lane.profile.surfaces {
            let (Some(first), Some(last)) = (surface.points.first(), surface.points.last()) else {
                continue;
            };
            let from = contact(first, section.foot_radius).0;
            let to = contact(last, section.foot_radius).0;
            canvas.polyline(
                view,
                [(from, lane.d), (to, lane.d)],
                &format!(
                    r#"stroke="{}" stroke-width="5" stroke-linecap="butt""#,
                    palette::footing(surface.footing)
                ),
            );
        }
    }
}

/// The capsule at the body's state, in the side view.
fn capsule(canvas: &mut Canvas, view: &Viewport, scene: &TraversalScene) {
    let radius = scene.params.body.radius;
    let bottom = (scene.body.s, scene.body.z);
    let top = (scene.body.s, scene.body.z + scene.params.body.spine());
    let stroke = format!(r#"stroke="{}" stroke-width="1.5""#, palette::BODY);
    let outline = format!(r#"fill="none" {stroke}"#);
    canvas.circle(view, bottom, radius, &outline);
    canvas.circle(view, top, radius, &outline);
    canvas.polyline(
        view,
        [(bottom.0 - radius, bottom.1), (top.0 - radius, top.1)],
        &stroke,
    );
    canvas.polyline(
        view,
        [(bottom.0 + radius, bottom.1), (top.0 + radius, top.1)],
        &stroke,
    );
}

fn legend(canvas: &mut Canvas, width: f32, y: f32) {
    let entries = [
        (
            palette::footing(crate::section::Footing::Fixed),
            "foothold, fixed",
        ),
        (
            palette::footing(crate::section::Footing::Loose {
                mass: 0.0,
                resting: true,
            }),
            "foothold, loose at rest",
        ),
        (
            palette::footing(crate::section::Footing::Loose {
                mass: 0.0,
                resting: false,
            }),
            "foothold, loose moving",
        ),
        (palette::BODY_FLOOR, "body floor (capsule centre)"),
        (palette::CEILING, "ceiling (top sphere centre)"),
    ];
    let mut x = MARGIN;
    for (colour, name) in entries {
        canvas.pixel_line(
            (x, y - 4.0),
            (x + 18.0, y - 4.0),
            &format!(r#"stroke="{colour}" stroke-width="3""#),
        );
        canvas.text(
            x + 22.0,
            y,
            name,
            &format!(r#"font-size="11" fill="{}""#, palette::LABEL),
        );
        x += 30.0 + 7.0 * name.len() as f32;
        if x > width - MARGIN {
            break;
        }
    }
}
