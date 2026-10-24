use std::fmt::Write;

/// A rectangle of the drawing that shows one view, mapping metres to pixels
/// at one scale on both axes, so slopes and gaps read at their true shape.
#[derive(Clone, Copy)]
pub struct Viewport {
    /// Pixel position of the viewport's top-left corner.
    pub left: f32,
    pub top: f32,
    /// Pixels per metre, on both axes.
    pub scale: f32,
    /// The metres shown at the viewport's left edge and top edge.
    pub s_min: f32,
    pub up_max: f32,
}

impl Viewport {
    /// Pixel position of `(s, up)`: `up` is `z` in the side view, `d` in the
    /// top view (the body's left at the top).
    pub fn at(&self, s: f32, up: f32) -> (f32, f32) {
        (
            self.left + (s - self.s_min) * self.scale,
            self.top + (self.up_max - up) * self.scale,
        )
    }
}

/// An SVG document built up element by element.
pub struct Canvas {
    body: String,
    width: f32,
    height: f32,
}

impl Canvas {
    pub fn new(width: f32, height: f32) -> Self {
        Self {
            body: String::new(),
            width,
            height,
        }
    }

    pub fn polyline(
        &mut self,
        view: &Viewport,
        points: impl IntoIterator<Item = (f32, f32)>,
        style: &str,
    ) {
        let mut coordinates = String::new();
        for (s, up) in points {
            let (x, y) = view.at(s, up);
            let _ = write!(coordinates, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(
            self.body,
            r#"<polyline points="{coordinates}" fill="none" {style}/>"#
        );
    }

    pub fn polygon(
        &mut self,
        view: &Viewport,
        points: impl IntoIterator<Item = (f32, f32)>,
        style: &str,
    ) {
        let mut coordinates = String::new();
        for (s, up) in points {
            let (x, y) = view.at(s, up);
            let _ = write!(coordinates, "{x:.1},{y:.1} ");
        }
        let _ = writeln!(self.body, r#"<polygon points="{coordinates}" {style}/>"#);
    }

    pub fn circle(&mut self, view: &Viewport, centre: (f32, f32), radius: f32, style: &str) {
        let (x, y) = view.at(centre.0, centre.1);
        let r = radius * view.scale;
        let _ = writeln!(
            self.body,
            r#"<circle cx="{x:.1}" cy="{y:.1}" r="{r:.1}" {style}/>"#
        );
    }

    /// Text at a pixel position.
    pub fn text(&mut self, x: f32, y: f32, text: &str, style: &str) {
        let _ = writeln!(
            self.body,
            r#"<text x="{x:.1}" y="{y:.1}" font-family="Helvetica, Arial, sans-serif" {style}>{text}</text>"#
        );
    }

    /// A line between pixel positions.
    pub fn pixel_line(&mut self, from: (f32, f32), to: (f32, f32), style: &str) {
        let _ = writeln!(
            self.body,
            r#"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" {style}/>"#,
            from.0, from.1, to.0, to.1
        );
    }

    /// Starts a group whose drawing is cut to the pixel rectangle from
    /// `top_left` to `bottom_right`; [`Canvas::end_clip`] closes it.
    pub fn begin_clip(&mut self, id: &str, top_left: (f32, f32), bottom_right: (f32, f32)) {
        let (x, y) = top_left;
        let (width, height) = (bottom_right.0 - x, bottom_right.1 - y);
        let _ = writeln!(
            self.body,
            r#"<clipPath id="{id}"><rect x="{x:.1}" y="{y:.1}" width="{width:.1}" height="{height:.1}"/></clipPath><g clip-path="url(#{id})">"#
        );
    }

    pub fn end_clip(&mut self) {
        let _ = writeln!(self.body, "</g>");
    }

    pub fn finish(self) -> String {
        format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\">\n\
             <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n{body}</svg>\n",
            w = self.width,
            h = self.height,
            body = self.body
        )
    }
}
