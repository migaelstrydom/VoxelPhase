//! Debug overlay system for displaying runtime information.
//!
//! Provides ECS resources for accumulating debug key-value pairs (rendered as
//! text overlays), debug log entries (printed to stdout on F3), and debug shapes
//! (rendered in the 3D scene).

use std::collections::BTreeMap;

use nalgebra::Point3;

use crate::rendering::Colour;

/// Runtime toggles for debug instrumentation.
///
/// Each flag gates a piece of optional diagnostic work. Disabled flags should
/// skip their measurement entirely so the instrumentation doesn't pay any cost
/// in release builds where it isn't needed.
pub struct DebugConfig {
    /// If true, display the frame rate as `FPS` in the overlay.
    pub show_fps: bool,
    /// If true, measure per-frame CPU work (excluding the vsync acquire wait)
    /// and display it as `CPU ms` in the overlay.
    pub show_cpu_ms: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            show_fps: true,
            show_cpu_ms: false,
        }
    }
}

/// ECS resource for accumulating debug text to display.
///
/// Debug entries are stored as key-value pairs and sorted alphabetically
/// by key before rendering, ensuring stable line positions.
///
/// # Usage
/// ```ignore
/// // In a system:
/// fn my_system(mut debug: Write<DebugLines>) {
///     debug.add("FPS", format!("{:.0}", fps));
///     debug.add("Position", format!("{:.1}, {:.1}, {:.1}", x, y, z));
/// }
/// ```
///
/// The overlay renderer clears entries each frame after rendering.
#[derive(Default)]
pub struct DebugLines {
    entries: BTreeMap<String, String>,
}

impl DebugLines {
    /// Add or update a debug entry.
    ///
    /// If a key already exists, its value is replaced.
    /// Keys are sorted alphabetically when rendering.
    pub fn add(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.entries.insert(key.into(), value.into());
    }

    /// Iterate over entries in sorted key order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.entries.iter().map(|(k, v)| (k.as_str(), v.as_str()))
    }

    /// Clear all entries. Called after rendering each frame.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

/// ECS resource for accumulating debug logs to print to stdout.
///
/// Debug entries are stored as key-value pairs and sorted alphabetically
/// by key before printing, ensuring stable output order.
///
/// # Usage
/// ```ignore
/// // In a system:
/// fn my_system(mut debug: Write<DebugLog>) {
///     debug.add("ContactCount", format!("{}", count));
///     debug.add("SleepingBodies", format!("{}", sleeping));
/// }
///
/// // At end of frame, if F3 is pressed:
/// debug_log.print_and_clear(true);  // prints to stdout
/// // Or if F3 not pressed:
/// debug_log.print_and_clear(false); // just clears
/// ```
#[derive(Default)]
pub struct DebugLog {
    entries: BTreeMap<String, String>,
}

impl DebugLog {
    /// Add or update a debug log entry.
    ///
    /// If a key already exists, its value is replaced.
    /// Keys are sorted alphabetically when printing.
    pub fn add(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.entries.insert(key.into(), value.into());
    }

    /// Print all entries to stdout if `should_print` is true, then clear.
    ///
    /// This should be called at the end of each frame with `should_print`
    /// set based on whether F3 is pressed.
    pub fn print_and_clear(&mut self, should_print: bool) {
        if should_print && !self.entries.is_empty() {
            println!("\n=== Debug Log ===");
            for (key, value) in &self.entries {
                println!("{}: {}", key, value);
            }
            println!("=================\n");
        }
        self.entries.clear();
    }
}

/// A geometric primitive for debug visualisation.
///
/// Shapes are stored with minimal data at creation time. Tessellation into
/// renderable vertices is deferred to draw time, allowing frustum culling
/// to skip off-screen shapes before doing any mesh generation.
pub enum DebugShape {
    Sphere {
        center: Point3<f32>,
        radius: f32,
        colour: Colour,
    },
    Line {
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
        colour: Colour,
    },
    Triangle {
        vertices: [Point3<f32>; 3],
        colour: Colour,
    },
    Capsule {
        center: Point3<f32>,
        rotation: nalgebra::UnitQuaternion<f32>,
        half_height: f32,
        radius: f32,
        colour: Colour,
    },
}

impl DebugShape {
    /// Bounding sphere for frustum culling.
    ///
    /// Returns (centre, radius) enclosing the entire shape.
    pub fn bounding_sphere(&self) -> (Point3<f32>, f32) {
        match self {
            DebugShape::Sphere { center, radius, .. } => (*center, *radius),
            DebugShape::Line {
                start, end, radius, ..
            } => {
                let mid = nalgebra::center(start, end);
                let half_len = nalgebra::distance(start, end) * 0.5;
                (mid, half_len + radius)
            }
            DebugShape::Triangle { vertices, .. } => {
                let centroid = Point3::from(
                    (vertices[0].coords + vertices[1].coords + vertices[2].coords) / 3.0,
                );
                let max_dist = vertices
                    .iter()
                    .map(|v| nalgebra::distance(&centroid, v))
                    .fold(0.0f32, f32::max);
                (centroid, max_dist)
            }
            DebugShape::Capsule {
                center,
                half_height,
                ..
            } => (*center, *half_height),
        }
    }
}

/// Debug overlay shapes to render in 3D.
///
/// Shapes are split into opaque and transparent buckets, which are drawn
/// during the opaque and transparent render passes respectively. All shapes
/// are stored as lightweight [`DebugShape`] descriptors; mesh tessellation
/// is deferred to render time.
#[derive(Default)]
pub struct DebugOverlays {
    /// Shapes drawn during the opaque render pass.
    opaque_shapes: Vec<DebugShape>,
    /// Shapes drawn during the transparent render pass (alpha-blended).
    transparent_shapes: Vec<DebugShape>,
}

#[allow(dead_code)]
impl DebugOverlays {
    pub fn add_sphere(&mut self, position: Point3<f32>, radius: f32, colour: Colour) {
        self.transparent_shapes.push(DebugShape::Sphere {
            center: position,
            radius,
            colour,
        });
    }

    pub fn add_point(&mut self, position: Point3<f32>) {
        self.add_sphere(position, 0.06, Colour::RED);
    }

    pub fn add_line(&mut self, start: Point3<f32>, end: Point3<f32>, colour: Colour) {
        self.add_line_with_radius(start, end, 0.02, colour);
    }

    pub fn add_line_with_radius(
        &mut self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
        colour: Colour,
    ) {
        self.transparent_shapes.push(DebugShape::Line {
            start,
            end,
            radius,
            colour,
        });
    }

    /// Add a capsule overlay (Y-axis oriented, positioned and rotated in world space).
    pub fn add_capsule(
        &mut self,
        center: Point3<f32>,
        rotation: nalgebra::UnitQuaternion<f32>,
        half_height: f32,
        radius: f32,
        colour: Colour,
    ) {
        self.transparent_shapes.push(DebugShape::Capsule {
            center,
            rotation,
            half_height,
            radius,
            colour,
        });
    }

    /// Add a transparent triangle overlay.
    ///
    /// Triangles are rendered with alpha blending during the transparent pass,
    /// making them useful for visualising mesh patches, collision geometry, etc.
    pub fn add_triangle(&mut self, vertices: [Point3<f32>; 3], colour: Colour) {
        self.transparent_shapes
            .push(DebugShape::Triangle { vertices, colour });
    }

    /// Shapes to draw during the opaque render pass.
    pub fn opaque_shapes(&self) -> &[DebugShape] {
        &self.opaque_shapes
    }

    /// Shapes to draw during the transparent render pass.
    pub fn transparent_shapes(&self) -> &[DebugShape] {
        &self.transparent_shapes
    }

    pub fn clear(&mut self) {
        self.opaque_shapes.clear();
        self.transparent_shapes.clear();
    }
}
