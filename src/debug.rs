//! Debug overlay system for displaying runtime information.
//!
//! Provides an ECS resource for accumulating debug key-value pairs that are
//! rendered as text overlays. Lines are sorted by key for stable positioning.

use std::collections::BTreeMap;

use nalgebra::Point3;

use crate::rendering::Colour;

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

/// Debug overlay shapes to render in 3D.
#[derive(Default)]
pub struct DebugOverlays {
    spheres: Vec<DebugSphere>,
    lines: Vec<DebugLine>,
}

#[allow(dead_code)]
impl DebugOverlays {
    pub fn add_sphere(&mut self, position: Point3<f32>, radius: f32, colour: Colour) {
        self.spheres.push(DebugSphere {
            position,
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
        self.lines.push(DebugLine {
            start,
            end,
            radius,
            colour,
        });
    }

    pub fn spheres(&self) -> &[DebugSphere] {
        &self.spheres
    }

    pub fn lines(&self) -> &[DebugLine] {
        &self.lines
    }

    pub fn clear(&mut self) {
        self.spheres.clear();
        self.lines.clear();
    }
}

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub struct DebugSphere {
    pub position: Point3<f32>,
    pub radius: f32,
    pub colour: Colour,
}

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub struct DebugLine {
    pub start: Point3<f32>,
    pub end: Point3<f32>,
    pub radius: f32,
    pub colour: Colour,
}
