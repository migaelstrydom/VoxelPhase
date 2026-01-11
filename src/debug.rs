//! Debug overlay system for displaying runtime information.
//!
//! Provides an ECS resource for accumulating debug key-value pairs that are
//! rendered as text overlays. Lines are sorted by key for stable positioning.

use std::collections::BTreeMap;

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
