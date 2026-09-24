//! The sea: a store at a fixed level that never runs dry or fills (§12).
//!
//! Its region is the spans it owns in the span graph, flooded once at load
//! from the map's open edges. It never re-floods: at 50–100k spans that would
//! cost a frame's budget many times over. It grows only by absorbing a
//! lowland basin that has filled to sea level over a weir (§9.1, step 5).

/// The sea.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ocean {
    /// Sea level, world y.
    pub level: f32,
    /// Authored swell amplitude, m.
    pub swell: f32,
    /// Bumped whenever it claims more spans, so its mesh and mask rebuild.
    pub region_version: u32,
}
