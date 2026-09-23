/// One timed stretch of a frame on the GPU.
///
/// Spans are bounded at render-pass edges and never inside a pass. A tiled GPU
/// — every Apple GPU, under MoltenVK — runs a pass's fragment work for all its
/// draws together, tile by tile, so a timestamp between two draws of one pass
/// measures nothing. `Scene` therefore covers opaque geometry, the sorted
/// blended surfaces and particles alike.
///
/// ```text
///   shadow cb:  [Shadow]
///   draw cb:    [FireSim] [Scene: sky, opaque, blended, particles] [Resolve]
///               [Composite: water, fire, overlay] [Bloom]
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuSpan {
    Shadow,
    FireSim,
    Scene,
    Resolve,
    Composite,
    Bloom,
}

impl GpuSpan {
    pub const COUNT: usize = 6;

    /// Every span, in the order the GPU runs them.
    pub const ALL: [GpuSpan; Self::COUNT] = [
        GpuSpan::Shadow,
        GpuSpan::FireSim,
        GpuSpan::Scene,
        GpuSpan::Resolve,
        GpuSpan::Composite,
        GpuSpan::Bloom,
    ];

    pub fn label(self) -> &'static str {
        match self {
            GpuSpan::Shadow => "shadow",
            GpuSpan::FireSim => "fire_sim",
            GpuSpan::Scene => "scene",
            GpuSpan::Resolve => "resolve",
            GpuSpan::Composite => "composite",
            GpuSpan::Bloom => "bloom",
        }
    }

    /// The query written where the span opens; the one after it closes it.
    pub(super) fn begin_query(self) -> u32 {
        self as u32 * 2
    }

    pub(super) fn end_query(self) -> u32 {
        self.begin_query() + 1
    }

    /// Queries a whole frame needs.
    pub(super) const QUERY_COUNT: u32 = Self::COUNT as u32 * 2;
}
