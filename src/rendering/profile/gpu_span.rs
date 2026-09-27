/// One timed stretch of a frame on the GPU.
///
/// Spans are bounded at render-pass edges and never inside a pass. A tiled GPU
/// — every Apple GPU, under MoltenVK — runs a pass's fragment work for all its
/// draws together, tile by tile, so a timestamp between two draws of one pass
/// measures nothing. `Scene` therefore covers opaque geometry, the sorted
/// blended surfaces and particles alike, up to the water; `Water` covers the
/// refraction copy, the water and everything blended in front of it.
///
/// ```text
///   shadow cb:  [Shadow]
///   probe cb:   [Probes: reflection probe faces and their mips]
///   draw cb:    [FireSim] [Scene: sky, opaque, blended beyond the water]
///               [Water: copy, water, blended in front of it] [Resolve]
///               [Composite: fire, overlay] [Bloom]
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuSpan {
    Shadow,
    Probes,
    FireSim,
    Scene,
    /// The refraction copy and the scene pass it resumes: the water and
    /// what is blended in front of it. Absent from a frame with no water.
    Water,
    Resolve,
    Composite,
    Bloom,
}

impl GpuSpan {
    pub const COUNT: usize = 8;

    /// Every span, in the order the GPU runs them.
    pub const ALL: [GpuSpan; Self::COUNT] = [
        GpuSpan::Shadow,
        GpuSpan::Probes,
        GpuSpan::FireSim,
        GpuSpan::Scene,
        GpuSpan::Water,
        GpuSpan::Resolve,
        GpuSpan::Composite,
        GpuSpan::Bloom,
    ];

    pub fn label(self) -> &'static str {
        match self {
            GpuSpan::Shadow => "shadow",
            GpuSpan::Probes => "probes",
            GpuSpan::FireSim => "fire_sim",
            GpuSpan::Scene => "scene",
            GpuSpan::Water => "water",
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
