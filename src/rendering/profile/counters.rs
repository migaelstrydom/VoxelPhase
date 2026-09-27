/// How much work a frame asked of the renderer.
///
/// Deterministic where the timings are not: two runs of the same scene give
/// the same counts, so a change meant to cut work can be checked against them
/// without the noise a stopwatch brings.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RenderCounters {
    /// Mesh draws recorded straight into the opaque pass.
    pub opaque_draws: u32,
    /// Mesh draws held back for the sorted blended flush. Each is recorded
    /// twice there, back faces then front.
    pub blended_draws: u32,
    /// Mesh draws composited after tonemapping (debug geometry).
    pub overlay_draws: u32,
    /// Triangles across every mesh draw, counted once however many times the
    /// draw is recorded.
    pub triangles: u64,
    /// Draws replayed into the sun shadow pass.
    pub shadow_casters: u32,
    /// Particles sorted, built and drawn.
    pub particles: u32,
    /// Reflection probes live this frame.
    pub probes: u32,
    /// Probe faces captured this frame.
    pub probe_faces: u32,
    /// Draws recorded into probe faces, summed over the faces.
    pub probe_draws: u32,
    /// Vertex and index bytes copied into a mesh buffer: streamed into the
    /// frame's, or uploaded into the resident arena because a model or the
    /// terrain was new or had changed.
    pub uploaded_mesh_bytes: u64,
    /// Times a mesh buffer ran out of room and was reallocated, carrying
    /// everything already written across.
    pub buffer_growths: u32,
}

impl RenderCounters {
    /// Count a mesh copied into the frame's buffers, and whether making room
    /// for it reallocated them.
    pub fn record_upload(&mut self, bytes: usize, grew: bool) {
        self.uploaded_mesh_bytes += bytes as u64;
        if grew {
            self.buffer_growths += 1;
        }
    }

    /// Every mesh draw, whichever pass it went to.
    pub fn mesh_draws(&self) -> u32 {
        self.opaque_draws + self.blended_draws + self.overlay_draws
    }
}
