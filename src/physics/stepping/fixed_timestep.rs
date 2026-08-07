/// Fixed-timestep accumulator for deterministic physics stepping.
///
/// Accumulates variable-rate frame time and drains it in fixed-size chunks,
/// clamped to a maximum substep budget. Reusable across different stepping
/// strategies.
pub struct FixedTimestep {
    fixed_dt: f32,
    max_substeps: u32,
    accumulator: f32,
}

impl FixedTimestep {
    pub fn new(fixed_dt: f32, max_substeps: u32) -> Self {
        Self {
            fixed_dt: fixed_dt.max(1e-5),
            max_substeps,
            accumulator: 0.0,
        }
    }

    /// Accumulate a frame's worth of time and return the number of fixed
    /// substeps to run this frame. The accumulator is clamped so that at
    /// most `max_substeps` are returned, preventing spiral-of-death when
    /// frame time spikes.
    pub fn accumulate(&mut self, frame_dt: f32) -> u32 {
        self.accumulator += frame_dt.max(0.0);

        let max_carry = self.fixed_dt * self.max_substeps as f32;
        if self.accumulator > max_carry {
            self.accumulator = max_carry;
        }

        if self.accumulator < self.fixed_dt {
            return 0;
        }

        let substeps = (self.accumulator / self.fixed_dt).floor() as u32;
        let substeps = substeps.min(self.max_substeps);
        self.accumulator -= substeps as f32 * self.fixed_dt;
        substeps
    }

    pub fn fixed_dt(&self) -> f32 {
        self.fixed_dt
    }
}
