/// Sequential stepper: generate contacts once, then substep N times.
///
/// This is the stepping pattern used by PGS and PGS+NGS solvers, where
/// contacts are generated once per frame and reused across multiple
/// velocity/position solve passes.

use crate::debug::DebugLines;
use crate::physics::impulses::PhysicsImpulse;
use crate::physics::static_geometry::StaticGeometry;
use crate::physics::world::PhysicsWorld;

use super::fixed_timestep::FixedTimestep;
use super::{StepResult, Stepper};

pub struct SequentialStepper {
    timestep: FixedTimestep,
}

impl SequentialStepper {
    pub fn new(fixed_dt: f32, max_substeps: u32) -> Self {
        Self {
            timestep: FixedTimestep::new(fixed_dt, max_substeps),
        }
    }
}

impl Stepper for SequentialStepper {
    fn step(
        &mut self,
        world: &mut PhysicsWorld,
        frame_dt: f32,
        static_geometry: &dyn StaticGeometry,
        impulses: &[PhysicsImpulse],
        debug_lines: &mut DebugLines,
    ) -> StepResult {
        let substeps = self.timestep.accumulate(frame_dt);
        if substeps == 0 {
            return StepResult { substeps: 0 };
        }

        let dt = self.timestep.fixed_dt();
        world.update_contacts(dt, static_geometry, impulses, debug_lines);
        for _ in 0..substeps {
            world.substep(dt, static_geometry);
        }

        StepResult { substeps }
    }

    fn fixed_dt(&self) -> f32 {
        self.timestep.fixed_dt()
    }
}
