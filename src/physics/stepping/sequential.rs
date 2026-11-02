/// Sequential stepper: generate contacts once, then substep N times.
///
/// This is the stepping pattern used by PGS and PGS+NGS solvers, where
/// contacts are generated once per frame and reused across multiple
/// velocity/position solve passes.
use std::borrow::Cow;

use crate::debug::DebugLines;
use crate::physics::force_provider::SubstepForceProvider;
use crate::physics::impulses::PhysicsImpulse;
use crate::physics::static_geometry::StaticGeometry;
use crate::physics::world::PhysicsWorld;

use super::fixed_timestep::FixedTimestep;
use super::{StepResult, Stepper};

pub struct SequentialStepper {
    timestep: FixedTimestep,
    /// One-shot impulses handed to a frame that ran no substep, held for the
    /// next frame that does. A frame shorter than the fixed step (slow
    /// motion, or more than 240 frames a second) steps nothing, and an
    /// impulse is delivered when contacts are generated, which only a
    /// stepping frame does.
    held_impulses: Vec<PhysicsImpulse>,
}

impl SequentialStepper {
    pub fn new(fixed_dt: f32, max_substeps: u32) -> Self {
        Self {
            timestep: FixedTimestep::new(fixed_dt, max_substeps),
            held_impulses: Vec::new(),
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
        force_providers: &[&dyn SubstepForceProvider],
        debug_lines: &mut DebugLines,
    ) -> StepResult {
        let substeps = self.timestep.accumulate(frame_dt);
        if substeps == 0 {
            self.held_impulses.extend_from_slice(impulses);
            return StepResult { substeps: 0 };
        }

        let dt = self.timestep.fixed_dt();
        let held = std::mem::take(&mut self.held_impulses);
        let impulses = if held.is_empty() {
            Cow::Borrowed(impulses)
        } else {
            Cow::Owned([held.as_slice(), impulses].concat())
        };
        world.update_contacts(dt, substeps, static_geometry, &impulses, debug_lines);
        for _ in 0..substeps {
            world.substep(dt, static_geometry, force_providers);
        }

        StepResult { substeps }
    }

    fn fixed_dt(&self) -> f32 {
        self.timestep.fixed_dt()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    const FIXED_DT: f32 = 1.0 / 240.0;

    /// An impulse handed to a frame too short to step is delivered by the
    /// next frame that does, not dropped.
    #[test]
    fn an_impulse_on_a_frame_without_a_substep_waits_for_the_next() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        world.attach_collider(body, ColliderDesc::sphere(0.5).density(1000.0));
        let blast = PhysicsImpulse::radial(Point3::new(-1.0, 0.0, 0.0), 5.0, 500.0, 0.0);

        let mut stepper = SequentialStepper::new(FIXED_DT, 12);
        let mut lines = DebugLines::default();
        let short = stepper.step(
            &mut world,
            FIXED_DT * 0.4,
            &EmptyGeometry,
            &[blast],
            &[],
            &mut lines,
        );
        assert_eq!(
            short.substeps, 0,
            "the first frame must be too short to step"
        );

        let long = stepper.step(
            &mut world,
            FIXED_DT * 0.8,
            &EmptyGeometry,
            &[],
            &[],
            &mut lines,
        );
        assert_eq!(long.substeps, 1);
        let vx = world.body(body).unwrap().linear_velocity().x;
        assert!(vx > 0.1, "the blast never reached the body: vx = {vx}");
    }
}
