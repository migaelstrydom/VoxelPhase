use std::time::Instant;

use specs::shred::RunningTime;
use specs::{Read, System, SystemData, World};

use super::timings::SystemTimings;

/// A system that times the one it wraps.
///
/// Adds only a shared read of [`SystemTimings`] to the inner system's data,
/// so wrapping changes nothing about what may run in parallel with what.
pub struct Timed<S> {
    inner: S,
    /// The dispatcher name, which is what the time is reported under.
    name: &'static str,
}

impl<S> Timed<S> {
    pub fn new(inner: S, name: &'static str) -> Self {
        Self { inner, name }
    }
}

impl<'a, S> System<'a> for Timed<S>
where
    S: System<'a>,
    S::SystemData: SystemData<'a>,
{
    type SystemData = (S::SystemData, Read<'a, SystemTimings>);

    fn run(&mut self, (data, timings): Self::SystemData) {
        let start = Instant::now();
        self.inner.run(data);
        timings.record(self.name, start.elapsed());
    }

    fn running_time(&self) -> RunningTime {
        self.inner.running_time()
    }

    fn setup(&mut self, world: &mut World) {
        <Read<'a, SystemTimings> as SystemData>::setup(world);
        self.inner.setup(world);
    }

    fn dispose(self, world: &mut World) {
        self.inner.dispose(world);
    }
}
