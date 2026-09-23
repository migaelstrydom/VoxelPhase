use specs::{Dispatcher, DispatcherBuilder, RunNow, System, SystemData};

use super::timed::Timed;

/// A `DispatcherBuilder` that wraps every system it is given in [`Timed`].
///
/// Mirrors the builder's own `with` and `with_thread_local`, so a dispatcher
/// is declared exactly as before and no system can be added untimed by
/// forgetting a wrapper.
pub struct TimedDispatcherBuilder<'a, 'b> {
    inner: DispatcherBuilder<'a, 'b>,
}

impl<'a, 'b> TimedDispatcherBuilder<'a, 'b> {
    pub fn new() -> Self {
        Self {
            inner: DispatcherBuilder::new(),
        }
    }

    pub fn with<S>(self, system: S, name: &'static str, dependencies: &[&str]) -> Self
    where
        S: for<'c> System<'c> + Send + 'a,
        for<'c> <S as System<'c>>::SystemData: SystemData<'c>,
    {
        Self {
            inner: self
                .inner
                .with(Timed::new(system, name), name, dependencies),
        }
    }

    /// Thread-local systems have no dispatcher name; `name` is only what the
    /// time is reported under.
    pub fn with_thread_local<S>(self, system: S, name: &'static str) -> Self
    where
        S: for<'c> System<'c> + 'b,
        for<'c> <S as System<'c>>::SystemData: SystemData<'c>,
        Timed<S>: for<'c> RunNow<'c>,
    {
        Self {
            inner: self.inner.with_thread_local(Timed::new(system, name)),
        }
    }

    pub fn build(self) -> Dispatcher<'a, 'b> {
        self.inner.build()
    }
}

impl Default for TimedDispatcherBuilder<'_, '_> {
    fn default() -> Self {
        Self::new()
    }
}
