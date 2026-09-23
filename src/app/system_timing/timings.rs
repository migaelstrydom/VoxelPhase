use std::sync::Mutex;
use std::time::Duration;

/// One system's time in one frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemTime {
    /// The name the system was registered with in the dispatcher.
    pub name: &'static str,
    pub elapsed: Duration,
}

/// The current frame's system times, as systems finish.
///
/// Written through a shared borrow, behind a lock, so that every timed system
/// only *reads* the resource. A write borrow would make each timed system
/// conflict with every other and serialise the whole parallel dispatch.
#[derive(Debug, Default)]
pub struct SystemTimings {
    /// Times recorded so far this frame, in the order systems finished.
    frame: Mutex<Vec<SystemTime>>,
}

impl SystemTimings {
    pub fn record(&self, name: &'static str, elapsed: Duration) {
        self.frame
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(SystemTime { name, elapsed });
    }

    /// The frame's times, slowest first, leaving the record empty for the
    /// next frame.
    pub fn take(&self) -> Vec<SystemTime> {
        let mut times = std::mem::take(
            &mut *self
                .frame
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner()),
        );
        times.sort_by(|a, b| b.elapsed.cmp(&a.elapsed));
        times
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn take_sorts_slowest_first_and_empties_the_frame() {
        let timings = SystemTimings::default();
        timings.record("fast", Duration::from_micros(10));
        timings.record("slow", Duration::from_millis(3));

        let taken = timings.take();
        assert_eq!(taken[0].name, "slow");
        assert_eq!(taken[1].name, "fast");
        assert!(timings.take().is_empty());
    }
}
