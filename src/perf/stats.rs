use std::time::Duration;

/// A duration in milliseconds.
pub fn as_ms(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

pub fn mean_ms(times: &[Duration]) -> f64 {
    if times.is_empty() {
        return 0.0;
    }
    times.iter().map(|&t| as_ms(t)).sum::<f64>() / times.len() as f64
}

pub fn max_ms(times: &[Duration]) -> f64 {
    times.iter().map(|&t| as_ms(t)).fold(0.0, f64::max)
}

/// The `quantile` (0 to 1) of `times`, nearest rank.
pub fn percentile_ms(times: &[Duration], quantile: f64) -> f64 {
    if times.is_empty() {
        return 0.0;
    }
    let mut sorted = times.to_vec();
    sorted.sort_unstable();
    let index = ((sorted.len() - 1) as f64 * quantile).round() as usize;
    as_ms(sorted[index])
}

/// The median of `values`, upper of the two middle values for an even count.
///
/// # Panics
///
/// If `values` is empty.
pub fn median(values: impl Iterator<Item = Duration>) -> Duration {
    let mut values: Vec<Duration> = values.collect();
    values.sort_unstable();
    values[values.len() / 2]
}
