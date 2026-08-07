use super::super::framework::{BenchRunResult, BenchSample};

// ═══════════════════════════════════════════════════════════════════════════
// Position assertions
// ═══════════════════════════════════════════════════════════════════════════

/// Assert that the final Y position is within `[expected - tolerance, expected + tolerance]`.
pub fn assert_final_y_near(run: &BenchRunResult, expected: f32, tolerance: f32) {
    let last = run.samples.last().expect("run should have samples");
    assert!(
        (last.y - expected).abs() <= tolerance,
        "final y={:.4} not near expected {expected:.4} (tol={tolerance})",
        last.y
    );
}

/// Assert that the final Y position is within the given range.
pub fn assert_final_y_in_range(run: &BenchRunResult, min_y: f32, max_y: f32) {
    let last = run.samples.last().expect("run should have samples");
    assert!(
        last.y > min_y && last.y < max_y,
        "final y={:.4} not in range ({min_y}, {max_y})",
        last.y
    );
}

/// Assert that the tracked body never dropped below `min_y` during the run.
pub fn assert_above_floor(run: &BenchRunResult, min_y: f32) {
    let actual_min = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    assert!(
        actual_min > min_y,
        "body fell below floor: min_y={actual_min:.4} (limit={min_y})"
    );
}

/// Assert the final X position is centered within `tolerance` of zero.
pub fn assert_stayed_centered_x(run: &BenchRunResult, tolerance: f32) {
    let last = run.samples.last().expect("run should have samples");
    assert!(
        last.x.abs() < tolerance,
        "body drifted from center: x={:.4} (tolerance={tolerance})",
        last.x
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Velocity / settling assertions
// ═══════════════════════════════════════════════════════════════════════════

/// Assert that the max linear and angular speeds in the last `tail_seconds`
/// are below the given thresholds.
pub fn assert_settled(run: &BenchRunResult, tail_seconds: f32, max_linear: f32, max_angular: f32) {
    let (tail_linear, tail_angular) = run.tail_max_speeds(tail_seconds);
    assert!(
        tail_linear < max_linear,
        "tail linear speed too high: {tail_linear:.6} (limit={max_linear})"
    );
    assert!(
        tail_angular < max_angular,
        "tail angular speed too high: {tail_angular:.6} (limit={max_angular})"
    );
}

/// Assert that the peak linear speed anywhere in the run exceeds the threshold.
pub fn assert_gained_speed(run: &BenchRunResult, min_peak_speed: f32) {
    let peak = run
        .samples
        .iter()
        .map(|s| s.linear_speed)
        .fold(0.0f32, f32::max);
    assert!(
        peak > min_peak_speed,
        "body should gain speed: peak={peak:.3} (min={min_peak_speed})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Contact assertions
// ═══════════════════════════════════════════════════════════════════════════

/// Assert that the final sample has at least `min_count` active contacts.
pub fn assert_resting_contacts(run: &BenchRunResult, min_count: usize) {
    let last = run.samples.last().expect("run should have samples");
    assert!(
        last.contact_count >= min_count,
        "expected resting contacts >= {min_count}, got {}",
        last.contact_count
    );
}

/// Assert that max contact depth in the final sample is within bounds.
pub fn assert_no_deep_penetration(run: &BenchRunResult, max_depth: f32) {
    let last = run.samples.last().expect("run should have samples");
    assert!(
        last.max_contact_depth <= max_depth,
        "unexpected deep penetration: depth={:.6} (limit={max_depth})",
        last.max_contact_depth
    );
}

/// Assert that after `skip_frames` initial samples, the contact count never
/// exceeds `max_contacts`.
pub fn assert_max_contacts_after_settle(
    run: &BenchRunResult,
    skip_frames: usize,
    max_contacts: usize,
) {
    let actual_max = run
        .samples
        .iter()
        .skip(skip_frames)
        .map(|s| s.contact_count)
        .max()
        .unwrap_or(0);
    assert!(
        actual_max <= max_contacts,
        "too many contacts after settling: max={actual_max} (limit={max_contacts})"
    );
}

/// Assert that in the tail window, the minimum contact count is at least
/// `min_count`.
pub fn assert_tail_min_contacts(run: &BenchRunResult, tail_seconds: f32, min_count: usize) {
    let last = run.samples.last().expect("run should have samples");
    let tail_start = (last.sim_time - tail_seconds).max(0.0);
    let min_contacts = run
        .samples
        .iter()
        .filter(|s| s.sim_time >= tail_start)
        .map(|s| s.contact_count)
        .min()
        .unwrap_or(0);
    assert!(
        min_contacts >= min_count,
        "tail min contacts={min_contacts} (required >= {min_count})"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Jitter analysis
// ═══════════════════════════════════════════════════════════════════════════

/// Results from analysing a trajectory for jitter.
#[derive(Debug)]
pub struct JitterReport {
    /// Number of frames where x moved backwards (dx < 0).
    pub x_reversals: usize,
    /// Number of frames where y moved upward (dy > 0).
    pub y_reversals: usize,
    /// Maximum frame-to-frame jerk in x (|d²x/dt²|).
    pub max_x_jerk: f32,
    /// Maximum frame-to-frame jerk in y.
    pub max_y_jerk: f32,
    /// Number of frames where contact count changed.
    pub contact_flips: usize,
    /// Total frames analysed.
    pub frames_analysed: usize,
}

/// Analyse a sequence of samples for trajectory jitter within a time window.
///
/// Only samples with `sim_time <= end_time` are considered. Detects position
/// reversals, acceleration spikes (jerk), and contact count flickering.
pub fn analyse_jitter(samples: &[BenchSample], fixed_dt: f32, end_time: f32) -> JitterReport {
    let window: Vec<&BenchSample> = samples.iter().filter(|s| s.sim_time <= end_time).collect();

    let mut x_reversals = 0usize;
    let mut y_reversals = 0usize;
    let mut contact_flips = 0usize;

    let mut velocities: Vec<(f32, f32)> = Vec::new();
    for w in window.windows(2) {
        let dx = w[1].x - w[0].x;
        let dy = w[1].y - w[0].y;
        let vx = dx / fixed_dt;
        let vy = dy / fixed_dt;
        velocities.push((vx, vy));

        if dx < -1e-6 {
            x_reversals += 1;
        }
        if dy > 1e-6 {
            y_reversals += 1;
        }
        if w[1].contact_count != w[0].contact_count {
            contact_flips += 1;
        }
    }

    let mut max_x_jerk = 0.0f32;
    let mut max_y_jerk = 0.0f32;
    for w in velocities.windows(2) {
        let jerk_x = ((w[1].0 - w[0].0) / fixed_dt).abs();
        let jerk_y = ((w[1].1 - w[0].1) / fixed_dt).abs();
        max_x_jerk = max_x_jerk.max(jerk_x);
        max_y_jerk = max_y_jerk.max(jerk_y);
    }

    JitterReport {
        x_reversals,
        y_reversals,
        max_x_jerk,
        max_y_jerk,
        contact_flips,
        frames_analysed: window.len(),
    }
}

/// Assert no vertical speed spikes after an initial settling period.
///
/// Computes vertical speed from consecutive y positions and checks that
/// the maximum after `skip_frames` is below `max_y_speed`.
pub fn assert_no_vertical_jitter(
    run: &BenchRunResult,
    fixed_dt: f32,
    skip_frames: usize,
    max_y_speed: f32,
) {
    let y_speeds: Vec<f32> = run
        .samples
        .windows(2)
        .map(|w| (w[1].y - w[0].y).abs() / fixed_dt)
        .collect();
    let max_after_settle = y_speeds
        .iter()
        .skip(skip_frames)
        .fold(0.0f32, |a, &b| a.max(b));
    assert!(
        max_after_settle < max_y_speed,
        "vertical speed spike (jitter): max={max_after_settle:.4} (limit={max_y_speed})"
    );
}

/// Assert that the Y position stays within bounds for all samples.
pub fn assert_y_in_range(run: &BenchRunResult, min_y: f32, max_y: f32) {
    let actual_min = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    let actual_max = run.samples.iter().map(|s| s.y).fold(0.0f32, f32::max);
    assert!(
        actual_min > min_y,
        "body dropped below range: min_y={actual_min:.4} (limit={min_y})"
    );
    assert!(
        actual_max < max_y,
        "body exceeded range: max_y={actual_max:.4} (limit={max_y})"
    );
}
