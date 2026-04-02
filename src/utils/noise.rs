/// Simple 2D Perlin-like noise generator for procedural textures.
///
/// Uses value noise with smoothstep interpolation for decent quality
/// and fast generation suitable for texture creation.

/// Generate 2D value noise with optional periodicity for tileable textures.
///
/// - `period`: If Some(p), the noise will repeat every p units, making it tileable.
fn noise_2d_periodic(x: f32, y: f32, seed: u32, period: Option<i32>) -> f32 {
    let xi = x.floor() as i32;
    let yi = y.floor() as i32;

    let xf = x - x.floor();
    let yf = y - y.floor();

    // Smoothstep interpolation
    let u = smoothstep(xf);
    let v = smoothstep(yf);

    // Get corner values with optional wrapping
    let c00 = hash_2d_periodic(xi, yi, seed, period);
    let c10 = hash_2d_periodic(xi + 1, yi, seed, period);
    let c01 = hash_2d_periodic(xi, yi + 1, seed, period);
    let c11 = hash_2d_periodic(xi + 1, yi + 1, seed, period);

    // Bilinear interpolation
    let cx0 = lerp(c00, c10, u);
    let cx1 = lerp(c01, c11, u);
    lerp(cx0, cx1, v)
}

/// Fractional Brownian Motion with optional periodicity for tileable textures.
///
/// - `period`: If Some(p), the noise will repeat every p units at the base frequency.
pub fn fbm_2d_periodic(
    x: f32,
    y: f32,
    octaves: u32,
    persistence: f32,
    lacunarity: f32,
    seed: u32,
    period: Option<i32>,
) -> f32 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_value = 0.0;

    for i in 0..octaves {
        // Scale period with frequency to maintain tiling at each octave
        let octave_period = period.map(|p| (p as f32 * frequency) as i32);
        total +=
            noise_2d_periodic(x * frequency, y * frequency, seed + i, octave_period) * amplitude;
        max_value += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }

    total / max_value
}

/// Hash function for 2D coordinates with optional periodicity.
fn hash_2d_periodic(x: i32, y: i32, seed: u32, period: Option<i32>) -> f32 {
    // Apply wrapping if period is specified
    let (wx, wy) = if let Some(p) = period {
        (x.rem_euclid(p), y.rem_euclid(p))
    } else {
        (x, y)
    };

    // Simple integer hash
    let mut n = (wx as u32)
        .wrapping_mul(374761393)
        .wrapping_add((wy as u32).wrapping_mul(668265263))
        .wrapping_add(seed);

    n = (n ^ (n >> 13)).wrapping_mul(1274126177);
    n = n ^ (n >> 16);

    (n & 0x7fffffff) as f32 / 0x7fffffff as f32
}

/// Smoothstep interpolation (3t² - 2t³).
fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

/// Linear interpolation.
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_noise_range() {
        for i in 0..100 {
            let x = i as f32 * 0.1;
            let y = i as f32 * 0.2;
            let n = noise_2d_periodic(x, y, 42, None);
            assert!(n >= 0.0 && n <= 1.0, "noise should be in [0, 1], got {}", n);
        }
    }

    #[test]
    fn test_fbm_range() {
        let n = fbm_2d_periodic(5.5, 3.2, 4, 0.5, 2.0, 123, None);
        assert!(n >= 0.0 && n <= 1.0, "fbm should be in [0, 1], got {}", n);
    }

    #[test]
    fn test_periodic_tiling() {
        let period = Some(10);
        let n1 = noise_2d_periodic(0.5, 0.5, 42, period);
        let n2 = noise_2d_periodic(10.5, 0.5, 42, period);
        assert!(
            (n1 - n2).abs() < 0.001,
            "periodic noise should tile, got {} and {}",
            n1,
            n2
        );
    }
}
