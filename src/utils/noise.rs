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

/// Generate 3D value noise with optional periodicity.
fn noise_3d_periodic(x: f32, y: f32, z: f32, seed: u32, period: Option<i32>) -> f32 {
    let xi = x.floor() as i32;
    let yi = y.floor() as i32;
    let zi = z.floor() as i32;

    let xf = x - x.floor();
    let yf = y - y.floor();
    let zf = z - z.floor();

    let u = smoothstep(xf);
    let v = smoothstep(yf);
    let w = smoothstep(zf);

    // Get eight corner values
    let c000 = hash_3d_periodic(xi, yi, zi, seed, period);
    let c100 = hash_3d_periodic(xi + 1, yi, zi, seed, period);
    let c010 = hash_3d_periodic(xi, yi + 1, zi, seed, period);
    let c110 = hash_3d_periodic(xi + 1, yi + 1, zi, seed, period);
    let c001 = hash_3d_periodic(xi, yi, zi + 1, seed, period);
    let c101 = hash_3d_periodic(xi + 1, yi, zi + 1, seed, period);
    let c011 = hash_3d_periodic(xi, yi + 1, zi + 1, seed, period);
    let c111 = hash_3d_periodic(xi + 1, yi + 1, zi + 1, seed, period);

    // Trilinear interpolation
    let cx00 = lerp(c000, c100, u);
    let cx10 = lerp(c010, c110, u);
    let cx01 = lerp(c001, c101, u);
    let cx11 = lerp(c011, c111, u);
    let cxy0 = lerp(cx00, cx10, v);
    let cxy1 = lerp(cx01, cx11, v);
    lerp(cxy0, cxy1, w)
}

/// Fractional Brownian Motion in 3D.
pub fn fbm_3d(
    x: f32,
    y: f32,
    z: f32,
    octaves: u32,
    persistence: f32,
    lacunarity: f32,
    seed: u32,
) -> f32 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_value = 0.0;

    for i in 0..octaves {
        total += noise_3d_periodic(x * frequency, y * frequency, z * frequency, seed + i, None)
            * amplitude;
        max_value += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }

    total / max_value
}

/// Hash function for 3D coordinates with optional periodicity.
fn hash_3d_periodic(x: i32, y: i32, z: i32, seed: u32, period: Option<i32>) -> f32 {
    let (wx, wy, wz) = if let Some(p) = period {
        (x.rem_euclid(p), y.rem_euclid(p), z.rem_euclid(p))
    } else {
        (x, y, z)
    };

    let mut n = (wx as u32)
        .wrapping_mul(374761393)
        .wrapping_add((wy as u32).wrapping_mul(668265263))
        .wrapping_add((wz as u32).wrapping_mul(1440670171))
        .wrapping_add(seed);

    n = (n ^ (n >> 13)).wrapping_mul(1274126177);
    n = n ^ (n >> 16);

    (n & 0x7fffffff) as f32 / 0x7fffffff as f32
}

/// Hash function for 2D coordinates with optional periodicity.
fn hash_2d_periodic(x: i32, y: i32, seed: u32, period: Option<i32>) -> f32 {
    hash_2d_periodic_xy(x, y, seed, period.map(|p| (p, p)))
}

/// As [`hash_2d_periodic`], with a period per axis.
fn hash_2d_periodic_xy(x: i32, y: i32, seed: u32, period: Option<(i32, i32)>) -> f32 {
    let (wx, wy) = if let Some((px, py)) = period {
        (x.rem_euclid(px.max(1)), y.rem_euclid(py.max(1)))
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
    fn test_noise_3d_range() {
        for i in 0..100 {
            let x = i as f32 * 0.1;
            let y = i as f32 * 0.2;
            let z = i as f32 * 0.15;
            let n = noise_3d_periodic(x, y, z, 42, None);
            assert!(
                n >= 0.0 && n <= 1.0,
                "3D noise should be in [0, 1], got {}",
                n
            );
        }
    }

    #[test]
    fn test_fbm_3d_range() {
        let n = fbm_3d(5.5, 3.2, 1.7, 4, 0.5, 2.0, 123);
        assert!(
            n >= 0.0 && n <= 1.0,
            "fbm_3d should be in [0, 1], got {}",
            n
        );
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

/// Quintic fade (6t⁵ - 15t⁴ + 10t³).
///
/// Used instead of [`smoothstep`] wherever a field will be *differentiated*.
/// Smoothstep's own derivative is continuous but its second derivative is not,
/// so a normal map derived from it creases along every lattice line. Quintic is
/// flat to second order at both ends and leaves no such seam.
fn quintic(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Unit gradient vector for a lattice point, wrapped to `period` if given.
fn gradient_2d_periodic(x: i32, y: i32, seed: u32, period: Option<(i32, i32)>) -> (f32, f32) {
    let angle = hash_2d_periodic_xy(x, y, seed, period) * std::f32::consts::TAU;
    (angle.cos(), angle.sin())
}

/// Generate 2D *gradient* (Perlin) noise, periodic if a period is given.
///
/// Value noise interpolates a random value per lattice point, which forces
/// every lattice line to be an extremum of the field and makes its derivative
/// vanish there. That is invisible in the value itself and glaring in its
/// slope: a normal map built from value noise prints the lattice as
/// axis-aligned banding. Gradient noise interpolates a random *slope* per
/// lattice point and is zero-valued at the lattice instead, which leaves no
/// preferred direction for a derivative to pick up.
///
/// Returned in `[0, 1]` to match [`noise_2d_periodic`], so the two are
/// interchangeable at call sites.
pub fn perlin_2d_periodic(x: f32, y: f32, seed: u32, period: Option<i32>) -> f32 {
    perlin_2d_periodic_xy(x, y, seed, period.map(|p| (p, p)))
}

/// [`perlin_2d_periodic`] on a lattice that wraps at a different period in
/// each axis.
///
/// What a *stretched* field needs. Sampling one axis at a lower frequency than
/// the other is how anisotropy is produced, and a single square period then
/// cannot wrap both axes at the edge of the same tile: the best a common
/// period can do is divide both frequencies, which tiles the field several
/// times over within the texture and prints rows of repeated features.
pub fn perlin_2d_periodic_xy(x: f32, y: f32, seed: u32, period: Option<(i32, i32)>) -> f32 {
    let xi = x.floor();
    let yi = y.floor();
    let xf = x - xi;
    let yf = y - yi;
    let (xi, yi) = (xi as i32, yi as i32);

    let corner = |dx: i32, dy: i32| {
        let (gx, gy) = gradient_2d_periodic(xi + dx, yi + dy, seed, period);
        gx * (xf - dx as f32) + gy * (yf - dy as f32)
    };

    let u = quintic(xf);
    let v = quintic(yf);
    let bottom = lerp(corner(0, 0), corner(1, 0), u);
    let top = lerp(corner(0, 1), corner(1, 1), u);
    let value = lerp(bottom, top, v);

    // 2D gradient noise is bounded by ±√2/2; centre it on 0.5.
    (value * std::f32::consts::SQRT_2 * 0.5 + 0.5).clamp(0.0, 1.0)
}

/// Fractional Brownian Motion over [`perlin_2d_periodic`].
///
/// Mirrors [`fbm_2d_periodic`] exactly, including how the period scales with
/// each octave's frequency so that every octave tiles. Kept separate rather
/// than switching the existing function's noise source, because terrain
/// generation is built on that one and changing its field would reshape every
/// level.
pub fn fbm_perlin_2d_periodic(
    x: f32,
    y: f32,
    octaves: u32,
    persistence: f32,
    lacunarity: f32,
    seed: u32,
    period: Option<i32>,
) -> f32 {
    fbm_perlin_2d_periodic_xy(
        x,
        y,
        octaves,
        persistence,
        lacunarity,
        seed,
        period.map(|p| (p, p)),
    )
}

/// [`fbm_perlin_2d_periodic`] over [`perlin_2d_periodic_xy`], for a field
/// sampled at a different frequency in each axis.
pub fn fbm_perlin_2d_periodic_xy(
    x: f32,
    y: f32,
    octaves: u32,
    persistence: f32,
    lacunarity: f32,
    seed: u32,
    period: Option<(i32, i32)>,
) -> f32 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_value = 0.0;

    for i in 0..octaves {
        let octave_period = period.map(|(px, py)| {
            (
                (px as f32 * frequency) as i32,
                (py as f32 * frequency) as i32,
            )
        });
        total += perlin_2d_periodic_xy(x * frequency, y * frequency, seed + i, octave_period)
            * amplitude;
        max_value += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }

    total / max_value
}

#[cfg(test)]
mod gradient_noise_tests {
    use super::*;

    /// The whole reason this exists: it will be differentiated, so it must tile
    /// without a seam or a normal map built from it puts a ridge along every
    /// repeat.
    #[test]
    fn gradient_noise_tiles_at_its_period() {
        let period = 8;
        for step in 0..40 {
            let x = step as f32 * 0.2;
            let y = step as f32 * 0.13;
            let inside = perlin_2d_periodic(x, y, 7, Some(period));
            let wrapped = perlin_2d_periodic(x + period as f32, y, 7, Some(period));
            assert!(
                (inside - wrapped).abs() < 1e-5,
                "seam at ({x}, {y}): {inside} vs {wrapped}"
            );
        }
    }

    /// Gradient noise is zero at every lattice point by construction. A field
    /// that is *extremal* there instead — value noise — is what prints the
    /// lattice into a derived normal map.
    #[test]
    fn gradient_noise_is_neutral_on_the_lattice() {
        for x in 0..6 {
            for y in 0..6 {
                let at_lattice = perlin_2d_periodic(x as f32, y as f32, 3, None);
                assert!(
                    (at_lattice - 0.5).abs() < 1e-5,
                    "lattice point ({x}, {y}) is {at_lattice}, not neutral"
                );
            }
        }
    }

    /// Stays inside the unit range its callers assume, and actually uses it —
    /// a field collapsed near 0.5 would produce no relief at all.
    #[test]
    fn gradient_noise_spans_a_useful_part_of_the_unit_range() {
        let mut low: f32 = 1.0;
        let mut high: f32 = 0.0;
        for step in 0..4000 {
            let value = perlin_2d_periodic(step as f32 * 0.137, step as f32 * 0.081, 11, None);
            assert!((0.0..=1.0).contains(&value), "out of range: {value}");
            low = low.min(value);
            high = high.max(value);
        }
        assert!(high - low > 0.6, "only spans {low}..{high}");
    }
}

/// Unit-ish gradient for a 3D lattice point: one of the twelve cube-edge
/// directions, which is the classic set and has no axis bias.
fn gradient_3d(x: i32, y: i32, z: i32, seed: u32) -> (f32, f32, f32) {
    const EDGES: [(f32, f32, f32); 12] = [
        (1.0, 1.0, 0.0),
        (-1.0, 1.0, 0.0),
        (1.0, -1.0, 0.0),
        (-1.0, -1.0, 0.0),
        (1.0, 0.0, 1.0),
        (-1.0, 0.0, 1.0),
        (1.0, 0.0, -1.0),
        (-1.0, 0.0, -1.0),
        (0.0, 1.0, 1.0),
        (0.0, -1.0, 1.0),
        (0.0, 1.0, -1.0),
        (0.0, -1.0, -1.0),
    ];
    let pick = (hash_3d_periodic(x, y, z, seed, None) * 12.0) as usize;
    EDGES[pick.min(11)]
}

/// 3D gradient (Perlin) noise, **signed**: roughly `-1..1`, zero on average.
///
/// Signed rather than the `0..1` of the other functions here because the one
/// thing it is for — displacing a solid's surface and differentiating the
/// result into normals — wants a deviation, not a level. Gradient rather than
/// value noise for the reason [`perlin_2d_periodic`] gives: a surface built
/// from value noise shows its lattice in its shading.
pub fn perlin_3d(x: f32, y: f32, z: f32, seed: u32) -> f32 {
    let (xi, yi, zi) = (x.floor(), y.floor(), z.floor());
    let (xf, yf, zf) = (x - xi, y - yi, z - zi);
    let (xi, yi, zi) = (xi as i32, yi as i32, zi as i32);

    let corner = |dx: i32, dy: i32, dz: i32| {
        let (gx, gy, gz) = gradient_3d(xi + dx, yi + dy, zi + dz, seed);
        gx * (xf - dx as f32) + gy * (yf - dy as f32) + gz * (zf - dz as f32)
    };

    let (u, v, w) = (quintic(xf), quintic(yf), quintic(zf));
    let x00 = lerp(corner(0, 0, 0), corner(1, 0, 0), u);
    let x10 = lerp(corner(0, 1, 0), corner(1, 1, 0), u);
    let x01 = lerp(corner(0, 0, 1), corner(1, 0, 1), u);
    let x11 = lerp(corner(0, 1, 1), corner(1, 1, 1), u);
    lerp(lerp(x00, x10, v), lerp(x01, x11, v), w).clamp(-1.0, 1.0)
}

/// Fractional Brownian Motion over [`perlin_3d`], normalised so the result
/// stays in `-1..1`.
pub fn fbm_perlin_3d(x: f32, y: f32, z: f32, octaves: u32, persistence: f32, seed: u32) -> f32 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_value = 0.0;
    for i in 0..octaves {
        total += perlin_3d(
            x * frequency,
            y * frequency,
            z * frequency,
            seed.wrapping_add(i),
        ) * amplitude;
        max_value += amplitude;
        amplitude *= persistence;
        frequency *= 2.0;
    }
    total / max_value
}

#[cfg(test)]
mod perlin_3d_tests {
    use super::*;

    #[test]
    fn signed_gradient_noise_is_centred_on_zero_and_bounded() {
        let mut sum = 0.0;
        let mut count = 0;
        for i in 0..2000 {
            let t = i as f32;
            let n = fbm_perlin_3d(t * 0.137, t * 0.071 + 3.0, t * 0.193 - 1.0, 4, 0.5, 9);
            assert!((-1.0..=1.0).contains(&n), "{n}");
            sum += n;
            count += 1;
        }
        let mean = sum / count as f32;
        assert!(mean.abs() < 0.08, "mean {mean}");
    }

    /// Gradient noise is zero at every lattice point; that is what keeps its
    /// slope from printing the lattice.
    #[test]
    fn gradient_noise_vanishes_on_the_lattice() {
        for i in -3..3 {
            assert!(perlin_3d(i as f32, (i * 2) as f32, 5.0, 4).abs() < 1e-6);
        }
    }
}
