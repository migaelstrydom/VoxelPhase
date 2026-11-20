//! The papyrus: its recto, the side written on, and its verso.
//!
//! Papyrus is two layers of pith strips laid crosswise and pressed, so its
//! fibres run across the recto and down the verso. The recto carries two
//! columns of script in faded sepia, either side of where the figure is
//! laid in gold (`mesh.rs`): close to, it reads as a page of a book; from
//! across a level, as lines. The script is uncial capitals in an unbroken
//! run, as a Greek scribe wrote, but no word in it can be read: the game
//! is wordless (`docs/WORLD_AND_STORY.md`).

use std::f32::consts::TAU;

use super::super::shared::textures::{Rgb, TextureRng};
use super::mesh::{FIGURE_SIZE, SHEET_HEIGHT, SHEET_WIDTH};
use crate::utils::noise::{fbm_2d_periodic, perlin_2d_periodic_xy};

/// The recto's texture, the whole open sheet: wide enough that the script's
/// strokes are a few texels thick.
pub const RECTO_WIDTH: u32 = 1280;
pub const RECTO_HEIGHT: u32 = (RECTO_WIDTH as f32 * SHEET_HEIGHT / SHEET_WIDTH) as u32;

/// The verso's, tiled along the rolls.
pub const VERSO_SIZE: u32 = 512;

/// What the verso's texture spans, m, square: the sheet's height.
pub const VERSO_SPAN: f32 = SHEET_HEIGHT;

/// Papyrus, fresh and aged, and the fibres' darker and lighter streaks.
const PAPYRUS: Rgb = Rgb {
    r: 0.8,
    g: 0.67,
    b: 0.45,
};
const PAPYRUS_AGED: Rgb = Rgb {
    r: 0.62,
    g: 0.46,
    b: 0.27,
};
const FIBRE_DARK: Rgb = Rgb {
    r: 0.66,
    g: 0.52,
    b: 0.32,
};
const FIBRE_LIGHT: Rgb = Rgb {
    r: 0.88,
    g: 0.78,
    b: 0.58,
};

/// Carbon ink faded to sepia, and how much of the papyrus it hides: a
/// letter's share is drawn between the two, as the pen ran dry and was
/// dipped again.
const INK: Rgb = Rgb {
    r: 0.25,
    g: 0.15,
    b: 0.08,
};
const INK_COVER: (f32, f32) = (0.5, 0.85);

/// Margins round the script, m: from the sheet's top and bottom, from where
/// it runs into the rolls, and from the figure.
const MARGIN_EDGE: f32 = 0.024;
const MARGIN_ROLL: f32 = 0.022;
const MARGIN_FIGURE: f32 = 0.016;

/// A letter's height and width, the gap after it, the line's pitch, and
/// the pen's width, m.
const LETTER_HEIGHT: f32 = 0.0048;
const LETTER_WIDTH: f32 = 0.0039;
const LETTER_GAP: f32 = 0.0012;
const LINE_PITCH: f32 = 0.0092;
const PEN: f32 = 0.00068;

/// Where one sheet is glued over the next (a kollesis), m from the
/// recto's left edge, how wide the overlap is, how much darker, and how
/// far along its strips the next sheet's pattern is moved, m, so the two
/// are seen to be different sheets.
const KOLLESEIS: [f32; 2] = [0.064, 0.418];
const KOLLESIS_WIDTH: f32 = 0.007;
const KOLLESIS_SHADE: f32 = 0.28;
const KOLLESIS_SHIFT: f32 = 0.0113;

/// Where the pith strips lie, m: one every so far across the fibres, and
/// how wide the darker seam is where one overlaps the next.
const STRIP_PITCH: f32 = 0.024;
const STRIP_SEAM: f32 = 0.0012;

/// The script's letters: uncials, each the strokes of a pen in a unit box,
/// up `+y`.
const LETTERS: &[&[&[(f32, f32)]]] = &[
    // Alpha
    &[
        &[(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)],
        &[(0.22, 0.38), (0.78, 0.38)],
    ],
    // Gamma
    &[&[(0.2, 0.0), (0.2, 1.0), (0.9, 1.0)]],
    // Delta
    &[&[(0.0, 0.0), (0.5, 1.0), (1.0, 0.0), (0.0, 0.0)]],
    // Eta
    &[
        &[(0.1, 0.0), (0.1, 1.0)],
        &[(0.9, 0.0), (0.9, 1.0)],
        &[(0.1, 0.5), (0.9, 0.5)],
    ],
    // Iota
    &[&[(0.5, 0.0), (0.5, 1.0)]],
    // Kappa
    &[
        &[(0.15, 0.0), (0.15, 1.0)],
        &[(0.9, 1.0), (0.15, 0.45)],
        &[(0.38, 0.62), (0.9, 0.0)],
    ],
    // Lambda
    &[&[(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)]],
    // Mu
    &[&[
        (0.05, 0.0),
        (0.05, 1.0),
        (0.5, 0.35),
        (0.95, 1.0),
        (0.95, 0.0),
    ]],
    // Nu
    &[&[(0.1, 0.0), (0.1, 1.0), (0.9, 0.0), (0.9, 1.0)]],
    // Pi
    &[
        &[(0.15, 0.0), (0.15, 1.0)],
        &[(0.0, 1.0), (1.0, 1.0)],
        &[(0.85, 1.0), (0.85, 0.0)],
    ],
    // Tau
    &[&[(0.0, 1.0), (1.0, 1.0)], &[(0.5, 1.0), (0.5, 0.0)]],
    // Upsilon
    &[
        &[(0.0, 1.0), (0.5, 0.45), (1.0, 1.0)],
        &[(0.5, 0.45), (0.5, 0.0)],
    ],
];

/// The round letters, drawn as arcs: omicron, theta, lunate sigma and
/// epsilon. Each is the arc's start and end, as turns from `+x`, and
/// whether a bar crosses it.
const ROUND_LETTERS: &[(f32, f32, bool)] = &[
    (0.0, 1.0, false),
    (0.0, 1.0, true),
    (0.12, 0.88, false),
    (0.12, 0.88, true),
];

/// The recto: horizontal fibres, the strips they were cut in, age at the
/// edges, and two columns of script either side of the figure's square.
/// RGBA, sRGB.
pub fn recto_texture(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let surface = Surface::new(rng);
    let ink = script(rng);

    let mut pixels = Vec::with_capacity((RECTO_WIDTH * RECTO_HEIGHT * 4) as usize);
    for y in 0..RECTO_HEIGHT {
        for x in 0..RECTO_WIDTH {
            let along = (x as f32 + 0.5) / RECTO_WIDTH as f32 * SHEET_WIDTH;
            let down = (y as f32 + 0.5) / RECTO_HEIGHT as f32 * SHEET_HEIGHT;
            let edge = down.min(SHEET_HEIGHT - down);
            let sheet = KOLLESEIS.iter().filter(|&&k| along > k).count();
            let mut papyrus = surface.at(along, down + sheet as f32 * KOLLESIS_SHIFT, edge);
            let seam = KOLLESEIS
                .iter()
                .map(|&k| (along - k).abs())
                .fold(f32::MAX, f32::min);
            if seam < 0.5 * KOLLESIS_WIDTH {
                let depth = 1.0 - seam / (0.5 * KOLLESIS_WIDTH);
                papyrus = papyrus.lerp(FIBRE_DARK, KOLLESIS_SHADE * depth.sqrt());
            }
            let cover = ink[(y * RECTO_WIDTH + x) as usize];
            papyrus.lerp(INK, cover).write_rgba(&mut pixels);
        }
    }
    pixels
}

/// The verso: the same papyrus with its fibres running down, unwritten.
/// Tiles, since it is wound round the rolls. RGBA, sRGB.
pub fn verso_texture(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let surface = Surface::new(rng).verso();

    let mut pixels = Vec::with_capacity((VERSO_SIZE * VERSO_SIZE * 4) as usize);
    for y in 0..VERSO_SIZE {
        for x in 0..VERSO_SIZE {
            let across = (x as f32 + 0.5) / VERSO_SIZE as f32 * VERSO_SPAN;
            let down = (y as f32 + 0.5) / VERSO_SIZE as f32 * VERSO_SPAN;
            surface.at(down, across, f32::MAX).write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Papyrus, as a function of where on the sheet, m: `along` the fibres and
/// `across` them, and how far from a cut edge.
struct Surface {
    fibre_seed: u32,
    cross_seed: u32,
    strip_seed: u32,
    mottle_seed: u32,
    /// Whether it must wrap at [`VERSO_SPAN`] both ways.
    tiles: bool,
    /// How strongly its fibres show.
    contrast: f32,
}

/// Fibres to a metre, across them and along: fine and long; and how
/// strongly they show, on the recto and on the verso. The verso's are
/// fainter, so the rolls read as wound sheet and not as grained wood. The
/// layer beneath, its fibres crossing these, shows through faintly.
const FIBRES_ACROSS: f32 = 900.0;
const FIBRES_ALONG: f32 = 55.0;
const FIBRE_CONTRAST: f32 = 0.38;
const VERSO_FIBRE_CONTRAST: f32 = 0.2;
const CROSS_CONTRAST: f32 = 0.12;
/// Patches of age to a metre.
const MOTTLE: f32 = 9.0;
/// How far in from a cut edge the papyrus has darkened, m.
const EDGE_AGE: f32 = 0.014;

impl Surface {
    fn new(rng: &mut TextureRng) -> Self {
        Self {
            fibre_seed: rng.u32(),
            cross_seed: rng.u32(),
            strip_seed: rng.u32(),
            mottle_seed: rng.u32(),
            tiles: false,
            contrast: FIBRE_CONTRAST,
        }
    }

    /// Cycles of a pattern of `per_metre` across [`VERSO_SPAN`], whole.
    fn period(&self, per_metre: f32) -> i32 {
        (per_metre * VERSO_SPAN).round().max(1.0) as i32
    }

    /// The pattern's cycles to a metre: as asked, or for a tiling surface,
    /// the nearest that fits [`VERSO_SPAN`] a whole number of times.
    fn density(&self, per_metre: f32) -> f32 {
        if self.tiles {
            self.period(per_metre) as f32 / VERSO_SPAN
        } else {
            per_metre
        }
    }

    /// As the verso: wrapping, since it is wound round the rolls, and its
    /// fibres fainter.
    fn verso(self) -> Self {
        Self {
            tiles: true,
            contrast: VERSO_FIBRE_CONTRAST,
            ..self
        }
    }

    fn at(&self, along: f32, across: f32, edge: f32) -> Rgb {
        let fibres = |along: f32, across: f32, seed: u32| {
            perlin_2d_periodic_xy(
                along * self.density(FIBRES_ALONG),
                across * self.density(FIBRES_ACROSS),
                seed,
                self.tiles
                    .then(|| (self.period(FIBRES_ALONG), self.period(FIBRES_ACROSS))),
            ) - 0.5
        };
        let fibre = fibres(along, across, self.fibre_seed);
        let cross = fibres(across, along, self.cross_seed);

        let pitch = 1.0 / self.density(1.0 / STRIP_PITCH);
        let strips = self.period(1.0 / STRIP_PITCH) as f32;
        let mut strip = (across / pitch).floor();
        let seam = (across - strip * pitch).min((strip + 1.0) * pitch - across);
        if self.tiles {
            strip = strip.rem_euclid(strips);
        }
        let strip_shade = perlin_2d_periodic_xy(strip * 0.71, 0.5, self.strip_seed, None) - 0.5;
        let mottle = fbm_2d_periodic(
            along * self.density(MOTTLE),
            across * self.density(MOTTLE),
            4,
            0.5,
            2.0,
            self.mottle_seed,
            self.tiles.then(|| self.period(MOTTLE)),
        );

        // Noise about its middle spans about a quarter either way.
        let streak = (fibre * self.contrast + cross * CROSS_CONTRAST) * 4.0;
        let mut colour = if streak > 0.0 {
            PAPYRUS.lerp(FIBRE_LIGHT, streak.min(1.0))
        } else {
            PAPYRUS.lerp(FIBRE_DARK, (-streak).min(1.0))
        };
        colour = colour.scale(1.0 + strip_shade * 0.1);
        if seam < STRIP_SEAM {
            colour = colour.lerp(FIBRE_DARK, 0.35 * (1.0 - seam / STRIP_SEAM));
        }
        let age = ((mottle - 0.45) * 2.2).clamp(0.0, 1.0) * 0.45
            + (1.0 - (edge / EDGE_AGE).min(1.0)).powi(2) * 0.6;
        colour.lerp(PAPYRUS_AGED, age.min(1.0))
    }
}

/// How much of each texel of the recto the script's ink covers, 0 to 1.
fn script(rng: &mut TextureRng) -> Vec<f32> {
    let mut ink = vec![0.0f32; (RECTO_WIDTH * RECTO_HEIGHT) as usize];
    let figure_left = 0.5 * (SHEET_WIDTH - FIGURE_SIZE);
    let columns = [
        (MARGIN_ROLL, figure_left - MARGIN_FIGURE),
        (
            SHEET_WIDTH - figure_left + MARGIN_FIGURE,
            SHEET_WIDTH - MARGIN_ROLL,
        ),
    ];
    let lines = ((SHEET_HEIGHT - 2.0 * MARGIN_EDGE - LETTER_HEIGHT) / LINE_PITCH) as usize + 1;
    for (left, right) in columns {
        for line in 0..lines {
            let base = MARGIN_EDGE + LETTER_HEIGHT + line as f32 * LINE_PITCH;
            // A paragraph's last line stops short.
            let end = if rng.pick(6) == 0 {
                left + (right - left) * rng.range(0.3, 0.8)
            } else {
                right
            };
            let mut x = left;
            while x + LETTER_WIDTH <= end {
                let slant = rng.range(-0.06, 0.06);
                let wobble = rng.range(-0.0003, 0.0003);
                let cover = rng.range(INK_COVER.0, INK_COVER.1);
                let letter = |p: (f32, f32)| {
                    (
                        x + (p.0 + slant * p.1) * LETTER_WIDTH,
                        base + wobble - p.1 * LETTER_HEIGHT,
                    )
                };
                let pick = rng.pick((LETTERS.len() + ROUND_LETTERS.len()) as u32) as usize;
                for stroke in letter_strokes(pick) {
                    let points: Vec<(f32, f32)> = stroke.into_iter().map(letter).collect();
                    for pair in points.windows(2) {
                        ink_segment(&mut ink, pair[0], pair[1], cover);
                    }
                }
                x += LETTER_WIDTH + LETTER_GAP;
            }
        }
    }
    ink
}

/// The strokes of letter `index`: one of [`LETTERS`], or past them one of
/// [`ROUND_LETTERS`].
fn letter_strokes(index: usize) -> Vec<Vec<(f32, f32)>> {
    if let Some(letter) = LETTERS.get(index) {
        return letter.iter().map(|stroke| stroke.to_vec()).collect();
    }
    let (from, to, bar) = ROUND_LETTERS[index - LETTERS.len()];
    let arc = (0..=12)
        .map(|i| {
            let a = TAU * (from + (to - from) * i as f32 / 12.0);
            (0.5 + 0.5 * a.cos(), 0.5 + 0.5 * a.sin())
        })
        .collect();
    let mut strokes = vec![arc];
    if bar {
        strokes.push(vec![(0.05, 0.5), (0.8, 0.5)]);
    }
    strokes
}

/// Lay a pen stroke from `a` to `b` (m on the sheet, `y` down) into `ink`,
/// hiding `cover` of the papyrus, its edge softened over a texel.
fn ink_segment(ink: &mut [f32], a: (f32, f32), b: (f32, f32), cover: f32) {
    let texel = SHEET_WIDTH / RECTO_WIDTH as f32;
    let to_texel = |m: f32| m / texel;
    let (ax, ay, bx, by) = (to_texel(a.0), to_texel(a.1), to_texel(b.0), to_texel(b.1));
    let half = to_texel(PEN) * 0.5;
    let reach = half + 1.0;
    let x0 = (ax.min(bx) - reach).floor().max(0.0) as u32;
    let x1 = (ax.max(bx) + reach).ceil().min(RECTO_WIDTH as f32 - 1.0) as u32;
    let y0 = (ay.min(by) - reach).floor().max(0.0) as u32;
    let y1 = (ay.max(by) + reach).ceil().min(RECTO_HEIGHT as f32 - 1.0) as u32;
    let (dx, dy) = (bx - ax, by - ay);
    let length2 = (dx * dx + dy * dy).max(1e-6);
    for y in y0..=y1 {
        for x in x0..=x1 {
            let (px, py) = (x as f32 + 0.5 - ax, y as f32 + 0.5 - ay);
            let t = ((px * dx + py * dy) / length2).clamp(0.0, 1.0);
            let (ex, ey) = (px - dx * t, py - dy * t);
            let distance = (ex * ex + ey * ey).sqrt();
            let hidden = (half + 0.5 - distance).clamp(0.0, 1.0) * cover;
            let cell = &mut ink[(y * RECTO_WIDTH + x) as usize];
            *cell = cell.max(hidden);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_textures_are_whole() {
        assert_eq!(
            recto_texture(7).len(),
            (RECTO_WIDTH * RECTO_HEIGHT * 4) as usize
        );
        assert_eq!(
            verso_texture(7).len(),
            (VERSO_SIZE * VERSO_SIZE * 4) as usize
        );
    }

    /// The script keeps out of the figure's square, so the gold lies on
    /// bare papyrus.
    #[test]
    fn the_script_leaves_the_figure_bare() {
        let ink = script(&mut TextureRng::new(3));
        let texel = SHEET_WIDTH / RECTO_WIDTH as f32;
        let left = ((0.5 * (SHEET_WIDTH - FIGURE_SIZE)) / texel) as u32;
        let right = ((0.5 * (SHEET_WIDTH + FIGURE_SIZE)) / texel) as u32;
        let inked = |x0: u32, x1: u32| {
            (0..RECTO_HEIGHT)
                .flat_map(|y| (x0..x1).map(move |x| (y * RECTO_WIDTH + x) as usize))
                .filter(|&i| ink[i] > 0.0)
                .count()
        };
        assert_eq!(inked(left, right), 0);
        assert!(inked(0, left) > 1000);
        assert!(inked(right, RECTO_WIDTH) > 1000);
    }
}
