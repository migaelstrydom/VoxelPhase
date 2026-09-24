//! Text cut into a surface: a maker's mark, a label, the stamp on a block of
//! metal.
//!
//! An engraving is laid over a baked texture rather than modelled, because
//! the cut is a fraction of a millimetre deep and geometry that fine would be
//! thousands of triangles to say what two channels of a texel already can.
//! The cut recolours the surface — darkened, for an etched mark that is matte
//! where the polish was, or a colour of its own, for a mark that grew an
//! oxide film — and lowers the relief height the texture carries in its
//! alpha, which the shader turns into a shaded wall on one side of each
//! stroke and a lit one on the other.
//!
//! ```text
//!   Engraving ──▶ lines ──▶ fontdue ──▶ coverage mask ─┐
//!   baked texture (RGBA) ──────────────────────────────┴──▶ cut() ──▶ RGBA
//!                                                               rgb = fill
//!                                                               alpha = height
//! ```

use fontdue::{Font, FontSettings};

use crate::core::error::{EngineError, EngineResult};
use crate::rendering::colour::Colour;
use crate::rendering::typeface::JETBRAINS_MONO;

/// How tall a capital is, as a fraction of the size type is set at. The
/// typeface's own proportion, used to centre a line on its capitals rather
/// than on its descenders.
const CAP_HEIGHT: f32 = 0.73;

/// One line of engraved text.
#[derive(Clone, Debug)]
pub struct EngravedLine {
    pub text: String,
    /// Centre of the line's capitals, in texture coordinates: `(0, 0)` is the
    /// texture's first texel, `v` runs down its rows.
    pub centre: (f32, f32),
    /// Size the type is set at, as a fraction of the texture's width.
    pub size: f32,
    /// Which way the line is set about `centre`.
    pub align: Align,
}

/// Where a line sits relative to its `centre` horizontally.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Align {
    Left,
    Centre,
}

/// Lines of text to cut into a texture, and how deep and how dark the cut is.
#[derive(Clone, Debug)]
pub struct Engraving {
    /// Names what is engraved, for the texture cache: two engravings with the
    /// same id over the same bake must cut the same texture.
    pub id: &'static str,
    pub lines: Vec<EngravedLine>,
    /// Depth of the cut, in texture widths. What the material's relief is set
    /// to, since the alpha carries height across exactly this range.
    pub depth: f32,
    /// What colour the bottom of the cut is.
    pub fill: CutFill,
}

/// What colour the bottom of a cut is.
#[derive(Clone, Copy, Debug)]
pub enum CutFill {
    /// The surface's own colour, multiplied by this: an etched mark.
    Darkened(f32),
    /// A colour of its own, whatever the surface was: a mark that grew an
    /// oxide film, or was filled with paint.
    Coloured(Colour),
}

impl CutFill {
    /// The colour at the bottom of a cut into a surface of colour `surface`.
    fn bottom(self, surface: [f32; 3]) -> [f32; 3] {
        match self {
            CutFill::Darkened(shade) => surface.map(|channel| channel * shade),
            CutFill::Coloured(colour) => [colour.r, colour.g, colour.b],
        }
    }
}

impl Engraving {
    /// Cut this engraving into a square RGBA8 texture `size` texels across.
    ///
    /// The texture's alpha must be height (`1` at the surface), which it is
    /// for any bake whose pattern carries no relief of its own.
    pub fn cut(&self, pixels: &mut [u8], size: u32) -> EngineResult<()> {
        let mask = self.mask(size)?;
        for (texel, coverage) in pixels.chunks_exact_mut(4).zip(mask) {
            if coverage <= 0.0 {
                continue;
            }
            let surface = [0, 1, 2].map(|channel| texel[channel] as f32 / 255.0);
            let bottom = self.fill.bottom(surface);
            for channel in 0..3 {
                let colour = surface[channel] + (bottom[channel] - surface[channel]) * coverage;
                texel[channel] = (colour.clamp(0.0, 1.0) * 255.0).round() as u8;
            }
            let height = texel[3] as f32 / 255.0;
            texel[3] = (height.min(1.0 - coverage) * 255.0).round() as u8;
        }
        Ok(())
    }

    /// How much of each texel the engraving cuts away, in `[0, 1]`.
    pub fn mask(&self, size: u32) -> EngineResult<Vec<f32>> {
        let font = Font::from_bytes(JETBRAINS_MONO, FontSettings::default())
            .map_err(|e| EngineError::InvalidState(format!("Failed to load font: {e}")))?;
        let mut mask = vec![0.0; (size * size) as usize];
        for line in &self.lines {
            set_line(&font, line, size, &mut mask);
        }
        Ok(mask)
    }
}

/// Rasterise one line into `mask`, keeping the deeper cut where strokes meet.
fn set_line(font: &Font, line: &EngravedLine, size: u32, mask: &mut [f32]) {
    let px = line.size * size as f32;
    let glyphs: Vec<_> = line.text.chars().map(|ch| font.rasterize(ch, px)).collect();
    let width: f32 = glyphs
        .iter()
        .map(|(metrics, _)| metrics.advance_width)
        .sum();

    let mut pen = match line.align {
        Align::Left => line.centre.0 * size as f32,
        Align::Centre => line.centre.0 * size as f32 - width * 0.5,
    };
    let baseline = line.centre.1 * size as f32 + px * CAP_HEIGHT * 0.5;

    for (metrics, bitmap) in &glyphs {
        let left = (pen + metrics.xmin as f32).round() as i64;
        let top = (baseline - metrics.ymin as f32 - metrics.height as f32).round() as i64;
        for row in 0..metrics.height {
            for col in 0..metrics.width {
                let (x, y) = (left + col as i64, top + row as i64);
                if x < 0 || y < 0 || x >= size as i64 || y >= size as i64 {
                    continue;
                }
                let coverage = bitmap[row * metrics.width + col] as f32 / 255.0;
                let texel = &mut mask[(y as u32 * size + x as u32) as usize];
                *texel = texel.max(coverage);
            }
        }
        pen += metrics.advance_width;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIZE: u32 = 128;

    fn engraving(text: &str, align: Align) -> Engraving {
        Engraving {
            id: "test",
            lines: vec![EngravedLine {
                text: text.to_string(),
                centre: (0.5, 0.5),
                size: 0.3,
                align,
            }],
            depth: 0.002,
            fill: CutFill::Darkened(0.4),
        }
    }

    /// The column range and row range the mask marks, as fractions of the
    /// texture.
    fn extent(mask: &[f32]) -> ((f32, f32), (f32, f32)) {
        let marked: Vec<(u32, u32)> = (0..SIZE * SIZE)
            .filter(|i| mask[*i as usize] > 0.5)
            .map(|i| (i % SIZE, i / SIZE))
            .collect();
        let span = |values: Vec<u32>| {
            let lo = *values.iter().min().unwrap() as f32 / SIZE as f32;
            let hi = *values.iter().max().unwrap() as f32 / SIZE as f32;
            (lo, hi)
        };
        (
            span(marked.iter().map(|m| m.0).collect()),
            span(marked.iter().map(|m| m.1).collect()),
        )
    }

    /// A centred capital sits on its centre both ways: the line is centred on
    /// its capitals, not on the box a descender would need.
    /// A coloured fill replaces the surface's colour at the bottom of a
    /// stroke outright, whatever the surface was.
    #[test]
    fn a_coloured_fill_ignores_the_surface() {
        let blue = Colour::new(0.2, 0.4, 0.9, 1.0);
        for surface in [[1.0, 1.0, 1.0], [0.1, 0.5, 0.3]] {
            assert_eq!(CutFill::Coloured(blue).bottom(surface), [0.2, 0.4, 0.9]);
        }
    }

    #[test]
    fn a_centred_line_is_centred_on_its_capitals() {
        let mask = engraving("W", Align::Centre).mask(SIZE).unwrap();
        let ((left, right), (top, bottom)) = extent(&mask);
        assert!(((left + right) * 0.5 - 0.5).abs() < 0.03, "{left}..{right}");
        assert!(((top + bottom) * 0.5 - 0.5).abs() < 0.03, "{top}..{bottom}");
    }

    #[test]
    fn a_left_aligned_line_starts_at_its_anchor() {
        let mask = engraving("74", Align::Left).mask(SIZE).unwrap();
        let ((left, _), _) = extent(&mask);
        assert!((left - 0.5).abs() < 0.04, "starts at {left}");
    }

    /// The cut lowers the height and darkens the colour where it lands, and
    /// leaves the rest of the surface exactly as it was.
    #[test]
    fn the_cut_marks_only_where_the_text_is() {
        let text = engraving("Au", Align::Centre);
        let mask = text.mask(SIZE).unwrap();
        let mut pixels = vec![200u8; (SIZE * SIZE * 4) as usize];
        for texel in pixels.chunks_exact_mut(4) {
            texel[3] = 255;
        }
        text.cut(&mut pixels, SIZE).unwrap();

        for (texel, coverage) in pixels.chunks_exact(4).zip(&mask) {
            if *coverage == 0.0 {
                assert_eq!(texel, [200, 200, 200, 255]);
            } else if *coverage == 1.0 {
                assert_eq!(texel[3], 0);
                assert_eq!(texel[0], (200.0 * 0.4_f32).round() as u8);
            }
        }
        assert!(
            mask.iter().any(|c| *c == 1.0),
            "the strokes are cut in full"
        );
    }
}
