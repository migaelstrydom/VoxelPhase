//! What a cube can be made of, and the element it is stamped with.
//!
//! A cube of a pure element is sold with its entry from the periodic table
//! engraved on one face: atomic number in the corner, symbol large in the
//! middle, name and standard atomic weight beneath.
//!
//! ```text
//!   ┌──────────────┐
//!   │ 74           │
//!   │      W       │
//!   │   Tungsten   │
//!   │    183.84    │
//!   └──────────────┘
//! ```

use serde::Deserialize;

use crate::rendering::colour::Colour;
use crate::rendering::engraving::{Align, CutFill, EngravedLine, Engraving};
use crate::rendering::substance::{self, Substance};

/// How deep the stamp is cut, in widths of the face. A few tenths of a
/// millimetre on a hand-sized cube: enough for the edges of each stroke to
/// catch the light, not enough to read as carved.
const STAMP_DEPTH: f32 = 0.002;

/// What the colour at the bottom of a stroke is multiplied by. An etched mark
/// is matte, and matte on polished metal reads dark.
const STAMP_SHADE: f32 = 0.62;

/// The colour a laser mark on titanium comes out: the heat grows an oxide
/// film a few tens of nanometres thick, and interference in it shows blue.
const ANODISED_BLUE: Colour = Colour::new(0.3, 0.62, 1.0, 1.0);

/// What a cube can be made of.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
pub enum Metal {
    #[default]
    Tungsten,
    Gold,
    Copper,
    Aluminium,
    Bismuth,
    Lithium,
    Osmium,
    Titanium,
}

/// An element's entry in the periodic table, as it is printed.
pub struct Element {
    pub symbol: &'static str,
    pub name: &'static str,
    pub atomic_number: u32,
    /// Standard atomic weight, to the precision it is conventionally quoted.
    pub atomic_weight: &'static str,
}

impl Metal {
    /// The one declaration of what the cube is made of: the collider takes the
    /// coefficients, the material takes the finish and the colour.
    pub fn substance(self) -> Substance {
        match self {
            Metal::Tungsten => substance::TUNGSTEN,
            Metal::Gold => substance::GOLD,
            Metal::Copper => substance::COPPER,
            Metal::Aluminium => substance::ALUMINIUM,
            Metal::Bismuth => substance::BISMUTH,
            Metal::Lithium => substance::LITHIUM,
            Metal::Osmium => substance::OSMIUM,
            Metal::Titanium => substance::TITANIUM,
        }
    }

    pub fn element(self) -> Element {
        match self {
            Metal::Tungsten => Element {
                symbol: "W",
                name: "Tungsten",
                atomic_number: 74,
                atomic_weight: "183.84",
            },
            Metal::Gold => Element {
                symbol: "Au",
                name: "Gold",
                atomic_number: 79,
                atomic_weight: "196.97",
            },
            Metal::Copper => Element {
                symbol: "Cu",
                name: "Copper",
                atomic_number: 29,
                atomic_weight: "63.546",
            },
            Metal::Aluminium => Element {
                symbol: "Al",
                name: "Aluminium",
                atomic_number: 13,
                atomic_weight: "26.982",
            },
            Metal::Bismuth => Element {
                symbol: "Bi",
                name: "Bismuth",
                atomic_number: 83,
                atomic_weight: "208.98",
            },
            Metal::Lithium => Element {
                symbol: "Li",
                name: "Lithium",
                atomic_number: 3,
                atomic_weight: "6.94",
            },
            Metal::Osmium => Element {
                symbol: "Os",
                name: "Osmium",
                atomic_number: 76,
                atomic_weight: "190.23",
            },
            Metal::Titanium => Element {
                symbol: "Ti",
                name: "Titanium",
                atomic_number: 22,
                atomic_weight: "47.867",
            },
        }
    }

    /// What colour the stamp comes out: etched dark on every metal but
    /// titanium, which the marking laser anodises.
    fn stamp_fill(self) -> CutFill {
        match self {
            Metal::Titanium => CutFill::Coloured(ANODISED_BLUE),
            _ => CutFill::Darkened(STAMP_SHADE),
        }
    }

    /// The element's entry, laid out to be engraved across one face.
    pub fn stamp(self) -> Engraving {
        let element = self.element();
        let line = |text: String, centre, size, align| EngravedLine {
            text,
            centre,
            size,
            align,
        };
        Engraving {
            id: element.symbol,
            lines: vec![
                line(
                    element.atomic_number.to_string(),
                    (0.1, 0.14),
                    0.14,
                    Align::Left,
                ),
                line(element.symbol.to_string(), (0.5, 0.44), 0.42, Align::Centre),
                line(element.name.to_string(), (0.5, 0.72), 0.11, Align::Centre),
                line(
                    element.atomic_weight.to_string(),
                    (0.5, 0.86),
                    0.11,
                    Align::Centre,
                ),
            ],
            depth: STAMP_DEPTH,
            fill: self.stamp_fill(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Metal; 8] = [
        Metal::Tungsten,
        Metal::Gold,
        Metal::Copper,
        Metal::Aluminium,
        Metal::Bismuth,
        Metal::Lithium,
        Metal::Osmium,
        Metal::Titanium,
    ];

    /// The texture cache tells stamps apart by id alone, so two metals must
    /// never share one.
    #[test]
    fn every_metal_has_its_own_stamp() {
        for (i, a) in ALL.iter().enumerate() {
            for b in &ALL[i + 1..] {
                assert_ne!(a.stamp().id, b.stamp().id);
            }
        }
    }

    /// Everything stamped stays on the face: nothing runs off an edge into
    /// the bevel, where the texture is not laid out for it.
    #[test]
    fn the_stamp_fits_inside_the_face() {
        const SIZE: u32 = 128;
        for metal in ALL {
            let mask = metal.stamp().mask(SIZE).unwrap();
            let margin = (SIZE as f32 * 0.04) as u32;
            for (i, coverage) in mask.iter().enumerate() {
                let (x, y) = (i as u32 % SIZE, i as u32 / SIZE);
                let near_edge =
                    x < margin || y < margin || x >= SIZE - margin || y >= SIZE - margin;
                assert!(
                    !(near_edge && *coverage > 0.0),
                    "{:?} stamps at ({x}, {y})",
                    metal
                );
            }
        }
    }
}
