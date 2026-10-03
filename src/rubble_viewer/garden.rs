//! The stations of `levels/rubble_garden.level.ron`, the rubble play-test
//! level, as scenarios: what each station promises a player, held as a test.

use nalgebra::Point3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::driver::Run;
use super::scenario::{Blast, Scenario};
use super::scenarios::largest;

const GARDEN: &str = include_str!("../../levels/rubble_garden.level.ron");

/// Every garden scenario, in the order `--list` prints them.
pub fn scenarios() -> Vec<Scenario> {
    vec![
        tall_columns(),
        short_columns(),
        boundary_column(),
        table_one_leg(),
        table_every_leg(),
        hoodoo_stem(),
        island_chip(),
        stalactite_root(),
        short_bridge(),
        long_bridge(),
        lip_root(),
        lip_underside(),
        hill_free_for_all(),
        fins_and_walls(),
    ]
}

fn grenade(x: f32, y: f32, z: f32) -> Blast {
    Blast::grenade(Point3::new(x, y, z))
}

/// Passes when something of at least `min` samples fell.
fn something_fell(run: &Run, min: usize) -> Result<(), String> {
    match largest(run) {
        n if n >= min => Ok(()),
        n => Err(format!(
            "expected a fragment of at least {min} samples, largest {n}"
        )),
    }
}

/// Passes when nothing of `max` samples or more fell.
fn nothing_big_fell(run: &Run, max: usize) -> Result<(), String> {
    match largest(run) {
        n if n < max => Ok(()),
        n => Err(format!(
            "expected nothing of {max} samples or more to fall, a fragment of {n} did"
        )),
    }
}

fn tall_columns() -> Scenario {
    Scenario {
        name: "garden_tall_columns",
        description:
            "garden back row: a grenade at the foot of each 10 m column drops the thin ones whole",
        level: GARDEN,
        blasts: vec![
            grenade(15.9, 2.6, 84.0),
            grenade(22.4, 2.6, 84.0),
            grenade(27.9, 2.6, 84.0),
            grenade(32.2, 2.6, 84.0),
            grenade(35.9, 2.6, 84.0),
            grenade(39.1, 2.6, 84.0),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 80),
    }
}

fn short_columns() -> Scenario {
    Scenario {
        name: "garden_short_columns",
        description: "garden front row: a grenade at the foot of each 3.5 m column drops it",
        level: GARDEN,
        blasts: vec![
            grenade(29.2, 2.6, 77.0),
            grenade(33.4, 2.6, 77.0),
            grenade(37.1, 2.6, 77.0),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 15),
    }
}

fn boundary_column() -> Scenario {
    Scenario {
        name: "garden_boundary_column",
        description: "garden corner: a column against the segment's edge drops like any other",
        level: GARDEN,
        blasts: vec![grenade(1.4, 2.6, 95.0)],
        known_gap: None,
        expect: |run| something_fell(run, 8),
    }
}

fn table_one_leg() -> Scenario {
    Scenario {
        name: "garden_table_one_leg",
        description: "garden table: with one leg blown it stands on the other three",
        level: GARDEN,
        blasts: vec![grenade(23.6, 3.0, 39.6)],
        known_gap: None,
        expect: |run| nothing_big_fell(run, 50),
    }
}

fn table_every_leg() -> Scenario {
    Scenario {
        name: "garden_table_every_leg",
        description: "garden table: with every leg blown the slab falls",
        level: GARDEN,
        blasts: vec![
            grenade(23.6, 3.0, 39.6),
            grenade(28.4, 3.0, 39.6),
            grenade(23.6, 3.0, 44.4),
            grenade(28.4, 3.0, 44.4),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 500),
    }
}

fn hoodoo_stem() -> Scenario {
    Scenario {
        name: "garden_hoodoo_stem",
        description: "garden hoodoo: blowing the stem drops the cap",
        level: GARDEN,
        blasts: vec![grenade(14.6, 4.0, 48.0)],
        known_gap: None,
        expect: |run| something_fell(run, 150),
    }
}

fn island_chip() -> Scenario {
    Scenario {
        name: "garden_island_chip",
        description: "garden island: a grenade on its edge chips it, and it stays in the sky",
        level: GARDEN,
        blasts: vec![grenade(17.5, 14.2, 34.0)],
        known_gap: None,
        expect: |run| nothing_big_fell(run, 50),
    }
}

fn stalactite_root() -> Scenario {
    Scenario {
        name: "garden_stalactite_root",
        description: "garden pavilion: a grenade at the root of the thickest stalactite drops it",
        level: GARDEN,
        blasts: vec![grenade(22.2, 7.4, 59.0)],
        known_gap: Some(
            "open: the stalactite falls, but 11 more samples are left paper-thin than before",
        ),
        expect: |run| something_fell(run, 5),
    }
}

fn short_bridge() -> Scenario {
    Scenario {
        name: "garden_short_bridge",
        description: "garden bridges: the 6 m deck cut at both ends, two grenades each, falls",
        level: GARDEN,
        blasts: vec![
            grenade(8.8, 8.0, 8.0),
            grenade(8.8, 7.5, 8.0),
            grenade(13.2, 8.0, 8.0),
            grenade(13.2, 7.5, 8.0),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 50),
    }
}

fn long_bridge() -> Scenario {
    Scenario {
        name: "garden_long_bridge",
        description: "garden bridges: the 28 m deck cut at both ends, two grenades each, falls",
        level: GARDEN,
        blasts: vec![
            grenade(8.8, 8.0, 24.0),
            grenade(8.8, 7.5, 24.0),
            grenade(35.2, 8.0, 24.0),
            grenade(35.2, 7.5, 24.0),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 300),
    }
}

fn lip_root() -> Scenario {
    Scenario {
        name: "garden_lip_root",
        description: "garden cliff: grenades along the root of the 0.7 m lip drop it",
        level: GARDEN,
        blasts: vec![
            grenade(83.6, 12.0, 50.5),
            grenade(83.6, 12.0, 52.0),
            grenade(83.6, 12.0, 53.5),
        ],
        known_gap: None,
        expect: |run| something_fell(run, 10),
    }
}

fn lip_underside() -> Scenario {
    Scenario {
        name: "garden_lip_underside",
        description: "garden cliff: grenades under the thinnest lip, against the face",
        level: GARDEN,
        blasts: vec![grenade(83.5, 10.5, 51.0), grenade(83.5, 10.5, 53.0)],
        known_gap: None,
        expect: |_| Ok(()),
    }
}

/// Grenades thrown at random over the cave hill: the free-for-all.
fn hill_free_for_all() -> Scenario {
    let mut rng = StdRng::seed_from_u64(7);
    let blasts = (0..30)
        .map(|_| {
            let (x, z) = (rng.gen_range(54.0..70.0), rng.gen_range(50.0..66.0));
            let r = ((x - 62.0f32).powi(2) + (z - 58.0f32).powi(2)).sqrt();
            let surface = 2.0 + 4.5 * (1.0 + (std::f32::consts::PI * r / 11.0).cos());
            grenade(x, surface + rng.gen_range(-1.5..0.5), z)
        })
        .collect();
    Scenario {
        name: "garden_hill",
        description: "garden hill: 30 seeded grenades into the caves; nothing left may float",
        level: GARDEN,
        blasts,
        known_gap: Some("open: blast 18 leaves one more sample standing free than before"),
        expect: |_| Ok(()),
    }
}

fn fins_and_walls() -> Scenario {
    Scenario {
        name: "garden_fins_and_walls",
        description: "garden fins and slab walls: a grenade through the root of each",
        level: GARDEN,
        blasts: vec![
            grenade(54.0, 3.0, 33.0),
            grenade(58.0, 3.0, 33.0),
            grenade(62.0, 3.0, 33.0),
            grenade(66.0, 3.0, 33.0),
            grenade(62.0, 3.0, 11.2),
            grenade(62.0, 3.0, 16.7),
            grenade(62.0, 3.0, 21.5),
        ],
        known_gap: None,
        expect: |_| Ok(()),
    }
}
