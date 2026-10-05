//! Runs a scenario: builds its terrain, sets off its blasts, audits after each.

use std::collections::HashSet;

use nalgebra::{Point3, Vector3};

use super::scenario::{Blast, Scenario};
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::physics::{PhysicsImpulse, PhysicsWorld, RigidBodyHandle, SequentialStepper, Stepper};
use crate::rubble::{
    Bricks, Cut, Flight, Outcome, Piece, Plan, RubblePlanner, ScreeRules, Settler, Settling,
    Verdict,
};
use crate::terrain::{LooseSamples, Occupancy, TerrainWorld, VoxelMaterial};

/// How deep a boulder's bricks may hold air in its holes and hollows, in
/// voxels. A brick's bevels skim the air at a concave corner by a few tenths
/// of a voxel; deeper is a gap the solver would stand things on.
const GAP_DEPTH: f32 = 0.5;
/// The frame rate scree is flown and boulders stepped at, as in the game.
const FRAME_SECONDS: f32 = 1.0 / 60.0;
/// The game's physics substep and its cap per frame.
const PHYSICS_DT: f32 = 1.0 / 240.0;
const MAX_SUBSTEPS: u32 = 12;
/// How long a blast's boulders are given to come to rest and be deposited. A
/// pile of thirty from the hill's shell takes 12 s to rest (E26).
const REST_SECONDS: f32 = 20.0;
/// How far below the terrain's bounds a boulder is counted as lost.
const LOST_MARGIN: f32 = 4.0;
/// How far a boulder may be from where free flight puts it after its first
/// frame, in metres. It starts clear of the ground by the bricks' inset, so
/// it touches nothing in that frame unless it started inside something and
/// the solver pushed it out, which moves it without adding energy.
const EJECTION_TOLERANCE: f32 = 0.005;

/// One piece of what a blast cut loose, as the report needs it: a fragment,
/// or a part of one too intricate to be one body.
#[derive(Debug, Clone, Copy)]
pub struct FragmentRecord {
    /// Which of the blast's fragments it is, or is a part of.
    pub source: usize,
    pub samples: usize,
    pub volume: f32,
    pub centroid: Point3<f32>,
    pub material: VoxelMaterial,
    pub fate: Fate,
    /// Samples drawn paper-thin in it, and in the whole fragment it came
    /// from: cracking may not make a piece thinner than its fragment was.
    pub paper_thin: usize,
    pub paper_thin_whole: usize,
    /// Whether it is a skin: no sample of it bears load.
    pub skin: bool,
    /// For a boulder, the air samples in its holes, gaps and hollows that
    /// its bricks hold: none, or the solver stands things on empty space.
    pub air_held: usize,
    /// How many pieces, joined face to face, its samples make: one, or a
    /// body moves things that only air joins.
    pub in_pieces: usize,
    /// For a boulder that came to rest, what the settler last made of it:
    /// `None` if it was never ready to be deposited.
    pub settled: Option<Verdict>,
}

/// What became of a fragment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fate {
    /// Crumbled where it broke.
    Dust,
    /// Fell as scree and crumbled where it landed, `frames` later, `drop`
    /// metres below where it broke.
    Landed { frames: usize, drop: f32 },
    /// Fell as scree and never landed: out of time, or out of the world.
    Expired { frames: usize },
    /// Became a boulder and went to sleep `frames` later, `drop` metres below
    /// where it broke. `ejected` if its first frame left it away from where
    /// free flight would have.
    Rested {
        frames: usize,
        drop: f32,
        ejected: bool,
    },
    /// Became a boulder and was still moving when time ran out.
    Restless { ejected: bool },
    /// Became a boulder and fell out of the world.
    Lost,
}

impl Fate {
    /// Whether it became a boulder.
    pub fn is_boulder(&self) -> bool {
        matches!(
            self,
            Fate::Rested { .. } | Fate::Restless { .. } | Fate::Lost
        )
    }
}

/// Where scree on `flight` comes down, against `terrain`.
fn fly(mut flight: Flight, terrain: &TerrainWorld) -> Fate {
    let rules = ScreeRules::default();
    let floor_y = terrain.bounds().min.y - rules.floor_margin;
    let start = flight.position;
    for frames in 1.. {
        match flight.step(FRAME_SECONDS, &rules, terrain, floor_y) {
            Outcome::Flying => {}
            Outcome::Landed(at) => {
                return Fate::Landed {
                    frames,
                    drop: start.y - at.y,
                };
            }
            Outcome::Expired => return Fate::Expired { frames },
        }
    }
    unreachable!("a flight ends within its lifetime")
}

/// A boulder being followed.
struct Followed {
    /// Index of its fragment in the blast.
    index: usize,
    body: RigidBodyHandle,
    start: Point3<f32>,
    /// The velocity the blast's shove gives it.
    thrown: Vector3<f32>,
    settling: Settling,
}

impl Followed {
    /// Where free flight puts it after `substeps` steps of `dt`, integrated
    /// as the solver does: velocity first, then position.
    fn flown(&self, substeps: u32, dt: f32, gravity: Vector3<f32>) -> Point3<f32> {
        let n = substeps as f32;
        self.start + self.thrown * (n * dt) + gravity * (dt * dt * n * (n + 1.0) / 2.0)
    }
}

/// Step one blast's boulders on `terrain` until each is deposited, would not
/// deposit, fell out of the world or ran out of time, the blast's `shove`
/// thrown on the first frame as the game throws it. Each is deposited as the
/// game's `SettleSystem` does it, and the terrain remeshed the same frame.
fn settle(
    world: &mut PhysicsWorld,
    followed: &mut [Followed],
    shove: PhysicsImpulse,
    terrain: &mut TerrainWorld,
) -> Vec<(usize, Fate, Option<Verdict>)> {
    let mut stepper = SequentialStepper::new(PHYSICS_DT, MAX_SUBSTEPS);
    let mut lines = DebugLines::default();
    let settler = Settler::default();
    let floor_y = terrain.bounds().min.y - LOST_MARGIN;
    let mut ejected = vec![false; followed.len()];
    let mut fates: Vec<Option<Fate>> = vec![None; followed.len()];
    let mut verdicts: Vec<Option<Verdict>> = vec![None; followed.len()];
    let gone = |fate: &Option<Fate>, verdict: &Option<Verdict>| {
        *fate == Some(Fate::Lost) || *verdict == Some(Verdict::Deposited)
    };
    let frames = (REST_SECONDS / FRAME_SECONDS) as usize;
    for frame in 1..=frames {
        let shoves = if frame == 1 { vec![shove] } else { Vec::new() };
        let substeps = stepper
            .step(world, FRAME_SECONDS, &*terrain, &shoves, &[], &mut lines)
            .substeps;
        let gravity = world.config().gravity;
        lines.clear();
        let mut ready = Vec::new();
        for (k, f) in followed.iter_mut().enumerate() {
            if gone(&fates[k], &verdicts[k]) {
                continue;
            }
            let Some(body) = world.body(f.body) else {
                fates[k] = Some(Fate::Lost);
                continue;
            };
            let at = body.position();
            if frame == 1
                && (at - f.flown(substeps, PHYSICS_DT, gravity)).norm() > EJECTION_TOLERANCE
            {
                ejected[k] = true;
            }
            if at.y < floor_y {
                fates[k] = Some(Fate::Lost);
                world.remove_body(f.body);
                continue;
            }
            let asleep = world.is_sleeping(f.body);
            if asleep && fates[k].is_none() {
                fates[k] = Some(Fate::Rested {
                    frames: frame,
                    drop: f.start.y - at.y,
                    ejected: ejected[k],
                });
            }
            if settler.ready(&mut f.settling, asleep, FRAME_SECONDS) {
                ready.push(k);
            }
        }

        let handles: Vec<RigidBodyHandle> = ready.iter().map(|&k| followed[k].body).collect();
        let mut deposited = false;
        for &k in &ready {
            let f = &mut followed[k];
            let verdict = settler.deposit(&mut f.settling, f.body, &handles, world, terrain);
            verdicts[k] = Some(verdict);
            if verdict == Verdict::Deposited {
                world.remove_body(f.body);
                deposited = true;
            }
        }
        if deposited {
            terrain.update();
        }

        let done = fates.iter().zip(&verdicts).all(|(fate, verdict)| {
            gone(fate, verdict) || (fate.is_some() && *verdict == Some(Verdict::Refused))
        });
        if done {
            break;
        }
    }
    followed
        .iter()
        .zip(fates.into_iter().zip(verdicts))
        .zip(ejected)
        .map(|((f, (fate, verdict)), ejected)| {
            (f.index, fate.unwrap_or(Fate::Restless { ejected }), verdict)
        })
        .collect()
}

/// What becomes of everything `cut` holds, played out against `terrain`:
/// one record a piece.
fn play(cut: Cut, planner: &mut RubblePlanner, terrain: &mut TerrainWorld) -> Vec<FragmentRecord> {
    let shove = cut.shove;
    let thin_whole: Vec<usize> = cut
        .fragments
        .iter()
        .map(|f| f.paper_thin_samples())
        .collect();
    let mut world = PhysicsWorld::default();
    let mut followed = Vec::new();
    let mut records: Vec<FragmentRecord> = planner
        .plan(cut)
        .into_iter()
        .enumerate()
        .map(|(index, piece)| {
            let Piece {
                fragment,
                source,
                plan,
            } = piece;
            let mut record = FragmentRecord {
                source,
                samples: fragment.sample_count(),
                volume: fragment.volume(),
                centroid: fragment.world_centroid(),
                material: fragment.material(),
                fate: Fate::Dust,
                paper_thin: fragment.paper_thin_samples(),
                paper_thin_whole: thin_whole[source],
                skin: fragment.bearing_samples() == 0,
                air_held: 0,
                in_pieces: 1,
                settled: None,
            };
            record.fate = match plan {
                Plan::Dust { .. } => Fate::Dust,
                Plan::Scree { flight, .. } => fly(flight, terrain),
                Plan::Boulder(boulder) => {
                    let occupancy = fragment.occupancy();
                    record.air_held = air_held_by(&boulder.bricks, &occupancy, GAP_DEPTH);
                    record.in_pieces = pieces_of(&occupancy);
                    let mass = boulder.mass;
                    let placed = boulder.place(&mut world);
                    let thrown = shove
                        .impulse_at(placed.centre)
                        .map_or(Vector3::zeros(), |j| j / mass);
                    followed.push(Followed {
                        index,
                        body: placed.body,
                        start: placed.centre,
                        thrown,
                        settling: Settling::new(fragment, placed.centre),
                    });
                    Fate::Lost
                }
            };
            record
        })
        .collect();
    for (index, fate, verdict) in settle(&mut world, &mut followed, shove, terrain) {
        records[index].fate = fate;
        records[index].settled = verdict;
    }
    records
}

/// How many of the air samples in a boulder's holes, gaps and hollows its
/// bricks hold deeper than `depth` voxels: empty space the solver treats as
/// rock (`Occupancy::encloses`). A notch in a corner, which any convex brick
/// fills, is not one.
fn air_held_by(bricks: &Bricks, occupancy: &Occupancy, depth: f32) -> usize {
    let [nx, ny, nz] = occupancy.dims();
    let depth = depth * occupancy.spacing();
    (0..nx)
        .flat_map(|x| (0..ny).flat_map(move |y| (0..nz).map(move |z| [x, y, z])))
        .filter(|&s| occupancy.encloses(s))
        .filter(|s| {
            let at = occupancy.world_position(Vector3::new(s[0] as f32, s[1] as f32, s[2] as f32));
            bricks.bricks.iter().any(|brick| {
                let local = bricks
                    .rotation
                    .inverse_transform_vector(&(at - brick.centre));
                brick.hull.faces.iter().all(|face| {
                    let on = brick.hull.vertices[face.vertex_indices[0] as usize];
                    face.normal.dot(&(local - on)) < -depth
                })
            })
        })
        .count()
}

/// How many pieces, joined face to face, the samples of `occupancy` make.
fn pieces_of(occupancy: &Occupancy) -> usize {
    let [nx, ny, nz] = occupancy.dims();
    let mut seen = HashSet::new();
    let mut pieces = 0;
    for start in (0..nx).flat_map(|x| (0..ny).flat_map(move |y| (0..nz).map(move |z| [x, y, z]))) {
        if !occupancy.is_solid(start) || !seen.insert(start) {
            continue;
        }
        pieces += 1;
        let mut stack = vec![start];
        while let Some(s) = stack.pop() {
            for axis in 0..3 {
                for n in [s[axis].wrapping_sub(1), s[axis] + 1] {
                    let mut next = s;
                    next[axis] = n;
                    if n < occupancy.dims()[axis] && occupancy.is_solid(next) && seen.insert(next) {
                        stack.push(next);
                    }
                }
            }
        }
    }
    pieces
}

/// The terrain as the audits find it.
#[derive(Debug, Clone, Copy)]
pub struct Audit {
    /// Solid standing free.
    pub loose: LooseSamples,
    /// Samples drawn paper-thin.
    pub paper_thin: usize,
}

impl Audit {
    fn of(terrain: &TerrainWorld) -> Self {
        Self {
            loose: terrain.loose_samples(),
            paper_thin: terrain.paper_thin_samples(),
        }
    }
}

/// What one blast did.
#[derive(Debug, Clone)]
pub struct BlastRecord {
    pub blast: Blast,
    pub fragments: Vec<FragmentRecord>,
    /// The terrain after the blast, before anything it cut loose came back.
    pub cut: Audit,
    /// The terrain once the blast's boulders were deposited.
    pub settled: Audit,
}

/// A scenario played out.
#[derive(Debug, Clone)]
pub struct Run {
    pub scenario: &'static str,
    /// The terrain as authored: floating islands, paper-thin strips.
    pub before: Audit,
    pub blasts: Vec<BlastRecord>,
    /// Open mesh edges once the terrain has remeshed after the last blast.
    pub open_edges: usize,
}

impl Run {
    /// The size in samples of every fragment cut loose, whole, before any
    /// was cut into parts.
    pub fn fragment_sizes(&self) -> Vec<usize> {
        self.blasts
            .iter()
            .flat_map(|blast| {
                let mut sizes: Vec<usize> = Vec::new();
                for piece in &blast.fragments {
                    if sizes.len() <= piece.source {
                        sizes.resize(piece.source + 1, 0);
                    }
                    sizes[piece.source] += piece.samples;
                }
                sizes
            })
            .collect()
    }

    /// Every piece cut loose, in order.
    pub fn fragments(&self) -> impl Iterator<Item = &FragmentRecord> {
        self.blasts.iter().flat_map(|b| b.fragments.iter())
    }

    /// Breaches of what every run is held to: no blast leaves more terrain
    /// standing free, or more drawn paper-thin, than there was before it, and
    /// its deposits leave no rock standing free; and
    /// every piece of scree falls clear of where it broke and lands, and every
    /// boulder comes to rest in the world without being thrown out of the
    /// ground, and is deposited. Scree that lands on its first frame started inside the
    /// ground, and scree that never lands fell through it.
    pub fn violations(&self) -> Vec<String> {
        let mut found = Vec::new();
        for (index, record) in self.blasts.iter().enumerate() {
            for fragment in &record.fragments {
                let problem = match fragment.fate {
                    // A skin pressed flat on the ground it broke from has
                    // nowhere to fall, and crumbles where it lies.
                    Fate::Landed { frames: 1, .. } if !fragment.skin => {
                        "scree landed on its first frame"
                    }
                    Fate::Expired { .. } => "scree never landed",
                    Fate::Rested { ejected: true, .. } | Fate::Restless { ejected: true } => {
                        "boulder was thrown out of the ground on its first frame"
                    }
                    Fate::Restless { .. } => "boulder never came to rest",
                    Fate::Lost => "boulder fell out of the world",
                    Fate::Rested { .. } if fragment.settled != Some(Verdict::Deposited) => {
                        "boulder came to rest and was never deposited"
                    }
                    _ if fragment.air_held > 0 => "boulder's bricks hold air",
                    _ if fragment.in_pieces > 1 => "boulder is pieces joined only by air",
                    _ => continue,
                };
                found.push(format!(
                    "blast {index}: {} samples at {:?}: {problem}",
                    fragment.samples, fragment.centroid
                ));
            }
            // The drawn pieces of one fragment between them draw no more of
            // it paper-thin than the whole fragment did.
            let mut thin: Vec<(usize, usize)> = Vec::new();
            for piece in record.fragments.iter().filter(|p| p.fate != Fate::Dust) {
                if thin.len() <= piece.source {
                    thin.resize(piece.source + 1, (0, 0));
                }
                thin[piece.source].0 += piece.paper_thin;
                thin[piece.source].1 = piece.paper_thin_whole;
            }
            for (source, (pieces, whole)) in thin.into_iter().enumerate() {
                if pieces > whole {
                    found.push(format!(
                        "blast {index}: fragment {source} cracked into pieces drawing {pieces} \
                         samples paper-thin, up from {whole}"
                    ));
                }
            }
        }
        let mut base = self.before;
        for (index, record) in self.blasts.iter().enumerate() {
            let at = record.blast.centre;
            let (cut, settled) = (record.cut, record.settled);
            if cut.loose.all > base.loose.all {
                found.push(format!(
                    "blast {index} at {at:?} left {} samples standing free, up from {}",
                    cut.loose.all, base.loose.all
                ));
            }
            if cut.paper_thin > base.paper_thin {
                found.push(format!(
                    "blast {index} at {at:?} left {} samples paper-thin, up from {}",
                    cut.paper_thin, base.paper_thin
                ));
            }
            // A deposit may leave a pebble too thin to bear lying loose, but
            // not rock.
            if settled.loose.bearing > cut.loose.bearing {
                found.push(format!(
                    "blast {index} at {at:?}: its deposits left {} bearing samples standing \
                     free, up from {}",
                    settled.loose.bearing, cut.loose.bearing
                ));
            }
            base = settled;
        }
        if self.open_edges > 0 {
            found.push(format!(
                "{} open mesh edges after the last blast",
                self.open_edges
            ));
        }
        found
    }
}

/// Play `scenario` and record what each blast did.
pub fn run(scenario: &Scenario) -> Result<Run, String> {
    let mut terrain = scenario.terrain()?;
    terrain.update();
    let before = Audit::of(&terrain);

    let mut planner = RubblePlanner::default();
    let blasts = scenario
        .blasts
        .iter()
        .map(|&blast| {
            let cut = Cut {
                shove: Explosion::new(blast.centre).physics_impulse(),
                fragments: terrain.detonate(blast.centre, &blast.charge),
            };
            // Rubble falls on the terrain as remeshed after the blast.
            terrain.update();
            let audit = Audit::of(&terrain);
            let fragments = play(cut, &mut planner, &mut terrain);
            BlastRecord {
                blast,
                fragments,
                cut: audit,
                settled: Audit::of(&terrain),
            }
        })
        .collect();
    terrain.update();

    Ok(Run {
        scenario: scenario.name,
        before,
        blasts,
        open_edges: terrain.open_edge_count(),
    })
}
