//! Turning a boulder that has come to rest back into terrain.
//!
//! A boulder costs the solver for as long as it is a body. Once it has slept
//! for `delay` with nothing else near it, its fragment is stamped back into
//! the field where it lies (`TerrainWorld::deposit`) and the body goes: the
//! pile is ground again, to be walked on, dug into and dammed with.
//!
//! ```text
//!   RubbleSpawnSystem ──▶ boulder + Settling { fragment, placed }
//!                                       │
//!   SettleSystem ───────────────────────┘ each frame:
//!     Settler::ready: awake, or asleep < delay ──▶ wait
//!     Settler::deposit, for each one ready:
//!       a body within a voxel of it that is not ready too ──▶ wait (it
//!         could end up inside); a pile that is all ready goes together
//!     TerrainWorld::deposit refused (across a join, past every segment)
//!       ──▶ stays a body until it wakes
//!     deposited ──▶ body removed, entity deleted; the terrain remeshes
//!                   this frame and the water re-lays around it
//! ```
//!
//! Every condition that can go wrong keeps the boulder a body, which is what
//! it was before Phase 4: the cull still bounds what never settles.

use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Entity, Join, Read, ReadStorage, System, Write,
    WriteStorage,
};

use super::budget::ResidentBoulder;
use crate::collision::AABB;
use crate::components::RigidBodyComponent;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::terrain::{Fragment, TerrainWorld};
use crate::time::Time;

/// A boulder that may become terrain again.
#[derive(Component)]
#[storage(DenseVecStorage)]
pub struct Settling {
    /// What it is made of, on the lattice it was cut from.
    fragment: Fragment,
    /// Its body's pose when it was placed, where the fragment's own pose
    /// has it.
    placed: Isometry3<f32>,
    /// Seconds it has slept without waking.
    asleep: f32,
    /// The terrain would not take it where it lies; not asked again until
    /// it moves.
    refused: bool,
}

impl Settling {
    /// `fragment` made a body whose origin was at `centre` when placed.
    pub fn new(fragment: Fragment, centre: Point3<f32>) -> Self {
        Self {
            fragment,
            placed: Translation3::from(centre.coords).into(),
            asleep: 0.0,
            refused: false,
        }
    }
}

/// What [`Settler::deposit`] made of a boulder ready to deposit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Another body is within reach of it.
    Crowded,
    /// The terrain would not take it where it lies.
    Refused,
    /// Stamped into the terrain: the caller removes its body.
    Deposited,
}

/// When a sleeping boulder becomes terrain.
#[derive(Debug, Clone, Copy)]
pub struct Settler {
    /// Seconds a boulder sleeps before it is deposited: a body asleep for a
    /// moment may yet be woken by what is still falling around it.
    pub delay: f32,
    /// How close another body may come, in voxels of the boulder's lattice.
    /// The deposited surface can stand up to half a voxel proud of the rock,
    /// and nothing may end up inside it.
    pub clearance: f32,
}

impl Default for Settler {
    fn default() -> Self {
        Self {
            delay: 1.0,
            clearance: 1.0,
        }
    }
}

impl Settler {
    /// Advance `settling`'s clock by `dt`, its body `asleep` or not, and say
    /// whether it is ready to deposit: asleep for `delay`, and not refused
    /// where it lies.
    pub fn ready(&self, settling: &mut Settling, asleep: bool, dt: f32) -> bool {
        if !asleep {
            settling.asleep = 0.0;
            settling.refused = false;
            return false;
        }
        settling.asleep += dt;
        settling.asleep >= self.delay && !settling.refused
    }

    /// Deposit `settling`, a boulder [`Self::ready`] passed whose body is
    /// `body`, into `terrain`, unless a body not among `ready` (the boulders
    /// deposited with it) reaches it.
    pub fn deposit(
        &self,
        settling: &mut Settling,
        body: RigidBodyHandle,
        ready: &[RigidBodyHandle],
        physics: &PhysicsWorld,
        terrain: &mut TerrainWorld,
    ) -> Verdict {
        let Some(rigid) = physics.body(body) else {
            return Verdict::Refused;
        };
        let now = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
        let moved = now * settling.placed.inverse();
        let reach = Vector3::repeat(self.clearance * settling.fragment.voxel_size());
        let bounds = settling.fragment.bounds(&moved);
        let around = AABB::new(bounds.min - reach, bounds.max + reach);
        if crowds(physics, body, ready, &around) {
            return Verdict::Crowded;
        }
        if terrain.deposit(&settling.fragment, &moved) {
            Verdict::Deposited
        } else {
            settling.refused = true;
            Verdict::Refused
        }
    }
}

/// Whether any body but `own` and those `ready` with it that is not static
/// reaches into `region`.
fn crowds(
    physics: &PhysicsWorld,
    own: RigidBodyHandle,
    ready: &[RigidBodyHandle],
    region: &AABB,
) -> bool {
    physics.bodies().iter().any(|(index, body)| {
        let handle = RigidBodyHandle(index);
        handle != own
            && !ready.contains(&handle)
            && !body.is_static()
            && body.colliders().iter().any(|handle| {
                physics.collider(*handle).is_some_and(|collider| {
                    let centre = collider.world_center(body.position(), body.rotation());
                    region.intersects_sphere(centre, collider.shape().bounding_radius())
                })
            })
    })
}

/// Deposits the boulders that have come to rest.
#[derive(Default)]
pub struct SettleSystem {
    settler: Settler,
}

impl<'a> System<'a> for SettleSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Option<Write<'a, TerrainWorld>>,
        Read<'a, Time>,
        WriteStorage<'a, Settling>,
        WriteStorage<'a, ResidentBoulder>,
        ReadStorage<'a, RigidBodyComponent>,
    );

    fn run(
        &mut self,
        (entities, mut physics, terrain, time, mut settling, mut boulders, bodies): Self::SystemData,
    ) {
        let Some(mut terrain) = terrain else {
            return;
        };
        let dt = time.delta_seconds();
        let ready: Vec<(Entity, RigidBodyHandle)> = (&entities, &mut settling, &bodies)
            .join()
            .filter_map(|(entity, boulder, body)| {
                let asleep = physics.world.is_sleeping(body.0);
                self.settler
                    .ready(boulder, asleep, dt)
                    .then_some((entity, body.0))
            })
            .collect();
        let handles: Vec<RigidBodyHandle> = ready.iter().map(|&(_, body)| body).collect();
        let mut deposited = Vec::new();
        for &(entity, body) in &ready {
            let Some(boulder) = settling.get_mut(entity) else {
                continue;
            };
            let verdict =
                self.settler
                    .deposit(boulder, body, &handles, &physics.world, &mut terrain);
            if verdict == Verdict::Deposited {
                deposited.push((entity, body));
            }
        }
        for (entity, body) in deposited {
            physics.world.remove_body(body);
            settling.remove(entity);
            boulders.remove(entity);
            let _ = entities.delete(entity);
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::level::loader::parse_level;
    use crate::level_check::build_terrain;
    use crate::physics::{ColliderDesc, RigidBodyDesc};
    use crate::terrain::BlastConfig;

    const FRAME: f32 = 1.0 / 60.0;

    /// An arch whose span both blasts cut loose, as `rubble_viewer`'s
    /// `arch_both_legs`.
    const ARCH: &str = r#"
Level(
    name: "settle: arch",
    segments: [(
        name: "main",
        terrain: Terrain(
            voxel_size: 0.5,
            bounds: (min: (-16.0, -8.0, -16.0), max: (16.0, 8.0, 16.0)),
            base_height: 0.0,
            material_layers: [(depth: 999.0, material: Rock)],
            features: [],
            volumes: [
                Arch(from: (-2.0, 0.0, 0.0), to: (2.0, 0.0, 0.0), radius: 2.0, thickness: 2.0),
            ],
        ),
    )],
    placements: [Root(segment: "main")],
    player_spawn: (10.0, 1.0, 10.0),
)
"#;

    /// The arch's terrain, its span cut loose, and that span as a boulder
    /// asleep where it broke, with the physics world it sleeps in.
    struct Fixture {
        terrain: TerrainWorld,
        physics: PhysicsWorld,
        body: RigidBodyHandle,
        settling: Settling,
        centre: Point3<f32>,
    }

    fn fixture(lifted: f32) -> Fixture {
        let level = parse_level(ARCH).expect("the arch parses");
        let mut terrain = build_terrain(&level);
        let mut fragments = Vec::new();
        for x in [2.0, -2.0] {
            let at = Point3::new(x, 0.25, 0.0);
            fragments.extend(terrain.detonate(at, &BlastConfig::fixed_radius(1.5)));
        }
        let span = fragments
            .into_iter()
            .max_by_key(Fragment::sample_count)
            .expect("the blasts cut the span loose");
        let centre = span.world_centroid();
        let mut physics = PhysicsWorld::default();
        // Every body starts asleep.
        let body =
            physics.create_body(RigidBodyDesc::dynamic().position(centre + Vector3::y() * lifted));
        physics.attach_collider(body, ColliderDesc::sphere(0.5));
        Fixture {
            terrain,
            physics,
            body,
            settling: Settling::new(span, centre),
            centre,
        }
    }

    /// Sleep `settling` for `seconds` and say whether it is then ready.
    fn sleep(settler: &Settler, settling: &mut Settling, seconds: f32) -> bool {
        let mut ready = false;
        for _ in 0..(seconds / FRAME).round() as usize {
            ready = settler.ready(settling, true, FRAME);
        }
        ready
    }

    #[test]
    fn a_boulder_is_ready_only_after_sleeping_for_the_delay() {
        let mut f = fixture(0.0);
        let settler = Settler::default();
        assert!(!sleep(&settler, &mut f.settling, 0.5 * settler.delay));
        assert!(sleep(&settler, &mut f.settling, 0.6 * settler.delay));
        assert!(!settler.ready(&mut f.settling, false, FRAME), "woken");
        assert!(!sleep(&settler, &mut f.settling, 0.5 * settler.delay));
    }

    #[test]
    fn a_body_beside_it_blocks_the_deposit_unless_it_is_ready_too() {
        let mut f = fixture(0.0);
        let settler = Settler::default();
        let crate_body = f
            .physics
            .create_body(RigidBodyDesc::dynamic().position(f.centre + Vector3::y() * 1.5));
        f.physics
            .attach_collider(crate_body, ColliderDesc::sphere(0.5));
        assert!(sleep(&settler, &mut f.settling, 1.1 * settler.delay));

        let verdict = settler.deposit(
            &mut f.settling,
            f.body,
            &[f.body],
            &f.physics,
            &mut f.terrain,
        );
        assert_eq!(verdict, Verdict::Crowded);
        let verdict = settler.deposit(
            &mut f.settling,
            f.body,
            &[f.body, crate_body],
            &f.physics,
            &mut f.terrain,
        );
        assert_eq!(verdict, Verdict::Deposited);
        assert!(
            f.terrain.segments()[0].voxel_at(f.centre).is_solid(),
            "the span is back where it rests"
        );
    }

    #[test]
    fn a_boulder_past_every_segment_is_refused_until_it_moves() {
        let mut f = fixture(100.0);
        let settler = Settler::default();
        assert!(sleep(&settler, &mut f.settling, 1.1 * settler.delay));
        let verdict = settler.deposit(
            &mut f.settling,
            f.body,
            &[f.body],
            &f.physics,
            &mut f.terrain,
        );
        assert_eq!(verdict, Verdict::Refused);
        assert!(!settler.ready(&mut f.settling, true, FRAME), "asked again");
        settler.ready(&mut f.settling, false, FRAME);
        assert!(
            sleep(&settler, &mut f.settling, 1.1 * settler.delay),
            "woken and asleep again"
        );
    }
}
