//! Brazier spawnable: the bronze tripod that ends a level, lit by arriving
//! at it.
//!
//! ```text
//!   casting (Clay) ── three legs on lion's paws, rising in an S to hug a
//!        │            shallow bowl and curling into volutes over its lip
//!   bowl (Clay) ───── a shell cut level at the lip, sooted inside
//!   mouldings ─────── rolled lip, belly band, a stretcher ring round the
//!        │            legs, three ring handles standing on the lip (Strand)
//!   coals (Clay) ──── banked in the bowl; glow once it is lit
//!        └──tarnish──▶ one bronze mesh + the coals, sculpted once a run
//!
//!   BrazierDef ──spawn──▶ one heavy body (a hull from its three feet to
//!                         its lip), Goal + Brazier (`objective`)
//! ```
//!
//! It belongs to reason, not to the garden (`docs/VISUAL_DIRECTION.md`):
//! cast bronze, kept bright, of true proportions, on the model of the
//! Roman tripods found at Pompeii. It is an ordinary body, heavy enough
//! that walking into it does not tip it and light enough that a blast
//! does; knocked over, it can no longer be lit (`objective::brazier`).
//!
//! `visual_bench objective` shows it.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::{Arc, OnceLock};

use nalgebra::{Point3, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, SolidFace};
use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{Clay, Ellipsoid, Lump, Relief, Strand};
use crate::collision::convex_hull::ConvexHull;
use crate::components::{
    MaterialModulation, ModelInstance, Orientation, Position, Renderable, RigidBodyComponent,
    Velocity,
};
use crate::core::error::EngineResult;
use crate::fire::components::FireSeat;
use crate::geometry::Polyhedron;
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::objective::{Brazier, Goal};
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Emission, Material, MaterialId, SurfaceFinish};
use crate::rendering::reflection::Reflects;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_perlin_3d;

/// Slots of its materials.
const BRONZE: usize = 0;
const CHARCOAL: usize = 1;
const EMBERS: usize = 2;

/// How high the bowl's widest ring is, m, and the bowl's half-width there
/// and depth below it.
const BOWL_Y: f32 = 1.0;
const BOWL_RADIUS: f32 = 0.34;
const BOWL_DEPTH: f32 = 0.2;
/// How thick the bowl is cast, m.
const BOWL_WALL: f32 = 0.016;
/// How far above its widest ring the bowl's lip is, m: a little shoulder
/// turning in.
const LIP_RISE: f32 = 0.035;
/// Radius of the rolled lip, m.
const LIP_ROLL: f32 = 0.015;

/// How far out from the middle the feet stand, m.
const FOOT_REACH: f32 = 0.47;

/// Each leg's line from its paw to its volute: how far out from the
/// middle, how high, and how thick, m. It rises inward to a waist, swells
/// again to meet the bowl, hugs the bowl's side up to the lip and curls
/// over outward.
const LEG: [(f32, f32, f32); 13] = [
    (0.462, 0.1, 0.036),
    (0.43, 0.2, 0.031),
    (0.355, 0.4, 0.026),
    (0.305, 0.6, 0.027),
    (0.29, 0.78, 0.03),
    (0.305, 0.885, 0.029),
    (0.345, 0.97, 0.026),
    (0.36, 1.04, 0.024),
    (0.388, 1.092, 0.021),
    (0.428, 1.104, 0.019),
    (0.457, 1.075, 0.017),
    (0.444, 1.043, 0.015),
    (0.418, 1.053, 0.014),
];

/// Stations a leg's line is laid through from one of its points to the
/// next.
const LEG_STEPS: usize = 4;

/// How far one lump of the casting runs into the next, m.
const CASTING_BLEND: f32 = 0.018;

/// How high the stretcher ring runs round the legs, m, and how thick it is.
const STRETCHER_Y: f32 = 0.42;
const STRETCHER_RADIUS: f32 = 0.011;

/// The raised band round the bowl's belly: how far below its widest ring,
/// m, and how thick.
const BAND_DROP: f32 = 0.075;
const BAND_RADIUS: f32 = 0.007;

/// The ring handles standing on the lip between the legs: the ring's
/// radius and thickness, m.
const HANDLE_RADIUS: f32 = 0.068;
const HANDLE_THICKNESS: f32 = 0.0105;

/// Sides round a moulding, and stations round a ring.
const MOULDING_SIDES: u32 = 12;
const RING_STATIONS: usize = 48;

/// Side of the grid each casting is skinned from, m.
const CASTING_GRAIN: f32 = 0.0065;
const COALS_GRAIN: f32 = 0.0075;

/// Bronze where it is rubbed bright, where it has only aged, and where it
/// has darkened in the hollows; the soot in the bowl (linear). On a metal
/// these tint what it mirrors.
const BRIGHT: Vector3<f32> = Vector3::new(0.95, 0.66, 0.33);
const AGED: Vector3<f32> = Vector3::new(0.55, 0.32, 0.12);
const DARKENED: Vector3<f32> = Vector3::new(0.12, 0.08, 0.05);
const SOOT: Vector3<f32> = Vector3::new(0.03, 0.025, 0.02);

/// Charcoal, and the ash that whitens its tops (linear).
const CHARCOAL_COLOUR: Vector3<f32> = Vector3::new(0.022, 0.02, 0.019);
const ASH: Vector3<f32> = Vector3::new(0.32, 0.3, 0.28);

/// Cast bronze, kept: a soft mirror.
const BRONZE_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.3,
    metallic: 1.0,
};

/// The coals' glow once lit, as luminance: well over the bloom threshold,
/// under the flame that stands on them.
const EMBER_GLOW: f32 = 2.2;
const EMBER_COLOUR: Colour = Colour {
    r: 1.0,
    g: 0.36,
    b: 0.08,
    a: 1.0,
};

/// The flame's volume over the coals: as wide as the bowl, a little taller
/// than a man's forearm.
const FLAME_SIZE: Vector3<f32> = Vector3::new(0.56, 0.85, 0.56);
/// How far below the lip the flame's base sits, m: down among the coals.
const FLAME_SUNK: f32 = 0.06;

/// What it weighs, kg: far more than cast bronze would, because a walker's
/// drive pushes with a couple of kilonewtons and the tripod tips over a
/// lever a quarter of a metre long. A blast still throws it over.
const MASS: f32 = 700.0;
const FRICTION: f32 = 0.7;
const RESTITUTION: f32 = 0.1;
/// Its collider's corners: round each foot, how far either side of the
/// foot's middle, radians; between the feet, how far out and how high off
/// the ground, m, so the sides there stand plumb and a walker pushes it
/// rather than lifting it, without the ground meeting anything but the
/// feet; and the corners round the lip.
const FOOT_SPREAD: f32 = 0.12;
const BETWEEN_FEET: (f32, f32) = (0.36, 0.05);
const LIP_CORNERS: usize = 12;

#[derive(Deserialize)]
pub struct BrazierDef {
    /// Ground point under the middle of its feet.
    pub pos: (f32, f32, f32),

    /// Horizontal distance that counts as arriving.
    #[serde(default = "BrazierDef::default_radius")]
    pub radius: f32,
}

impl BrazierDef {
    /// How far round it its feet reach, m: what it stands on.
    pub const FOOTPRINT_RADIUS: f32 = FOOT_REACH + 0.09;

    pub fn default_radius() -> f32 {
        2.0
    }

    /// Its materials, one to a slot: bronze, charcoal, and the embers in it
    /// that glow once it is lit.
    pub fn materials() -> Vec<Material> {
        let charcoal = Material::coloured(Colour::WHITE).with_finish(SurfaceFinish {
            roughness: 0.92,
            metallic: 0.0,
        });
        vec![
            Material::coloured(Colour::WHITE)
                .with_finish(BRONZE_FINISH)
                .with_reflections(Reflects::Surroundings),
            charcoal.clone(),
            charcoal.with_emission(Emission {
                colour: EMBER_COLOUR,
                strength: EMBER_GLOW,
                ..Emission::default()
            }),
        ]
    }

    /// Its model with its ground point at the origin, in the materials
    /// whose ids are `ids`. Sculpted the first time it is asked for.
    pub fn model(ids: &[MaterialId]) -> Model {
        static SCULPTED: OnceLock<Vec<MeshPrimitive>> = OnceLock::new();
        let primitives = SCULPTED
            .get_or_init(sculpt)
            .iter()
            .map(|mesh| MeshPrimitive {
                material: ids[mesh.material.0 as usize],
                ..mesh.clone()
            })
            .collect();
        Model::flat(vec![ModelPart {
            local_transform: Transform::default(),
            primitives,
        }])
    }

    /// Where its flame sits, from its ground point.
    fn flame_base() -> Vector3<f32> {
        Vector3::new(0.0, BOWL_Y + LIP_RISE - FLAME_SUNK, 0.0)
    }

    /// Make its body in `physics` standing on `ground`. Returns the body and
    /// where its origin, the centre of its mass, lies from the ground point.
    ///
    /// One hull round the whole of it, from its feet to its lip. It stands
    /// on its three feet as the tripod does, and its sides between the feet
    /// stand plumb, so a walker pushes it over the ground rather than
    /// wedging in under the bowl and lifting it.
    fn create_body(
        physics: &mut PhysicsWorld,
        ground: Point3<f32>,
    ) -> (RigidBodyHandle, Vector3<f32>) {
        let body = physics.create_body(RigidBodyDesc::dynamic().position(ground));
        // A collider's shape is taken to be centred on its offset.
        let hull = tripod_hull();
        let centre = hull.centroid();
        let hull = Arc::new(hull.translated(-centre));
        let density = MASS / hull.compute_volume();
        physics.attach_collider(
            body,
            ColliderDesc::convex_hull(hull)
                .offset_translation(centre)
                .density(density)
                .friction(FRICTION)
                .restitution(RESTITUTION),
        );
        let moved = physics.recenter_on_colliders(body);
        (body, moved)
    }
}

impl Spawnable for BrazierDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(Self::materials()
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ground = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let (body, moved) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            Self::create_body(&mut physics.world, ground)
        };

        let mut model = Self::model(materials);
        for vertex in model
            .parts
            .iter_mut()
            .flat_map(|part| part.primitives.iter_mut())
            .flat_map(|primitive| primitive.vertices.iter_mut())
        {
            vertex.pos -= moved;
        }

        let brazier = Brazier::new(FireSeat {
            base: Self::flame_base() - moved,
            size: FLAME_SIZE,
        });

        vec![world
            .create_entity()
            .with(Position(ground.coords + moved))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(model)))
            .with(Renderable)
            .with(MaterialModulation(brazier.embers()))
            .with(Goal::new(self.radius))
            .with(brazier)
            .build()]
    }
}

/// Which way leg `index` stands out from the middle, on the ground.
fn leg_direction(index: usize) -> Vector3<f32> {
    let angle = FRAC_PI_2 + TAU * index as f32 / 3.0;
    Vector3::new(angle.cos(), 0.0, angle.sin())
}

/// A point `out` from the middle along `direction`, `up` high.
fn at(direction: Vector3<f32>, out: f32, up: f32) -> Vector3<f32> {
    direction * out + Vector3::y() * up
}

/// The hull it collides as: two corners at each foot on the ground, one
/// between each pair of feet a little off it, and a ring round the lip.
fn tripod_hull() -> ConvexHull {
    let lip = BOWL_Y + LIP_RISE + LIP_ROLL;
    let mut corners = Vec::new();
    for index in 0..3 {
        let middle = FRAC_PI_2 + TAU * index as f32 / 3.0;
        for side in [-FOOT_SPREAD, FOOT_SPREAD] {
            let a = middle + side;
            corners.push(Vector3::new(
                FOOT_REACH * a.cos(),
                0.0,
                FOOT_REACH * a.sin(),
            ));
        }
        let between = middle + TAU / 6.0;
        let (out, up) = BETWEEN_FEET;
        corners.push(Vector3::new(out * between.cos(), up, out * between.sin()));
    }
    let radius = BOWL_RADIUS + LIP_ROLL;
    for i in 0..LIP_CORNERS {
        let a = TAU * (i as f32 + 0.5) / LIP_CORNERS as f32;
        corners.push(Vector3::new(radius * a.cos(), lip, radius * a.sin()));
    }
    let shape = Polyhedron::convex_hull(corners);
    let faces: Vec<SolidFace> = shape
        .faces
        .iter()
        .map(|face| SolidFace {
            vertex_indices: face.clone(),
            // Any corner off the face lies behind it.
            opposite_vertex: (0..shape.vertices.len())
                .find(|i| !face.contains(i))
                .unwrap_or(0),
        })
        .collect();
    build_convex_hull(&shape.vertices, &faces)
}

/// The whole brazier against its slots: one bronze mesh, and the coals.
fn sculpt() -> Vec<MeshPrimitive> {
    let bronze_slot = MaterialId(BRONZE as u32);
    let mut bronze = casting().skin(CASTING_GRAIN, bronze_slot);
    append(&mut bronze, bowl().skin(CASTING_GRAIN, bronze_slot));
    append(&mut bronze, mouldings(bronze_slot));
    tarnish(&mut bronze);

    let mut charcoal = coals().skin(COALS_GRAIN, MaterialId(CHARCOAL as u32));
    paint_coals(&mut charcoal);
    let mut embers = charcoal.split_off(smouldering);
    embers.material = MaterialId(EMBERS as u32);

    vec![bronze, charcoal, embers]
}

/// Add `more` to `mesh`.
fn append(mesh: &mut MeshPrimitive, more: MeshPrimitive) {
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(more.vertices);
    mesh.indices
        .extend(more.indices.into_iter().map(|index| index + first));
}

/// A smooth line through `points` (out, up, thickness), sampled `steps`
/// times from each point to the next: Catmull-Rom, which passes through
/// every point.
fn smooth(points: &[(f32, f32, f32)], steps: usize) -> Vec<(f32, f32, f32)> {
    let get = |i: isize| {
        let p = points[i.clamp(0, points.len() as isize - 1) as usize];
        Vector3::new(p.0, p.1, p.2)
    };
    let mut line = Vec::new();
    for span in 0..points.len() as isize - 1 {
        let (p0, p1, p2, p3) = (get(span - 1), get(span), get(span + 1), get(span + 2));
        for step in 0..steps {
            let t = step as f32 / steps as f32;
            let (t2, t3) = (t * t, t * t * t);
            let p = 0.5
                * (2.0 * p1
                    + (p2 - p0) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                    + (3.0 * p1 - p0 - 3.0 * p2 + p3) * t3);
            line.push((p.x, p.y, p.z));
        }
    }
    let last = points[points.len() - 1];
    line.push(last);
    line
}

/// The legs and their paws, cast as one with the stretcher's collars.
fn casting() -> Clay {
    let line = smooth(&LEG, LEG_STEPS);
    let mut clay = Clay::new(CASTING_BLEND);
    for index in 0..3 {
        let d = leg_direction(index);
        let across = Vector3::y().cross(&d);
        let p = |out: f32, up: f32, side: f32| {
            let v = at(d, out, up) + across * side;
            (v.x, v.y, v.z)
        };
        let yaw = -d.z.atan2(d.x);

        // The leg, a limb from each station to the next. Joined hard: limbs
        // laid end to end share the ball at each joint, and a soft join
        // would swell every one of them into a bead.
        clay = clay.smoothing(0.0);
        for pair in line.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            clay = clay.add(
                Lump::limb(p(a.0, a.1, 0.0), p(b.0, b.1, 0.0), (a.2, b.2)),
                (1.0, 1.0, 1.0),
            );
        }

        // The paw on its disc: a low plinth, the pad, the fetlock where it
        // gathers into the leg, and four toes curled over its front, each
        // its own, with a crease between.
        clay = clay
            .smoothing(0.004)
            .add(
                Lump::Ball(
                    Ellipsoid::new((0.078, 0.024, 0.078), p(FOOT_REACH + 0.01, 0.006, 0.0))
                        .yawed(yaw),
                ),
                (1.0, 1.0, 1.0),
            )
            .smoothing(0.016)
            .add(
                Lump::Ball(
                    Ellipsoid::new((0.056, 0.04, 0.052), p(FOOT_REACH + 0.004, 0.058, 0.0))
                        .yawed(yaw),
                ),
                (1.0, 1.0, 1.0),
            )
            .add(
                Lump::mass((0.042, 0.05, 0.042), p(FOOT_REACH - 0.004, 0.118, 0.0)),
                (1.0, 1.0, 1.0),
            )
            .smoothing(0.005);
        for (toe, side) in [-0.042f32, -0.015, 0.015, 0.042].into_iter().enumerate() {
            // Short and plump, fanned round the front of the pad; the outer
            // toes sit back and a little lower, as a lion's do.
            let outer = toe == 0 || toe == 3;
            let back = if outer { 0.016 } else { 0.0 };
            let fan = d * (FOOT_REACH + 0.05 - back) + across * side;
            let toe_yaw = yaw - side * 2.2;
            clay = clay.add(
                Lump::Ball(
                    Ellipsoid::new((0.026, 0.022, 0.0175), (fan.x, 0.03 - back * 0.3, fan.z))
                        .yawed(toe_yaw),
                ),
                (1.0, 1.0, 1.0),
            );
        }
        clay = clay.smoothing(CASTING_BLEND);

        // A collar where the stretcher passes through the leg, and a boss
        // where the leg leaves the bowl for its volute.
        let collar = point_on_leg(&line, STRETCHER_Y);
        clay = clay
            .add(
                Lump::mass((0.03, 0.017, 0.03), p(collar.0, STRETCHER_Y, 0.0)),
                (1.0, 1.0, 1.0),
            )
            .add(Lump::ball(0.026, p(0.36, 1.035, 0.0)), (1.0, 1.0, 1.0))
            .smoothing(CASTING_BLEND);
    }
    // The paws stand flat on their discs.
    clay.cut(Lump::half_space(Vector3::zeros(), Vector3::y()), 0.0)
}

/// Where the leg's line crosses height `y`: how far out it is, and how
/// thick.
fn point_on_leg(line: &[(f32, f32, f32)], y: f32) -> (f32, f32) {
    for pair in line.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if (a.1 - y) * (b.1 - y) <= 0.0 && (b.1 - a.1).abs() > 1e-6 {
            let t = (y - a.1) / (b.1 - a.1);
            return (a.0 + (b.0 - a.0) * t, a.2 + (b.2 - a.2) * t);
        }
    }
    (line[0].0, line[0].2)
}

/// The bowl: a shell cut level at its lip.
fn bowl() -> Clay {
    let centre = (0.0, BOWL_Y, 0.0);
    let inner = (
        BOWL_RADIUS - BOWL_WALL,
        BOWL_DEPTH - BOWL_WALL,
        BOWL_RADIUS - BOWL_WALL,
    );
    Clay::new(0.0)
        .add(
            Lump::mass((BOWL_RADIUS, BOWL_DEPTH, BOWL_RADIUS), centre),
            (1.0, 1.0, 1.0),
        )
        .cut(Lump::mass(inner, centre), 0.004)
        .cut(
            Lump::half_space(Vector3::new(0.0, BOWL_Y + LIP_RISE, 0.0), -Vector3::y()),
            0.003,
        )
}

/// The bowl's half-width at height `y`.
fn bowl_radius_at(y: f32) -> f32 {
    let h = ((y - BOWL_Y) / BOWL_DEPTH).clamp(-1.0, 1.0);
    BOWL_RADIUS * (1.0 - h * h).sqrt()
}

/// A ring of `radius` about `centre`, in the plane square to `normal`.
fn ring(centre: Vector3<f32>, normal: Vector3<f32>, radius: f32, thickness: f32) -> Strand {
    let normal = normal.normalize();
    let spare = if normal.y.abs() < 0.9 {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let u = normal.cross(&spare).normalize();
    let v = normal.cross(&u);
    let path = (0..=RING_STATIONS)
        .map(|i| {
            let a = TAU * i as f32 / RING_STATIONS as f32;
            (centre + (u * a.cos() + v * a.sin()) * radius, thickness)
        })
        .collect();
    Strand::through(path).sides(MOULDING_SIDES).rings(1)
}

/// The rolled lip, the band round the belly, the stretcher ring, and the
/// three ring handles standing on the lip between the legs.
fn mouldings(material: MaterialId) -> MeshPrimitive {
    let mut mesh = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    };
    let lay = |strand: Strand, mesh: &mut MeshPrimitive| {
        strand.lay(mesh, |_| (Vector3::new(1.0, 1.0, 1.0), 1.0));
    };

    let lip_y = BOWL_Y + LIP_RISE;
    lay(
        ring(
            Vector3::new(0.0, lip_y, 0.0),
            Vector3::y(),
            bowl_radius_at(lip_y) - BOWL_WALL * 0.5,
            LIP_ROLL,
        ),
        &mut mesh,
    );
    let band_y = BOWL_Y - BAND_DROP;
    lay(
        ring(
            Vector3::new(0.0, band_y, 0.0),
            Vector3::y(),
            bowl_radius_at(band_y),
            BAND_RADIUS,
        ),
        &mut mesh,
    );
    let line = smooth(&LEG, LEG_STEPS);
    let (stretcher, _) = point_on_leg(&line, STRETCHER_Y);
    lay(
        ring(
            Vector3::new(0.0, STRETCHER_Y, 0.0),
            Vector3::y(),
            stretcher,
            STRETCHER_RADIUS,
        ),
        &mut mesh,
    );

    for index in 0..3 {
        // Halfway round between two legs.
        let between = leg_direction(index) + leg_direction((index + 1) % 3);
        let out = between.normalize();
        let tangent = Vector3::y().cross(&out);
        let seat = bowl_radius_at(lip_y) - BOWL_WALL * 0.5;
        let centre = at(
            out,
            seat,
            lip_y + LIP_ROLL + HANDLE_RADIUS - HANDLE_THICKNESS * 0.5,
        );
        // Seen face on from outside: the ring's plane is square to the way
        // out.
        lay(
            ring(centre, out, HANDLE_RADIUS, HANDLE_THICKNESS),
            &mut mesh,
        );
        // Two lugs that hold it to the lip.
        for side in [-1.0, 1.0] {
            let lug = at(out, seat, lip_y + LIP_ROLL * 0.8) + tangent * side * 0.03;
            append(
                &mut mesh,
                Ellipsoid::ball(0.014, lug).mesh(Colour::WHITE, material),
            );
        }
    }
    mesh
}

/// The coals banked in the bowl: a bed, and lumps heaped on it, highest in
/// the middle.
fn coals() -> Clay {
    let mut clay = Clay::new(0.012).add(
        Lump::mass((0.27, 0.075, 0.27), (0.0, BOWL_Y - 0.075, 0.0)),
        (1.0, 1.0, 1.0),
    );
    let mut seed = 0x9E37_79B9u32;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32
    };
    for _ in 0..34 {
        let (round, out, size) = (next() * TAU, next().sqrt() * 0.22, 0.03 + 0.03 * next());
        let heap = 0.035 * (1.0 - out / 0.22);
        clay = clay.add(
            Lump::mass(
                (
                    size,
                    size * (0.6 + 0.3 * next()),
                    size * (0.8 + 0.3 * next()),
                ),
                (
                    out * round.cos(),
                    BOWL_Y - 0.03 + heap + 0.01 * next(),
                    out * round.sin(),
                ),
            ),
            (1.0, 1.0, 1.0),
        );
    }
    clay.relief(Relief::Lumps {
        frequency: 40.0,
        depth: 0.008,
        seed: 17,
    })
}

/// A number from smooth noise at `p`, about -1 to 1.
fn noise(p: Vector3<f32>, scale: f32, seed: u32) -> f32 {
    fbm_perlin_3d(p.x * scale, p.y * scale, p.z * scale, 3, 0.5, seed)
}

/// What the years and the fire have done to the bronze: bright where it
/// stands proud and is handled (the lip, the volutes, the toes), darker in
/// the hollows, black with soot inside the bowl.
fn tarnish(mesh: &mut MeshPrimitive) {
    let inner = Vector3::new(
        BOWL_RADIUS - BOWL_WALL,
        BOWL_DEPTH - BOWL_WALL,
        BOWL_RADIUS - BOWL_WALL,
    );
    let lip_y = BOWL_Y + LIP_RISE;
    for vertex in &mut mesh.vertices {
        let p = vertex.pos;
        let closed = ((1.0 - vertex.ao) / 0.75).clamp(0.0, 1.0);
        let blotch = noise(p, 6.0, 31);
        let fine = noise(p, 45.0, 32);
        let handled = smoothstep(lip_y - 0.03, lip_y + 0.02, p.y) + smoothstep(0.08, 0.02, p.y);
        let rubbed =
            (0.45 + 0.3 * blotch + 0.1 * fine + 0.25 * vertex.normal.y.max(0.0) + 0.35 * handled
                - 1.2 * closed)
                .clamp(0.0, 1.0);
        let mut bronze = AGED
            .lerp(&DARKENED, (closed * 1.3).min(1.0) * 0.7)
            .lerp(&BRIGHT, rubbed * 0.75);

        // Inside the bowl, under the lip: soot.
        let q = (p - Vector3::new(0.0, BOWL_Y, 0.0))
            .component_div(&inner)
            .norm();
        let inside = q < 1.0 + 0.6 * BOWL_WALL / inner.x && p.y < lip_y - 0.004;
        if inside {
            let toward_lip = smoothstep(lip_y - 0.05, lip_y - 0.005, p.y);
            bronze = SOOT.lerp(&bronze, toward_lip * 0.6);
        }
        vertex.color = Vector4::new(bronze.x, bronze.y, bronze.z, 1.0);
    }
}

/// Whether the coals at `p` are the ones that glow once the brazier is lit:
/// patches and seams through the heap, not the whole of it.
fn smouldering(p: Vector3<f32>) -> bool {
    noise(p, 26.0, 51) + 0.6 * noise(p, 9.0, 52) > 0.12
}

/// Charcoal, whitened with ash where it faces up.
fn paint_coals(mesh: &mut MeshPrimitive) {
    for vertex in &mut mesh.vertices {
        let ash = (smoothstep(0.3, 0.95, vertex.normal.y)
            * (0.5 + 0.5 * noise(vertex.pos, 30.0, 41)))
        .clamp(0.0, 1.0);
        let c = CHARCOAL_COLOUR.lerp(&ASH, ash * 0.3);
        vertex.color = Vector4::new(c.x, c.y, c.z, 1.0);
    }
}

/// 0 at `from`, 1 at `to`, eased between; `from` may be above `to`.
fn smoothstep(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::PhysicsConfig;

    /// The leg's line runs up from the paw, inside the footprint, and ends
    /// in its volute over the lip.
    #[test]
    fn a_leg_rises_from_its_paw_to_above_the_lip() {
        let line = smooth(&LEG, LEG_STEPS);
        // It starts in the paw's fetlock, which stands on the disc.
        assert!(
            line[0].1 - line[0].2 < 0.08,
            "the leg starts {} m up",
            line[0].1
        );
        assert!(line.iter().all(|p| p.0 < BrazierDef::FOOTPRINT_RADIUS));
        let top = line.iter().map(|p| p.1).fold(f32::MIN, f32::max);
        assert!(top > BOWL_Y + LIP_RISE);
    }

    /// Stood on flat ground and left alone it stays upright and where it
    /// was: its mass sits well inside its feet.
    #[test]
    fn it_stands_on_its_three_feet() {
        const DT: f32 = 1.0 / 240.0;
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let (body, moved) = BrazierDef::create_body(&mut world, Point3::origin());
        world.wake_body(body);

        let mass = world.body(body).expect("the brazier").mass();
        assert!((mass - MASS).abs() < 1.0, "it weighs {mass} kg");
        assert!(
            moved.y > 0.3 && moved.y < 0.8,
            "its mass is {} m up",
            moved.y
        );

        let floor = FlatQuadGeometry::new(20.0);
        let mut debug = DebugLines::default();
        for _ in 0..(3.0 / DT) as usize {
            world.update_contacts(DT, 1, &floor, &[], &mut debug);
            world.substep(DT, &floor, &[]);
        }
        let body = world.body(body).expect("the brazier");
        let up = body.rotation() * Vector3::y();
        assert!(up.y > 0.999, "it leans to {up:?}");
        let drift = body.position().coords - moved;
        assert!(drift.norm() < 0.02, "it moved by {drift:?}");
    }

    /// The sculpture is whole: indices name vertices, nothing strays far
    /// from the brazier, and the coals sit inside the bowl.
    #[test]
    fn the_sculpture_is_whole() {
        let meshes = sculpt();
        assert_eq!(meshes.len(), 3);
        for mesh in &meshes {
            assert!(!mesh.indices.is_empty());
            let count = mesh.vertices.len() as u32;
            assert!(mesh.indices.iter().all(|&i| i < count));
            for vertex in &mesh.vertices {
                assert!(vertex.pos.iter().all(|x| x.is_finite()));
                assert!(
                    vertex.pos.y > -0.005 && vertex.pos.y < 1.3,
                    "a vertex at {:?}",
                    vertex.pos
                );
                let out = Vector3::new(vertex.pos.x, 0.0, vertex.pos.z).norm();
                assert!(out < BrazierDef::FOOTPRINT_RADIUS + 0.01);
            }
        }
        for vertex in meshes[CHARCOAL]
            .vertices
            .iter()
            .chain(&meshes[EMBERS].vertices)
        {
            let out = Vector3::new(vertex.pos.x, 0.0, vertex.pos.z).norm();
            assert!(out < BOWL_RADIUS - BOWL_WALL, "a coal lies {out} m out");
            assert!(
                vertex.pos.y < BOWL_Y + LIP_RISE + 0.02,
                "a coal heaps over the lip"
            );
        }
    }
}
