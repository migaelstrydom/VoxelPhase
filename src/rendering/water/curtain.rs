//! A fall's curtain: the body of water drawn from its lip down to where it
//! lands, and the foam there.
//!
//! ```text
//!        end 0      middle      end 1       three strands down each fall: its
//!   lip ───●──────────●──────────●───       own arc from the middle of its lip,
//!          ┃╲         ┃╲         ┃╲         and at each end the mean of its arc
//!          ┃ ╲        ┃ ╲        ┃ ╲        and the arc of the fall its lip runs
//!          ┃  ╲       ┃  ╲       ┃  ╲       on into. Two falls side by side
//!   foot ~~┃~~~●~~~~~~┃~~~●~~~~~~┃~~~●~~    share the strand between them, so
//!                                           a row of falls is one curtain
//! ```
//!
//! A lip running across the lattice is a staircase of edges facing two
//! ways. Its falls' middles lie on a straight line, and so do the strands
//! between them: the curtain hangs from the staircase along its first row
//! and is smooth from the second. Each strand's top face follows its arc;
//! its under face lies below by the water's thickness, the flow over the
//! lip by its speed there, thinning as it falls. Where a fall ends and no
//! other runs on from it, the body is closed by a cap between the two, so
//! a thick fall is seen to be thick from its side, and a thin one frays out.
//!
//! The faces are wound by their normals and the sheets' pipeline culls back
//! faces: from in front only the top is seen, from behind only the under
//! face. Where it lands, a patch of foam lies on the water round its foot.

use nalgebra::{Point3, Vector2, Vector3, Vector4};
use smallvec::SmallVec;

use crate::water::falling::{along_edge, edge_ends, fall_time, GRAVITY};
use crate::water::field::Fall;
use crate::water::geometry::COLUMN_SIZE;

use super::river_mesh::RiverMesh;
use super::vertex::RiverVertex;

/// Rows of vertices down a curtain, after its lip. Fixed, so a curtain
/// keeps its vertices as its water rises and falls.
pub(super) const CURTAIN_ROWS: u32 = 8;

/// How far out past its foot the foam where a fall lands reaches, m: so far
/// for any fall, and this much further per metre it falls; and how far back
/// behind its foot, under the fall.
const PLUNGE_REACH: f32 = 0.5;
const PLUNGE_REACH_PER_FALL: f32 = 0.15;
const MAX_PLUNGE_REACH: f32 = 1.8;
const PLUNGE_BACK: f32 = 0.45;
/// How far above the water its foam is drawn, m, clear of the surface's
/// own depth.
pub(super) const PLUNGE_LIFT: f32 = 0.02;
/// Where across its foam each of its rows lies: behind the foot, at it, and
/// at its outer edge (`RiverVertex::sheet`). The first is just short of -1,
/// so the whole patch reads as past the fall.
const PLUNGE_ROWS: [f32; 3] = [-0.98, 0.0, 1.0];

/// Thinnest and thickest a falling body of water is drawn, m.
const MIN_THICKNESS: f32 = 0.02;
const MAX_THICKNESS: f32 = 2.0;

/// Depths of water going over a lip over which a fall's free end stops
/// fraying out into strands and is the closed end of a body, m.
const FRAYING_DEPTH: (f32, f32) = (0.15, 0.8);

/// Speeds water leaves a lip at over which it goes from sliding glassy over
/// the brink to breaking white as it leaves, m/s, and how white the fastest
/// leaves short of a jet's.
const AIRY_SPEED: (f32, f32) = (2.0, 6.0);
const FASTEST_AIR: f32 = 0.8;

/// How far back into the hole it bursts out of a jet's body is drawn, m, so
/// it is seen to come from inside.
const JET_SET_BACK: f32 = 0.6;

/// How far out from its hole's mouth a jet has drawn in from the hole's
/// whole height to its own thickness, m.
const PLUG_RUN: f32 = 1.0;

/// The least time of flight a jet's body is drawn through, s.
const PLUG_TIME: f32 = 0.3;

/// Least flow over an edge, per metre of it, that a curtain draws as full,
/// m²/s: a curtain is drawn as deep as the water pours over its edge, so
/// the draw cuts it where it only trickles.
const FULL_POUR: f32 = 1.0;

/// One fall's water in flight, as its curtain is drawn through it.
#[derive(Debug, Clone, Copy)]
struct Tongue {
    /// The middle of its lip, on top of its water.
    lip: Point3<f32>,
    /// Its level velocity along the arc drawn, m/s.
    level: Vector2<f32>,
    /// How far its top falls, m, and for how long, s.
    fall: f32,
    duration: f32,
    /// What it carries per metre of lip, m²/s, and how thick it leaves.
    pour: f32,
    depth: f32,
    /// How much air it has taken in as it leaves, 0 to 1.
    air: f32,
    /// How far back from its lip its body starts, m: a jet's, inside the
    /// hole it bursts out of.
    set_back: f32,
    /// How far over its top, and under its underside, the hole it bursts
    /// out of stands open, m: it fills the hole behind its mouth, and
    /// draws in to its own thickness over the first of its flight.
    plug: f32,
}

/// A strand's water at one moment of its fall.
#[derive(Debug, Clone, Copy)]
struct Section {
    /// The top of the water.
    top: Point3<f32>,
    velocity: Vector3<f32>,
    thickness: f32,
}

impl Tongue {
    fn of(fall: &Fall) -> Self {
        let mut height = fall.height();
        let mut duration = fall_time(height);
        let mut level = if duration > 0.0 {
            fall.throw() / duration
        } else {
            Vector2::zeros()
        };
        let (air, set_back, plug) = if fall.driven {
            // A jet that meets water as it comes out still comes out: its
            // body is drawn on into that water the way it was thrown, far
            // enough to close the hole's mouth over the water's surface.
            if duration < PLUG_TIME {
                duration = PLUG_TIME;
                height = 0.5 * GRAVITY * PLUG_TIME * PLUG_TIME;
                level = fall.velocity;
            }
            let plug = 0.5 * (fall.opening - fall.depth).max(0.0);
            (1.0, JET_SET_BACK, plug)
        } else {
            let speed = fall.velocity.norm();
            let air = FASTEST_AIR * smoothstep(AIRY_SPEED.0, AIRY_SPEED.1, speed);
            (air, 0.0, 0.0)
        };
        Self {
            lip: fall.lip,
            level,
            fall: height,
            duration,
            pour: fall.pour,
            depth: fall.depth,
            air,
            set_back,
            plug,
        }
    }

    /// Its water `share` of the way through its fall, by time.
    fn at(&self, share: f32) -> Section {
        let t = self.duration * share;
        let out = self.level * t;
        let velocity = Vector3::new(self.level.x, -GRAVITY * t, self.level.y);
        // The flow over the lip keeps its rate as it speeds up: it thins.
        let thickness = self.pour / velocity.norm().max(1.0e-3);
        let filling = self.plug * (1.0 - smoothstep(0.0, PLUG_RUN, out.norm()));
        Section {
            top: Point3::new(
                self.lip.x + out.x,
                self.lip.y - self.fall * share * share + filling,
                self.lip.z + out.y,
            ),
            velocity,
            thickness: thickness.clamp(
                MIN_THICKNESS,
                self.depth.clamp(MIN_THICKNESS, MAX_THICKNESS),
            ) + 2.0 * filling,
        }
    }

    /// How deep its curtain is drawn: how much pours, no deeper than the
    /// water it pours from.
    fn drawn_depth(&self) -> f32 {
        self.depth.min(self.pour / FULL_POUR)
    }
}

/// One line of vertices down a curtain: the water of one fall, or between
/// two whose lips run on into each other, the mean of theirs.
#[derive(Debug, Clone)]
struct Strand {
    tongues: SmallVec<[Tongue; 2]>,
    /// How far its water is moved from its tongues', level: a free end's
    /// strand is its fall's arc moved to the end of the lip.
    shift: Vector2<f32>,
    /// Which side of the curtain's middle it lies on: -1 or 1 at an end,
    /// along the level direction square to its flight; 0 in the middle.
    side: f32,
    /// How far it frays out: a free end's, by how thin its water is.
    fray: f32,
    /// Whether the fall ends here, with no other running on from it.
    free: bool,
    /// Where its top hangs from: the surface it pours off, x and z and
    /// height. None where the water it leaves has no surface drawn.
    hung: Option<(Vector2<f32>, f32)>,
}

impl Strand {
    fn section(&self, share: f32) -> Section {
        let n = self.tongues.len() as f32;
        let mut top = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut thickness = 0.0;
        for tongue in &self.tongues {
            let section = tongue.at(share);
            top += section.top.coords;
            velocity += section.velocity;
            thickness += section.thickness;
        }
        Section {
            top: Point3::from(top / n) + Vector3::new(self.shift.x, 0.0, self.shift.y),
            velocity: velocity / n,
            thickness: thickness / n,
        }
    }

    fn mean(&self, of: impl Fn(&Tongue) -> f32) -> f32 {
        self.tongues.iter().map(of).sum::<f32>() / self.tongues.len() as f32
    }

    /// The way its water flies, level, `share` of the way down.
    fn out(&self, section: &Section) -> Vector2<f32> {
        Vector2::new(section.velocity.x, section.velocity.z)
            .try_normalize(1.0e-4)
            .unwrap_or_else(Vector2::x)
    }

    /// What every vertex of the strand carries: how deep it is drawn, where
    /// its streaks hang from, and which side it is on. `drop` and the
    /// thickness are the vertex's own.
    fn vertex(&self, at: Point3<f32>, section: &Section, drop: f32) -> RiverVertex {
        let lip = self.section(0.0).top;
        RiverVertex {
            xz: Vector2::new(at.x, at.z),
            surface: at.y,
            depth: self.mean(Tongue::drawn_depth),
            flow: Vector2::new(lip.x, lip.z),
            face: self.out(section),
            sheet: Vector4::new(drop, self.mean(|t| t.fall), self.side, section.thickness),
            normal: Vector3::y(),
            fray: self.fray,
            air: self.mean(|t| t.air),
            life: RiverVertex::ALWAYS,
        }
    }

    /// The top of its water on row `row`, and the face normal there: square
    /// to its flight, out and up. Row 0 is where it hangs from.
    fn top(&self, row: u32) -> (Point3<f32>, Vector3<f32>, Section) {
        let section = self.section(row as f32 / CURTAIN_ROWS as f32);
        let out = self.out(&section);
        let level = Vector2::new(section.velocity.x, section.velocity.z).norm();
        let sinking = -section.velocity.y;
        let normal = Vector3::new(out.x * sinking, level, out.y * sinking)
            .try_normalize(1.0e-6)
            .unwrap_or_else(Vector3::y);
        let at = match (row, self.hung) {
            (0, Some((xz, height))) => Point3::new(xz.x, height, xz.y),
            (0, None) => {
                let back = out * self.mean(|t| t.set_back);
                section.top - Vector3::new(back.x, 0.0, back.y)
            }
            _ => section.top,
        };
        (at, normal, section)
    }

    /// The vertex of `face` on row `row`.
    fn body(&self, row: u32, face: Face) -> RiverVertex {
        let (top, normal, section) = self.top(row);
        let share = row as f32 / CURTAIN_ROWS as f32;
        let drop = self.mean(|t| t.fall) * share * share;
        let (at, normal) = match face {
            Face::Top => (top, normal),
            Face::Under => (top - normal * section.thickness, -normal),
            // A cap's two edges: on the top face and on the under face,
            // both facing out sideways from the curtain's end.
            Face::CapTop | Face::CapUnder => {
                let out = self.out(&section);
                let sideways = Vector3::new(-out.y, 0.0, out.x) * self.side;
                let at = if face == Face::CapTop {
                    top
                } else {
                    top - normal * section.thickness
                };
                (at, sideways)
            }
        };
        RiverVertex {
            normal,
            ..self.vertex(at, &section, drop)
        }
    }

    /// A vertex of the foam where it lands, lying on the water at its foot:
    /// `across` its patch, -1 behind the foot under the fall, 0 at the
    /// foot, 1 at its outer edge. Its drop is its fall and 1 more and that,
    /// past its foot: what the draw takes for foam.
    fn plunge(&self, across: f32) -> RiverVertex {
        let section = self.section(1.0);
        let fall = self.mean(|t| t.fall);
        let reach = (PLUNGE_REACH + PLUNGE_REACH_PER_FALL * fall).min(MAX_PLUNGE_REACH);
        let out = self.out(&section)
            * if across < 0.0 {
                across * PLUNGE_BACK
            } else {
                across * reach
            };
        let at = section.top + Vector3::new(out.x, PLUNGE_LIFT, out.y);
        RiverVertex {
            fray: self.free as u32 as f32,
            ..self.vertex(at, &section, fall + 1.0 + across)
        }
    }
}

/// Which face of a falling body of water a vertex is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Face {
    /// Its outer surface, facing out and up.
    Top,
    /// The surface under it, facing in and down.
    Under,
    /// The closed end of a fall, on its top face's edge and its under
    /// face's.
    CapTop,
    CapUnder,
}

/// One fall's curtain.
#[derive(Debug, Clone)]
pub(super) struct Curtain {
    /// The strands at its lip's two ends (`edge_ends`) and, between them,
    /// its middle.
    strands: [Strand; 3],
    /// When its fall began and stopped pouring (`RiverVertex::life`).
    life: Vector2<f32>,
}

impl Curtain {
    /// The curtain of `fall` as the water's clock stands at `clock`,
    /// running on at each end of its lip into the fall `beside` it there,
    /// if any, and hung along its lip from the surface at `hung`: at its
    /// two ends and, between them, its middle.
    pub fn new(
        fall: &Fall,
        beside: [Option<&Fall>; 2],
        hung: Option<[(Vector2<f32>, f32); 3]>,
        clock: f32,
    ) -> Self {
        let own = Tongue::of(fall);
        let (step, face) = (fall.step(), fall.face());
        let ledge = 0.5 * (fall.reach[0] + fall.reach[1]) * COLUMN_SIZE;
        let edge_middle = Vector2::new(fall.lip.x, fall.lip.z) - face * ledge;
        let thin = 1.0 - smoothstep(FRAYING_DEPTH.0, FRAYING_DEPTH.1, fall.depth);
        let end = |end: usize| {
            let along = along_edge(step, edge_ends(step)[end]);
            let along = Vector2::new(along.di as f32, along.dk as f32);
            let mut strand = match beside[end] {
                Some(other) => Strand {
                    tongues: smallvec::smallvec![own, Tongue::of(other)],
                    shift: Vector2::zeros(),
                    side: 0.0,
                    fray: 0.0,
                    free: false,
                    hung: None,
                },
                None => {
                    let corner = edge_middle + along * (0.5 * COLUMN_SIZE);
                    let at = corner + face * (fall.reach[end] * COLUMN_SIZE);
                    Strand {
                        tongues: smallvec::smallvec![own],
                        shift: at - Vector2::new(fall.lip.x, fall.lip.z),
                        side: 0.0,
                        fray: thin,
                        free: true,
                        hung: None,
                    }
                }
            };
            let out = strand.out(&strand.section(0.0));
            let square = Vector2::new(-out.y, out.x);
            strand.side = if along.dot(&square) < 0.0 { -1.0 } else { 1.0 };
            strand.hung = hung.map(|h| h[2 * end]);
            strand
        };
        let middle = Strand {
            tongues: smallvec::smallvec![own],
            shift: Vector2::zeros(),
            side: 0.0,
            fray: 0.0,
            free: false,
            hung: hung.map(|h| h[1]),
        };
        let stopped = fall
            .spent
            .map_or(RiverVertex::ALWAYS.y, |spent| clock - spent);
        Self {
            strands: [end(0), middle, end(1)],
            life: Vector2::new(clock - fall.age, stopped),
        }
    }

    /// Whether the fall ends at each end of its lip.
    fn free(&self) -> [bool; 2] {
        [self.strands[0].free, self.strands[2].free]
    }

    /// How many vertices a curtain whose lip ends where `free` says has.
    pub fn vertex_count(free: [bool; 2]) -> usize {
        let rows = CURTAIN_ROWS as usize + 1;
        let caps = free.iter().filter(|&&f| f).count();
        3 * rows * 2 + 3 * PLUNGE_ROWS.len() + caps * 2 * rows
    }

    /// Its vertices: its top face, strand by strand, each from lip to foot;
    /// its under face likewise; its foam, strand by strand; then the cap of
    /// each free end, row by row, top edge then under edge.
    pub fn vertices(&self) -> Vec<RiverVertex> {
        let mut out = Vec::with_capacity(Self::vertex_count(self.free()));
        for face in [Face::Top, Face::Under] {
            for strand in &self.strands {
                out.extend((0..=CURTAIN_ROWS).map(|row| strand.body(row, face)));
            }
        }
        for strand in &self.strands {
            out.extend(PLUNGE_ROWS.iter().map(|&across| strand.plunge(across)));
        }
        for strand in [&self.strands[0], &self.strands[2]] {
            if strand.free {
                for row in 0..=CURTAIN_ROWS {
                    out.push(strand.body(row, Face::CapTop));
                    out.push(strand.body(row, Face::CapUnder));
                }
            }
        }
        for vertex in &mut out {
            vertex.life = self.life;
        }
        out
    }

    /// Join the vertices of a curtain whose lip ends where `free` says,
    /// already in `mesh` from vertex `first` on, into its triangles.
    pub fn join(mesh: &mut RiverMesh, first: u32, free: [bool; 2]) {
        let rows = CURTAIN_ROWS + 1;
        for face in 0..2 {
            let base = first + face * 3 * rows;
            let across: Vec<[u32; 3]> = (0..rows)
                .map(|row| [base + row, base + rows + row, base + 2 * rows + row])
                .collect();
            strip(mesh, &across);
        }
        let foam = first + 6 * rows;
        let depth = PLUNGE_ROWS.len() as u32;
        let across: Vec<[u32; 3]> = (0..depth)
            .map(|row| [foam + row, foam + depth + row, foam + 2 * depth + row])
            .collect();
        strip(mesh, &across);
        let mut cap = foam + 3 * depth;
        for end in free {
            if end {
                let down: Vec<[u32; 2]> = (0..rows)
                    .map(|row| [cap + 2 * row, cap + 2 * row + 1])
                    .collect();
                strip(mesh, &down);
                cap += 2 * rows;
            }
        }
    }
}

fn smoothstep(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The triangles of a patch of foam: `centre` joined to each pair of
/// neighbours round `ring`, wound to face up.
pub(super) fn fan(mesh: &mut RiverMesh, centre: u32, ring: &[u32]) {
    for pair in ring.windows(2) {
        let triangle = wound(mesh, [centre, pair[0], pair[1]]);
        mesh.sheet_indices.extend(triangle);
    }
}

/// The triangles of a body of water's surface: `rows` of vertices across
/// it, each joined to the next along the row and to the row after, each
/// triangle wound to face the way its vertices' normals do, so the pipeline
/// culls the faces turned away. A face of a curtain is rows across its
/// strands; a jet, rows of a ring closed by its first vertex again.
pub(super) fn strip<R: AsRef<[u32]>>(mesh: &mut RiverMesh, rows: &[R]) {
    for pair in rows.windows(2) {
        let (near, far) = (pair[0].as_ref(), pair[1].as_ref());
        for i in 0..near.len().min(far.len()).saturating_sub(1) {
            let (a, b, c, d) = (near[i], near[i + 1], far[i + 1], far[i]);
            for [p, q, r] in [[a, b, c], [a, c, d]] {
                mesh.sheet_indices.extend(wound(mesh, [p, q, r]));
            }
        }
    }
}

/// The triangle `corners` ordered to face the mean of their normals.
fn wound(mesh: &RiverMesh, corners: [u32; 3]) -> [u32; 3] {
    let at = |i: u32| {
        let v = &mesh.vertices[i as usize];
        (Vector3::new(v.xz.x, v.surface, v.xz.y), v.normal)
    };
    let [(p, np), (q, nq), (r, nr)] = corners.map(at);
    let [a, b, c] = corners;
    if (q - p).cross(&(r - p)).dot(&(np + nq + nr)) >= 0.0 {
        [a, b, c]
    } else {
        [a, c, b]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::SpanIndex;

    /// A fall over a +x edge at x = 2, from 10 m to 6 m, 0.4 m deep,
    /// carrying 0.8 m²/s: off the cell whose column's middle is at z.
    fn fall(z: f32) -> Fall {
        let duration = fall_time(4.0);
        Fall {
            from: SpanIndex { slot: 0, flat: 0 },
            direction: 0,
            lip: Point3::new(2.0, 10.0, z),
            reach: [0.0; 2],
            velocity: Vector2::new(2.0, 0.0),
            depth: 0.4,
            opening: 0.4,
            pour: 0.8,
            driven: false,
            drowned: false,
            foot: Point3::new(2.0 + 2.0 * duration, 6.0, z),
            landing: Point3::new(2.0 + 2.0 * duration, 6.0, z),
            into: None,
            age: 60.0,
            spent: None,
            aloft: 0.0,
            settling: 0.0,
        }
    }

    fn at(v: &RiverVertex) -> Vector3<f32> {
        Vector3::new(v.xz.x, v.surface, v.xz.y)
    }

    #[test]
    fn its_top_runs_from_its_lip_to_its_foot() {
        let curtain = Curtain::new(&fall(0.25), [None, None], None, 0.0);
        let vertices = curtain.vertices();
        let rows = CURTAIN_ROWS as usize + 1;
        let middle = &vertices[rows..2 * rows];
        assert!((at(&middle[0]) - Vector3::new(2.0, 10.0, 0.25)).norm() < 1e-5);
        assert!(
            (middle[0].normal - Vector3::y()).norm() < 1e-5,
            "level at the lip"
        );
        let foot = middle[rows - 1];
        assert!((at(&foot) - fall(0.25).foot.coords).norm() < 1e-4);
        assert!(foot.normal.x > 0.9, "facing out once it falls fast");
        assert_eq!(foot.sheet.x, foot.sheet.y);
        assert_eq!(vertices.len(), Curtain::vertex_count([true, true]));
    }

    #[test]
    fn a_free_end_hangs_from_the_end_of_its_lip_and_a_shared_one_from_between_two_falls() {
        let (near, far) = (fall(0.25), fall(0.75));
        // The +x edge's ends lie towards −z, then +z.
        let curtain = Curtain::new(&near, [None, Some(&far)], None, 0.0);
        let vertices = curtain.vertices();
        let rows = CURTAIN_ROWS as usize + 1;
        assert!((vertices[0].xz - Vector2::new(2.0, 0.0)).norm() < 1e-5);
        assert!((vertices[2 * rows].xz - Vector2::new(2.0, 0.5)).norm() < 1e-5);
        let other = Curtain::new(&far, [Some(&near), None], None, 0.0).vertices();
        for row in 0..rows {
            let (ours, theirs) = (vertices[2 * rows + row], other[row]);
            assert_eq!((ours.xz, ours.surface), (theirs.xz, theirs.surface));
            assert_eq!(ours.normal, theirs.normal, "row {row}");
        }
        assert_eq!(vertices.len(), Curtain::vertex_count([true, false]));
    }

    #[test]
    fn its_under_face_leaves_the_lip_its_depth_down_and_closes_in_as_it_falls() {
        let curtain = Curtain::new(&fall(0.25), [None, None], None, 0.0);
        let vertices = curtain.vertices();
        let rows = CURTAIN_ROWS as usize + 1;
        let gap = |row: usize| {
            let (top, under) = (&vertices[rows + row], &vertices[4 * rows + row]);
            assert!((top.normal + under.normal).norm() < 1e-6, "faces opposite");
            (at(top) - at(under)).norm()
        };
        assert!((gap(0) - 0.4).abs() < 1e-5);
        assert!(gap(rows - 1) < 0.1 && gap(rows - 1) >= MIN_THICKNESS - 1e-6);
        assert!((1..rows).all(|row| gap(row) < gap(row - 1)));
    }

    #[test]
    fn its_triangles_face_their_normals() {
        let mut mesh = RiverMesh::default();
        let curtain = Curtain::new(&fall(0.25), [None, None], None, 0.0);
        mesh.vertices = curtain.vertices();
        Curtain::join(&mut mesh, 0, [true, true]);
        assert!(!mesh.sheet_indices.is_empty());
        for triangle in mesh.sheet_indices.chunks(3) {
            let [p, q, r] = [0, 1, 2].map(|i| {
                let v = &mesh.vertices[triangle[i] as usize];
                (at(v), v.normal)
            });
            let facing = (q.0 - p.0).cross(&(r.0 - p.0));
            assert!(facing.dot(&(p.1 + q.1 + r.1)) >= 0.0);
        }
    }

    #[test]
    fn its_foam_lies_on_the_water_round_its_foot_and_reads_as_past_the_fall() {
        let curtain = Curtain::new(&fall(0.25), [None, None], None, 0.0);
        let vertices = curtain.vertices();
        let rows = CURTAIN_ROWS as usize + 1;
        let foam = &vertices[6 * rows..6 * rows + 9];
        let foot = fall(0.25).foot;
        for v in foam {
            assert!((v.surface - (foot.y + PLUNGE_LIFT)).abs() < 1e-5);
            assert!(v.sheet.x > v.sheet.y, "past its fall");
        }
        // The middle strand's: behind, at and beyond the foot.
        assert!(foam[3].xz.x < foot.x && foam[5].xz.x > foot.x);
        assert!((foam[4].xz.x - foot.x).abs() < 1e-5);
    }
}
