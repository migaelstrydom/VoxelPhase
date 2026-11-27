//! The convex hull of a cloud of points, as triangles.
//!
//! For shapes sampled off a smooth form, where neighbouring faces are so
//! nearly flush that no tolerance on "flat" tells one from the next: the
//! hull is grown a point at a time, and a point is judged only against the
//! faces it can see, so nothing is ever merged.
//!
//! ```text
//!   tetrahedron of four far-flung points
//!     └─► each further point: the faces it sees ──► their horizon
//!           └─► those faces gone, a fan from the point to the horizon in their place
//! ```
//!
//! It works in f64, and a point within a hair of the hull (a billionth of
//! its size) counts as inside it.

use nalgebra::Vector3;
use rustc_hash::FxHashMap;

/// How far outside a face a point must be to be seen from it, as a share
/// of the cloud's size.
const SEEN: f64 = 1e-9;

/// A face of the hull as it grows: its corners, anticlockwise seen from
/// outside, and its plane.
struct Face {
    corners: [usize; 3],
    normal: Vector3<f64>,
    offset: f64,
    alive: bool,
}

impl Face {
    fn new(points: &[Vector3<f64>], corners: [usize; 3]) -> Self {
        let [a, b, c] = corners.map(|i| points[i]);
        let normal = (b - a).cross(&(c - a)).normalize();
        Self {
            corners,
            normal,
            offset: normal.dot(&a),
            alive: true,
        }
    }

    /// How far outside its plane `p` is.
    fn height(&self, p: &Vector3<f64>) -> f64 {
        self.normal.dot(p) - self.offset
    }
}

/// The convex hull of `points`: the points that are its corners, and its
/// faces as triples of indices into them, anticlockwise seen from outside.
/// `None` if the points do not enclose a volume.
pub fn convex_hull(points: &[Vector3<f32>]) -> Option<(Vec<Vector3<f32>>, Vec<[usize; 3]>)> {
    let cloud: Vec<Vector3<f64>> = points.iter().map(|p| p.cast::<f64>()).collect();
    let size = cloud
        .iter()
        .flat_map(|p| cloud.iter().map(move |q| (p - q).norm()))
        .fold(0.0, f64::max);
    let seen = SEEN * size;
    let start = tetrahedron(&cloud, seen)?;
    let inside = start.iter().map(|&i| cloud[i]).sum::<Vector3<f64>>() / 4.0;

    let mut faces: Vec<Face> = Vec::new();
    // Which face each directed edge belongs to, among the living.
    let mut edges: FxHashMap<(usize, usize), usize> = FxHashMap::default();
    let [a, b, c, d] = start;
    for corners in [[a, b, c], [a, b, d], [a, c, d], [b, c, d]] {
        add_face(&cloud, &inside, &mut faces, &mut edges, corners);
    }

    for point in 0..cloud.len() {
        if start.contains(&point) {
            continue;
        }
        let p = cloud[point];
        let seeing: Vec<usize> = (0..faces.len())
            .filter(|&f| faces[f].alive && faces[f].height(&p) > seen)
            .collect();
        if seeing.is_empty() {
            continue;
        }
        // The horizon: edges of the faces it sees whose other side it does
        // not.
        let mut horizon = Vec::new();
        for &f in &seeing {
            let [a, b, c] = faces[f].corners;
            for (from, to) in [(a, b), (b, c), (c, a)] {
                let beyond = edges.get(&(to, from)).copied();
                if beyond.is_some_and(|other| !seeing.contains(&other)) {
                    horizon.push((from, to));
                }
            }
        }
        for &f in &seeing {
            faces[f].alive = false;
            let [a, b, c] = faces[f].corners;
            for edge in [(a, b), (b, c), (c, a)] {
                edges.remove(&edge);
            }
        }
        for (from, to) in horizon {
            add_face(&cloud, &inside, &mut faces, &mut edges, [from, to, point]);
        }
    }

    // Only the points the faces use, in the order first used.
    let mut kept = vec![usize::MAX; cloud.len()];
    let mut corners = Vec::new();
    let triangles = faces
        .iter()
        .filter(|face| face.alive)
        .map(|face| {
            face.corners.map(|i| {
                if kept[i] == usize::MAX {
                    kept[i] = corners.len();
                    corners.push(points[i]);
                }
                kept[i]
            })
        })
        .collect();
    Some((corners, triangles))
}

/// Add a face on `corners` to `faces`, wound to face away from `inside`,
/// and its edges to `edges`.
fn add_face(
    cloud: &[Vector3<f64>],
    inside: &Vector3<f64>,
    faces: &mut Vec<Face>,
    edges: &mut FxHashMap<(usize, usize), usize>,
    corners: [usize; 3],
) {
    let mut face = Face::new(cloud, corners);
    if face.height(inside) > 0.0 {
        face = Face::new(cloud, [corners[0], corners[2], corners[1]]);
    }
    let index = faces.len();
    let [a, b, c] = face.corners;
    for edge in [(a, b), (b, c), (c, a)] {
        edges.insert(edge, index);
    }
    faces.push(face);
}

/// Four points of `cloud` that enclose a volume, as far apart as a quick
/// search finds them.
fn tetrahedron(cloud: &[Vector3<f64>], seen: f64) -> Option<[usize; 4]> {
    let farthest = |score: &dyn Fn(&Vector3<f64>) -> f64| {
        (0..cloud.len()).max_by(|&i, &j| score(&cloud[i]).total_cmp(&score(&cloud[j])))
    };
    let a = farthest(&|p| -p.x)?;
    let b = farthest(&|p| (p - cloud[a]).norm())?;
    let line = (cloud[b] - cloud[a]).try_normalize(seen)?;
    let c = farthest(&|p| {
        let off = p - cloud[a];
        (off - line * off.dot(&line)).norm()
    })?;
    let normal = (cloud[b] - cloud[a])
        .cross(&(cloud[c] - cloud[a]))
        .try_normalize(seen * seen)?;
    let d = farthest(&|p| normal.dot(&(p - cloud[a])).abs())?;
    (normal.dot(&(cloud[d] - cloud[a])).abs() > seen).then_some([a, b, c, d])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every edge is shared by two faces, one each way round; every point
    /// is on or inside every face.
    fn assert_closed_and_convex(
        points: &[Vector3<f32>],
        corners: &[Vector3<f32>],
        faces: &[[usize; 3]],
    ) {
        let mut edges = FxHashMap::default();
        for face in faces {
            for k in 0..3 {
                *edges.entry((face[k], face[(k + 1) % 3])).or_insert(0) += 1;
            }
        }
        for (&(from, to), &count) in &edges {
            assert_eq!(
                count, 1,
                "edge ({from}, {to}) is used {count} times one way"
            );
            assert_eq!(
                edges.get(&(to, from)),
                Some(&1),
                "edge ({from}, {to}) is open"
            );
        }
        for face in faces {
            let [a, b, c] = face.map(|i| corners[i]);
            let normal = (b - a).cross(&(c - a)).normalize();
            for p in points {
                assert!(normal.dot(&(p - a)) < 1e-5, "{p:?} is outside a face");
            }
        }
    }

    #[test]
    fn a_cube_with_points_inside_and_on_its_faces_is_its_eight_corners() {
        let mut points = Vec::new();
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    points.push(Vector3::new(x, y, z));
                }
            }
        }
        points.push(Vector3::new(0.2, -0.3, 0.1));
        points.push(Vector3::new(1.0, 0.0, 0.0));
        points.push(Vector3::new(0.5, 1.0, 0.5));
        let (corners, faces) = convex_hull(&points).unwrap();
        assert_eq!(corners.len(), 8);
        assert_eq!(faces.len(), 12);
        assert_closed_and_convex(&points, &corners, &faces);
    }

    #[test]
    fn a_flattened_sphere_sampled_in_rings_is_closed_and_convex() {
        // Rings of a lens, nearly flush with one another near its rim:
        // where merging by flatness goes wrong.
        let mut points = Vec::new();
        for ring in 0..7 {
            let lat = -1.4 + 2.8 * ring as f32 / 6.0;
            for round in 0..20 {
                let lon = round as f32 / 20.0 * std::f32::consts::TAU;
                points.push(Vector3::new(
                    1.5 * lat.cos() * lon.sin(),
                    0.2 * lat.sin(),
                    2.0 * lat.cos() * lon.cos(),
                ));
            }
        }
        points.push(Vector3::new(0.0, 0.2, 0.0));
        points.push(Vector3::new(0.0, -0.2, 0.0));
        let (corners, faces) = convex_hull(&points).unwrap();
        assert_eq!(corners.len(), points.len());
        assert_eq!(faces.len(), 2 * corners.len() - 4);
        assert_closed_and_convex(&points, &corners, &faces);
    }

    #[test]
    fn points_on_a_plane_have_no_hull() {
        let points: Vec<Vector3<f32>> = (0..10)
            .map(|i| Vector3::new(i as f32, (i * i) as f32 * 0.1, 0.0))
            .collect();
        assert!(convex_hull(&points).is_none());
    }
}
