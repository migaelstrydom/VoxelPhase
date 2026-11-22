use nalgebra::{Point3, Vector2, Vector3};

/// Spans narrower than this, m, are no span at all: a patch that narrow is a
/// point, one that thin a line.
const MIN_SPAN: f32 = 1.0e-3;

/// What a pair met over: the shape of a hit's contact patch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Patch {
    /// A corner or a curve: the contacts meet at one place.
    Point,
    /// An edge: contacts along a line of `length`, m.
    Line { length: f32 },
    /// A face: contacts spanning `area`, m².
    Face { area: f32 },
}

impl Patch {
    /// The patch `points` span, seen along `normal` (unit length).
    pub(crate) fn spanned(points: &[Point3<f32>], normal: &Vector3<f32>) -> Self {
        let across = plane_coordinates(points, normal);
        let length = widest(&across);
        if length < MIN_SPAN {
            return Patch::Point;
        }
        let area = hull_area(across);
        if area < length * MIN_SPAN {
            Patch::Line { length }
        } else {
            Patch::Face { area }
        }
    }
}

/// `points` in coordinates on the plane across `normal`.
fn plane_coordinates(points: &[Point3<f32>], normal: &Vector3<f32>) -> Vec<Vector2<f32>> {
    let helper = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = normal.cross(&helper).normalize();
    let v = normal.cross(&u);
    points
        .iter()
        .map(|p| Vector2::new(p.coords.dot(&u), p.coords.dot(&v)))
        .collect()
}

/// The greatest distance between any two of `points`.
fn widest(points: &[Vector2<f32>]) -> f32 {
    let mut widest = 0.0f32;
    for (i, a) in points.iter().enumerate() {
        for b in &points[i + 1..] {
            widest = widest.max((a - b).norm());
        }
    }
    widest
}

/// Area of the convex hull of `points` (Andrew's monotone chain).
fn hull_area(mut points: Vec<Vector2<f32>>) -> f32 {
    if points.len() < 3 {
        return 0.0;
    }
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let turn = |o: &Vector2<f32>, a: &Vector2<f32>, b: &Vector2<f32>| {
        (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
    };
    let mut hull: Vec<Vector2<f32>> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &Vector2<f32>>> = if pass == 0 {
            Box::new(points.iter())
        } else {
            Box::new(points.iter().rev())
        };
        for p in ordered {
            while hull.len() >= start + 2
                && turn(&hull[hull.len() - 2], &hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(*p);
        }
        hull.pop();
    }
    let mut twice = 0.0;
    for i in 0..hull.len() {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        twice += a.x * b.y - a.y * b.x;
    }
    twice.abs() * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    fn on_ground(xz: &[(f32, f32)]) -> Vec<Point3<f32>> {
        xz.iter().map(|&(x, z)| Point3::new(x, 0.0, z)).collect()
    }

    #[test]
    fn one_contact_or_a_cluster_is_a_point() {
        let up = Vector3::y();
        assert_eq!(Patch::spanned(&on_ground(&[(1.0, 2.0)]), &up), Patch::Point);
        assert_eq!(
            Patch::spanned(&on_ground(&[(1.0, 2.0), (1.0002, 2.0003)]), &up),
            Patch::Point
        );
    }

    #[test]
    fn contacts_in_a_row_are_a_line_as_long_as_the_row() {
        let up = Vector3::y();
        let edge = on_ground(&[(0.0, 0.0), (0.5, 0.0)]);
        assert_eq!(Patch::spanned(&edge, &up), Patch::Line { length: 0.5 });
        let row = on_ground(&[(0.0, 0.0), (0.2, 0.0), (0.5, 0.0)]);
        assert_eq!(Patch::spanned(&row, &up), Patch::Line { length: 0.5 });
    }

    #[test]
    fn a_face_spans_its_corners_hull() {
        let up = Vector3::y();
        let square = on_ground(&[(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.25, 0.25)]);
        match Patch::spanned(&square, &up) {
            Patch::Face { area } => assert!((area - 0.25).abs() < 1e-5, "area {area}"),
            other => panic!("a square read as {other:?}"),
        }
    }

    /// The patch is measured across the normal, whatever way it faces: a
    /// face on a wall spans the same as one on the floor.
    #[test]
    fn a_patch_is_measured_across_its_normal() {
        let wall: Vec<Point3<f32>> = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5)]
            .iter()
            .map(|&(y, z)| Point3::new(3.0, y, z))
            .collect();
        match Patch::spanned(&wall, &Vector3::x()) {
            Patch::Face { area } => assert!((area - 0.25).abs() < 1e-5, "area {area}"),
            other => panic!("a wall's square read as {other:?}"),
        }
    }
}
