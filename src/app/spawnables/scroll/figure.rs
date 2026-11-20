//! The figures the scrolls carry: propositions of Euclid's *Elements*, each
//! drawn as Euclid drew it, in straight lines and circles and the points
//! they name.

use std::f32::consts::{FRAC_PI_2, TAU};

use nalgebra::{Rotation2, Vector2};
use serde::Deserialize;

/// A proposition of the *Elements* whose figure a scroll carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
pub enum Proposition {
    /// Book I, 1: on a given straight line, an equilateral triangle, its
    /// apex where two circles, each through the other's centre, cross.
    Equilateral,
    /// Book I, 47: in a right-angled triangle, the square on the side under
    /// the right angle equals the squares on the other two; the three
    /// squares, the line from the right angle across the great one, and
    /// the four lines that prove it.
    Pythagoras,
    /// Book IV, 5: about a given triangle, a circle, centred where the
    /// lines square to two of its sides through their middles meet.
    Circumcircle,
    /// Book IV, 11: in a given circle, an equilateral and equiangular
    /// pentagon, with the two lines from its apex that make the triangle
    /// it is drawn from.
    Pentagon,
}

impl Proposition {
    /// Every proposition a scroll can carry.
    pub const ALL: [Proposition; 4] = [
        Proposition::Equilateral,
        Proposition::Pythagoras,
        Proposition::Circumcircle,
        Proposition::Pentagon,
    ];

    /// Its figure, fitted to the square from (-1, -1) to (1, 1), up `+y`.
    pub fn figure(self) -> Figure {
        match self {
            Proposition::Equilateral => equilateral(),
            Proposition::Pythagoras => pythagoras(),
            Proposition::Circumcircle => circumcircle(),
            Proposition::Pentagon => pentagon(),
        }
        .fitted()
    }
}

/// Sides to a circle: round at the size a figure is drawn.
const CIRCLE_SEGMENTS: usize = 72;

/// A figure as the pen drew it.
#[derive(Clone, Debug, Default)]
pub struct Figure {
    /// Each stroke of the pen, as the points it runs through: a straight
    /// line is two, a circle many and closed. Corners are where one stroke
    /// ends and the next begins, so they stay sharp.
    pub strokes: Vec<Vec<Vector2<f32>>>,

    /// The points the proposition names, where Euclid sets a letter: a dot
    /// here, since nothing is written in words.
    pub points: Vec<Vector2<f32>>,
}

impl Figure {
    /// With a straight line from `a` to `b`.
    fn line(mut self, a: Vector2<f32>, b: Vector2<f32>) -> Self {
        self.strokes.push(vec![a, b]);
        self
    }

    /// With straight lines round `corners`, closed, and each corner named.
    fn polygon(mut self, corners: &[Vector2<f32>]) -> Self {
        for (i, &a) in corners.iter().enumerate() {
            self = self.line(a, corners[(i + 1) % corners.len()]);
        }
        self.points.extend_from_slice(corners);
        self
    }

    /// With the circle about `centre` of radius `radius`.
    fn circle(mut self, centre: Vector2<f32>, radius: f32) -> Self {
        let stroke = (0..=CIRCLE_SEGMENTS)
            .map(|i| {
                let a = TAU * i as f32 / CIRCLE_SEGMENTS as f32;
                centre + Vector2::new(a.cos(), a.sin()) * radius
            })
            .collect();
        self.strokes.push(stroke);
        self
    }

    /// With `points` named.
    fn naming(mut self, points: &[Vector2<f32>]) -> Self {
        self.points.extend_from_slice(points);
        self
    }

    /// Every point it draws or names.
    fn all_points(&self) -> impl Iterator<Item = &Vector2<f32>> {
        self.strokes.iter().flatten().chain(&self.points)
    }

    /// The same figure moved and scaled to fill the square from (-1, -1) to
    /// (1, 1) in its longer direction, centred in the other.
    fn fitted(self) -> Self {
        let (mut low, mut high) = (Vector2::repeat(f32::MAX), Vector2::repeat(f32::MIN));
        for p in self.all_points() {
            low = low.inf(p);
            high = high.sup(p);
        }
        let middle = (low + high) * 0.5;
        let scale = 2.0 / (high - low).max();
        let fit = |p: Vector2<f32>| (p - middle) * scale;
        Self {
            strokes: self
                .strokes
                .into_iter()
                .map(|stroke| stroke.into_iter().map(fit).collect())
                .collect(),
            points: self.points.into_iter().map(fit).collect(),
        }
    }
}

fn v(x: f32, y: f32) -> Vector2<f32> {
    Vector2::new(x, y)
}

/// I.1: the line AB, the circles about A through B and about B through A,
/// and the triangle to C where they cross.
fn equilateral() -> Figure {
    let (a, b) = (v(-1.0, 0.0), v(1.0, 0.0));
    let c = v(0.0, 3f32.sqrt());
    Figure::default()
        .circle(a, 2.0)
        .circle(b, 2.0)
        .polygon(&[a, b, c])
}

/// I.47 on a 3-4-5 triangle, the hypotenuse BC level with its square
/// hung below it, as the manuscripts draw it: AL square to it through the
/// right angle A, and AD, FC, AE, BK, the lines that pair the triangles.
fn pythagoras() -> Figure {
    let (b, c) = (v(-2.5, 0.0), v(2.5, 0.0));
    let a = v(-0.7, 2.4);
    let (d, e) = (v(-2.5, -5.0), v(2.5, -5.0));
    // The squares on AB and AC, each outward from the triangle.
    let ab_out = v(-2.4, 1.8);
    let (f, g) = (b + ab_out, a + ab_out);
    let ac_out = v(2.4, 3.2);
    let (h, k) = (a + ac_out, c + ac_out);
    let l = v(a.x, d.y);
    Figure::default()
        .polygon(&[a, b, c])
        .polygon(&[b, d, e, c])
        .polygon(&[a, g, f, b])
        .polygon(&[a, c, k, h])
        .line(a, l)
        .line(a, d)
        .line(f, c)
        .line(a, e)
        .line(b, k)
        .naming(&[l])
}

/// IV.5: the triangle ABC in its circle about F, the lines DF and EF square
/// to AB and AC through their middles, and FA, FB, FC.
fn circumcircle() -> Figure {
    let on_circle = |degrees: f32| {
        let a = degrees.to_radians();
        v(a.cos(), a.sin())
    };
    let (a, b, c) = (on_circle(104.0), on_circle(208.0), on_circle(332.0));
    let f = v(0.0, 0.0);
    let (d, e) = ((a + b) * 0.5, (a + c) * 0.5);
    Figure::default()
        .circle(f, 1.0)
        .polygon(&[a, b, c])
        .line(d, f)
        .line(e, f)
        .line(f, a)
        .line(f, b)
        .line(f, c)
        .naming(&[d, e, f])
}

/// IV.11: the pentagon ABCDE in its circle, apex up, and AC, AD.
fn pentagon() -> Figure {
    let step = Rotation2::new(TAU / 5.0);
    let corners: Vec<Vector2<f32>> = (0..5)
        .scan(v(FRAC_PI_2.cos(), FRAC_PI_2.sin()), |at, _| {
            let here = *at;
            *at = step * *at;
            Some(here)
        })
        .collect();
    Figure::default()
        .circle(v(0.0, 0.0), 1.0)
        .polygon(&corners)
        .line(corners[0], corners[2])
        .line(corners[0], corners[3])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every figure fills its square in one direction and stays inside it.
    #[test]
    fn each_figure_fills_its_square() {
        for proposition in Proposition::ALL {
            let figure = proposition.figure();
            let (mut low, mut high) = (Vector2::repeat(f32::MAX), Vector2::repeat(f32::MIN));
            for p in figure.all_points() {
                low = low.inf(p);
                high = high.sup(p);
            }
            assert!(
                low.min() >= -1.0 - 1e-5 && high.max() <= 1.0 + 1e-5,
                "{proposition:?}"
            );
            assert!((high - low).max() > 2.0 - 1e-4, "{proposition:?}");
            assert!(figure.strokes.iter().all(|stroke| stroke.len() >= 2));
            assert!(!figure.points.is_empty(), "{proposition:?} names no point");
        }
    }

    /// I.47's triangle is right-angled at A and its squares are squares.
    #[test]
    fn pythagoras_is_drawn_true() {
        let (b, c, a) = (v(-2.5, 0.0), v(2.5, 0.0), v(-0.7, 2.4));
        assert!((b - a).dot(&(c - a)).abs() < 1e-5);
        assert!(((b - a).norm() - 3.0).abs() < 1e-5);
        assert!(((c - a).norm() - 4.0).abs() < 1e-5);
        assert!(v(-2.4, 1.8).dot(&(b - a)).abs() < 1e-5);
        assert!(v(2.4, 3.2).dot(&(c - a)).abs() < 1e-5);
    }
}
