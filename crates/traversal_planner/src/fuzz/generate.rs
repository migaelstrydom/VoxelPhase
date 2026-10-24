use crate::plan::{BodyState, Limb};
use crate::scene::{AuthoredGround, Form, Ground, Intent, Shape, TraversalScene};
use crate::section::Footing;

use super::rng::Rng;

/// Where the course of features ends along the path, in metres: past any
/// strip.
const COURSE_END: f32 = 12.0;
/// The player's speeds, in m/s: crouch, walk, sprint.
const SPEEDS: [f32; 3] = [2.0, 5.0, 8.0];

/// A random scene for `seed`: a bank to start on, then a course of features
/// along the path (runs, lips and drops, gaps, plank bridges, stepping
/// stones on and off the line, tilted and loose stones, ramps, overhangs),
/// planned at a crouch, a walk or a sprint.
pub fn generate(seed: u64) -> TraversalScene {
    let mut rng = Rng::new(seed);
    let mut course = Course {
        shapes: Vec::new(),
        at: rng.range(0.3, 1.0),
        height: 0.0,
    };
    course.block(-1.0, course.at, Footing::Fixed, None);
    while course.at < COURSE_END {
        match rng.next_u64() % 8 {
            0 => course.run(&mut rng),
            1 => course.lip(&mut rng),
            2 => course.gap(&mut rng),
            3 => course.planks(&mut rng),
            4 => course.stones(&mut rng),
            5 => course.ramp(&mut rng),
            6 => course.overhang(&mut rng),
            _ => course.run(&mut rng),
        }
    }
    let base = rng
        .chance(0.5)
        .then(|| course.lowest() - rng.range(0.3, 1.0));
    let speed = rng.pick(&SPEEDS);
    TraversalScene {
        ground: Ground::Authored(AuthoredGround {
            base,
            shapes: course.shapes,
        }),
        body: BodyState {
            s: 0.0,
            z: 0.25,
            speed,
            climb: 0.0,
        },
        lead: rng.pick(&[Limb::LeftFoot, Limb::RightFoot]),
        intent: Intent { speed },
        params: Default::default(),
        layout: Default::default(),
        expect: None,
    }
}

/// The shapes laid so far, and where the next feature starts.
struct Course {
    shapes: Vec<Shape>,
    at: f32,
    /// Height of the ground the course is at.
    height: f32,
}

impl Course {
    fn block(&mut self, from: f32, to: f32, footing: Footing, across: Option<(f32, f32)>) {
        self.shapes.push(Shape {
            form: Form::Block {
                from,
                to,
                top: self.height,
                tilt: 0.0,
            },
            across,
            yaw: 0.0,
            footing,
        });
    }

    fn lowest(&self) -> f32 {
        self.shapes
            .iter()
            .flat_map(|shape| {
                let top = shape.top();
                shape
                    .footprint()
                    .into_iter()
                    .filter_map(move |(s, _)| top.map(|top| top.at(s, 0.0)))
            })
            .fold(0.0, f32::min)
    }

    fn run(&mut self, rng: &mut Rng) {
        let length = rng.range(0.3, 2.0);
        self.block(self.at, self.at + length, Footing::Fixed, None);
        self.at += length;
    }

    fn lip(&mut self, rng: &mut Rng) {
        let rise = rng.range(0.03, 0.6) * if rng.chance(0.5) { 1.0 } else { -1.0 };
        self.height += rise;
        self.run(rng);
    }

    fn gap(&mut self, rng: &mut Rng) {
        self.at += rng.range(0.1, 1.0);
        self.run(rng);
    }

    fn planks(&mut self, rng: &mut Rng) {
        let footing = Footing::Loose {
            mass: rng.range(200.0, 3000.0),
            resting: true,
        };
        for _ in 0..(2 + rng.next_u64() % 4) {
            self.at += rng.range(0.15, 0.7);
            let length = rng.range(0.4, 1.0);
            let half_span = rng.range(0.8, 1.2);
            let offset = rng.range(-0.1, 0.1);
            self.shapes.push(Shape {
                form: Form::Block {
                    from: self.at,
                    to: self.at + length,
                    top: self.height,
                    tilt: 0.0,
                },
                across: Some((offset - half_span, offset + half_span)),
                yaw: rng.range(-0.2, 0.2),
                footing,
            });
            self.at += length;
        }
        self.at += rng.range(0.15, 0.7);
        self.run(rng);
    }

    fn stones(&mut self, rng: &mut Rng) {
        for _ in 0..(2 + rng.next_u64() % 4) {
            self.at += rng.range(0.2, 0.6);
            let length = rng.range(0.25, 0.45);
            let centre = rng.pick(&[0.0, 0.15, -0.15, 0.25, -0.25]);
            let half_width = rng.range(0.1, 0.25);
            let footing = if rng.chance(0.3) {
                Footing::Loose {
                    mass: rng.range(20.0, 200.0),
                    resting: rng.chance(0.9),
                }
            } else {
                Footing::Fixed
            };
            self.shapes.push(Shape {
                form: Form::Block {
                    from: self.at,
                    to: self.at + length,
                    top: self.height + rng.range(-0.05, 0.08),
                    tilt: if rng.chance(0.3) {
                        rng.range(-0.5, 0.5)
                    } else {
                        0.0
                    },
                },
                across: Some((centre - half_width, centre + half_width)),
                yaw: rng.range(-0.3, 0.3),
                footing,
            });
            self.at += length;
        }
        self.at += rng.range(0.2, 0.6);
        self.run(rng);
    }

    fn ramp(&mut self, rng: &mut Rng) {
        let length = rng.range(1.0, 3.0);
        let rise = rng.range(-0.6, 0.6);
        self.shapes.push(Shape {
            form: Form::Ramp {
                from: self.at,
                to: self.at + length,
                start_top: self.height,
                end_top: self.height + rise,
            },
            across: None,
            yaw: 0.0,
            footing: Footing::Fixed,
        });
        self.height += rise;
        self.at += length;
    }

    fn overhang(&mut self, rng: &mut Rng) {
        let length = rng.range(0.5, 2.5);
        self.shapes.push(Shape {
            form: Form::Roof {
                from: self.at,
                to: self.at + length,
                bottom: self.height + rng.range(1.0, 2.0),
            },
            across: None,
            yaw: 0.0,
            footing: Footing::Fixed,
        });
        self.block(self.at, self.at + length, Footing::Fixed, None);
        self.at += length;
    }
}
