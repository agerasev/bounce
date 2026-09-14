//! A small rigid-body playground. Physics owns bodies and dragging; rendering
//! owns shared textures. Debug force traversal borrows the world immutably.
mod physics;
mod render;

use derive_more::{Deref, DerefMut};
use glam::Vec2;
use hsl::HSL;
use phy::{Rot2, Solver, Var};
use physics::{Body, Shape};
use rand::{Rng, RngExt};
use rand_distr::Uniform;
pub use render::{DrawMode, TextureStorage};
use rgb::Rgb;

#[derive(Clone, Deref, DerefMut)]
pub struct Item<S: Solver> {
    #[deref]
    #[deref_mut]
    pub body: Body<S>,
    pub shape: Shape,
    pub color: Rgb<f32>,
}

pub struct World<S: Solver> {
    /// Half of world sides
    size: Vec2,
    items: Vec<Item<S>>,
    drag: Option<(usize, Vec2, Vec2)>,
    // Reused across RK4 stages; force evaluation itself only reads the bodies.
    forces: Vec<geom2::pressure::ContactLoad>,
    contacts: Vec<physics::ContactShape>,
}

impl<S: Solver> World<S> {
    pub fn new(size: Vec2) -> Self {
        Self {
            size,
            items: Vec::new(),
            drag: None,
            forces: Vec::new(),
            contacts: Vec::new(),
        }
    }

    pub fn size(&self) -> Vec2 {
        self.size
    }

    pub fn wall_size(&self) -> Vec2 {
        self.size - physics::WALL_OFFSET * self.size.min_element()
    }

    pub fn drag_acquire(&mut self, pos: Vec2) {
        // Circles are drawn over rectangles; within a layer the last item wins.
        self.drag = self
            .items
            .iter()
            .enumerate()
            .filter(|(_, item)| item.contains(pos))
            .max_by_key(|(i, item)| (matches!(item.shape, Shape::Circle { .. }), *i))
            .map(|(i, item)| (i, pos, item.rot.inverse().transform(pos - *item.pos)));
    }

    pub fn drag_move(&mut self, pos: Vec2) {
        if let Some((_, target, ..)) = &mut self.drag {
            *target = pos;
        }
    }
    pub fn drag_release(&mut self) {
        self.drag = None;
    }

    pub fn n_items(&self) -> usize {
        self.items.len()
    }
    pub fn remove_item(&mut self, i: usize) -> Item<S> {
        self.drag = self.drag.and_then(|(index, target, local)| {
            (index != i).then_some((index - usize::from(index > i), target, local))
        });
        self.items.remove(i)
    }
    pub fn insert_item(&mut self, item: Item<S>) {
        self.items.push(item);
    }

    pub fn resize(&mut self, size: Vec2) {
        self.drag_release();
        self.size = size;
    }
}

pub fn sample_item<S: Solver>(mut rng: impl Rng, box_size: Vec2) -> Item<S> {
    let radius: f32 = rng.sample(Uniform::new(0.1, 0.3).unwrap());
    let mass = physics::MASF * radius;
    let eff_size = (box_size - Vec2::splat(radius)).max(Vec2::ZERO);
    let shape = if rng.sample(Uniform::new(0.0, 1.0).unwrap()) < 0.5 {
        Shape::Circle { radius }
    } else {
        Shape::Rectangle {
            size: Vec2::new(radius, radius * rng.random_range(0.5..1.0)),
        }
    };
    Item {
        body: Body {
            mass,
            pos: Var::new(Vec2::new(
                rng.sample(Uniform::new_inclusive(-eff_size.x, eff_size.x).unwrap()),
                rng.sample(Uniform::new_inclusive(-eff_size.y, eff_size.y).unwrap()),
            )),
            vel: Var::default(),
            inm: physics::INMF * mass * radius,
            rot: Var::new(Rot2::default()),
            asp: Var::default(),
        },
        color: Rgb::from(
            HSL {
                h: rng.sample(Uniform::new(0.0, 360.0).unwrap()),
                s: 1.0,
                l: 0.5,
            }
            .to_rgb(),
        ) / 255.0,
        shape,
    }
}
