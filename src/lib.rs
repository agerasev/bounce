mod physics;

use crate::physics::{Actor, Body, Shape, WALL_OFFSET};
use derive_more::derive::{Deref, DerefMut};
use glam::{Affine2, Vec2, Vec4, Vec4Swizzles};
use hsl::HSL;
use phy::{Rot2, Solver, Var};
use rand::Rng;
use rand_distr::Uniform;
use rgb::Rgb;
use wgame::{
    Library,
    fs::Path,
    gfx::{
        Scene,
        types::{Color, color},
    },
    image::Image,
    prelude::*,
    texture::{Texture, TextureSettings},
};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Default, Debug)]
pub enum DrawMode {
    #[default]
    Normal,
    Debug,
}

/// Drawing border thickness factor
const BORDERX: f32 = 1.0 / 24.0;

#[derive(Clone, Deref, DerefMut)]
pub struct Item<S: Solver> {
    #[deref]
    #[deref_mut]
    pub body: Body<S>,
    pub shape: Shape,

    pub texture: Texture,
    pub color: Rgb<f32>,
}

impl<S: Solver> Item<S> {
    pub fn draw(&self, lib: &Library, scene: &mut Scene, mode: DrawMode) {
        let (size, order) = match &self.shape {
            Shape::Circle { radius } => (Vec2::splat(*radius), 1),
            Shape::Rectangle { size } => (*size, 0),
        };
        match mode {
            DrawMode::Normal => {
                scene.add(
                    &lib.shapes()
                        .unit_quad()
                        .transform(Affine2::from_scale_angle_translation(
                            size,
                            self.rot.angle(),
                            *self.pos,
                        ))
                        .fill_texture(&self.texture)
                        .multiply_color(self.color)
                        .order(order),
                );
            }
            DrawMode::Debug => match &self.shape {
                Shape::Circle { radius } => {
                    /*
                    draw_circle_lines(
                        self.pos.x,
                        self.pos.y,
                        *radius,
                        BORDERX * radius,
                        self.color,
                    ),
                    */
                }
                Shape::Rectangle { .. } => {
                    // Draw later
                }
            },
        }
        if let Shape::Rectangle { .. } = &self.shape {
            /*
            draw_rectangle_lines_ex(
                self.pos.x,
                self.pos.y,
                2.0 * size.x,
                2.0 * size.y,
                BORDERX * size.min_element(),
                DrawRectangleParams {
                    offset: Vec2::new(0.5, 0.5),
                    rotation: self.rot.angle(),
                    color: match mode {
                        DrawMode::Normal => color::BLACK,
                        DrawMode::Debug => self.color,
                    },
                },
            );
            */
        }
    }
}

pub struct World<S: Solver> {
    /// Half of world sides
    size: Vec2,
    items: Vec<Item<S>>,
    drag: Option<(usize, Vec2, Vec2)>,
}

impl<S: Solver> World<S> {
    pub fn new(size: Vec2) -> Self {
        Self {
            size,
            items: Vec::new(),
            drag: None,
        }
    }

    pub fn size(&self) -> Vec2 {
        self.size
    }

    pub fn drag_acquire(&mut self, pos: Vec2) {
        self.drag = self.items.iter().enumerate().find_map(|(i, item)| {
            let rel_pos = pos - *item.pos;
            if rel_pos.length() < item.shape.radius() {
                let rpos = item.rot.inverse().transform(rel_pos);
                Some((i, pos, rpos))
            } else {
                None
            }
        })
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
        self.drag = None;
        self.items.remove(i)
    }
    pub fn insert_item(&mut self, item: Item<S>) {
        self.items.push(item);
    }

    pub fn resize(&mut self, size: Vec2) {
        self.size = size;
    }
    pub fn draw(&self, lib: &Library, scene: &mut Scene, mode: DrawMode) {
        let wall_size = self.size - WALL_OFFSET * self.size.min_element();
        match mode {
            DrawMode::Normal => {
                let thickness = 2.0 * WALL_OFFSET * self.size.max_element();
                let wall_size = wall_size + 0.5 * thickness;
                scene.add(
                    &lib.shapes()
                        .rectangle((
                            -wall_size + Vec2::splat(thickness),
                            wall_size - Vec2::splat(thickness),
                        ))
                        .fill_color(color::WHITE)
                        .order(-1000),
                );
                /*
                draw_rectangle_lines(
                    -wall_size.x,
                    -wall_size.y,
                    2.0 * wall_size.x,
                    2.0 * wall_size.y,
                    thickness,
                    color::WHITE,
                );
                */
            }
            DrawMode::Debug => {
                /*
                draw_rectangle_lines(
                    -wall_size.x,
                    -wall_size.y,
                    2.0 * wall_size.x,
                    2.0 * wall_size.y,
                    0.3 * BORDERX,
                    color::WHITE,
                ),
                */
            }
        }
        for item in &self.items {
            item.draw(lib, scene, mode);
        }
    }
}

pub fn sample_item<S: Solver>(
    mut rng: impl Rng,
    box_size: Vec2,
    textures: &TextureStorage,
) -> Item<S> {
    let radius: f32 = rng.sample(Uniform::new(0.1, 0.3).unwrap());
    let mass = physics::MASF * radius;
    let eff_size = (box_size - Vec2::splat(radius)).max(Vec2::ZERO);
    let shape = if rng.sample(Uniform::new(0.0, 1.0).unwrap()) < 0.5 {
        Shape::Circle { radius }
    } else {
        Shape::Rectangle {
            size: Vec2::splat(radius),
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
        color: HSL {
            h: rng.sample(Uniform::new(0.0, 360.0).unwrap()),
            s: 1.0,
            l: 0.5,
        }
        .to_rgb()
        .into(),
        texture: match &shape {
            Shape::Circle { .. } => textures.ball.clone(),
            Shape::Rectangle { .. } => textures.noise.clone(),
        },
        shape,
    }
}

const FORCEX: f32 = 0.05;

pub struct DrawActor<'a> {
    pub lib: &'a Library,
    pub scene: &'a mut Scene,
}

impl<S: Solver> Actor<S> for DrawActor<'_> {
    fn apply(&mut self, _: &mut Body<S>, pos: Vec2, force: Vec2) {
        let fpos = pos + FORCEX * force;
        // Draw an arrow
        self.scene.add(
            &self
                .lib
                .shapes()
                .triangle(
                    fpos,
                    pos - BORDERX * FORCEX * force.perp(),
                    pos + BORDERX * FORCEX * force.perp(),
                )
                .fill_color(color::WHITE),
        );
    }
}

fn noisy_texture<R: Rng>(
    rng: R,
    lib: &Library,
    width: u32,
    height: u32,
    base: Rgb<f32>,
    var: Rgb<f32>,
) -> Texture {
    lib.make_texture(
        &Image::with_data(
            (width, height),
            rng.sample_iter(Uniform::new(0.0, 1.0).unwrap())
                .take(width as usize * height as usize)
                .map(|a| {
                    Vec4::from(((base.to_vec4() + a * var.to_vec4()).xyz(), 1.0)).to_rgba_f16()
                })
                .collect::<Vec<_>>(),
        ),
        TextureSettings::nearest(),
    )
}

pub struct TextureStorage {
    ball: Texture,
    noise: Texture,
}

impl TextureStorage {
    pub async fn load(base: impl AsRef<Path>, rng: &mut impl Rng, lib: &Library) -> Self {
        Self {
            ball: lib
                .load_texture(
                    format!("{}/ball.png", base.as_ref()),
                    TextureSettings::linear(),
                )
                .await
                .unwrap(),
            noise: noisy_texture(
                rng,
                lib,
                32,
                32,
                Rgb::new(0.75, 0.75, 0.75),
                Rgb::new(0.25, 0.25, 0.25),
            ),
        }
    }
}
