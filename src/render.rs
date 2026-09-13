//! Shared assets and drawing. All world access here is immutable.
use crate::{World, physics::Shape};
use glam::{Affine2, Vec2, Vec4, Vec4Swizzles};
use phy::Solver;
use rand::Rng;
use rand_distr::Uniform;
use rgb::Rgb;
use wgame::{
    Library, Result,
    gfx::{
        Scene,
        types::{Color, color},
    },
    image::Image,
    prelude::*,
    texture::{Texture, TextureSettings},
};

#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub enum DrawMode {
    #[default]
    Normal,
    Debug,
}

const BORDER: f32 = 0.006;
const FORCE_SCALE: f32 = 0.12;

pub struct TextureStorage {
    ball: Texture,
    noise: Texture,
}

impl TextureStorage {
    pub fn new(rng: &mut impl Rng, lib: &Library) -> Result<Self> {
        let ball = Image::decode_auto(include_bytes!("../assets/ball.png"))
            .map_err(|err| err.context("cannot decode embedded ball.png"))?;
        Ok(Self {
            ball: lib.make_texture(&ball, TextureSettings::linear()),
            noise: noisy_texture(
                rng,
                lib,
                32,
                32,
                Rgb::new(0.75, 0.75, 0.75),
                Rgb::new(0.25, 0.25, 0.25),
            ),
        })
    }
}

fn outline(
    lib: &Library,
    scene: &mut Scene,
    size: Vec2,
    transform: Affine2,
    tint: Rgb<f32>,
    width: f32,
    order: i32,
) {
    // Join the four edges without overlapping their corner pixels.
    for (min, max) in [
        (
            Vec2::new(-size.x - width / 2.0, -size.y - width / 2.0),
            Vec2::new(size.x + width / 2.0, -size.y + width / 2.0),
        ),
        (
            Vec2::new(-size.x - width / 2.0, size.y - width / 2.0),
            Vec2::new(size.x + width / 2.0, size.y + width / 2.0),
        ),
        (
            Vec2::new(-size.x - width / 2.0, -size.y + width / 2.0),
            Vec2::new(-size.x + width / 2.0, size.y - width / 2.0),
        ),
        (
            Vec2::new(size.x - width / 2.0, -size.y + width / 2.0),
            Vec2::new(size.x + width / 2.0, size.y - width / 2.0),
        ),
    ] {
        scene.add(
            &lib.shapes()
                .rectangle((min, max))
                .transform(transform)
                .fill_color(tint)
                .order(order),
        );
    }
}

impl<S: Solver> World<S> {
    pub fn draw(
        &self,
        lib: &Library,
        textures: &TextureStorage,
        scene: &mut Scene,
        mode: DrawMode,
    ) {
        let wall = self.wall_size();
        if mode == DrawMode::Normal {
            scene.add(
                &lib.shapes()
                    .rectangle((-wall, wall))
                    .fill_color(color::WHITE)
                    .order(-1000),
            );
        }
        outline(
            lib,
            scene,
            wall,
            Affine2::IDENTITY,
            if mode == DrawMode::Normal {
                color::BLACK
            } else {
                color::WHITE
            },
            BORDER,
            -999,
        );
        for item in &self.items {
            let (size, order, texture) = match item.shape {
                Shape::Circle { radius } => (Vec2::splat(radius), 1, &textures.ball),
                Shape::Rectangle { size } => (size, 0, &textures.noise),
            };
            let transform = Affine2::from_angle_translation(item.rot.angle(), *item.pos);
            if mode == DrawMode::Normal {
                scene.add(
                    &lib.shapes()
                        .unit_quad()
                        .transform(Affine2::from_scale(size))
                        .transform(transform)
                        .fill_texture(texture)
                        .multiply_color(item.color)
                        .order(order),
                );
            }
            let tint = if mode == DrawMode::Normal {
                color::BLACK
            } else {
                item.color
            };
            match item.shape {
                Shape::Circle { radius } if mode == DrawMode::Debug => {
                    scene.add(
                        &lib.shapes()
                            .unit_circle()
                            .stroke_color(BORDER / radius, tint)
                            .scale(radius)
                            .transform(transform)
                            .order(order),
                    );
                    scene.add(
                        &lib.shapes()
                            .line(Vec2::ZERO, Vec2::new(radius, 0.0), BORDER)
                            .transform(transform)
                            .fill_color(tint)
                            .order(order),
                    );
                }
                Shape::Rectangle { size } => {
                    outline(lib, scene, size, transform, tint, BORDER, order)
                }
                _ => (),
            }
        }
        if mode == DrawMode::Debug {
            self.visit_forces(|pos, force| {
                let vector = FORCE_SCALE * force;
                let length = vector.length();
                if !length.is_finite() || length < BORDER {
                    return;
                }
                let tip = pos + vector;
                let direction = vector / length;
                let head = length.min(0.035);
                let base = tip - head * direction;
                scene.add(
                    &lib.shapes()
                        .line(pos, base, BORDER)
                        .fill_color(color::WHITE)
                        .order(2),
                );
                scene.add(
                    &lib.shapes()
                        .triangle(
                            tip,
                            base + 0.4 * head * direction.perp(),
                            base - 0.4 * head * direction.perp(),
                        )
                        .fill_color(color::WHITE)
                        .order(2),
                );
            });
        }
        if let Some((index, target, local)) = self.drag {
            let item = &self.items[index];
            let point = *item.pos + item.rot.transform(local);
            scene.add(
                &lib.shapes()
                    .line(point, target, BORDER)
                    .fill_color(color::MAGENTA)
                    .order(3),
            );
            scene.add(
                &lib.shapes()
                    .unit_circle()
                    .stroke_color(0.2, color::MAGENTA)
                    .scale(0.025)
                    .move_to(target)
                    .order(3),
            );
        }
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
