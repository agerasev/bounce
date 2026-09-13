use crate::{
    Item, World,
    physics::{Shape, WALL_OFFSET},
};
use glam::{Affine2, Vec2, Vec4, Vec4Swizzles};
use phy::Solver;
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

impl<S: Solver> Item<S> {
    fn draw(&self, lib: &Library, textures: &TextureStorage, scene: &mut Scene, mode: DrawMode) {
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
                        .fill_texture(match self.shape {
                            Shape::Circle { .. } => &textures.ball,
                            Shape::Rectangle { .. } => &textures.noise,
                        })
                        .multiply_color(self.color)
                        .order(order),
                );
            }
            DrawMode::Debug => match &self.shape {
                Shape::Circle { .. } => {
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

impl<S: Solver> World<S> {
    pub fn draw(
        &self,
        lib: &Library,
        textures: &TextureStorage,
        scene: &mut Scene,
        mode: DrawMode,
    ) {
        let wall_size = self.size - WALL_OFFSET * self.size.min_element();
        match mode {
            DrawMode::Normal => {
                let thickness = 2.0 * WALL_OFFSET * self.size.max_element();
                let wall_size = wall_size + 0.5 * thickness;
                scene.add(
                    &lib.shapes()
                        .rectangle((
                            -wall_size + Vec2::splat(0.5 * thickness),
                            wall_size - Vec2::splat(0.5 * thickness),
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
            item.draw(lib, textures, scene, mode);
        }
    }
}
const FORCEX: f32 = 0.05;

pub struct DrawActor<'a> {
    pub lib: &'a Library,
    pub scene: &'a mut Scene,
}

impl DrawActor<'_> {
    pub fn apply(&mut self, pos: Vec2, force: Vec2) {
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
