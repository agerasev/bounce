use std::time::Duration;

use bounce::{DrawActor, DrawMode, TextureStorage, World, sample_item};
use metaphysics::numerical::Solver;
use rand::{SeedableRng, rngs::SmallRng};
use wgame::{
    Library, Window,
    app::time::Instant,
    gfx::types::{Color, color},
    glam::{Affine2, Vec2},
    prelude::*,
    shapes::ShapeExt,
    typography::TextAlign,
};

#[wgame::window(title = "Wgame example", size = (1200, 900), resizable = true, vsync = true)]
async fn main(mut window: Window<'_>) {
    let gfx = Library::new(window.graphics());

    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
    let textures = TextureStorage::load("assets", &mut rng, &gfx).await;

    // let font = gfx.load_font("assets/free-sans-bold.ttf").await.unwrap();
    // let mut font_raster = None;
    // let mut text = None;

    let mut viewport = Vec2::ZERO;
    let scale = 640.0;

    let mut toy_box: Option<World> = None;
    let mut mode = DrawMode::Normal;

    let mut time = Instant::now();
    while let Some(mut frame) = window.next_frame().await.unwrap() {
        if let Some((width, height)) = frame.resized() {
            viewport = Vec2::new(width as f32, height as f32);
            toy_box = Some(match toy_box.take() {
                None => {
                    let mut toy_box = World::new(viewport / scale);
                    for _ in 0..8 {
                        toy_box.insert_item(sample_item(&mut rng, toy_box.size(), &textures));
                    }
                    toy_box
                }
                Some(mut toy_box) => {
                    toy_box.resize(viewport / scale);
                    toy_box
                }
            });

            // let raster = font_raster.insert(font.rasterize(height as f32 / 10.0));
            // text = Some(raster.text("Hello, World!"));
        }
        let toy_box = toy_box.as_mut().unwrap();

        frame.clear(match mode {
            DrawMode::Normal => color::BLACK.mix(color::WHITE, 0.5),
            DrawMode::Debug => color::BLACK.to_rgba(),
        });

        let camera = frame
            .physical_camera()
            .transform(Affine2::from_scale(Vec2::splat(scale)));
        let mut scene = frame.scene();
        scene.camera = camera;

        {
            let now = Instant::now();
            let frame_time = now - time;
            time = now;
            let dt = frame_time
                .min(Duration::from_millis(40))
                .div_f32(if mode == DrawMode::Debug { 10.0 } else { 1.0 });
            Solver.solve_step(toy_box, dt.as_secs_f32());
        }

        {
            toy_box.draw(&gfx, &mut scene, mode);
            if mode == DrawMode::Debug {
                toy_box.compute_derivs_ext(&mut DrawActor {
                    lib: &gfx,
                    scene: &mut scene,
                });
            }

            /*
            if mode == DrawMode::Normal {
                draw_text_aligned(
                    &format!("{}", toy_box.n_items()),
                    viewport.x - 30.0,
                    60.0,
                    TextAlign::Right,
                    Some(&font),
                    40.0,
                    color::WHITE,
                );
            }
            */
        }
    }
}
/*
pub async fn main() {
    while !is_key_down(KeyCode::Escape) {
        {
            if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
                toy_box.insert_item(sample_item(&mut rng, toy_box.size, &textures));
            }
            if toy_box.n_items() != 0
                && (is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract))
            {
                toy_box.remove_item(rng.sample(Uniform::new(0, toy_box.n_items())));
            }

            let mouse_pos = camera.screen_to_world(Vec2::from(mouse_position()));
            if is_mouse_button_pressed(MouseButton::Left) {
                toy_box.drag_acquire(mouse_pos);
            }
            if is_mouse_button_released(MouseButton::Left) {
                toy_box.drag_release();
            }
            if is_mouse_button_down(MouseButton::Left) {
                toy_box.drag_move(mouse_pos);
            } else {
                toy_box.drag_release();
            }

            if is_key_pressed(KeyCode::Backslash) {
                mode = match mode {
                    DrawMode::Normal => DrawMode::Debug,
                    DrawMode::Debug => DrawMode::Normal,
                }
            }
        }

        next_frame().await
    }

    Ok(())
}
*/
