use std::time::Duration;

use bounce::{DrawActor, DrawMode, TextureStorage, World, sample_item};
use glam::{Vec4, Vec4Swizzles};
use metaphysics::{Rk4, Solver};
use rand::{Rng, SeedableRng, rngs::SmallRng};
use rand_distr::Uniform;
use wgame::{
    Event, Library, Window,
    app::time::Instant,
    gfx::types::{Color, color},
    glam::{Affine2, Vec2},
    input::{
        event::{ElementState, MouseButton},
        keyboard::{KeyCode, PhysicalKey},
    },
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

    let mut toy_box: Option<World<Rk4>> = None;
    let mut mode = DrawMode::Normal;

    let mut events = window.input();
    let mut mouse_pos = Vec2::ZERO;
    let mut mouse_down = false;

    let mut time = Instant::now();
    'frame_loop: while let Some(mut frame) = window.next_frame().await.unwrap() {
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
        let camera = frame
            .physical_camera()
            .transform(Affine2::from_scale_angle_translation(
                Vec2::splat(0.5 * scale),
                0.0,
                0.5 * viewport,
            ));

        while let Some(event) = events.try_next() {
            match event {
                Event::KeyboardInput { event, .. } => {
                    if event.state.is_pressed()
                        && !event.repeat
                        && let PhysicalKey::Code(key) = event.physical_key
                    {
                        match key {
                            KeyCode::Escape => break 'frame_loop,
                            KeyCode::Equal | KeyCode::NumpadAdd => {
                                toy_box.insert_item(sample_item(
                                    &mut rng,
                                    toy_box.size(),
                                    &textures,
                                ));
                            }
                            KeyCode::Minus | KeyCode::NumpadSubtract => {
                                if toy_box.n_items() != 0 {
                                    toy_box.remove_item(
                                        rng.sample(Uniform::new(0, toy_box.n_items()).unwrap()),
                                    );
                                }
                            }
                            KeyCode::Backslash => {
                                mode = match mode {
                                    DrawMode::Normal => DrawMode::Debug,
                                    DrawMode::Debug => DrawMode::Normal,
                                }
                            }
                            _ => (),
                        }
                    }
                }
                Event::MouseInput { state, button, .. } => match (state, button) {
                    (ElementState::Pressed, MouseButton::Left) => {
                        mouse_down = true;
                        toy_box.drag_acquire(mouse_pos);
                    }
                    (ElementState::Released, MouseButton::Left) => {
                        mouse_down = false;
                        toy_box.drag_release();
                    }
                    _ => (),
                },
                Event::CursorMoved { position, .. } => {
                    let world_pos = camera.logical_to_world(Vec4::new(
                        2.0 * position.x as f32 / viewport.x - 1.0,
                        1.0 - 2.0 * position.y as f32 / viewport.y,
                        0.0,
                        1.0,
                    ));
                    mouse_pos = world_pos.xy();

                    if mouse_down {
                        toy_box.drag_move(mouse_pos);
                    }
                }
                Event::CursorLeft { .. } => {
                    mouse_down = false;
                }
                _ => (),
            }
        }

        frame.clear(match mode {
            DrawMode::Normal => color::BLACK.mix(color::WHITE, 0.5),
            DrawMode::Debug => color::BLACK.to_rgba(),
        });

        let mut scene = frame.scene();
        scene.camera = camera;

        {
            let now = Instant::now();
            let frame_time = now - time;
            time = now;
            let dt = frame_time
                .min(Duration::from_millis(40))
                .div_f32(if mode == DrawMode::Debug { 10.0 } else { 1.0 });
            Rk4.solve_step(toy_box, dt.as_secs_f32());
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
