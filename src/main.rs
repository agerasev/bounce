//! Interactive controls are listed in the window and README. Simulation uses a
//! 240 Hz fixed RK4 step. Catch-up is capped at 24 steps per frame (100 ms);
//! excess elapsed time is discarded after stalls, rather than increasing dt.
//! Pause, focus loss, reset and speed changes discard accumulated time.
use std::time::Duration;

use bounce::{DrawMode, TextureStorage, World, sample_item};
use glam::{Affine2, Vec2};
use phy::{Rk4, Solver};
use rand::{Rng, SeedableRng, rngs::SmallRng};
use wgame::{
    Event, Library, Result, Window,
    app::time::Instant,
    gfx::types::{Color, color},
    input::{
        event::{ElementState, MouseButton},
        keyboard::{KeyCode, PhysicalKey},
    },
    prelude::*,
};

const SEED: u64 = 0xdeadbeef;
const SCALE: f32 = 640.0;
const STEP: Duration = Duration::from_nanos(4_166_667);
const MAX_STEPS: u32 = 24;

fn reset(size: Vec2, rng: &mut SmallRng) -> World<Rk4> {
    *rng = SmallRng::seed_from_u64(SEED);
    let mut world = World::new(size);
    for _ in 0..8 {
        world.insert_item(sample_item(&mut *rng, world.wall_size()));
    }
    world
}

#[wgame::window(title = "Bounce", size = (1200, 900), resizable = true, vsync = true)]
async fn main(mut window: Window<'_>) -> Result<()> {
    let gfx = Library::new(window.graphics());
    // Asset generation has its own RNG so changing a texture cannot change reset.
    let textures = TextureStorage::new(&mut SmallRng::seed_from_u64(SEED), &gfx)?;
    let font_data =
        wgame::typography::FontData::new(include_bytes!("../assets/DejaVuSans.ttf").to_vec(), 0)
            .map_err(|err| err.context("cannot decode embedded DejaVuSans.ttf"))?;
    let font = gfx.make_font(&font_data);
    let raster = font.rasterize(18.0);
    let help = raster.text("+/-: add/remove   Drag: left mouse   Esc: quit");
    let mut status = raster.text("");
    let mut previous_status = String::new();
    let mut rng = SmallRng::seed_from_u64(SEED);
    let mut world = None;
    let mut mode = DrawMode::Normal;
    let mut paused = false;
    let mut slow = false;
    let mut focused = true;
    let mut events = window.input();
    // Keep physical coordinates so a stationary cursor remains valid after resize.
    let mut mouse = None;
    let mut last = Instant::now();
    let mut accumulated = Duration::ZERO;

    'frames: while let Some(mut frame) = window.next_frame().await? {
        let size = frame.size();
        let viewport = Vec2::new(size.0 as f32, size.1 as f32);
        let mut reset_clock = false;
        let world = world.get_or_insert_with(|| reset(viewport / SCALE, &mut rng));
        if frame.resized().is_some() {
            world.resize(viewport / SCALE);
            reset_clock = true;
        }
        let camera = frame
            .physical_camera()
            .transform(Affine2::from_scale_angle_translation(
                Vec2::splat(0.5 * SCALE),
                0.0,
                0.5 * viewport,
            ));
        while let Some(event) = events.try_next() {
            match event {
                Event::KeyboardInput { event, .. } if event.state.is_pressed() && !event.repeat => {
                    if let PhysicalKey::Code(key) = event.physical_key {
                        match key {
                            KeyCode::Escape => {
                                frame.discard();
                                break 'frames;
                            }
                            KeyCode::Equal | KeyCode::NumpadAdd => {
                                world.insert_item(sample_item(&mut rng, world.wall_size()))
                            }
                            KeyCode::Minus | KeyCode::NumpadSubtract if world.n_items() > 0 => {
                                world.remove_item(rng.random_range(0..world.n_items()));
                            }
                            KeyCode::Backslash => {
                                mode = if mode == DrawMode::Normal {
                                    DrawMode::Debug
                                } else {
                                    DrawMode::Normal
                                }
                            }
                            KeyCode::Space => {
                                paused = !paused;
                                reset_clock = true;
                            }
                            KeyCode::KeyS => {
                                slow = !slow;
                                reset_clock = true;
                            }
                            KeyCode::KeyR => {
                                *world = reset(viewport / SCALE, &mut rng);
                                reset_clock = true;
                            }
                            _ => (),
                        }
                    }
                }
                Event::MouseInput {
                    state,
                    button: MouseButton::Left,
                    ..
                } => match state {
                    ElementState::Pressed => {
                        if let Some(pos) = mouse.and_then(|pos| camera.screen_to_world(pos, size)) {
                            world.drag_acquire(pos);
                        }
                    }
                    ElementState::Released => world.drag_release(),
                },
                Event::CursorMoved { position, .. } => {
                    let pixel = Vec2::new(position.x as f32, position.y as f32);
                    mouse = Some(pixel);
                    if let Some(pos) = camera.screen_to_world(pixel, size) {
                        world.drag_move(pos);
                    }
                }
                Event::CursorLeft { .. } => {
                    world.drag_release();
                    mouse = None;
                }
                Event::Focused(value) => {
                    focused = value;
                    world.drag_release();
                    reset_clock = true;
                }
                _ => (),
            }
        }
        let now = Instant::now();
        if reset_clock || paused || !focused {
            accumulated = Duration::ZERO;
        } else {
            let elapsed = (now - last).min(STEP * MAX_STEPS);
            accumulated += if slow { elapsed / 10 } else { elapsed };
            for _ in 0..MAX_STEPS {
                if accumulated < STEP {
                    break;
                }
                Rk4.solve_step(world, STEP.as_secs_f32());
                accumulated -= STEP;
            }
        }
        last = now;
        frame.clear(if mode == DrawMode::Normal {
            color::BLACK.mix(color::WHITE, 0.5)
        } else {
            color::BLACK.to_rgba()
        });
        let mut scene = frame.scene();
        scene.camera = camera;
        world.draw(&gfx, &textures, &mut scene, mode);
        scene.render();

        let label = format!(
            "{} bodies | Space: {} | S: {} | \\: {} | R: reset{}",
            world.n_items(),
            if paused { "resume" } else { "pause" },
            if slow { "normal speed" } else { "slow motion" },
            if mode == DrawMode::Normal {
                "debug"
            } else {
                "normal"
            },
            if focused { "" } else { " | unfocused" }
        );
        if label != previous_status {
            status = raster.text(&label);
            previous_status = label;
        }
        let camera = frame.physical_camera();
        let mut overlay = frame.scene();
        overlay.camera = camera;
        overlay.add(
            &gfx.shapes()
                .rectangle((Vec2::ZERO, Vec2::new(viewport.x, 64.0)))
                .fill_color(color::BLACK),
        );
        overlay.add(&status.scale(raster.size()).move_to(Vec2::new(12.0, 24.0)));
        overlay.add(&help.scale(raster.size()).move_to(Vec2::new(12.0, 49.0)));
        overlay.render();
        frame.present();
    }
    Ok(())
}
