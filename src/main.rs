//! Simulation uses a 240 Hz fixed RK4 step, capped at 24 catch-up steps per frame.
//! The host owns layout/input/presentation; controls communicate through actions.
mod controls;
mod ui;
use bounce::{DrawMode, TextureStorage, World, sample_item};
use controls::{Action, Controls};
use glam::{Affine2, Vec2};
use phy::{Rk4, Solver};
use rand::{RngExt, SeedableRng, rngs::SmallRng};
use std::{cell::RefCell, rc::Rc, time::Duration};
use wgame::{
    Library, Result, Window, WindowHost,
    app::time::Instant,
    canvas::{Button, Event, Key},
    gfx::types::{Color, color},
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

#[wgame::window(title = "Bounce", logical_size = (1200.0, 900.0), resizable = true, vsync = true)]
async fn main(window: Window<'_>) -> Result<()> {
    let controls = Rc::new(RefCell::new(Controls::default()));
    let ui_controls = controls.clone();
    let host = wgame_egui::EguiWindow::new(window, move |ui, canvas| {
        ui::layout(ui, canvas, &mut ui_controls.borrow_mut())
    });
    run(host, controls).await
}

async fn run(mut host: impl WindowHost, shared: Rc<RefCell<Controls>>) -> Result<()> {
    let gfx = Library::new(host.graphics());
    // Asset generation cannot change the simulation's reset sequence.
    let textures = TextureStorage::new(&mut SmallRng::seed_from_u64(SEED), &gfx)?;
    let mut rng = SmallRng::seed_from_u64(SEED);
    let mut world = None;
    let mut last = Instant::now();
    let mut accumulated = Duration::ZERO;
    let mut previous_size = Vec2::ZERO;
    'frames: while let Some(mut frame) = host.next_frame().await? {
        let mut controls = shared.borrow_mut();
        let logical_size = frame.logical_size();
        let playground = Vec2::new(logical_size.0 as f32, logical_size.1 as f32);
        let world = world.get_or_insert_with(|| reset(playground / SCALE, &mut rng));
        let mut reset_clock = playground != previous_size;
        if reset_clock && frame.visible() {
            world.drag_release();
            world.resize(playground / SCALE);
            previous_size = playground;
        }
        let camera = frame
            .logical_camera()
            .transform(Affine2::from_scale_angle_translation(
                Vec2::splat(0.5 * SCALE),
                0.0,
                0.5 * playground,
            ));
        let world_point =
            |position| camera.screen_to_world(position * frame.scale_factor() as f32, frame.size());
        for event in &frame.input().events {
            match *event {
                Event::Key {
                    key,
                    pressed: true,
                    repeat: false,
                } => {
                    let action = match key {
                        Key::Escape => {
                            drop(controls);
                            frame.discard();
                            break 'frames;
                        }
                        Key::Plus => Some(Action::Add),
                        Key::Minus => Some(Action::Remove),
                        Key::Character('\\') => Some(Action::Mode),
                        Key::Space => Some(Action::Pause),
                        Key::Character('s') => Some(Action::Slow),
                        Key::Character('r') => Some(Action::Reset),
                        _ => None,
                    };
                    if let Some(action) = action {
                        controls.actions.push(action);
                    }
                }
                Event::Button {
                    button: Button::Primary,
                    pressed,
                    position,
                } => {
                    if pressed {
                        if let Some(pos) = world_point(position) {
                            world.drag_acquire(pos);
                        }
                    } else {
                        world.drag_release();
                    }
                }
                Event::Moved(position) => {
                    if let Some(pos) = world_point(position) {
                        world.drag_move(pos);
                    }
                }
                Event::Cancelled | Event::Focused(_) => {
                    world.drag_release();
                    reset_clock = true;
                }
                _ => {}
            }
        }
        for action in std::mem::take(&mut controls.actions) {
            match action {
                Action::Add => world.insert_item(sample_item(&mut rng, world.wall_size())),
                Action::Remove if world.n_items() > 0 => {
                    world.remove_item(rng.random_range(0..world.n_items()));
                }
                Action::Mode => {
                    controls.mode = if controls.mode == DrawMode::Normal {
                        DrawMode::Debug
                    } else {
                        DrawMode::Normal
                    }
                }
                Action::Pause => {
                    controls.paused = !controls.paused;
                    reset_clock = true;
                }
                Action::Slow => {
                    controls.slow = !controls.slow;
                    reset_clock = true;
                }
                Action::Reset => {
                    *world = reset(playground / SCALE, &mut rng);
                    reset_clock = true;
                }
                _ => {}
            }
        }
        let now = Instant::now();
        if reset_clock || controls.paused || !frame.input().window_focused || !frame.visible() {
            accumulated = Duration::ZERO;
        } else {
            let elapsed = (now - last).min(STEP * MAX_STEPS);
            accumulated += if controls.slow { elapsed / 10 } else { elapsed };
            for _ in 0..MAX_STEPS {
                if accumulated < STEP {
                    break;
                }
                Rk4.solve_step(world, STEP.as_secs_f32());
                accumulated -= STEP;
            }
        }
        last = now;
        controls.bodies = world.n_items();
        frame.clear(if controls.mode == DrawMode::Normal {
            color::BLACK.mix(color::WHITE, 0.5)
        } else {
            color::BLACK.to_rgba()
        });
        let visible = frame.visible();
        let mut scene = frame.scene();
        scene.camera = camera;
        if visible {
            world.draw(&gfx, &textures, &mut scene, controls.mode);
        }
        scene.render();
        drop(controls);
        frame.present();
    }
    Ok(())
}
