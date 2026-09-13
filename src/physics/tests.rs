use super::*;
use crate::sample_item;
use phy::{Rk4, Solver};
use rand::{Rng, SeedableRng, rngs::SmallRng};
use rgb::Rgb;

thread_local! {
    pub(super) static INTERFACE_CALLS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

fn item(shape: Shape, position: Vec2, angle: f32) -> Item<Rk4> {
    Item {
        body: Body {
            mass: 0.2,
            inm: 0.008,
            pos: Var::new(position),
            vel: Var::default(),
            rot: Var::new(Rot2::from_angle(angle)),
            asp: Var::default(),
        },
        shape,
        color: Rgb::new(1.0, 1.0, 1.0),
    }
}

fn disk(position: Vec2) -> Item<Rk4> {
    item(Shape::Circle { radius: 0.2 }, position, 0.0)
}
fn rectangle(position: Vec2, angle: f32) -> Item<Rk4> {
    item(
        Shape::Rectangle {
            size: Vec2::new(0.2, 0.15),
        },
        position,
        angle,
    )
}
fn pair(a: &Item<Rk4>, b: &Item<Rk4>) -> AppliedLoad {
    contact_pair(a, &a.contact_shape(), b, &b.contact_shape()).unwrap()
}
fn close(a: f64, b: f64, eps: f64) {
    assert!(
        (a - b).abs() <= eps * (1.0 + a.abs().max(b.abs())),
        "{a} != {b}"
    );
}

#[test]
fn pair_loads_preserve_force_and_angular_momentum() {
    for a in [disk(Vec2::ZERO), rectangle(Vec2::ZERO, 0.3)] {
        for b in [
            disk(Vec2::new(0.25, 0.1)),
            rectangle(Vec2::new(0.25, 0.1), -0.1),
        ] {
            let forward = pair(&a, &b).about(DVec2::ZERO);
            let reverse = pair(&b, &a).about(DVec2::ZERO);
            close(forward.force.x, -reverse.force.x, 1e-8);
            close(forward.force.y, -reverse.force.y, 1e-8);
            close(forward.torque, -reverse.torque, 1e-8);
            assert!(forward.force.dot(a.pos.as_dvec2() - b.pos.as_dvec2()) > 0.0);
        }
    }
}

#[test]
fn disk_is_not_spun_by_elastic_contact_but_rectangle_can_be() {
    let a = disk(Vec2::new(0.2, 0.16));
    let b = rectangle(Vec2::ZERO, 0.25);
    let load = pair(&a, &b);
    close(load.about(a.pos.as_dvec2()).torque, 0.0, 1e-8);
    assert!(load.opposite().about(b.pos.as_dvec2()).torque.abs() > 0.01);
}

#[test]
fn aligned_identical_square_fields_have_symmetric_response() {
    let square = |x| {
        item(
            Shape::Rectangle {
                size: Vec2::splat(0.25),
            },
            Vec2::new(x, 0.0),
            0.0,
        )
    };
    let a = square(-0.125);
    let b = square(0.125);
    // U(d) = P*h²*(4/3 - d/h + (d/h)^3/12) for identical square tent fields.
    let load = pair(&a, &b).about(DVec2::ZERO);
    close(load.force.x, -PRESSURE * 0.25 * 0.75, 1e-12);
    close(load.force.y, 0.0, 1e-12);
    close(load.torque, 0.0, 1e-12);
    let same = pair(&a, &a);
    close(same.load.force.length(), 0.0, 1e-12);
    close(same.load.torque, 0.0, 1e-12);
    let circle = disk(Vec2::ZERO);
    close(pair(&circle, &circle).load.force.length(), 0.0, 1e-12);
}

#[test]
fn contact_dissipation_has_nonpositive_power() {
    let mut a = rectangle(Vec2::ZERO, 0.25);
    let mut b = disk(Vec2::new(0.2, 0.15));
    let elastic = pair(&a, &b);
    for (velocity, spin) in [(Vec2::new(2.0, -3.0), 5.0), (Vec2::new(-20.0, 15.0), -50.0)] {
        *a.vel = velocity;
        *a.asp = spin;
        *b.vel = -0.7 * velocity;
        *b.asp = -0.4 * spin;
        let total = pair(&a, &b);
        let aa = total.about(a.pos.as_dvec2());
        let ea = elastic.about(a.pos.as_dvec2());
        let bb = total.opposite().about(b.pos.as_dvec2());
        let eb = elastic.opposite().about(b.pos.as_dvec2());
        let power = (aa.force - ea.force).dot(a.vel.as_dvec2())
            + (aa.torque - ea.torque) * spin as f64
            + (bb.force - eb.force).dot(b.vel.as_dvec2())
            + (bb.torque - eb.torque) * (*b.asp as f64);
        assert!(power < 0.0, "damping/friction power {power}");
    }
}

#[test]
fn all_walls_push_inward_and_preserve_disk_torque() {
    for normal in [Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y] {
        let a = disk(-0.9 * normal);
        let result = contact_wall(&a, &a.contact_shape(), -1.0, normal).unwrap();
        assert!(result.load.force.dot(normal.as_dvec2()) > 0.0);
        close(result.load.force.dot(normal.perp().as_dvec2()), 0.0, 1e-8);
        close(result.about(a.pos.as_dvec2()).torque, 0.0, 1e-8);
    }
}

#[test]
fn debug_force_observation_does_not_mutate_world() {
    let mut world = World::<Rk4>::new(Vec2::splat(1.0));
    world.insert_item(rectangle(Vec2::ZERO, 0.2));
    world.insert_item(disk(Vec2::new(0.2, 0.1)));
    let before: Vec<_> = world
        .items
        .iter()
        .map(|i| (*i.pos, *i.vel, i.rot.angle(), *i.asp))
        .collect();
    let observe = |world: &World<Rk4>| {
        let mut result = Vec::new();
        world.visit_forces(|p, f| result.push((p, f)));
        result
    };
    let first = observe(&world);
    assert_eq!(first, observe(&world));
    assert!(first.len() > 2);
    assert!(world.contacts.is_empty() && world.forces.is_empty());
    assert_eq!(
        before,
        world
            .items
            .iter()
            .map(|i| (*i.pos, *i.vel, i.rot.angle(), *i.asp))
            .collect::<Vec<_>>()
    );
    let net: DVec2 = first.iter().map(|(_, f)| f.as_dvec2()).sum();
    close(net.x, 0.0, 1e-5);
    close(net.y, 2.0 * 0.2 * GRAV.y as f64, 1e-5);
}

#[test]
fn crowded_seed_remains_finite_at_game_timestep() {
    let mut world = World::<Rk4>::new(Vec2::splat(0.5));
    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
    for _ in 0..64 {
        world.insert_item(sample_item(&mut rng, world.wall_size()));
    }
    let mut max_speed = 0.0_f32;
    let mut max_position = 0.0_f32;
    for _ in 0..480 {
        Rk4.solve_step(&mut world, 1.0 / 240.0);
        for item in &world.items {
            assert!(
                item.pos.is_finite()
                    && item.vel.is_finite()
                    && item.asp.is_finite()
                    && item.rot.angle().is_finite()
            );
            max_speed = max_speed.max(item.vel.length());
            max_position = max_position.max(item.pos.abs().max_element());
        }
    }
    eprintln!("crowded scene: peak speed {max_speed}, peak position {max_position}");
    assert!(max_speed < 100.0, "peak speed {max_speed}");
    assert!(max_position < 1.0, "wall escape: {max_position}");
}

#[test]
fn fully_submerged_body_still_has_conservative_wall_restoring_force() {
    let a = rectangle(Vec2::new(2.0, 0.0), 0.3);
    let load = contact_wall(&a, &a.contact_shape(), -1.0, Vec2::NEG_X).unwrap();
    // Entire body is in a region where wall pressure exceeds body pressure;
    // the additional center spring supplies the gradient of 0.5*k*(x-1)^2.
    close(load.load.force.x, -WALL_RECOVERY_STIFFNESS, 1e-10);
    close(load.load.force.y, 0.0, 1e-10);
    close(load.about(a.pos.as_dvec2()).torque, 0.0, 1e-10);
}

#[test]
fn ordinary_scene_dragging_and_resize_remain_finite() {
    let mut world = World::<Rk4>::new(Vec2::new(1.8, 1.3));
    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
    for _ in 0..8 {
        world.insert_item(sample_item(&mut rng, world.wall_size()));
    }
    let target = *world.items[0].pos;
    world.drag_acquire(target);
    assert!(world.drag.is_some());
    world.drag_move(target + Vec2::new(0.3, -0.2));
    for step in 0..1200 {
        if step == 60 {
            world.drag_release();
        }
        if step == 600 {
            world.resize(Vec2::splat(0.6));
            assert!(world.drag.is_none());
        }
        Rk4.solve_step(&mut world, 1.0 / 240.0);
        for item in &world.items {
            assert!(item.pos.is_finite() && item.vel.is_finite() && item.asp.is_finite());
            assert!(item.vel.length() < 100.0);
        }
    }
    for item in &world.items {
        assert!(item.pos.abs().max_element() < 0.8);
    }
}

#[test]
fn dropped_bodies_settle_within_five_seconds() {
    for body in [disk(Vec2::ZERO), rectangle(Vec2::ZERO, 0.3)] {
        let mut world = World::<Rk4>::new(Vec2::splat(1.0));
        world.insert_item(body);
        let mut tail_speed = 0.0_f32;
        for step in 0..1200 {
            Rk4.solve_step(&mut world, 1.0 / 240.0);
            let body = &world.items[0];
            if step >= 960 {
                tail_speed = tail_speed.max(body.vel.length() + 0.2 * body.asp.abs());
            }
        }
        eprintln!(
            "drop {:?}: final-second peak motion {tail_speed}",
            world.items[0].shape
        );
        assert!(
            tail_speed < 0.05,
            "body kept bouncing/rocking: {tail_speed}"
        );
    }
}

#[test]
fn sliding_ball_spins_up_and_reduces_contact_slip() {
    let mut world = World::<Rk4>::new(Vec2::new(5.0, 1.0));
    world.insert_item(disk(Vec2::ZERO));
    // Start at static equilibrium to isolate friction from drop/settling time.
    let floor_y = world.wall_size().y;
    let (mut lower, mut upper) = (floor_y - 0.2, floor_y);
    for _ in 0..32 {
        let body = &mut world.items[0];
        body.pos.y = 0.5 * (lower + upper);
        let support = contact_wall(body, &body.contact_shape(), -floor_y, Vec2::NEG_Y)
            .unwrap()
            .load
            .force
            .y;
        if -support < (body.mass * GRAV.y) as f64 {
            lower = body.pos.y;
        } else {
            upper = body.pos.y;
        }
    }
    *world.items[0].vel = Vec2::X;
    *world.items[0].asp = 0.0;
    for _ in 0..240 {
        Rk4.solve_step(&mut world, 1.0 / 240.0);
    }
    let body = &world.items[0];
    let floor = Pressure {
        geometry: HalfPlane {
            normal: Vec2::NEG_Y,
            offset: -world.wall_size().y,
        },
        field: PressureField {
            origin: DVec2::new(0.0, world.wall_size().y as f64),
            quadratic: 0.0,
            linear: DVec2::new(0.0, WALL_STIFFNESS),
            constant: 0.0,
        },
    };
    let mut slip = 0.0;
    let mut weight = 0.0;
    body.contact_shape().visit_wall(&floor, &mut |piece| {
        // Independent midpoint sampling of the whole contact patch.
        for sample in 0..32 {
            let (point, normal, pressure) = piece.sample((sample as f64 + 0.5) / 32.0);
            let w = pressure.max(0.0) * piece.length() * piece.weight / 32.0;
            slip += w * body.vel_at(point).dot(normal.perp()).abs();
            weight += w;
        }
    });
    assert!(weight > 0.0, "ball lost floor contact");
    slip /= weight;
    eprintln!(
        "sliding ball after 1 s: vx {}, spin {}, contact slip {slip}",
        body.vel.x, *body.asp
    );
    assert!(*body.asp > 1.0, "ball did not spin up: {}", *body.asp);
    assert!(body.vel.x > 0.1, "ball stopped instead of rolling");
    assert!(slip < 0.15, "excessive contact slip: {slip}");
}

#[test]
fn damped_impacts_agree_with_a_smaller_timestep() {
    for radius in [0.1, 0.2, 0.3] {
        let mut coarse = World::<Rk4>::new(Vec2::new(5.0, 1.0));
        let mut fine = World::<Rk4>::new(Vec2::new(5.0, 1.0));
        for world in [&mut coarse, &mut fine] {
            let mut body = item(Shape::Circle { radius }, Vec2::ZERO, 0.0);
            body.mass = MASF * radius;
            body.inm = INMF * body.mass * radius;
            *body.vel = Vec2::new(0.7, 0.0);
            world.insert_item(body);
        }
        let mut position_error = 0.0_f32;
        let mut velocity_error = 0.0_f32;
        for _ in 0..480 {
            Rk4.solve_step(&mut coarse, 1.0 / 240.0);
            for _ in 0..4 {
                Rk4.solve_step(&mut fine, 1.0 / 960.0);
            }
            let a = &coarse.items[0];
            let b = &fine.items[0];
            assert!(a.pos.is_finite() && a.vel.is_finite() && a.asp.is_finite());
            position_error = position_error.max((*a.pos - *b.pos).length());
            velocity_error =
                velocity_error.max((*a.vel - *b.vel).length() + radius * (*a.asp - *b.asp).abs());
        }
        eprintln!(
            "radius {radius}: timestep position error {position_error}, motion error {velocity_error}"
        );
        assert!(position_error < 0.001, "position error {position_error}");
        assert!(velocity_error < 0.01, "motion error {velocity_error}");
    }
}

/// Frozen poses make before/after timings comparable even when trajectories
/// diverge from floating-point rounding. One RK4 step performs four evaluations.
#[test]
#[ignore = "manual release-mode contact benchmark"]
fn benchmark_contact_evaluation() {
    use std::{hint::black_box, time::Instant};
    for (label, bodies, half_size, repetitions) in [
        ("8 ordinary", 8, Vec2::new(1.8, 1.3), 2000),
        ("64 sparse", 64, Vec2::splat(4.0), 500),
        ("64 packed", 64, Vec2::splat(0.5), 30),
        ("256 sparse", 256, Vec2::splat(8.0), 100),
    ] {
        let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
        let mut world = World::<Rk4>::new(half_size);
        for _ in 0..bodies {
            let mut item = sample_item(&mut rng, world.wall_size());
            // Include rotated rectangles, not just the default aligned spawn.
            *item.rot = Rot2::from_angle(rng.random_range(-1.0..1.0));
            world.insert_item(item);
        }
        Rk4.solve_step(&mut world, 0.0);
        INTERFACE_CALLS.with(|count| count.set(0));
        Rk4.solve_step(&mut world, 0.0);
        let calls = INTERFACE_CALLS.with(|count| count.get() / 4);
        let mut samples = [0.0; 3];
        for time in &mut samples {
            let start = Instant::now();
            for _ in 0..repetitions {
                Rk4.solve_step(black_box(&mut world), 0.0);
                black_box(&world.forces);
            }
            *time = start.elapsed().as_secs_f64() * 1e6 / (4 * repetitions) as f64;
        }
        samples.sort_by(f64::total_cmp);
        eprintln!(
            "{label}: {:.2} us/evaluation, {calls} interface calls/evaluation (median of 3)",
            samples[1]
        );
    }
}

/// Reference path uses original geom2 domains, with neither cached constraints
/// nor any body/cell bounds. This checks both optimizations against full contact.
struct OriginalDomain<'a>(&'a dyn PressureDomain);

impl PressureDomain for OriginalDomain<'_> {
    fn constraints(
        &self,
        visit: &mut dyn FnMut(PressureField, BoundaryOwnership),
    ) -> Result<(), PressureError> {
        self.0.constraints(visit)
    }
}

fn original_cells(shape: &ContactShape) -> Vec<Pressure<OriginalDomain<'_>>> {
    fn original<G: PressureDomain, const N: usize>(
        cell: &ContactCell<G, N>,
    ) -> Pressure<OriginalDomain<'_>> {
        Pressure {
            geometry: OriginalDomain(&cell.pressure.geometry.geometry),
            field: cell.pressure.field,
        }
    }
    match &shape.cells {
        ContactCells::Disk(cell) => vec![original(cell)],
        ContactCells::Rectangle(cells) => cells.iter().map(original).collect(),
    }
}

#[test]
fn bounds_and_prepared_domains_preserve_exhaustive_pair_loads() {
    let mut rng = SmallRng::seed_from_u64(0xaabb);
    let mut items: Vec<Item<Rk4>> = (0..40)
        .map(|_| {
            let mut item = sample_item(&mut rng, Vec2::splat(1.0));
            *item.rot = Rot2::from_angle(rng.random_range(-3.0..3.0));
            *item.vel = Vec2::new(rng.random_range(-2.0..2.0), rng.random_range(-2.0..2.0));
            *item.asp = rng.random_range(-5.0..5.0);
            item
        })
        .collect();
    items.extend([
        disk(Vec2::ZERO),
        disk(Vec2::new(0.4, 0.0)),
        disk(Vec2::new(0.3999, 0.0)),
        rectangle(Vec2::ZERO, 0.0),
        rectangle(Vec2::new(0.2, 0.0), 0.0),
        item(
            Shape::Rectangle {
                size: Vec2::new(0.7, 0.02),
            },
            Vec2::new(0.2, 0.2),
            0.7,
        ),
    ]);
    let shapes: Vec<_> = items.iter().map(Item::contact_shape).collect();
    let mut fast_calls = 0;
    let mut reference_calls = 0;
    for (i, a) in items.iter().enumerate() {
        for (j, b) in items.iter().enumerate().skip(i + 1) {
            let reference = a.pos.as_dvec2() + 0.5 * (b.pos.as_dvec2() - a.pos.as_dvec2());
            INTERFACE_CALLS.with(|count| count.set(0));
            let actual = contact_pair(a, &shapes[i], b, &shapes[j])
                .map_or(ContactLoad::default(), |load| load.about(reference));
            fast_calls += INTERFACE_CALLS.with(|count| count.get());
            let mut expected = ContactLoad::default();
            INTERFACE_CALLS.with(|count| count.set(0));
            for left in &original_cells(&shapes[i]) {
                for right in &original_cells(&shapes[j]) {
                    visit_interface(left, right, &mut |piece| {
                        expected += piece_load(piece, &a.body, Some(&b.body), reference);
                    });
                }
            }
            reference_calls += INTERFACE_CALLS.with(|count| count.get());
            close(actual.force.x, expected.force.x, 1e-9);
            close(actual.force.y, expected.force.y, 1e-9);
            close(actual.torque, expected.torque, 1e-9);
        }
    }
    assert!(
        fast_calls < reference_calls / 2,
        "{fast_calls} versus {reference_calls}"
    );
}

#[test]
fn wall_bounds_preserve_every_orientation_of_pressure_contact() {
    for angle in [0.0, 0.1, 0.8, 1.5, 2.7] {
        for body in [
            disk(Vec2::new(0.3, -0.4)),
            rectangle(Vec2::new(0.3, -0.4), angle),
        ] {
            let shape = body.contact_shape();
            for normal in [Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y] {
                // Center stays inside so this isolates pressure contact from
                // the additional outside-center wall recovery spring.
                let offset = body.pos.dot(normal) - 0.05;
                let wall = Pressure {
                    geometry: HalfPlane { normal, offset },
                    field: PressureField {
                        origin: (normal * offset).as_dvec2(),
                        quadratic: 0.0,
                        linear: -WALL_STIFFNESS * normal.as_dvec2(),
                        constant: 0.0,
                    },
                };
                let reference = body.pos.as_dvec2();
                let actual = contact_wall(&body, &shape, offset, normal)
                    .unwrap()
                    .about(reference);
                let mut expected = ContactLoad::default();
                for cell in &original_cells(&shape) {
                    visit_interface(cell, &wall, &mut |piece| {
                        expected += piece_load(piece, &body.body, None, reference)
                    });
                }
                close(actual.force.x, expected.force.x, 1e-9);
                close(actual.force.y, expected.force.y, 1e-9);
                close(actual.torque, expected.torque, 1e-9);
            }
        }
    }
}
