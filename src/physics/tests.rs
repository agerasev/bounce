use super::*;
use crate::sample_item;
use phy::{Rk4, Solver};
use rand::{SeedableRng, rngs::SmallRng};
use rgb::Rgb;

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
