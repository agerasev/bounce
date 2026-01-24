use super::{
    Item, World,
    geometry::{intersect_circle_and_plane, intersect_circles},
};
use glam::Vec2;
use metaphysics::{Rot2, Solver, System, Var, Visitor, angular_to_linear2, torque2};

/// Mass factor
pub const MASF: f32 = 1.0;
/// Moment of inertia factor
pub const INMF: f32 = 0.2;

/// Gravitaty
const GRAV: Vec2 = Vec2::new(0.0, 4.0);
/// Air resistance
const AIRF: f32 = 0.01;

/// Elasticity of balls
const ELAST: f32 = 30.0;

/// Damping factor.
const DAMP: f32 = 0.2;
/// Liquid friction
const FRICT: f32 = 0.4;

/// Attraction damping.
const ADAMP: f32 = 4.0;

#[derive(Clone, Debug)]
pub enum Shape {
    Circle {
        radius: f32,
    },
    Rectangle {
        /// Half len of rectangle sides
        size: Vec2,
    },
}

impl Shape {
    pub fn radius(&self) -> f32 {
        match self {
            Shape::Circle { radius } => *radius,
            Shape::Rectangle { size } => size.min_element(),
        }
    }
}

pub trait Actor<S: Solver> {
    /// Apply force to the specific point of the body.
    fn apply(&mut self, body: &mut Body<S>, pos: Vec2, force: Vec2);
}

struct DerivActor;
impl<S: Solver> Actor<S> for DerivActor {
    fn apply(&mut self, body: &mut Body<S>, pos: Vec2, force: Vec2) {
        body.vel.deriv += force / body.mass;
        body.asp.deriv += torque2(pos - *body.pos, force) / body.inm;
    }
}

/// Rigid body
#[derive(Clone, Default)]
pub struct Body<S: Solver> {
    pub mass: f32,
    pub pos: Var<Vec2, S>,
    pub vel: Var<Vec2, S>,

    /// Moment of inertia
    pub inm: f32,
    /// Rotation.
    pub rot: Var<Rot2, S>,
    /// Angular speed.
    pub asp: Var<f32, S>,
}

impl<S: Solver> Body<S> {
    fn vel_at(&self, p: Vec2) -> Vec2 {
        *self.vel + angular_to_linear2(*self.asp, p - *self.pos)
    }

    /// Influence item by directed deformation `def` at point of contact `pos` moving with velocity `vel`.
    pub fn contact(&mut self, actor: &mut impl Actor<S>, def: Vec2, pos: Vec2, vel: Vec2) {
        let vel = self.vel_at(pos) - vel;

        let norm = def.normalize_or_zero();
        // Elastic force (normal reaction)
        let elast_f = ELAST * def;

        // Damping force (parallel to `norm`)
        let damp_f = -DAMP * vel.dot(norm) * elast_f;
        // Liquid friction force (perpendicular to `norm`)
        let frict_f = -FRICT * vel.dot(norm.perp()) * elast_f.perp();
        // Total force
        let total_f = elast_f + damp_f + frict_f;

        actor.apply(self, pos, total_f);
    }

    /// Pin `loc_pos` point in local item coordinates to `target` point in world space.
    pub fn attract(&mut self, actor: &mut impl Actor<S>, target: Vec2, self_pos: Vec2) {
        let loc_pos = self.rot.transform(self_pos);
        let rel_pos = target - (*self.pos + loc_pos);
        let vel = *self.vel + angular_to_linear2(*self.asp, loc_pos);

        // Elastic attraction
        let elast_f = ELAST * rel_pos;
        // Constant damping
        let damp_f = -ADAMP * vel;
        // Total force
        let total_f = elast_f + damp_f;

        actor.apply(self, *self.pos + loc_pos, total_f);
    }
}

fn contact_wall<S: Solver>(
    actor: &mut impl Actor<S>,
    item: &mut Item<S>,
    offset: f32,
    normal: Vec2,
) {
    if let Some((area, barycenter)) =
        intersect_circle_and_plane(*item.pos, item.shape.radius(), offset, normal)
    {
        item.body
            .contact(actor, normal * area.sqrt(), barycenter, Vec2::ZERO);
    }
}

/// Wall offset factor
pub const WALL_OFFSET: f32 = 0.04;

impl<S: Solver> World<S> {
    pub fn compute_derivs_ext(&mut self, actor: &mut impl Actor<S>) {
        for item in self.items.iter_mut() {
            let radius = item.shape.radius();
            let body = &mut item.body;

            body.pos.deriv += *body.vel;
            body.rot.deriv += *body.asp;

            // Gravity
            actor.apply(body, *body.pos, GRAV * body.mass);

            // Air resistance
            body.vel.deriv += -(AIRF * radius / body.mass) * *body.vel;
            body.asp.deriv += -(AIRF * radius / body.inm) * *body.asp;

            // Walls
            let wall_size = self.size - WALL_OFFSET * self.size.min_element();
            contact_wall(actor, item, -wall_size.x, Vec2::new(1.0, 0.0));
            contact_wall(actor, item, -wall_size.x, Vec2::new(-1.0, 0.0));
            contact_wall(actor, item, -wall_size.y, Vec2::new(0.0, 1.0));
            contact_wall(actor, item, -wall_size.y, Vec2::new(0.0, -1.0));
        }

        for i in 1..self.items.len() {
            let (left, other_items) = self.items.split_at_mut(i);
            let this = left.last_mut().unwrap();
            for other in other_items {
                if let Some((area, barycenter)) = intersect_circles(
                    *this.pos,
                    this.shape.radius(),
                    *other.pos,
                    other.shape.radius(),
                ) {
                    let dir = (*other.pos - *this.pos).normalize_or_zero();
                    let def = area.sqrt();
                    this.contact(actor, -def * dir, barycenter, other.vel_at(barycenter));
                    other.contact(actor, def * dir, barycenter, this.vel_at(barycenter));
                }
            }
        }

        if let Some((i, target, loc_pos)) = self.drag {
            let item = &mut self.items[i];
            item.body.attract(actor, target, loc_pos);
        }
    }
}

impl<S: Solver> System<S> for World<S> {
    fn compute_derivs(&mut self, _dt: f32) {
        self.compute_derivs_ext(&mut DerivActor);
    }
    fn visit_vars<V: Visitor<Solver = S>>(&mut self, visitor: &mut V) {
        for ent in &mut self.items {
            visitor.apply(&mut ent.pos);
            visitor.apply(&mut ent.vel);
            visitor.apply(&mut ent.rot);
            visitor.apply(&mut ent.asp);
        }
    }
}
