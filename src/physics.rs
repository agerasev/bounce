use super::{Item, World};
use either::Either;
use geom2::{
    ArcVertex, Circle, Disk, HalfPlane, Integrable, Intersect, IntersectTo, LineSegment, Meta,
    MetaArcPolygon, MetaPolygon, Moment, Polygon,
};
use glam::Vec2;
use phy::{Rot2, Solver, System, Var, Visitor, angular_to_linear2, torque2};
use smallvec::SmallVec;

const AREA_EPS: f32 = 0.0;

/// Mass factor
pub const MASF: f32 = 1.0;
/// Moment of inertia factor
pub const INMF: f32 = 0.2;

/// Gravity
const GRAV: Vec2 = Vec2::new(0.0, 4.0);
/// Air resistance
const AIRF: f32 = 0.01;

/// Elasticity of balls
const ELAST: f32 = 200.0;

/// Damping factor.
const DAMP: f32 = 0.2;
/// Liquid friction
const FRICT: f32 = 0.4;

/// Mouse attraction damping.
const MOUSE_DAMP: f32 = 4.0;

/// Wall offset factor
pub const WALL_OFFSET: f32 = 0.04;

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

impl<S: Solver> Item<S> {
    pub fn geometry(&self) -> Either<Disk, Polygon<SmallVec<[Vec2; 4]>>> {
        match self.shape {
            Shape::Circle { radius } => Either::Left(Disk(Circle {
                center: *self.pos,
                radius,
            })),
            Shape::Rectangle { size } => {
                let vertex = Vec2::from_angle(self.rot.angle()).rotate(size);
                Either::Right(Polygon::<SmallVec<[Vec2; 4]>>::new(SmallVec::from([
                    *self.pos - vertex,
                    *self.pos - vertex.perp(),
                    *self.pos + vertex,
                    *self.pos + vertex.perp(),
                ])))
            }
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
        let damp_f = -MOUSE_DAMP * vel;
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
    let wall = HalfPlane { normal, offset };
    let overlay = match item.geometry() {
        Either::Left(left) => left.intersect(&wall).map(|x| x.moment()),
        Either::Right(right) => right
            .intersect_to(&wall)
            .map(|x: Polygon<SmallVec<[Vec2; 5]>>| x.moment()),
    };
    if let Some(overlay) = overlay
        && overlay.area > AREA_EPS
    {
        let dir = normal;
        let force = overlay.area;
        let poa = overlay.centroid;
        item.body.contact(actor, dir * force, poa, Vec2::ZERO);
    }
}

impl<S: Solver> Item<S> {
    pub fn collide(&mut self, other: &mut Self, actor: &mut impl Actor<S>) -> Option<()> {
        let (area, dir, poa) = match (self.geometry(), other.geometry()) {
            (Either::Left(self_circle), Either::Left(other_circle)) => {
                let overlay = self_circle.intersect(&other_circle)?;
                let Moment { area, centroid } = overlay.moment();
                (area, *other.pos - *self.pos, centroid)
            }
            (Either::Left(circle), Either::Right(polygon))
            | (Either::Right(polygon), Either::Left(circle)) => {
                let overlay: MetaArcPolygon<SmallVec<[Meta<ArcVertex, f32>; 8]>, f32> =
                    Meta::new(circle, -0.5).intersect_to(&Meta::new(polygon, 0.5))?;
                let Moment { area, centroid } = overlay.map_vertices(|x| x.inner).moment();
                let dir = overlay
                    .edges()
                    .map(|a| a.chord().vec() * a.meta)
                    .sum::<Vec2>()
                    .normalize_or_zero()
                    .perp();
                (
                    area,
                    match self.shape {
                        Shape::Circle { .. } => dir,
                        Shape::Rectangle { .. } => -dir,
                    },
                    centroid,
                )
            }
            (Either::Right(self_polygon), Either::Right(other_polygon)) => {
                let overlay: MetaPolygon<SmallVec<[Meta<Vec2, f32>; 8]>, f32> =
                    Meta::new(self_polygon, -0.5).intersect_to(&Meta::new(other_polygon, 0.5))?;
                let Moment { area, centroid } = overlay.map_vertices(|x| x.inner).moment();
                let dir = overlay
                    .edges()
                    .map(|l| l.vec() * l.meta)
                    .sum::<Vec2>()
                    .normalize_or_zero()
                    .perp();
                (area, dir, centroid)
            }
        };

        if area > AREA_EPS {
            let force = area; // .sqrt();
            self.contact(actor, -force * dir, poa, other.vel_at(poa));
            other.contact(actor, force * dir, poa, self.vel_at(poa));
            Some(())
        } else {
            None
        }
    }
}

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
                this.collide(other, actor);
            }
        }

        if let Some((i, target, loc_pos)) = self.drag {
            let item = &mut self.items[i];
            item.body.attract(actor, target, loc_pos);
        }
    }
}

impl<S: Solver> System<S> for World<S> {
    fn compute_derivs(&mut self, _: &S::Context) {
        self.compute_derivs_ext(&mut DerivActor);
    }
    fn visit_vars<V: Visitor<S>>(&mut self, visitor: &mut V) {
        for ent in &mut self.items {
            visitor.apply(&mut ent.pos);
            visitor.apply(&mut ent.vel);
            visitor.apply(&mut ent.rot);
            visitor.apply(&mut ent.asp);
        }
    }
}
