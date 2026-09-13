use super::{Item, World};
use either::Either;
use geom2::{
    ArcVertex, Circle, Disk, HalfPlane, Integrable, Intersect, IntersectTo, Meta, MetaArcPolygon,
    MetaPolygon, Moment, Polygon,
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
    pub fn contains(&self, pos: Vec2) -> bool {
        let local = self.rot.inverse().transform(pos - *self.pos);
        match self.shape {
            Shape::Circle { radius } => local.length_squared() <= radius * radius,
            Shape::Rectangle { size } => local.abs().cmple(size).all(),
        }
    }

    pub fn geometry(&self) -> Either<Disk, Polygon<SmallVec<[Vec2; 4]>>> {
        match self.shape {
            Shape::Circle { radius } => Either::Left(Disk(Circle {
                center: *self.pos,
                radius,
            })),
            Shape::Rectangle { size } => {
                let corners = [
                    -size,
                    Vec2::new(size.x, -size.y),
                    size,
                    Vec2::new(-size.x, size.y),
                ]
                .map(|p| *self.pos + self.rot.transform(p));
                Either::Right(Polygon::<SmallVec<[Vec2; 4]>>::new(SmallVec::from(corners)))
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct Force {
    pub pos: Vec2,
    pub vector: Vec2,
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
    pub fn contact(&self, def: Vec2, pos: Vec2, vel: Vec2) -> Force {
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

        Force {
            pos,
            vector: total_f,
        }
    }

    /// Pin `loc_pos` point in local item coordinates to `target` point in world space.
    pub fn attract(&self, target: Vec2, self_pos: Vec2) -> Force {
        let loc_pos = self.rot.transform(self_pos);
        let rel_pos = target - (*self.pos + loc_pos);
        let vel = *self.vel + angular_to_linear2(*self.asp, loc_pos);

        // Elastic attraction
        let elast_f = ELAST * rel_pos;
        // Constant damping
        let damp_f = -MOUSE_DAMP * vel;
        // Total force
        let total_f = elast_f + damp_f;

        Force {
            pos: *self.pos + loc_pos,
            vector: total_f,
        }
    }
}

fn contact_wall<S: Solver>(item: &Item<S>, offset: f32, normal: Vec2) -> Option<Force> {
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
        Some(item.body.contact(dir * force, poa, Vec2::ZERO))
    } else {
        None
    }
}

impl<S: Solver> Item<S> {
    pub fn collide(&self, other: &Self) -> Option<(Force, Force)> {
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
            Some((
                self.contact(-force * dir, poa, other.vel_at(poa)),
                other.contact(force * dir, poa, self.vel_at(poa)),
            ))
        } else {
            None
        }
    }
}

/// Visits linear forces without changing solver variables or their derivatives.
fn visit_forces<S: Solver>(
    items: &[Item<S>],
    size: Vec2,
    drag: Option<(usize, Vec2, Vec2)>,
    mut apply: impl FnMut(usize, Force),
) {
    let wall = size - WALL_OFFSET * size.min_element();
    for (i, item) in items.iter().enumerate() {
        apply(
            i,
            Force {
                pos: *item.pos,
                vector: GRAV * item.mass - AIRF * item.shape.radius() * *item.vel,
            },
        );
        for (offset, normal) in [
            (-wall.x, Vec2::X),
            (-wall.x, Vec2::NEG_X),
            (-wall.y, Vec2::Y),
            (-wall.y, Vec2::NEG_Y),
        ] {
            if let Some(force) = contact_wall(item, offset, normal) {
                apply(i, force);
            }
        }
        for (j, other) in items.iter().enumerate().skip(i + 1) {
            if let Some((left, right)) = item.collide(other) {
                apply(i, left);
                apply(j, right);
            }
        }
    }
    if let Some((i, target, local)) = drag {
        apply(i, items[i].attract(target, local));
    }
}

impl<S: Solver> World<S> {
    /// Observes linear forces at the current state. Rotational air drag is a pure
    /// torque, so it has no arrow at a point of application.
    pub fn visit_forces(&self, mut apply: impl FnMut(Vec2, Vec2)) {
        visit_forces(&self.items, self.size, self.drag, |_, force| {
            apply(force.pos, force.vector)
        });
    }
}

impl<S: Solver> System<S> for World<S> {
    fn compute_derivs(&mut self, _: &S::Context) {
        self.forces.clear();
        self.forces.resize(self.items.len(), (Vec2::ZERO, 0.0));
        let items = &self.items;
        visit_forces(items, self.size, self.drag, |i, force| {
            self.forces[i].0 += force.vector;
            self.forces[i].1 += torque2(force.pos - *items[i].pos, force.vector);
        });
        for (item, &(force, torque)) in self.items.iter_mut().zip(&self.forces) {
            let radius = item.shape.radius();
            let body = &mut item.body;
            body.pos.deriv = *body.vel;
            body.rot.deriv = *body.asp;
            body.vel.deriv = force / body.mass;
            body.asp.deriv = (torque - AIRF * radius * *body.asp) / body.inm;
        }
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
