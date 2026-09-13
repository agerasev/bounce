use super::{Item, World};
use geom2::{
    Disk, HalfPlane, Polygon,
    pressure::{
        ContactLoad, ContactPiece, Interface, Pressure, PressureDomain, PressureError,
        PressureField,
    },
};
use glam::{DVec2, Vec2};
use phy::{Rot2, Solver, System, Var, Visitor, angular_to_linear2};

/// Mass factor.
pub const MASF: f32 = 1.0;
/// Moment of inertia factor.
pub const INMF: f32 = 0.2;

const GRAV: Vec2 = Vec2::new(0.0, 4.0);
const AIRF: f32 = 0.01;
/// Peak interior pressure, per unit thickness. Not the old area-force factor.
const PRESSURE: f64 = 200.0;
/// Pressure gradient inside a compliant wall.
const WALL_STIFFNESS: f64 = 1000.0;
/// Additional center spring outside the arena: U = 0.5*k*outside_distance².
/// Finite body pressure alone stops restoring once fully submerged in a wall.
const WALL_RECOVERY_STIFFNESS: f64 = 4000.0;
/// Velocity-dependent traction factors; intentionally much lower than before.
const DAMP: f64 = 0.02;
const FRICT: f64 = 0.04;
/// Dragging is an independent point spring.
const MOUSE_STIFFNESS: f32 = 200.0;
const MOUSE_DAMP: f32 = 4.0;
pub const WALL_OFFSET: f32 = 0.04;

type Triangle = Pressure<Polygon<[Vec2; 3]>>;

#[derive(Clone, Debug)]
pub enum Shape {
    Circle { radius: f32 },
    Rectangle { size: Vec2 },
}

impl Shape {
    pub fn radius(&self) -> f32 {
        match self {
            Shape::Circle { radius } => *radius,
            Shape::Rectangle { size } => size.min_element(),
        }
    }

    fn bounding_radius(&self) -> f32 {
        match self {
            Shape::Circle { radius } => *radius,
            Shape::Rectangle { size } => size.length(),
        }
    }
}

/// Rebuilt once per derivative evaluation, then reused by all pair contacts.
#[derive(Clone, Debug)]
pub(crate) enum ContactShape {
    Disk(Pressure<Disk>),
    Rectangle([Triangle; 4]),
}

impl<S: Solver> Item<S> {
    pub fn contains(&self, pos: Vec2) -> bool {
        let local = self.rot.inverse().transform(pos - *self.pos);
        match self.shape {
            Shape::Circle { radius } => local.length_squared() <= radius * radius,
            Shape::Rectangle { size } => local.abs().cmple(size).all(),
        }
    }

    fn contact_shape(&self) -> ContactShape {
        match self.shape {
            Shape::Circle { radius } => ContactShape::Disk(
                Pressure::disk(Disk::new(*self.pos, radius), PRESSURE)
                    .expect("body must have finite position and positive size"),
            ),
            Shape::Rectangle { size } => {
                let rotation = self.rot.matrix();
                let corners = [
                    -size,
                    Vec2::new(size.x, -size.y),
                    size,
                    Vec2::new(-size.x, size.y),
                ]
                .map(|p| *self.pos + rotation * p);
                ContactShape::Rectangle(core::array::from_fn(|i| {
                    Pressure::triangle(
                        Polygon::new([corners[i], corners[(i + 1) % 4], *self.pos]),
                        [0.0, 0.0, PRESSURE],
                    )
                    .expect("body must have finite position and nondegenerate rectangle geometry")
                }))
            }
        }
    }
}

/// Force and independent torque about `pos`, not a force at an assumed centroid.
#[derive(Clone, Copy, Debug)]
struct AppliedLoad {
    pos: DVec2,
    load: ContactLoad,
}

impl AppliedLoad {
    fn force(pos: Vec2, force: Vec2) -> Self {
        Self {
            pos: pos.as_dvec2(),
            load: ContactLoad {
                force: force.as_dvec2(),
                torque: 0.0,
            },
        }
    }

    fn about(self, reference: DVec2) -> ContactLoad {
        ContactLoad {
            force: self.load.force,
            torque: self.load.torque + (self.pos - reference).perp_dot(self.load.force),
        }
    }

    fn opposite(self) -> Self {
        Self {
            pos: self.pos,
            load: ContactLoad {
                force: -self.load.force,
                torque: -self.load.torque,
            },
        }
    }
}

#[derive(Clone, Default)]
pub struct Body<S: Solver> {
    pub mass: f32,
    pub pos: Var<Vec2, S>,
    pub vel: Var<Vec2, S>,
    pub inm: f32,
    pub rot: Var<Rot2, S>,
    pub asp: Var<f32, S>,
}

impl<S: Solver> Body<S> {
    fn vel_at(&self, p: DVec2) -> DVec2 {
        self.vel.as_dvec2() + *self.asp as f64 * (p - self.pos.as_dvec2()).perp()
    }

    fn attract(&self, target: Vec2, self_pos: Vec2) -> AppliedLoad {
        let loc_pos = self.rot.transform(self_pos);
        let rel_pos = target - (*self.pos + loc_pos);
        let vel = *self.vel + angular_to_linear2(*self.asp, loc_pos);
        AppliedLoad::force(
            *self.pos + loc_pos,
            MOUSE_STIFFNESS * rel_pos - MOUSE_DAMP * vel,
        )
    }
}

fn visit_interface<G: PressureDomain, H: PressureDomain>(
    a: &Pressure<G>,
    b: &Pressure<H>,
    visit: &mut impl FnMut(ContactPiece),
) {
    match a.interface(b) {
        Ok(interface) => interface.for_each(visit),
        // No normal exists on identical field polynomials. Choose zero on the
        // tied cell interior; ordinary interfaces and shared-edge half weights
        // still contribute around it. Exactly coincident identical bodies have
        // no preferred separating direction and receive no arbitrary impulse.
        Err(PressureError::CoincidentFields) => (),
        Err(error) => panic!("invalid pressure contact: {error:?}"),
    }
}

impl ContactShape {
    fn visit_pair(&self, other: &Self, visit: &mut impl FnMut(ContactPiece)) {
        match (self, other) {
            (Self::Disk(a), Self::Disk(b)) => visit_interface(a, b, visit),
            (Self::Disk(a), Self::Rectangle(bs)) => {
                for b in bs {
                    visit_interface(a, b, visit);
                }
            }
            (Self::Rectangle(as_), Self::Disk(b)) => {
                for a in as_ {
                    visit_interface(a, b, visit);
                }
            }
            (Self::Rectangle(as_), Self::Rectangle(bs)) => {
                for a in as_ {
                    for b in bs {
                        visit_interface(a, b, visit);
                    }
                }
            }
        }
    }

    fn visit_wall(&self, wall: &Pressure<HalfPlane>, visit: &mut impl FnMut(ContactPiece)) {
        match self {
            Self::Disk(a) => visit_interface(a, wall, visit),
            Self::Rectangle(cells) => {
                for cell in cells {
                    visit_interface(cell, wall, visit);
                }
            }
        }
    }
}

/// The elastic contribution is analytic. Three positive Gauss weights integrate
/// dissipative traction separately; every sample has nonpositive relative power.
/// Applying the same sample to both bodies preserves action/reaction and torque.
fn piece_load<S: Solver>(
    piece: ContactPiece,
    a: &Body<S>,
    b: Option<&Body<S>>,
    reference: DVec2,
) -> ContactLoad {
    let mut total = piece.integrate(reference);
    for (fraction, weight) in [
        (0.1127016653792583, 5.0 / 18.0),
        (0.5, 4.0 / 9.0),
        (0.8872983346207417, 5.0 / 18.0),
    ] {
        let (point, normal, pressure) = piece.sample(fraction);
        let velocity = a.vel_at(point) - b.map_or(DVec2::ZERO, |b| b.vel_at(point));
        let tangent = normal.perp();
        let normal_speed = DAMP * velocity.dot(normal);
        let tangent_speed = FRICT * velocity.dot(tangent);
        // Bounded factors avoid an attractive local normal traction on fast
        // separation and unbounded drag during a deeply overlapping spawn.
        let force = -pressure.max(0.0)
            * piece.length()
            * piece.weight
            * weight
            * (normal * (normal_speed / (1.0 + normal_speed.abs()))
                + tangent * (tangent_speed / (1.0 + tangent_speed.abs())));
        total += ContactLoad {
            force,
            torque: (point - reference).perp_dot(force),
        };
    }
    total
}

fn contact_pair<S: Solver>(
    a: &Item<S>,
    a_shape: &ContactShape,
    b: &Item<S>,
    b_shape: &ContactShape,
) -> Option<AppliedLoad> {
    let reach = a.shape.bounding_radius() as f64 + b.shape.bounding_radius() as f64;
    if (a.pos.as_dvec2() - b.pos.as_dvec2()).length_squared() >= reach * reach {
        return None;
    }
    let reference = a.pos.as_dvec2() + 0.5 * (b.pos.as_dvec2() - a.pos.as_dvec2());
    let mut total = ContactLoad::default();
    a_shape.visit_pair(b_shape, &mut |piece| {
        total += piece_load(piece, &a.body, Some(&b.body), reference)
    });
    Some(AppliedLoad {
        pos: reference,
        load: total,
    })
}

fn contact_wall<S: Solver>(
    item: &Item<S>,
    shape: &ContactShape,
    offset: f32,
    normal: Vec2,
) -> Option<AppliedLoad> {
    if item.pos.dot(normal) - item.shape.bounding_radius() >= offset {
        return None;
    }
    let wall = Pressure {
        geometry: HalfPlane { normal, offset },
        field: PressureField {
            origin: (normal * offset).as_dvec2(),
            quadratic: 0.0,
            linear: -WALL_STIFFNESS * normal.as_dvec2(),
            constant: 0.0,
        },
    };
    let reference = item.pos.as_dvec2();
    let mut total = ContactLoad::default();
    shape.visit_wall(&wall, &mut |piece| {
        total += piece_load(piece, &item.body, None, reference)
    });
    let outside = offset as f64 - reference.dot(normal.as_dvec2());
    if outside > 0.0 {
        let elastic = WALL_RECOVERY_STIFFNESS * outside;
        let speed = item.vel.as_dvec2().dot(normal.as_dvec2());
        let damping = 1.4
            * (WALL_RECOVERY_STIFFNESS * item.mass as f64).sqrt()
            * (outside / item.shape.radius() as f64).min(1.0);
        total.force += normal.as_dvec2() * (elastic - damping * speed).max(0.0);
    }
    Some(AppliedLoad {
        pos: reference,
        load: total,
    })
}

fn visit_loads<S: Solver>(
    items: &[Item<S>],
    shapes: &[ContactShape],
    wall: Vec2,
    drag: Option<(usize, Vec2, Vec2)>,
    mut apply: impl FnMut(usize, AppliedLoad),
) {
    for (i, item) in items.iter().enumerate() {
        apply(
            i,
            AppliedLoad::force(
                *item.pos,
                GRAV * item.mass - AIRF * item.shape.radius() * *item.vel,
            ),
        );
        for (offset, normal) in [
            (-wall.x, Vec2::X),
            (-wall.x, Vec2::NEG_X),
            (-wall.y, Vec2::Y),
            (-wall.y, Vec2::NEG_Y),
        ] {
            if let Some(load) = contact_wall(item, &shapes[i], offset, normal) {
                apply(i, load);
            }
        }
        for (j, other) in items.iter().enumerate().skip(i + 1) {
            if let Some(load) = contact_pair(item, &shapes[i], other, &shapes[j]) {
                apply(i, load);
                apply(j, load.opposite());
            }
        }
    }
    if let Some((i, target, local)) = drag {
        apply(i, items[i].attract(target, local));
    }
}

impl<S: Solver> World<S> {
    /// Observe forces immutably. Independent contact torques are represented by
    /// equivalent force couples, so debug arrows retain the full contact load.
    /// Rotational air drag remains omitted from the arrows.
    pub fn visit_forces(&self, mut apply: impl FnMut(Vec2, Vec2)) {
        let shapes: Vec<_> = self.items.iter().map(Item::contact_shape).collect();
        visit_loads(
            &self.items,
            &shapes,
            self.wall_size(),
            self.drag,
            |i, load| {
                apply(load.pos.as_vec2(), load.load.force.as_vec2());
                let arm = 0.5 * self.items[i].shape.radius() as f64;
                let force = DVec2::X * (load.load.torque / (2.0 * arm));
                if force != DVec2::ZERO {
                    apply((load.pos - arm * DVec2::Y).as_vec2(), force.as_vec2());
                    apply((load.pos + arm * DVec2::Y).as_vec2(), -force.as_vec2());
                }
            },
        );
    }
}

impl<S: Solver> System<S> for World<S> {
    fn compute_derivs(&mut self, _: &S::Context) {
        self.forces.clear();
        self.forces.resize(self.items.len(), ContactLoad::default());
        self.contacts.clear();
        self.contacts
            .extend(self.items.iter().map(Item::contact_shape));
        let wall = self.wall_size();
        let items = &self.items;
        visit_loads(items, &self.contacts, wall, self.drag, |i, load| {
            self.forces[i] += load.about(items[i].pos.as_dvec2());
        });
        for (item, load) in self.items.iter_mut().zip(&self.forces) {
            let radius = item.shape.radius();
            let body = &mut item.body;
            body.pos.deriv = *body.vel;
            body.rot.deriv = *body.asp;
            body.vel.deriv = (load.force / body.mass as f64).as_vec2();
            body.asp.deriv =
                ((load.torque - (AIRF * radius * *body.asp) as f64) / body.inm as f64) as f32;
        }
    }

    fn visit_vars<V: Visitor<S>>(&mut self, visitor: &mut V) {
        for item in &mut self.items {
            visitor.apply(&mut item.pos);
            visitor.apply(&mut item.vel);
            visitor.apply(&mut item.rot);
            visitor.apply(&mut item.asp);
        }
    }
}

#[cfg(test)]
mod tests;
