use super::{Item, World};
use geom2::{
    Disk, HalfPlane, Polygon,
    pressure::{
        BoundaryOwnership, ContactLoad, ContactPiece, Interface, Pressure, PressureDomain,
        PressureError, PressureField,
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
/// Inverse-speed scales for bounded normal damping and tangential friction.
/// At speed 1/factor, the dissipative traction reaches half the local pressure.
const DAMP: f64 = 2.0;
const FRICT: f64 = 2.0;
/// Dragging is an independent point spring.
const MOUSE_STIFFNESS: f32 = 200.0;
const MOUSE_DAMP: f32 = 4.0;
pub const WALL_OFFSET: f32 = 0.04;

type Triangle = ContactCell<Polygon<[Vec2; 3]>, 3>;

/// Bounds of the actual pressure geometry, rebuilt at every RK4 stage.
#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: DVec2,
    max: DVec2,
}

impl Aabb {
    fn points(points: &[Vec2]) -> Self {
        let mut bounds = Self {
            min: DVec2::INFINITY,
            max: DVec2::NEG_INFINITY,
        };
        for point in points {
            bounds.min = bounds.min.min(point.as_dvec2());
            bounds.max = bounds.max.max(point.as_dvec2());
        }
        bounds
    }

    fn overlaps(self, other: Self) -> bool {
        // Keep touching boxes: shared cell edges can carry nonzero pressure.
        self.min.cmple(other.max).all() && other.min.cmple(self.max).all()
    }

    fn intersects_wall(self, normal: Vec2, offset: f32) -> bool {
        let nearest = DVec2::new(
            if normal.x >= 0.0 {
                self.min.x
            } else {
                self.max.x
            },
            if normal.y >= 0.0 {
                self.min.y
            } else {
                self.max.y
            },
        );
        nearest.dot(normal.as_dvec2()) <= offset as f64
    }
}

#[derive(Clone, Debug)]
struct ContactCell<G, const N: usize> {
    pressure: Pressure<PreparedDomain<G, N>>,
    bounds: Aabb,
}

/// Clipping rescans boundaries. Prepare validated constraints once per RK4
/// stage instead of reconstructing them on every scan of every cell pair.
#[derive(Clone, Debug)]
struct PreparedDomain<G, const N: usize> {
    geometry: G,
    constraints: [(PressureField, BoundaryOwnership); N],
}

impl<G, const N: usize> PressureDomain for PreparedDomain<G, N> {
    fn constraints(
        &self,
        visit: &mut dyn FnMut(PressureField, BoundaryOwnership),
    ) -> Result<(), PressureError> {
        for &(field, ownership) in &self.constraints {
            visit(field, ownership);
        }
        Ok(())
    }
}

impl<G: PressureDomain, const N: usize> ContactCell<G, N> {
    fn new(pressure: Pressure<G>, bounds: Aabb) -> Self {
        let zero = PressureField {
            origin: DVec2::ZERO,
            quadratic: 0.0,
            linear: DVec2::ZERO,
            constant: 0.0,
        };
        let mut constraints = [(zero, BoundaryOwnership::Closed); N];
        let mut count = 0;
        pressure
            .geometry
            .constraints(&mut |field, ownership| {
                constraints[count] = (field, ownership);
                count += 1;
            })
            .expect("body pressure geometry must be valid");
        assert_eq!(count, N);
        Self {
            pressure: Pressure {
                geometry: PreparedDomain {
                    geometry: pressure.geometry,
                    constraints,
                },
                field: pressure.field,
            },
            bounds,
        }
    }
}

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
}

/// Rebuilt once per derivative evaluation, then reused by all pair contacts.
#[derive(Clone, Debug)]
pub(crate) struct ContactShape {
    bounds: Aabb,
    cells: ContactCells,
}

#[derive(Clone, Debug)]
enum ContactCells {
    Disk(ContactCell<Disk, 1>),
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

    fn air_torque(&self) -> f64 {
        -(AIRF * self.shape.radius() * *self.asp) as f64
    }

    fn contact_shape(&self) -> ContactShape {
        match self.shape {
            Shape::Circle { radius } => {
                let pressure = Pressure::disk(Disk::new(*self.pos, radius), PRESSURE)
                    .expect("body must have finite position and positive size");
                let bounds = Aabb {
                    min: self.pos.as_dvec2() - DVec2::splat(radius as f64),
                    max: self.pos.as_dvec2() + DVec2::splat(radius as f64),
                };
                ContactShape {
                    bounds,
                    cells: ContactCells::Disk(ContactCell::new(pressure, bounds)),
                }
            }
            Shape::Rectangle { size } => {
                let rotation = self.rot.matrix();
                let corners = [
                    -size,
                    Vec2::new(size.x, -size.y),
                    size,
                    Vec2::new(-size.x, size.y),
                ]
                .map(|p| *self.pos + rotation * p);
                ContactShape {
                    bounds: Aabb::points(&corners),
                    cells: ContactCells::Rectangle(core::array::from_fn(|i| {
                        let vertices = [corners[i], corners[(i + 1) % 4], *self.pos];
                        let pressure = Pressure::triangle(
                            Polygon::new(vertices),
                            [0.0, 0.0, PRESSURE],
                        )
                        .expect(
                            "body must have finite position and nondegenerate rectangle geometry",
                        );
                        ContactCell::new(pressure, Aabb::points(&vertices))
                    })),
                }
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

    fn with_reference(self, pos: DVec2) -> Self {
        Self {
            pos,
            load: self.about(pos),
        }
    }

    /// Draw the resultant on its line of action, at the point nearest `pos`.
    /// A pure torque has no single-force representation and needs a couple.
    fn visit_forces(self, arm: f64, mut apply: impl FnMut(Vec2, Vec2)) {
        let ContactLoad { force, torque } = self.load;
        let force_squared = force.length_squared();
        if force_squared > 0.0 {
            // r × F = torque; translating along F leaves the moment unchanged.
            let point = self.pos - force.perp() * (torque / force_squared);
            if point.as_vec2().is_finite() {
                apply(point.as_vec2(), force.as_vec2());
                return;
            }
        }
        // Also retain a finite representation when a tiny resultant would put
        // its application point outside the renderer's numeric range.
        if force != DVec2::ZERO {
            apply(self.pos.as_vec2(), force.as_vec2());
        }
        if torque != 0.0 {
            let couple = DVec2::X * (torque / (2.0 * arm));
            apply((self.pos - arm * DVec2::Y).as_vec2(), couple.as_vec2());
            apply((self.pos + arm * DVec2::Y).as_vec2(), -couple.as_vec2());
        }
    }
}

/// A contact-based anchor for the equivalent force arrow. Only collected for
/// debug observation, using the same positive quadrature as contact damping.
/// This locates the arrow; it does not approximate its force or moment.
#[derive(Default)]
struct ContactCenter {
    weight: f64,
    moment: DVec2,
}

impl ContactCenter {
    fn add(&mut self, piece: ContactPiece, reference: DVec2) {
        for (fraction, weight) in CONTACT_QUADRATURE {
            let (point, _, pressure) = piece.sample(fraction);
            let weight = pressure.max(0.0) * piece.length() * piece.weight * weight;
            self.weight += weight;
            self.moment += weight * (point - reference);
        }
    }

    fn point(self, reference: DVec2) -> DVec2 {
        if self.weight > 0.0 {
            reference + self.moment / self.weight
        } else {
            reference
        }
    }
}

const CONTACT_QUADRATURE: [(f64, f64); 3] = [
    (0.1127016653792583, 5.0 / 18.0),
    (0.5, 4.0 / 9.0),
    (0.8872983346207417, 5.0 / 18.0),
];

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
    #[cfg(test)]
    tests::INTERFACE_CALLS.with(|count| count.set(count.get() + 1));
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
        match (&self.cells, &other.cells) {
            (ContactCells::Disk(a), ContactCells::Disk(b)) => {
                // Circles reject diagonal false positives left by their AABBs.
                let a_disk = &a.pressure.geometry.geometry;
                let b_disk = &b.pressure.geometry.geometry;
                let reach = a_disk.radius as f64 + b_disk.radius as f64;
                if (a_disk.center.as_dvec2() - b_disk.center.as_dvec2()).length_squared()
                    <= reach * reach
                {
                    visit_cell_pair(a, b, visit);
                }
            }
            (ContactCells::Disk(a), ContactCells::Rectangle(bs)) => {
                for b in bs {
                    visit_cell_pair(a, b, visit);
                }
            }
            (ContactCells::Rectangle(as_), ContactCells::Disk(b)) => {
                for a in as_ {
                    visit_cell_pair(a, b, visit);
                }
            }
            (ContactCells::Rectangle(as_), ContactCells::Rectangle(bs)) => {
                for a in as_ {
                    for b in bs {
                        visit_cell_pair(a, b, visit);
                    }
                }
            }
        }
    }

    fn visit_wall(&self, wall: &Pressure<HalfPlane>, visit: &mut impl FnMut(ContactPiece)) {
        match &self.cells {
            ContactCells::Disk(a) => visit_interface(&a.pressure, wall, visit),
            ContactCells::Rectangle(cells) => {
                for cell in cells {
                    if cell
                        .bounds
                        .intersects_wall(wall.geometry.normal, wall.geometry.offset)
                    {
                        visit_interface(&cell.pressure, wall, visit);
                    }
                }
            }
        }
    }
}

fn visit_cell_pair<G: PressureDomain, H: PressureDomain, const N: usize, const M: usize>(
    a: &ContactCell<G, N>,
    b: &ContactCell<H, M>,
    visit: &mut impl FnMut(ContactPiece),
) {
    if a.bounds.overlaps(b.bounds) {
        visit_interface(&a.pressure, &b.pressure, visit);
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
    for (fraction, weight) in CONTACT_QUADRATURE {
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

fn contact_pair<S: Solver, const DISPLAY: bool>(
    a: &Item<S>,
    a_shape: &ContactShape,
    b: &Item<S>,
    b_shape: &ContactShape,
) -> Option<AppliedLoad> {
    if !a_shape.bounds.overlaps(b_shape.bounds) {
        return None;
    }
    let reference = a.pos.as_dvec2() + 0.5 * (b.pos.as_dvec2() - a.pos.as_dvec2());
    let mut total = ContactLoad::default();
    let mut center = ContactCenter::default();
    a_shape.visit_pair(b_shape, &mut |piece| {
        total += piece_load(piece, &a.body, Some(&b.body), reference);
        if DISPLAY {
            center.add(piece, reference);
        }
    });
    let load = AppliedLoad {
        pos: reference,
        load: total,
    };
    Some(if DISPLAY {
        load.with_reference(center.point(reference))
    } else {
        load
    })
}

fn contact_wall<S: Solver, const DISPLAY: bool>(
    item: &Item<S>,
    shape: &ContactShape,
    offset: f32,
    normal: Vec2,
) -> Option<AppliedLoad> {
    if !shape.bounds.intersects_wall(normal, offset) {
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
    let mut center = ContactCenter::default();
    shape.visit_wall(&wall, &mut |piece| {
        total += piece_load(piece, &item.body, None, reference);
        if DISPLAY {
            center.add(piece, reference);
        }
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
    let load = AppliedLoad {
        pos: reference,
        load: total,
    };
    Some(if DISPLAY {
        load.with_reference(center.point(reference))
    } else {
        load
    })
}

fn visit_loads<S: Solver, const DISPLAY: bool>(
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
            if let Some(load) = contact_wall::<_, DISPLAY>(item, &shapes[i], offset, normal) {
                apply(i, load);
            }
        }
        for (j, other) in items.iter().enumerate().skip(i + 1) {
            if let Some(load) = contact_pair::<_, DISPLAY>(item, &shapes[i], other, &shapes[j]) {
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
    /// Observe forces immutably. Each nonzero resultant is placed on its line
    /// of action near the pressure-weighted contact center, reproducing the
    /// full contact torque with a single arrow.
    /// Pure torques require force couples instead.
    /// Rotational air drag is also shown as a pure torque.
    pub fn visit_forces(&self, mut apply: impl FnMut(Vec2, Vec2)) {
        let shapes: Vec<_> = self.items.iter().map(Item::contact_shape).collect();
        visit_loads::<_, true>(
            &self.items,
            &shapes,
            self.wall_size(),
            self.drag,
            |i, load| {
                let arm = 0.5 * self.items[i].shape.radius() as f64;
                load.visit_forces(arm, &mut apply);
            },
        );
        for item in &self.items {
            AppliedLoad {
                pos: item.pos.as_dvec2(),
                load: ContactLoad {
                    force: DVec2::ZERO,
                    torque: item.air_torque(),
                },
            }
            .visit_forces(0.5 * item.shape.radius() as f64, &mut apply);
        }
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
        visit_loads::<_, false>(items, &self.contacts, wall, self.drag, |i, load| {
            self.forces[i] += load.about(items[i].pos.as_dvec2());
        });
        for (item, load) in self.items.iter_mut().zip(&self.forces) {
            let air_torque = item.air_torque();
            let body = &mut item.body;
            body.pos.deriv = *body.vel;
            body.rot.deriv = *body.asp;
            body.vel.deriv = (load.force / body.mass as f64).as_vec2();
            body.asp.deriv = ((load.torque + air_torque) / body.inm as f64) as f32;
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
