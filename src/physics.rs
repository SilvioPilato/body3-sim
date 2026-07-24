use std::rc::Rc;

use macroquad::math::Vec2;

use crate::quadtree::{Quadtree, WalkDecision};

pub const GRAVITY: f32 = 100_000.0;

// Plummer softening replaces the bare 1/r^2 singularity with
// 1/(r^2 + softening^2), capping the peak close-encounter force and bounding
// the smallest resolvable encounter timescale to ~sqrt(softening^3/(G*m)).
// Fixed-dt Verlet stays symplectic (energy conserved) only while that
// timescale >= dt.
//
// `mass` must be the mass of the body being encountered — for a swarm
// orbiter that's the core, not another light body. Undershooting it
// understates the required softening and injects energy (see
// examples/stability_sweep.rs). Callers get this from `min_softening` via
// `simulation::integration_params` instead of hardcoding it.
pub fn min_softening(dt: f32, mass: f32) -> f32 {
    (dt * dt * GRAVITY * mass).cbrt()
}

// Fallback for callers without a scenario handy (benches, examples that pin a
// value deliberately). NOT the production value — production derives it.
pub const DEFAULT_SOFTENING: f32 = 1.0;

// Default Barnes-Hut opening-angle threshold. Kept as a named constant so
// callers without a SimulationConfig handy (benches, examples) pick up the
// production default. Production threads `SimulationConfig.theta_threshold`
// through the call chain instead of reading this constant directly.
pub const DEFAULT_TETHA_THRESHOLD: f32 = 1.8;

#[derive(Clone, Debug, Copy)]
pub struct PhysicsObject {
    pub position: Vec2,
    pub velocity: Vec2,
    pub mass: f32,
}

pub struct Physics;
pub struct Verlet;

pub trait PhysicsSystem {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32, theta: f32, softening: f32) -> Rc<Vec<PhysicsObject>>;
}

impl PhysicsSystem for Verlet {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32, theta: f32, softening: f32) -> Rc<Vec<PhysicsObject>> {
        let (objects, _) = Self::execute_cached(objects, dt, center, half_size, theta, softening, None);
        objects
    }
}

impl Verlet {
    // Same integration as `execute`, but accepts the previous step's acc_new
    // as this step's acc_old instead of recomputing it — valid because
    // nothing moves between one step's end and the next step's start, so
    // acc_new(t) was evaluated at the exact position acc_old(t+1) needs.
    // Pass `None` on the first call.
    pub fn execute_cached(
        objects: Rc<Vec<PhysicsObject>>,
        dt: f32,
        center: Vec2,
        half_size: f32,
        theta: f32,
        softening: f32,
        prev_acc_new: Option<&[Vec2]>,
    ) -> (Rc<Vec<PhysicsObject>>, Vec<Vec2>) {
        let mut objects = (*objects).clone();
        let acc_old = match prev_acc_new {
            Some(acc) => acc.to_vec(),
            None => Physics::compute_accelerations(&objects, center, half_size, theta, softening),
        };

        for (obj, acc) in objects.iter_mut().zip(acc_old.iter()) {
            obj.position += obj.velocity * dt + 0.5 * *acc * dt * dt;
        }

        let acc_new = Physics::compute_accelerations(&objects, center, half_size, theta, softening);

        for ((obj, a_old), a_new) in objects.iter_mut().zip(acc_old.iter()).zip(acc_new.iter()) {
            obj.velocity += 0.5 * (*a_old + *a_new) * dt;
        }

        (Rc::new(objects), acc_new)
    }
}

impl Physics {
    pub fn total_energy(objects: &[PhysicsObject], softening: f32) -> f32 {
        let kinetic: f32 = objects
            .iter()
            .map(|o| 0.5 * o.mass * o.velocity.length_squared())
            .sum();

        let potential: f32 = (0..objects.len())
            .flat_map(|i| (i + 1..objects.len()).map(move |j| (i, j)))
            .map(|(i, j)| {
                let dist_sq = Vec2::distance_squared(objects[i].position, objects[j].position) + softening * softening;
                -GRAVITY * objects[i].mass * objects[j].mass / dist_sq.sqrt()
            })
            .sum();

        kinetic + potential
    }

    // Same Barnes-Hut tree/theta test as compute_accelerations, applied to
    // potential energy: O(n log n) instead of exact total_energy's O(n^2).
    // walk_potential visits each pair from both sides, so its raw sum
    // double-counts everything uniformly; halved once at the end here.
    pub fn total_energy_approx(objects: &[PhysicsObject], center: Vec2, half_size: f32, theta: f32, softening: f32) -> f32 {
        let kinetic: f32 = objects
            .iter()
            .map(|o| 0.5 * o.mass * o.velocity.length_squared())
            .sum();

        let tree = Quadtree::build(objects, center, half_size);
        kinetic + Self::walk_potential(objects, &tree, theta, softening)
    }

    fn walk_potential(objects: &[PhysicsObject], tree: &Quadtree, theta: f32, softening: f32) -> f32 {
        let theta_sq = theta * theta;
        let mut total = 0.0f32;
        for i in 0..objects.len() {
            let mut pair_sum = 0.0f32;
            tree.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    for &j in indices {
                        if j != i {
                            let dist_sq = Vec2::distance_squared(objects[i].position, objects[j].position) + softening * softening;
                            pair_sum += -GRAVITY * objects[i].mass * objects[j].mass / dist_sq.sqrt();
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d_sq = Vec2::distance_squared(objects[i].position, node.center_of_mass);
                    let width = node.half_size * 2.0;
                    if d_sq == 0.0 || width * width > theta_sq * d_sq {
                        WalkDecision::Descend
                    } else {
                        let dist_sq = d_sq + softening * softening;
                        pair_sum += -GRAVITY * objects[i].mass * node.total_mass / dist_sq.sqrt();
                        WalkDecision::Skip
                    }
                }
            });
            total += pair_sum;
        }
        total * 0.5
    }

    fn compute_acceleration(pos_a: Vec2, pos_b: Vec2, mass_b: f32, softening: f32) -> Vec2 {
        let delta = pos_b - pos_a;
        let dist_sq = Vec2::distance_squared(pos_a, pos_b) + softening * softening;
        let dist = dist_sq.sqrt();
        (GRAVITY * mass_b) / (dist_sq * dist) * delta
    }

    pub fn compute_accelerations(objects: &[PhysicsObject], center: Vec2, half_size: f32, theta: f32, softening: f32) -> Vec<Vec2> {
        let tree = Quadtree::build(objects, center, half_size);
        Self::walk_forces(objects, &tree, theta, softening)
    }

    // `objects` must be the exact slice (same length and order) that `tree` was
    // built from. A mismatched slice isn't memory-unsafe but silently produces
    // wrong accelerations (or panics on an out-of-bounds index).
    pub fn walk_forces(objects: &[PhysicsObject], tree: &Quadtree, theta: f32, softening: f32) -> Vec<Vec2> {
        // Opening test in squared distance to avoid a sqrt per internal-node
        // visit: (2*half)/d > theta  <=>  (2*half)^2 > theta^2 * d^2 (all terms
        // positive). The visit count dominates the walk, and the sqrt was thrown
        // away on every descend, so this removes the hottest sqrt entirely.
        let theta_sq = theta * theta;
        let mut res = Vec::with_capacity(objects.len());
        for i in 0..objects.len() {
            let mut acc = Vec2::ZERO;
            tree.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    for &j in indices {
                        if j != i {
                            acc += Physics::compute_acceleration(objects[i].position, objects[j].position, objects[j].mass, softening);
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let dist_sq = Vec2::distance_squared(objects[i].position, node.center_of_mass);
                    let width = node.half_size * 2.0;
                    if dist_sq == 0.0 || width * width > theta_sq * dist_sq {
                        WalkDecision::Descend
                    } else {
                        acc += Physics::compute_acceleration(objects[i].position, node.center_of_mass, node.total_mass, softening);
                        WalkDecision::Skip
                    }
                }
            });
            res.push(acc);
        }
        res
    }
}