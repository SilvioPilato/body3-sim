use std::rc::Rc;

use macroquad::math::Vec2;

use crate::quadtree::{Quadtree, WalkDecision};

pub const GRAVITY: f32 = 100_000.0;

// Plummer softening replaces the bare 1/r^2 singularity with
// 1/(r^2 + softening^2), capping the peak close-encounter force at
// ~GRAVITY*m/softening^2 and the potential-well depth at ~GRAVITY*m/softening.
// That bounds the smallest resolvable encounter timescale to
// ~sqrt(softening^3 / (GRAVITY*m)); fixed-dt Verlet stays symplectic (energy
// conserved) only while that timescale >= dt.
//
// The mass in that expression is the mass of the body being encountered, and
// the binding encounter for every swarm orbiter is with the CORE (20000), not
// with another light body (1.0). Evaluating the criterion at the light mass —
// as this file previously did — understates the required softening by ~220x
// at the production dt, and the resulting energy injection is what made
// swarms fly apart (+54% energy over 10 simulated seconds at n=1000; see
// examples/stability_sweep.rs). Callers derive their softening from
// `min_softening` via `simulation::integration_params` instead of hardcoding.
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

pub struct Physics {

}
pub struct EulerSimple;
pub struct Verlet;

pub trait PhysicsSystem {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32, theta: f32, softening: f32) -> Rc<Vec<PhysicsObject>>;
}

impl PhysicsSystem for EulerSimple {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32, theta: f32, softening: f32) -> Rc<Vec<PhysicsObject>> {
        let mut objects = (*objects).clone();
        let accelerations = Physics::compute_accelerations(&objects, center, half_size, theta, softening);
        for (obj, accelleration) in objects.iter_mut().zip(accelerations.iter()) {
            obj.velocity += *accelleration * dt;
            obj.position += obj.velocity * dt;
        }
        
        Rc::new(objects)
    }
}

impl PhysicsSystem for Verlet {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32, theta: f32, softening: f32) -> Rc<Vec<PhysicsObject>> {
        let (objects, _) = Self::execute_cached(objects, dt, center, half_size, theta, softening, None);
        objects
    }
}

impl Verlet {
    // Same integration as `execute`, but accepts the previous step's acc_new
    // as this step's acc_old instead of recomputing it. Force only depends on
    // position, and acc_new(t) is evaluated at the exact position acc_old(t+1)
    // would be evaluated at (nothing moves between one step's end and the
    // next step's start) — so reusing it is exact, not an approximation.
    // Pass `None` on the first call (no prior acc_new exists yet).
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

    // Same Barnes-Hut approximation already trusted for compute_accelerations
    // (same tree, same `theta` opening-angle test), applied to potential
    // energy instead of force. O(n log n) instead of exact total_energy's
    // O(n^2), and — unlike Monte Carlo pair sampling — deterministic: no
    // run-to-run variance, no risk of missing a dominant close encounter or
    // flipping sign.
    //
    // Each unordered pair/cluster interaction is encountered from both
    // sides (body i's walk treats {i,cluster-containing-j} the same way
    // body j's walk treats {j,cluster-containing-i}), so walk_potential's
    // raw sum double-counts everything uniformly; total_energy_approx
    // halves it once at the end to correct for that.
    pub fn total_energy_approx(objects: &[PhysicsObject], center: Vec2, half_size: f32, theta: f32, softening: f32) -> f32 {
        let kinetic: f32 = objects
            .iter()
            .map(|o| 0.5 * o.mass * o.velocity.length_squared())
            .sum();

        let tree = Quadtree::build(objects, center, half_size);
        kinetic + Self::walk_potential(objects, &tree, theta, softening)
    }

    fn walk_potential(objects: &[PhysicsObject], tree: &Quadtree<'_>, theta: f32, softening: f32) -> f32 {
        let mut total = 0.0f32;
        for i in 0..objects.len() {
            let mut pair_sum = 0.0f32;
            tree.root.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    for &j in indices {
                        if j != i {
                            let dist_sq = Vec2::distance_squared(objects[i].position, objects[j].position) + softening * softening;
                            pair_sum += -GRAVITY * objects[i].mass * objects[j].mass / dist_sq.sqrt();
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d = Vec2::distance(objects[i].position, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > theta {
                        WalkDecision::Descend
                    } else {
                        let dist_sq = d * d + softening * softening;
                        pair_sum += -GRAVITY * objects[i].mass * node.total_mass / dist_sq.sqrt();
                        WalkDecision::Skip
                    }
                }
            }, &tree.objects);
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
    pub fn walk_forces(objects: &[PhysicsObject], tree: &Quadtree<'_>, theta: f32, softening: f32) -> Vec<Vec2> {
        let mut res = Vec::new();
        for i in 0..objects.len() {
            let mut acc = Vec2::ZERO;
            tree.root.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    // foglia: forza diretta, i è catturato dalla closure
                    for &j in indices {
                        if j != i {
                            acc += Physics::compute_acceleration(objects[i].position, objects[j].position, objects[j].mass, softening);
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d = Vec2::distance(objects[i].position, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > theta {
                        WalkDecision::Descend
                    } else {
                        acc += Physics::compute_acceleration(objects[i].position, node.center_of_mass, node.total_mass, softening);
                        WalkDecision::Skip
                    }
                }
            }, &tree.objects);
            res.push(acc);
        }
        res
    }
}