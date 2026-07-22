use std::rc::Rc;

use macroquad::math::Vec2;

use crate::quadtree::{NodeView, Quadtree, WalkDecision};

pub const GRAVITY: f32 = 100_000.0;
const SOFTENING: f32 = 0.001;
const TETHA_THRESHOLD: f32 = 0.5;

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
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32) -> Rc<Vec<PhysicsObject>>;
}

impl PhysicsSystem for EulerSimple {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32) -> Rc<Vec<PhysicsObject>> {
        let mut objects = (*objects).clone();
        let accelerations = Physics::compute_accelerations(&objects, center, half_size);
        for (obj, accelleration) in objects.iter_mut().zip(accelerations.iter()) {
            obj.velocity += *accelleration * dt;
            obj.position += obj.velocity * dt;
        }
        
        Rc::new(objects)
    }
}

impl PhysicsSystem for Verlet {
    fn execute(objects: Rc<Vec<PhysicsObject>>, dt: f32, center: Vec2, half_size: f32) -> Rc<Vec<PhysicsObject>> {
        let mut objects = (*objects).clone();
        let acc_old = Physics::compute_accelerations(&objects, center, half_size);

        for (obj, acc) in objects.iter_mut().zip(acc_old.iter()) {
            obj.position += obj.velocity * dt + 0.5 * *acc * dt * dt;
        }

        let acc_new = Physics::compute_accelerations(&objects, center, half_size);

        for ((obj, a_old), a_new) in objects.iter_mut().zip(acc_old.iter()).zip(acc_new.iter()) {
            obj.velocity += 0.5 * (*a_old + *a_new) * dt;
        }

        Rc::new(objects)
    }
}

impl Physics {
    pub fn total_energy(objects: &[PhysicsObject]) -> f32 {
        let kinetic: f32 = objects
            .iter()
            .map(|o| 0.5 * o.mass * o.velocity.length_squared())
            .sum();

        let potential: f32 = (0..objects.len())
            .flat_map(|i| (i + 1..objects.len()).map(move |j| (i, j)))
            .map(|(i, j)| {
                let dist_sq = Vec2::distance_squared(objects[i].position, objects[j].position) + SOFTENING * SOFTENING;
                -GRAVITY * objects[i].mass * objects[j].mass / dist_sq.sqrt()
            })
            .sum();

        kinetic + potential
    }

    fn compute_acceleration(pos_a: Vec2, pos_b: Vec2, mass_b: f32) -> Vec2 {
        let delta = pos_b - pos_a;
        let dist_sq = Vec2::distance_squared(pos_a, pos_b) + SOFTENING * SOFTENING;
        let dist = dist_sq.sqrt();
        (GRAVITY * mass_b) / (dist_sq * dist) * delta
    }

    pub fn compute_accelerations(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Vec<Vec2> {
        let tree = Quadtree::build(objects, center, half_size);
        Self::walk_forces(objects, &tree)
    }

    // `objects` must be the exact slice (same length and order) that `tree` was
    // built from. A mismatched slice isn't memory-unsafe but silently produces
    // wrong accelerations (or panics on an out-of-bounds index).
    pub fn walk_forces(objects: &[PhysicsObject], tree: &Quadtree<'_>) -> Vec<Vec2> {
        let mut res = Vec::new();
        for i in 0..objects.len() {
            let mut acc = Vec2::ZERO;
            tree.root.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    // foglia: forza diretta, i è catturato dalla closure
                    for &j in indices {
                        if j != i {
                            acc += Physics::compute_acceleration(objects[i].position, objects[j].position, objects[j].mass);
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d = Vec2::distance(objects[i].position, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > TETHA_THRESHOLD {
                        WalkDecision::Descend
                    } else {
                        acc += Physics::compute_acceleration(objects[i].position, node.center_of_mass, node.total_mass);
                        WalkDecision::Skip
                    }
                }
            }, &tree.objects);
            res.push(acc);
        }
        res
    }
}