use std::rc::Rc;

use body3_sim::physics::{Physics, PhysicsObject, PhysicsSystem, Verlet};
use body3_sim::quadtree::Quadtree;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use macroquad::math::vec2;

const SWARM_SIZES: [usize; 7] = [1000, 2000, 4000, 8000, 16000, 32000, 64000];
const SCREEN_SIZE: f32 = 800.0;
const PHYSICS_DT: f32 = 0.005;

fn build_objects(swarm_size: usize) -> Vec<PhysicsObject> {
    let sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size },
        screen_size: SCREEN_SIZE,
        physics_dt: PHYSICS_DT,
        time_scale: 1.0,
    });
    sim.objects().to_vec()
}

fn bench_quadtree_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("quadtree_build");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Quadtree::build(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_walk_forces(c: &mut Criterion) {
    let mut group = c.benchmark_group("walk_forces");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        let tree = Quadtree::build(&objects, center, half_size);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Physics::walk_forces(&objects, &tree));
        });
    }
    group.finish();
}

fn bench_compute_accelerations(c: &mut Criterion) {
    let mut group = c.benchmark_group("compute_accelerations");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Physics::compute_accelerations(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_verlet_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("verlet_step");
    for &n in &SWARM_SIZES {
        let objects = Rc::new(build_objects(n));
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Verlet::execute(objects.clone(), PHYSICS_DT, center, half_size));
        });
    }
    group.finish();
}

fn bench_clone_objects(c: &mut Criterion) {
    let mut group = c.benchmark_group("clone_objects");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| objects.to_vec());
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_quadtree_build,
    bench_walk_forces,
    bench_compute_accelerations,
    bench_verlet_step,
    bench_clone_objects
);
criterion_main!(benches);
