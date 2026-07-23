use std::rc::Rc;

use body3_sim::physics::{Physics, PhysicsSystem, Verlet};
use body3_sim::quadtree::Quadtree;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use macroquad::math::vec2;

const SWARM_SIZES: [usize; 7] = [1000, 2000, 4000, 8000, 16000, 32000, 64000];
const SCREEN_SIZE: f32 = 800.0;
const PHYSICS_DT: f32 = 0.005;

fn build_sim(swarm_size: usize) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size },
        screen_size: SCREEN_SIZE,
        physics_dt: PHYSICS_DT,
        time_scale: 1.0,
    })
}

fn bench_quadtree_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("quadtree_build");
    for &n in &SWARM_SIZES {
        let sim = build_sim(n);
        let objects = sim.objects().to_vec();
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Quadtree::build(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_walk_forces(c: &mut Criterion) {
    let mut group = c.benchmark_group("walk_forces");
    for &n in &SWARM_SIZES {
        let sim = build_sim(n);
        let objects = sim.objects().to_vec();
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
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
        let sim = build_sim(n);
        let objects = sim.objects().to_vec();
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Physics::compute_accelerations(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_verlet_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("verlet_step");
    for &n in &SWARM_SIZES {
        let sim = build_sim(n);
        let objects = Rc::new(sim.objects().to_vec());
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Verlet::execute(objects.clone(), PHYSICS_DT, center, half_size));
        });
    }
    group.finish();
}

// Unlike bench_verlet_step (Verlet::execute, always recomputes both force
// evals), this measures Verlet::execute_cached given a *warm* acc_old — the
// state any step past the first actually runs with in Simulation::update.
// The warm-up step runs once, outside the timed region; every timed call
// then repeats from that exact same fixed (objects, acc) pair, matching
// bench_verlet_step's "same input every call" methodology so the two groups
// are directly comparable. (An earlier iter_custom version that threaded
// state across consecutive real steps let the swarm disperse over thousands
// of steps within a single sample, which cut per-step cost on its own and
// inflated the apparent speedup at small n — a measurement artifact, not
// this optimization.)
fn bench_verlet_step_cached(c: &mut Criterion) {
    let mut group = c.benchmark_group("verlet_step_cached");
    for &n in &SWARM_SIZES {
        let sim = build_sim(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
        let initial = Rc::new(sim.objects().to_vec());
        let (warm_objects, warm_acc) = Verlet::execute_cached(initial, PHYSICS_DT, center, half_size, None);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Verlet::execute_cached(warm_objects.clone(), PHYSICS_DT, center, half_size, Some(&warm_acc)));
        });
    }
    group.finish();
}

fn bench_clone_objects(c: &mut Criterion) {
    let mut group = c.benchmark_group("clone_objects");
    for &n in &SWARM_SIZES {
        let sim = build_sim(n);
        let objects = sim.objects().to_vec();
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
    bench_verlet_step_cached,
    bench_clone_objects
);
criterion_main!(benches);
