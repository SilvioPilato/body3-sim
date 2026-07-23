// Walk-forces scaling diagnostic: isolate which hypothesis family explains
// the super-logarithmic growth (≈172x measured vs ≈102.5x O(n log n) predicted
// for n=1000 -> n=64000) at the PRE-fix opening angle theta = 0.5.
// (Production now defaults theta to 1.8 via SimulationConfig; this example
// pins 0.5 so its ratio stays comparable to the original measurement.)
//
// Three scenarios share the SAME spatial extent (annulus from
// central_swarm_radii(N_MAX)) so only the mass distribution / radial profile
// differ. We time Physics::walk_forces at N_MIN and N_MAX and report ratios.
//
// Interpretation key (stated BEFORE running):
//   A≈172, B≈102, C≈172  -> F1 (central body), F2 excluded.
//   A≈172, B≈172, C≈102  -> F2 (radial distribution), F1 excluded.
//   A≈172, B≈172, C≈172  -> F1 & F2 excluded; cause in F3/F4/F5.
//   all ≈102             -> F6 (n=1000 measurement-floor artifact).

use std::f32::consts::TAU;
use std::time::Instant;

use body3_sim::physics::{GRAVITY, Physics, PhysicsObject, DEFAULT_SOFTENING};
use body3_sim::quadtree::Quadtree;
use body3_sim::simulation::central_swarm_radii;

use macroquad::math::{Vec2, vec2};

const SCREEN: f32 = 800.0;
const N_MIN: usize = 1_000;
const N_MAX: usize = 64_000;
const CENTRAL_MASS: f32 = 20_000.0;
const LIGHT_MASS: f32 = 1.0;
const GOLDEN_ANGLE: f32 = TAU * 0.618_034;
// This diagnostic characterizes the PRE-fix opening angle (theta = 0.5), the
// value production used before theta became a SimulationConfig field. Kept
// explicit so the ratio printed below stays comparable to the original
// ≈172x measurement that motivated the parameterization.
const DIAGNOSTIC_THETA: f32 = 0.5;

struct Timing {
    ns_median: u128,
    iterations: u32,
}

fn pick_iterations(n: usize) -> u32 {
    if n <= 8_000 { 5 } else if n <= 32_000 { 2 } else { 1 }
}

fn time_walk(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Timing {
    let tree = Quadtree::build(objects, center, half_size);
    let k = pick_iterations(objects.len());
    let mut samples: Vec<u128> = Vec::with_capacity(k as usize);
    for _ in 0..k {
        let t0 = Instant::now();
        let _ = Physics::walk_forces(objects, &tree, DIAGNOSTIC_THETA, DEFAULT_SOFTENING);
        samples.push(t0.elapsed().as_nanos());
    }
    samples.sort_unstable();
    let ns_median = samples[samples.len() / 2];
    Timing { ns_median, iterations: k }
}

fn build_a(n: usize, min_r: f32, max_r: f32, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: vec2(cx, cy), velocity: Vec2::ZERO, mass: CENTRAL_MASS });
    for i in 0..n {
        let radius = min_r + (max_r - min_r) * (i as f32 / n.max(1) as f32);
        let angle = GOLDEN_ANGLE * i as f32;
        let dir = vec2(angle.cos(), angle.sin());
        let position = vec2(cx, cy) + dir * radius;
        let speed = (GRAVITY * CENTRAL_MASS / radius).sqrt();
        let tangent = vec2(-dir.y, dir.x) * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: LIGHT_MASS });
    }
    objects
}

fn build_b(n: usize, min_r: f32, max_r: f32, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: vec2(cx, cy), velocity: Vec2::ZERO, mass: LIGHT_MASS });
    for i in 0..n {
        let radius = min_r + (max_r - min_r) * (i as f32 / n.max(1) as f32);
        let angle = GOLDEN_ANGLE * i as f32;
        let dir = vec2(angle.cos(), angle.sin());
        let position = vec2(cx, cy) + dir * radius;
        let speed = (GRAVITY * LIGHT_MASS / radius).sqrt();
        let tangent = vec2(-dir.y, dir.x) * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: LIGHT_MASS });
    }
    objects
}

fn build_c(n: usize, min_r: f32, max_r: f32, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: vec2(cx, cy), velocity: Vec2::ZERO, mass: CENTRAL_MASS });
    let min_r2 = min_r * min_r;
    let max_r2 = max_r * max_r;
    for i in 0..n {
        let u = i as f32 / n.max(1) as f32;
        let radius = (min_r2 + u * (max_r2 - min_r2)).sqrt();
        let angle = GOLDEN_ANGLE * i as f32;
        let dir = vec2(angle.cos(), angle.sin());
        let position = vec2(cx, cy) + dir * radius;
        let speed = (GRAVITY * CENTRAL_MASS / radius).sqrt();
        let tangent = vec2(-dir.y, dir.x) * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: LIGHT_MASS });
    }
    objects
}

fn run(name: &str, build: fn(usize, f32, f32, Vec2) -> Vec<PhysicsObject>) {
    let (min_r, max_r) = central_swarm_radii(N_MAX);
    let half_size = (SCREEN / 2.0).max(max_r * 1.1);
    let center = vec2(SCREEN / 2.0, SCREEN / 2.0);

    let small = build(N_MIN, min_r, max_r, center);
    let large = build(N_MAX, min_r, max_r, center);

    let t_small = time_walk(&small, center, half_size);
    let t_large = time_walk(&large, center, half_size);

    let us_small = t_small.ns_median as f64 / 1_000.0;
    let ms_large = t_large.ns_median as f64 / 1_000_000.0;
    let ratio = t_large.ns_median as f64 / t_small.ns_median as f64;

    println!(
        "{name:<20} n={:<6} {:<8.2} us  n={:<6} {:<8.3} ms  ratio={:.1}x  (k={}/{})",
        N_MIN, us_small, N_MAX, ms_large, ratio,
        t_small.iterations, t_large.iterations,
    );
}

fn main() {
    println!("walk_diagnostic: walk_forces scaling, fixed annulus = central_swarm_radii(N_MAX)");
    let (min_r, max_r) = central_swarm_radii(N_MAX);
    println!("annulus: min_r={min_r:.3} max_r={max_r:.3}");
    println!();
    run("A current", build_a);
    run("B light-only", build_b);
    run("C uniform-area", build_c);
}