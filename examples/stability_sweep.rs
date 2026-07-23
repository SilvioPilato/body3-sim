// The ablation in examples/escape_diagnostic.rs showed that no combination of
// the spawn-side fixes stops the swarm from flying apart: total energy grows
// by tens of percent within the first ~2 sim seconds in EVERY variant. Verlet
// is symplectic, so a fixed-dt run should not do that — which points at the
// integration parameters, not the initial conditions.
//
// This sweep varies one knob at a time over a fixed span of simulated time and
// reports energy drift, to find which one actually controls the divergence.
// The quadtree root is refitted every step throughout, so out-of-root
// misfiling cannot confound the comparison.

use std::rc::Rc;

use body3_sim::physics::{
    DEFAULT_SOFTENING, DEFAULT_TETHA_THRESHOLD, GRAVITY, Physics, PhysicsObject, Verlet,
};
use body3_sim::simulation::central_swarm_radii;
use macroquad::math::{Vec2, vec2};

const N: usize = 1_000;
const SIM_TIME: f32 = 10.0;
const CENTRAL_MASS: f32 = 20_000.0;
const LIGHT_MASS: f32 = 1.0;

fn build_n(center: Vec2, n: usize) -> Vec<PhysicsObject> {
    let (min_r, max_r) = central_swarm_radii(n);
    let golden = std::f32::consts::TAU * 0.618_034;
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: center, velocity: Vec2::ZERO, mass: CENTRAL_MASS });
    for i in 0..n {
        let radius = min_r + (max_r - min_r) * (i as f32 / n as f32);
        let angle = golden * i as f32;
        let dir = vec2(angle.cos(), angle.sin());
        let speed = (GRAVITY * CENTRAL_MASS / radius).sqrt();
        objects.push(PhysicsObject {
            position: center + dir * radius,
            velocity: vec2(-dir.y, dir.x) * speed,
            mass: LIGHT_MASS,
        });
    }
    objects
}

fn fitted_root(objects: &[PhysicsObject]) -> (Vec2, f32) {
    let mut lo = vec2(f32::INFINITY, f32::INFINITY);
    let mut hi = vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for o in objects {
        lo = lo.min(o.position);
        hi = hi.max(o.position);
    }
    (((lo + hi) * 0.5), ((hi - lo).max_element() * 0.5 * 1.01).max(1.0))
}

fn run(dt: f32, theta: f32, softening: f32) -> (f32, f32) {
    run_n(N, dt, theta, softening)
}

fn run_n(n: usize, dt: f32, theta: f32, softening: f32) -> (f32, f32) {
    run_full(n, dt, theta, softening, false)
}

// fix_c: recompute each orbiter's circular speed from the acceleration it
// actually feels, instead of from `central_mass` alone.
fn apply_fix_c(objects: &mut [PhysicsObject], center: Vec2, theta: f32, softening: f32) {
    for o in objects.iter_mut() {
        o.velocity = Vec2::ZERO;
    }
    let (c, half) = fitted_root(objects);
    let acc = Physics::compute_accelerations(objects, c, half, theta, softening);
    for (obj, a) in objects.iter_mut().zip(acc.iter()).skip(1) {
        let d = obj.position - center;
        let r = d.length();
        if r <= 0.0 {
            continue;
        }
        let dir = d / r;
        let a_radial = -a.dot(dir);
        if a_radial <= 0.0 {
            continue;
        }
        obj.velocity = vec2(-dir.y, dir.x) * (a_radial * r).sqrt();
    }
}

fn run_full(n: usize, dt: f32, theta: f32, softening: f32, fix_c: bool) -> (f32, f32) {
    let center = vec2(500.0, 500.0);
    let mut initial = build_n(center, n);
    if fix_c {
        apply_fix_c(&mut initial, center, theta, softening);
    }
    let mut objects = Rc::new(initial);
    let e0 = Physics::total_energy(&objects, softening);
    let steps = (SIM_TIME / dt) as usize;

    for _ in 0..steps {
        let (c, half) = fitted_root(&objects);
        let (next, _) = Verlet::execute_cached(objects.clone(), dt, c, half, theta, softening, None);
        objects = next;
    }

    let e = Physics::total_energy(&objects, softening);
    let mut radii: Vec<f32> = objects.iter().map(|o| (o.position - center).length()).collect();
    radii.sort_by(f32::total_cmp);
    let p98 = radii[((radii.len() - 1) as f32 * 0.98) as usize];
    (100.0 * (e - e0) / e0.abs(), p98)
}

fn header(title: &str) {
    println!("\n=== {title} (n={N}, sim_time={SIM_TIME}s, fitted root) ===");
    println!("    value      steps   energy_drift        p98_r");
}

fn main() {
    header("dt sweep (theta=1.8, softening=1.0)");
    for dt in [0.005f32, 0.002, 0.001, 0.000_5, 0.000_25] {
        let (drift, p98) = run(dt, DEFAULT_TETHA_THRESHOLD, DEFAULT_SOFTENING);
        println!("  dt={dt:<8}  {:6}   {drift:+12.2}%   {p98:10.0}", (SIM_TIME / dt) as usize);
    }

    header("theta sweep (dt=0.005, softening=1.0)");
    for theta in [1.8f32, 1.0, 0.5, 0.2] {
        let (drift, p98) = run(0.005, theta, DEFAULT_SOFTENING);
        println!("  theta={theta:<5}   {:6}   {drift:+12.2}%   {p98:10.0}", (SIM_TIME / 0.005) as usize);
    }

    header("softening sweep (dt=0.005, theta=1.8)");
    for soft in [1.0f32, 2.0, 4.0, 8.0, 16.0, 32.0] {
        let (drift, p98) = run(0.005, DEFAULT_TETHA_THRESHOLD, soft);
        println!("  soft={soft:<6}  {:6}   {drift:+12.2}%   {p98:10.0}", (SIM_TIME / 0.005) as usize);
    }

    header("combined (small dt + small theta + larger softening)");
    for (dt, theta, soft) in [(0.001f32, 0.5f32, 4.0f32), (0.000_5, 0.5, 4.0), (0.000_5, 0.2, 8.0)] {
        let (drift, p98) = run(dt, theta, soft);
        println!(
            "  dt={dt} theta={theta} soft={soft}   {:6}   {drift:+12.2}%   {p98:10.0}",
            (SIM_TIME / dt) as usize
        );
    }

    // The shortest encounter timescale a Plummer-softened pair can have is
    // ~sqrt(softening^3 / (GRAVITY * m)); fixed-dt Verlet stays symplectic
    // only while that exceeds dt. physics.rs picked softening=1.0 against the
    // LIGHT mass (1.0), but every orbiter's binding encounter is with the
    // CENTRAL mass (20000), which is the term that actually sets the floor:
    //
    //   softening >= (dt^2 * GRAVITY * m_central)^(1/3)
    //
    // Check that this predicts the sweep, and that it holds as n grows (the
    // criterion has no n in it, so it should).
    println!("\n=== criterion check: softening >= (dt^2 * G * m_central)^(1/3) ===");
    println!("    n       dt      softening   predicted_min   energy_drift        p98_r");
    for n in [1_000usize, 8_000] {
        for dt in [0.005f32, 0.001] {
            let predicted = (dt * dt * GRAVITY * CENTRAL_MASS).cbrt();
            for soft in [DEFAULT_SOFTENING, predicted * 1.2] {
                let (drift, p98) = run_n(n, dt, DEFAULT_TETHA_THRESHOLD, soft);
                println!(
                    "  {n:6}  {dt:<7}  {soft:9.2}   {predicted:13.2}   {drift:+12.2}%   {p98:10.0}"
                );
            }
        }
    }

    // Correct softening alone does not settle the larger swarms. Does fix C
    // (orbits that account for the swarm's own mass) close the remaining gap?
    println!("\n=== fix C on top of the corrected softening (dt=0.005, theta=1.8) ===");
    println!("    n     softening   fix_C   energy_drift        p98_r   spawn_max_r");
    let soft = (0.005f32 * 0.005 * GRAVITY * CENTRAL_MASS).cbrt() * 1.2;
    for n in [1_000usize, 8_000, 44_000] {
        let spawn_max = central_swarm_radii(n).1;
        for fix_c in [false, true] {
            let (drift, p98) = run_full(n, 0.005, DEFAULT_TETHA_THRESHOLD, soft, fix_c);
            println!(
                "  {n:6}  {soft:9.2}   {:5}   {drift:+12.2}%   {p98:10.0}   {spawn_max:11.0}",
                if fix_c { "yes" } else { "no" }
            );
        }
    }
}
