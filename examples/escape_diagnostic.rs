// Ablation study for the three candidate fixes behind bodies leaving the view.
// Each is toggled independently against the same CentralSwarm initial layout,
// so their contributions can be ranked by measurement rather than intuition:
//
//   B) remove net spawn momentum   -> kills bulk drift of the whole system
//   C) circular speed from the MEASURED acceleration rather than from
//      `central_mass` alone -> orbits that account for the swarm's own mass
//   D) quadtree root refitted to the bodies' bounding box each step, instead
//      of the static spawn-time extent -> no body is ever outside the root,
//      so Quadtree::insert never misfiles one into a corner quadrant
//
// Metrics per checkpoint: radial extent (p98 and max), how many bodies sit
// outside the ORIGINAL root (the misfiling condition), and total-energy drift
// relative to t=0, which is the honest indicator of numerical blow-up.

use std::rc::Rc;

use body3_sim::physics::{GRAVITY, Physics, PhysicsObject, Verlet};
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig, central_swarm_radii};
use macroquad::math::{Vec2, vec2};

const STEPS: usize = 2_000;
const REPORT_AT: [usize; 5] = [0, 500, 1_000, 1_500, 2_000];

// Mirrors simulation.rs's central_swarm.
const CENTRAL_MASS: f32 = 20_000.0;
const LIGHT_MASS: f32 = 1.0;

#[derive(Clone, Copy)]
struct Variant {
    name: &'static str,
    fix_b: bool,
    fix_c: bool,
    fix_d: bool,
}

fn golden_angle() -> f32 {
    std::f32::consts::TAU * 0.618_034
}

// The production layout: positions and radii identical to central_swarm.
fn positions(n: usize, center: Vec2) -> Vec<(Vec2, f32)> {
    let (min_r, max_r) = central_swarm_radii(n);
    (0..n)
        .map(|i| {
            let radius = min_r + (max_r - min_r) * (i as f32 / n.max(1) as f32);
            let angle = golden_angle() * i as f32;
            let dir = vec2(angle.cos(), angle.sin());
            (center + dir * radius, radius)
        })
        .collect()
}

fn build(n: usize, center: Vec2, half: f32, v: Variant) -> Vec<PhysicsObject> {
    let layout = positions(n, center);
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: center, velocity: Vec2::ZERO, mass: CENTRAL_MASS });
    for &(position, _) in &layout {
        objects.push(PhysicsObject { position, velocity: Vec2::ZERO, mass: LIGHT_MASS });
    }

    if v.fix_c {
        // Circular speed from the acceleration the body actually feels in this
        // configuration: no shell-theorem assumption (which does not hold in
        // 2D with a 1/r^2 force), no need for radius-sorted indices.
        let acc = Physics::compute_accelerations(&objects, center, half, DEFAULT_THETA, DEFAULT_SOFT);
        for (obj, a) in objects.iter_mut().zip(acc.iter()).skip(1) {
            let d = obj.position - center;
            let r = d.length();
            if r <= 0.0 {
                continue;
            }
            let dir = d / r;
            let a_radial = -a.dot(dir); // positive => net pull toward center
            if a_radial <= 0.0 {
                continue;
            }
            obj.velocity = vec2(-dir.y, dir.x) * (a_radial * r).sqrt();
        }
    } else {
        for (obj, &(_, radius)) in objects.iter_mut().skip(1).zip(layout.iter()) {
            let d = obj.position - center;
            let dir = d / radius;
            let speed = (GRAVITY * CENTRAL_MASS / radius).sqrt();
            obj.velocity = vec2(-dir.y, dir.x) * speed;
        }
    }

    if v.fix_b {
        let m: f32 = objects.iter().map(|o| o.mass).sum();
        let p = objects.iter().fold(Vec2::ZERO, |acc, o| acc + o.velocity * o.mass);
        let v_com = p / m;
        for obj in objects.iter_mut() {
            obj.velocity -= v_com;
        }
    }

    objects
}

const DEFAULT_THETA: f32 = body3_sim::physics::DEFAULT_TETHA_THRESHOLD;
const DEFAULT_SOFT: f32 = body3_sim::physics::DEFAULT_SOFTENING;

// Root that always contains every body, so Quadtree::insert never misfiles.
fn fitted_root(objects: &[PhysicsObject]) -> (Vec2, f32) {
    let mut lo = vec2(f32::INFINITY, f32::INFINITY);
    let mut hi = vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for o in objects {
        lo = lo.min(o.position);
        hi = hi.max(o.position);
    }
    let center = (lo + hi) * 0.5;
    let half = (hi - lo).max_element() * 0.5 * 1.01;
    (center, half.max(1.0))
}

fn report(step: usize, objects: &[PhysicsObject], center: Vec2, half: f32, e0: f32) {
    let mut radii: Vec<f32> = objects.iter().map(|o| (o.position - center).length()).collect();
    radii.sort_by(f32::total_cmp);
    let p98 = radii[((radii.len() - 1) as f32 * 0.98) as usize];
    let max_r = radii[radii.len() - 1];
    let outside = objects
        .iter()
        .filter(|o| (o.position.x - center.x).abs() > half || (o.position.y - center.y).abs() > half)
        .count();
    let e = Physics::total_energy(objects, DEFAULT_SOFT);
    println!(
        "    {step:5}  {p98:10.0}  {max_r:12.0}  {:6.2}%   {:+9.2}%",
        100.0 * outside as f32 / objects.len() as f32,
        100.0 * (e - e0) / e0.abs()
    );
}

fn run(n: usize, v: Variant) {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        time_scale: 1.0,
        ..Default::default()
    };
    let center = vec2(config.screen_size / 2.0, config.screen_size / 2.0);
    let static_half = Simulation::world_extent(&config.scenario, config.screen_size);

    let objects = build(n, center, static_half, v);
    let e0 = Physics::total_energy(&objects, DEFAULT_SOFT);
    let mut objects = Rc::new(objects);
    let mut cached: Option<Vec<Vec2>> = None;

    println!("  -- {} --", v.name);
    println!("    step        p98_r         max_r   outside     energy_drift");

    for step in 0..=STEPS {
        if REPORT_AT.contains(&step) {
            report(step, &objects, center, static_half, e0);
        }
        if step == STEPS {
            break;
        }
        let (root_center, root_half) =
            if v.fix_d { fitted_root(&objects) } else { (center, static_half) };
        // Refitting the root invalidates the cached acceleration only in the
        // sense that it was computed on a different tree; the positions are
        // the same, so reuse stays exact for the static case and is dropped
        // for the fitted one to keep the comparison honest.
        let prev = if v.fix_d { None } else { cached.as_deref() };
        let (next, acc_new) = Verlet::execute_cached(
            objects.clone(),
            config.physics_dt,
            root_center,
            root_half,
            DEFAULT_THETA,
            DEFAULT_SOFT,
            prev,
        );
        objects = next;
        cached = Some(acc_new);
    }
}

fn main() {
    let variants = [
        Variant { name: "baseline", fix_b: false, fix_c: false, fix_d: false },
        Variant { name: "B only (no net momentum)", fix_b: true, fix_c: false, fix_d: false },
        Variant { name: "C only (measured orbital speed)", fix_b: false, fix_c: true, fix_d: false },
        Variant { name: "D only (fitted quadtree root)", fix_b: false, fix_c: false, fix_d: true },
        Variant { name: "C + D", fix_b: false, fix_c: true, fix_d: true },
        Variant { name: "B + C + D", fix_b: true, fix_c: true, fix_d: true },
    ];

    for n in [1_000usize, 8_000] {
        println!("\n================ CentralSwarm n={n} ================");
        for v in variants {
            run(n, v);
        }
    }
}
