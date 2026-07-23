// Acceptance test for the swarm-stability work. A fixed-dt Verlet integrator
// is symplectic: over a bounded run the total energy must stay put. It does
// not today (see docs/superpowers/plans/2026-07-23-swarm-stability.md), and
// that drift is what makes swarms fly off-screen.
//
// Kept deliberately small so it runs in a debug `cargo test`; the full sweeps
// live in examples/stability_sweep.rs.
use body3_sim::physics::Physics;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const STEPS: usize = 600;
const MAX_DRIFT_PERCENT: f32 = 5.0;

fn energy_drift_percent(n: usize, steps: usize) -> f32 {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        time_scale: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(config);
    let e0 = Physics::total_energy(sim.objects(), sim.config().softening);
    for _ in 0..steps {
        sim.update(sim.config().physics_dt);
    }
    let e = Physics::total_energy(sim.objects(), sim.config().softening);
    100.0 * (e - e0) / e0.abs()
}

#[test]
fn central_swarm_conserves_energy() {
    let drift = energy_drift_percent(300, STEPS);
    assert!(
        drift.abs() < MAX_DRIFT_PERCENT,
        "energy drift {drift:+.2}% exceeds +-{MAX_DRIFT_PERCENT}%"
    );
}

#[test]
fn central_swarm_stays_near_its_spawn_extent() {
    // Physical relaxation does expand a 2D cluster, so this is a loose bound:
    // it catches "blew up by orders of magnitude", not gentle spreading.
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 300 },
        time_scale: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(config);
    let center = macroquad::math::vec2(config.screen_size / 2.0, config.screen_size / 2.0);
    let spawn_max = body3_sim::simulation::central_swarm_radii(300).1;
    for _ in 0..STEPS {
        sim.update(sim.config().physics_dt);
    }
    let mut radii: Vec<f32> = sim
        .objects()
        .iter()
        .map(|o| (o.position - center).length())
        .collect();
    radii.sort_by(f32::total_cmp);
    let p98 = radii[((radii.len() - 1) as f32 * 0.98) as usize];
    assert!(
        p98 < spawn_max * 4.0,
        "p98 radius {p98:.0} vs spawn max {spawn_max:.0} (limit {:.0})",
        spawn_max * 4.0
    );
}

#[test]
fn swarm_orbiters_are_circular_in_the_actual_force_field() {
    // v^2 / r must equal the radial acceleration the body really feels.
    // Deriving the speed from the core mass alone ignores the swarm's own
    // mass, which at large n exceeds the core.
    let config = SimulationConfig::for_scenario(Scenario::CentralSwarm { swarm_size: 2000 });
    let sim = Simulation::new(config);
    let center = macroquad::math::vec2(config.screen_size / 2.0, config.screen_size / 2.0);
    let acc = {
        let (root_center, root_half) = body3_sim::quadtree::fitting_root(sim.objects());
        Physics::compute_accelerations(
            sim.objects(),
            root_center,
            root_half,
            config.theta_threshold,
            config.softening,
        )
    };

    let mut worst = 0.0f32;
    for (obj, a) in sim.objects().iter().zip(acc.iter()).skip(1) {
        let d = obj.position - center;
        let r = d.length();
        let a_radial = -a.dot(d / r);
        if a_radial <= 0.0 {
            continue;
        }
        let expected = (a_radial * r).sqrt();
        worst = worst.max((obj.velocity.length() - expected).abs() / expected);
    }
    assert!(worst < 0.02, "worst relative speed error {:.1}%", worst * 100.0);
}