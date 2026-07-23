// energy_theta_sweep: measure relative error of Physics::total_energy_approx
// (Barnes-Hut) vs exact total_energy, across opening-angle theta and swarm
// size, at the post-density-fix constant-density regime. Goal: pick theta that
// keeps energy approximation usable.
//
// Pre-fix (variable density) error was ~0.5% @ n=500, 30.8% @ n=8000,
// 200% @ n=44000. Post-fix density is constant, so error should stay flat in n;
// here we sweep theta to see how much accuracy trades against scaling speed.

use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const SIZES: [usize; 3] = [500, 8000, 44000];
const THETAS: [f32; 4] = [0.5, 1.5, 2.0, 3.0];

fn sim_for(n: usize, theta: f32) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
        theta_threshold: theta,
        softening: body3_sim::physics::DEFAULT_SOFTENING,
    })
}

fn main() {
    println!("energy_theta_sweep: relative error of total_energy_approx vs exact");
    println!("{:>7} {:>8} {:>18} {:>18} {:>12}", "n", "theta", "exact", "approx", "rel_err_%");
    for &n in &SIZES {
        for &theta in &THETAS {
            let sim = sim_for(n, theta);
            let exact = sim.total_energy();
            let approx = sim.total_energy_approx();
            let rel = ((approx - exact) / exact.abs().max(1e-12)).abs() * 100.0;
            println!("{:>7} {:>8.2} {:>18.4e} {:>18.4e} {:>12.3}", n, theta, exact, approx, rel);
        }
        println!();
    }
}