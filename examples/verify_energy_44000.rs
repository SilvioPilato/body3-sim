// verify_energy_44000: re-check the energy-divergence anomaly that was
// observed pre-fix at n=44000 (exact total_energy diverged from ~-3e11 to
// ~+3.9e16 within the first few logged frames, plausibly from close encounters
// at extreme density). Post density-fix + theta=1.8, density is constant and
// force aggregation is coarser; this prints the exact energy trajectory over
// 300 steps so we can see whether it stays bounded or still diverges.
//
// Headless (no window). exact energy is O(n^2) (~1s at n=44000); 10 samples
// adds ~10s, acceptable for a one-off verification.

use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const N: usize = 44_000;
const STEPS: u32 = 300;
const LOG_EVERY: u32 = 30;

fn main() {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: N },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
        // theta defaults to 1.8 (DEFAULT_TETHA_THRESHOLD); omit / use default.
        ..SimulationConfig::default()
    });

    let mut e0 = sim.total_energy();
    println!("step=  0 energy={:.4e}", e0);
    let mut max_abs = e0.abs();
    for step in 1..=STEPS {
        sim.update(sim.config().physics_dt);
        if step % LOG_EVERY == 0 {
            let e = sim.total_energy();
            println!("step={:>3} energy={:.4e} (rel {:.3})", step, e, (e - e0) / e0.abs().max(1e-12));
            e0 = e;
            if e.abs() > max_abs { max_abs = e.abs(); }
        }
    }
    println!();
    println!("max |energy| over run = {:.4e}", max_abs);
    // pre-fix anomaly: growth to ~+3.9e16. Bounded verdict: compare to e0 magnitude.
    let initial = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: N },
        screen_size: 800.0,
        ..SimulationConfig::default()
    });
    let init_e = initial.total_energy().abs();
    println!("initial |energy|       = {:.4e}", init_e);
    println!("growth factor          = {:.2}x", max_abs / init_e.max(1e-12));
}