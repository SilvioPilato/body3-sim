use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

// Physics::total_energy_approx is accurate at normal density (this
// codebase's default swarm_size is 1000) but its error grows sharply with
// density and is unusable at the extreme end (~200% relative error at
// n=44000 — see main.rs's comment on ENERGY_LOG_INTERVAL_FRAMES). This test
// pins the regime where it's actually valid; it deliberately does not cover
// n=8000+ since that's a known, accepted limitation, not a regression to
// guard against here.
#[test]
fn approx_energy_matches_exact_at_normal_density() {
    for swarm_size in [500, 2000] {
        let sim = Simulation::new(SimulationConfig {
            scenario: Scenario::CentralSwarm { swarm_size },
            screen_size: 800.0,
            physics_dt: 0.005,
            time_scale: 1.0,
            theta_threshold: 1.5,
        });

        let exact = sim.total_energy();
        let approx = sim.total_energy_approx();
        let relative_error = ((approx - exact) / exact).abs();

        assert!(
            relative_error < 0.10,
            "swarm_size={swarm_size} exact={exact} approx={approx} relative_error={relative_error}"
        );
    }
}
