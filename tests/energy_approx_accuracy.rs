use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

// Physics::total_energy_approx's relative error grows with PAIR AGGREGATION
// COUNT, not density (verified post-density-fix via examples/energy_theta_sweep:
// ~0.5% @ n=500, ~30% @ n=8000, ~185% @ n=44000 at theta=1.8, moving only
// ~10-20% across theta). The growth with n is intrinsic to Barnes-Hut pair
// aggregation and stays unusable for the energy display at high n. This test
// pins the small-n regime where the approximation is still under 10%; it
// deliberately does not cover n=8000+ since that's a known, accepted
// limitation, not a regression to guard against here.
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
