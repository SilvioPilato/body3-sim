use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use macroquad::math::Vec2;

fn sim_with_theta(theta: f32) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::RandomSwarm(body3_sim::simulation::RandomSwarmParams::default()),
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
        theta_threshold: theta,
    })
}

#[test]
fn default_theta_is_1_8() {
    assert_eq!(SimulationConfig::default().theta_threshold, 1.8);
}

#[test]
fn theta_override_changes_trajectory() {
    // The opening-angle theta materially changes force aggregation, so two
    // simulations that differ ONLY in theta_threshold must diverge within a
    // small number of steps. Use RandomSwarm for a bounded, deterministic
    // geometry (RandomSwarm radii are a fixed user param, not scaled by
    // central_swarm's sqrt(n) law — so this test stays independent of the
    // density fix). Pin only *that they differ*, not the exact trajectory.
    let mut s_low = sim_with_theta(0.5);
    let mut s_hi = sim_with_theta(2.0);
    for _ in 0..5 {
        s_low.update(0.005);
        s_hi.update(0.005);
    }
    let mut max_delta = 0.0_f32;
    for (a, b) in s_low.objects().iter().zip(s_hi.objects().iter()) {
        let d = Vec2::distance(a.position, b.position);
        if d > max_delta { max_delta = d; }
    }
    assert!(max_delta > 1.0, "theta had no effect on trajectory (max_delta={max_delta})");
}