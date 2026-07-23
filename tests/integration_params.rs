// tests/integration_params.rs
use body3_sim::physics::{min_softening, GRAVITY};
use body3_sim::simulation::{
    body_count, dominant_mass, integration_params, Scenario, Simulation, SimulationConfig,
};

#[test]
fn min_softening_inverts_the_encounter_timescale_criterion() {
    // The criterion is sqrt(softening^3 / (G*m)) >= dt. min_softening returns
    // the softening at which that holds with equality, so feeding its output
    // back through the timescale must reproduce dt.
    for &(dt, mass) in &[(0.005f32, 20_000.0f32), (1e-4, 40_000.0), (0.002, 5.0)] {
        let soft = min_softening(dt, mass);
        let timescale = (soft.powi(3) / (GRAVITY * mass)).sqrt();
        assert!(
            (timescale - dt).abs() / dt < 1e-3,
            "dt={dt} mass={mass}: timescale={timescale} != dt"
        );
    }
}

#[test]
fn min_softening_matches_the_measured_swarm_value() {
    // The value the sweep in examples/stability_sweep.rs found to work for
    // CentralSwarm at the production dt: predicted 36.8, first clean sweep
    // point 32, verified good at 44.
    let soft = min_softening(0.005, 20_000.0);
    assert!((soft - 36.84).abs() < 0.1, "got {soft}");
}

fn all_scenarios() -> Vec<Scenario> {
    use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams};
    vec![
        Scenario::CentralSwarm { swarm_size: 1000 },
        Scenario::DualCircle,
        Scenario::TriangleCircle,
        Scenario::Burrau1913,
        Scenario::SolarSystem,
        Scenario::FigureEight,
        Scenario::Circumbinary,
        Scenario::Trojan,
        Scenario::Slingshot,
        Scenario::GalaxyCollision { swarm_size: 2000 },
        Scenario::RandomSwarm(RandomSwarmParams::default()),
        Scenario::RandomNBody(RandomNBodyParams::default()),
    ]
}

#[test]
fn body_count_matches_what_the_scenario_actually_builds() {
    // Keeps the hand-written counts honest: integration_params picks the
    // timestep from body_count, so a wrong count silently mis-tunes physics.
    for scenario in all_scenarios() {
        let sim = Simulation::new(SimulationConfig { scenario, ..Default::default() });
        assert_eq!(
            body_count(&scenario),
            sim.objects().len(),
            "body_count disagrees with build_scenario for {scenario:?}"
        );
    }
}

#[test]
fn dominant_mass_is_the_heaviest_body_present() {
    for scenario in all_scenarios() {
        let sim = Simulation::new(SimulationConfig { scenario, ..Default::default() });
        let heaviest = sim.objects().iter().map(|o| o.mass).fold(0.0f32, f32::max);
        let claimed = dominant_mass(&scenario);
        assert!(
            claimed >= heaviest * 0.999,
            "{scenario:?}: dominant_mass {claimed} < heaviest body {heaviest}"
        );
    }
}

#[test]
fn every_scenario_satisfies_the_criterion() {
    for scenario in all_scenarios() {
        let (dt, softening) = integration_params(&scenario);
        let required = body3_sim::physics::min_softening(dt, dominant_mass(&scenario));
        assert!(
            softening >= required,
            "{scenario:?}: softening {softening} below required {required} at dt {dt}"
        );
    }
}

#[test]
fn defaults_come_from_the_scenario() {
    let config = SimulationConfig::default();
    let (dt, softening) = integration_params(&config.scenario);
    assert_eq!(config.physics_dt, dt);
    assert_eq!(config.softening, softening);
    // The measured-good swarm value, not the old 1.0.
    assert!(config.softening > 30.0, "got {}", config.softening);
}

#[test]
fn few_body_presets_keep_softening_small_relative_to_their_geometry() {
    // A softening comparable to the orbital radii would erase exactly the
    // close-encounter physics these presets exist to show.
    for (scenario, smallest_length) in [
        (Scenario::SolarSystem, 90.0f32),
        (Scenario::Slingshot, 60.0),
        (Scenario::Trojan, 250.0),
    ] {
        let (_, softening) = integration_params(&scenario);
        assert!(
            softening < smallest_length * 0.1,
            "{scenario:?}: softening {softening} is >10% of {smallest_length}"
        );
    }
}