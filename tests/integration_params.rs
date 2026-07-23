// tests/integration_params.rs
use body3_sim::physics::{min_softening, GRAVITY};

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