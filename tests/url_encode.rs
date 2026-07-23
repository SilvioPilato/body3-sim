// Pure round-trip + edge-case suite for the URL encode/decode pair in
// src/url.rs. No macroquad, no wasm, no browser — runs under plain `cargo test`.
use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, SimulationConfig};
use body3_sim::url::{decode, encode};

// ---- round-trip: every scenario, default vs. overridden ----

#[test]
fn roundtrip_central_swarm_default() {
    let config = SimulationConfig::default();
    assert_eq!(encode(&config), "");
    assert_eq!(decode(""), Some(config));
}

#[test]
fn roundtrip_central_swarm_override() {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 5000 },
        physics_dt: 0.01,
        ..SimulationConfig::default()
    };
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_dual_circle() {
    let config = SimulationConfig::for_scenario(Scenario::DualCircle);
    assert_eq!(encode(&config), "scenario=dualcircle");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_triangle_circle() {
    let config = SimulationConfig::for_scenario(Scenario::TriangleCircle);
    assert_eq!(encode(&config), "scenario=trianglecircle");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_burrau() {
    let config = SimulationConfig::for_scenario(Scenario::Burrau1913);
    assert_eq!(encode(&config), "scenario=burrau1913");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_solar_system() {
    let config = SimulationConfig::for_scenario(Scenario::SolarSystem);
    assert_eq!(encode(&config), "scenario=solarsystem");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_figure_eight() {
    let config = SimulationConfig::for_scenario(Scenario::FigureEight);
    assert_eq!(encode(&config), "scenario=figureeight");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_circumbinary() {
    let config = SimulationConfig::for_scenario(Scenario::Circumbinary);
    assert_eq!(encode(&config), "scenario=circumbinary");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_trojan() {
    let config = SimulationConfig::for_scenario(Scenario::Trojan);
    assert_eq!(encode(&config), "scenario=trojan");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_slingshot() {
    let config = SimulationConfig::for_scenario(Scenario::Slingshot);
    assert_eq!(encode(&config), "scenario=slingshot");
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_galaxy_collision() {
    let config = SimulationConfig::for_scenario(Scenario::GalaxyCollision { swarm_size: 3000 });
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_random_swarm() {
    let config = SimulationConfig::for_scenario(Scenario::RandomSwarm(RandomSwarmParams {
        swarm_size: 500,
        central_mass_range: (10_000.0, 50_000.0),
        ..RandomSwarmParams::default()
    }));
    assert_eq!(decode(&encode(&config)), Some(config));
}

#[test]
fn roundtrip_random_n_body() {
    let config = SimulationConfig::for_scenario(Scenario::RandomNBody(RandomNBodyParams {
        count: 12,
        mass_range: (1.0, 3000.0),
        seed: 7,
        ..RandomNBodyParams::default()
    }));
    assert_eq!(decode(&encode(&config)), Some(config));
}

// ---- edge cases ----

#[test]
fn unknown_scenario_returns_none() {
    assert_eq!(decode("scenario=garbage"), None);
}

#[test]
fn malformed_number_returns_none() {
    assert_eq!(decode("scenario=centralswarm&swarm_size=abc"), None);
    assert_eq!(decode("scenario=centralswarm&physics_dt=xyz"), None);
}

#[test]
fn empty_query_returns_default() {
    assert_eq!(decode(""), Some(SimulationConfig::default()));
    assert_eq!(decode("    "), Some(SimulationConfig::default()));
}

#[test]
fn trailing_junk_ignored() {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 1000 },
        ..SimulationConfig::default()
    };
    // an unknown key after a valid scenario field is ignored, not an error
    assert_eq!(decode(&format!("{}&xyz=123", encode(&config))), Some(config));
}

#[test]
fn multiple_overrides_applied() {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 2000 },
        physics_dt: 0.002,
        time_scale: 2.0,
        ..SimulationConfig::default()
    };
    assert_eq!(decode(&encode(&config)), Some(config));
}