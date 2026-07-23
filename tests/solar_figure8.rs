use body3_sim::physics::GRAVITY;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use macroquad::math::Vec2;

const CENTER: Vec2 = Vec2::new(500.0, 500.0);

fn sim(scenario: Scenario) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario,
        screen_size: 1000.0,
        time_scale: 1.0, // update(dt) == exactly one substep, deterministic
        ..SimulationConfig::default()
    })
}

#[test]
fn solar_system_planets_are_on_circular_orbits() {
    let s = sim(Scenario::SolarSystem);
    let objs = s.objects();
    assert_eq!(objs.len(), 7, "expected 1 sun + 6 planets");

    let sun = objs[0];
    assert!(objs[1..].iter().all(|p| p.mass < sun.mass), "sun must be heaviest");
    assert!((sun.position - CENTER).length() < 1e-3, "sun must sit at center");

    for p in &objs[1..] {
        let r = (p.position - CENTER).length();
        let expected = (GRAVITY * sun.mass / r).sqrt();
        let speed = p.velocity.length();
        assert!(
            (speed - expected).abs() / expected < 1e-3,
            "planet r={r} speed={speed} expected circular {expected}"
        );
    }
}

#[test]
fn solar_system_total_momentum_is_zero() {
    // Sun carries the counter-momentum so the whole system stays centered.
    let s = sim(Scenario::SolarSystem);
    let p = s
        .objects()
        .iter()
        .fold(Vec2::ZERO, |acc, o| acc + o.velocity * o.mass);
    assert!(p.length() < 1.0, "net momentum should cancel, got {p:?}");
}

#[test]
fn figure_eight_is_balanced_and_symmetric() {
    let s = sim(Scenario::FigureEight);
    let objs = s.objects();
    assert_eq!(objs.len(), 3);
    assert!(objs.iter().all(|o| (o.mass - objs[0].mass).abs() < 1e-6), "masses must be equal");

    let vsum = objs.iter().fold(Vec2::ZERO, |a, o| a + o.velocity);
    assert!(vsum.length() < 1e-2, "equal-mass momentum must cancel, got {vsum:?}");

    assert!((objs[2].position - CENTER).length() < 1e-3, "third body at center");
    let d1 = objs[0].position - CENTER;
    let d2 = objs[1].position - CENTER;
    assert!((d1 + d2).length() < 1e-3, "bodies 1 and 2 must mirror through center: {d1:?} {d2:?}");
}

#[test]
fn figure_eight_stays_bounded_over_a_full_orbit() {
    // ~5s of sim time at dt=0.005 is close to one figure-8 period (~5.2s at
    // this scale); the choreography must not fly apart.
    let mut s = sim(Scenario::FigureEight);
    for _ in 0..1000 {
        s.update(0.005);
    }
    for o in s.objects() {
        let d = (o.position - CENTER).length();
        assert!(d < 300.0, "figure-8 body escaped: dist={d}");
    }
}
