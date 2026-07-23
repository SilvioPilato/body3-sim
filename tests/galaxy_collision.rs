use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use macroquad::math::{vec2, Vec2};

const CENTER: Vec2 = Vec2::new(500.0, 500.0);
// mirror of the private GALAXY_* constants in simulation.rs
const SEP: f32 = 500.0;
const IMPACT: f32 = 150.0;
const APPROACH: f32 = 900.0;

fn sim(scenario: Scenario) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario,
        screen_size: 1000.0,
        time_scale: 1.0,
        ..SimulationConfig::default()
    })
}

#[test]
fn galaxy_collision_has_two_offset_cores_moving_toward_each_other() {
    let total = 200;
    let per = total / 2;
    let s = sim(Scenario::GalaxyCollision { swarm_size: total });
    let objs = s.objects();
    assert_eq!(objs.len(), total + 2, "two cores + total orbiters");

    // exactly two heavy cores; everything else is a light orbiter
    assert_eq!(objs.iter().filter(|o| o.mass > 1000.0).count(), 2);

    let core_a = objs[0];
    let core_b = objs[per + 1];
    assert!((core_a.position - (CENTER + vec2(-SEP / 2.0, IMPACT / 2.0))).length() < 1e-3);
    assert!((core_b.position - (CENTER + vec2(SEP / 2.0, -IMPACT / 2.0))).length() < 1e-3);
    // cores carry only the opposite bulk velocity (no orbital component)
    assert!((core_a.velocity - vec2(APPROACH, 0.0)).length() < 1e-3);
    assert!((core_b.velocity - vec2(-APPROACH, 0.0)).length() < 1e-3);
}

#[test]
fn galaxy_collision_net_momentum_is_small() {
    // Bulk momenta cancel exactly; each swarm's internal residual does not fully
    // cancel with the other, so only require it to be tiny next to a core's own.
    let s = sim(Scenario::GalaxyCollision { swarm_size: 2000 });
    let objs = s.objects();
    let total = objs.iter().fold(Vec2::ZERO, |a, o| a + o.velocity * o.mass);
    let char_p = objs.iter().map(|o| (o.velocity * o.mass).length()).fold(0.0_f32, f32::max);
    assert!(total.length() / char_p < 1e-2, "net momentum {:e} vs core {:e}", total.length(), char_p);
}

#[test]
fn galaxy_collision_stays_finite_through_the_merger() {
    let mut s = sim(Scenario::GalaxyCollision { swarm_size: 200 });
    for _ in 0..300 {
        s.update(0.005);
    }
    assert!(
        s.objects().iter().all(|o| o.position.is_finite() && o.velocity.is_finite()),
        "a body went non-finite during the merger"
    );
}
