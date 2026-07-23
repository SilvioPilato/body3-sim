use body3_sim::physics::GRAVITY;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use macroquad::math::Vec2;

const CENTER: Vec2 = Vec2::new(500.0, 500.0);

fn sim(scenario: Scenario) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario,
        screen_size: 1000.0,
        time_scale: 1.0,
        ..SimulationConfig::default()
    })
}

fn total_momentum(s: &Simulation) -> Vec2 {
    s.objects().iter().fold(Vec2::ZERO, |acc, o| acc + o.velocity * o.mass)
}

// Net momentum should cancel by construction; f32 rounding leaves a residual
// proportional to the largest single-body momentum, so compare relative to it.
fn assert_momentum_cancels(s: &Simulation) {
    let char_p = s
        .objects()
        .iter()
        .map(|o| (o.velocity * o.mass).length())
        .fold(0.0_f32, f32::max)
        .max(1.0);
    let rel = total_momentum(s).length() / char_p;
    assert!(rel < 1e-5, "net momentum should cancel: relative residual {rel:e}");
}

#[test]
fn circumbinary_two_stars_plus_planets() {
    let s = sim(Scenario::Circumbinary);
    let objs = s.objects();
    assert_eq!(objs.len(), 5, "2 stars + 3 planets");

    // the two stars are the heaviest bodies
    let (stars, planets) = objs.split_at(2);
    assert!(stars.iter().all(|st| planets.iter().all(|p| p.mass < st.mass)));

    // binary barycenter sits at the center
    let m: f32 = stars.iter().map(|s| s.mass).sum();
    let bary = stars.iter().fold(Vec2::ZERO, |a, s| a + s.position * s.mass) / m;
    assert!((bary - CENTER).length() < 1e-2, "barycenter off-center: {bary:?}");

    // planets orbit the pair as a point mass M = m_a + m_b
    for p in planets {
        let r = (p.position - CENTER).length();
        let expected = (GRAVITY * m / r).sqrt();
        assert!((p.velocity.length() - expected).abs() / expected < 1e-3);
    }

    assert_momentum_cancels(&s);
}

#[test]
fn trojan_planet_with_l4_l5_clusters() {
    let s = sim(Scenario::Trojan);
    let objs = s.objects();
    assert_eq!(objs.len(), 14, "sun + planet + 12 trojans");

    let sun = objs[0];
    assert!(objs[1..].iter().all(|b| b.mass < sun.mass), "sun heaviest");
    assert!((sun.position - CENTER).length() < 1e-3, "sun centered");

    // planet at angle 0
    let planet = objs[1];
    let pa = planet.position - CENTER;
    assert!(pa.y.abs() < 1e-3 && pa.x > 0.0, "planet should sit at angle 0");

    // trojans split evenly around +60 and -60 degrees
    let deg = |b: &body3_sim::physics::PhysicsObject| {
        let d = b.position - CENTER;
        d.y.atan2(d.x).to_degrees()
    };
    let near = |a: f32, target: f32| (a - target).abs() < 15.0;
    let l4 = objs[2..].iter().filter(|b| near(deg(b), 60.0)).count();
    let l5 = objs[2..].iter().filter(|b| near(deg(b), -60.0)).count();
    assert_eq!((l4, l5), (6, 6), "trojans must cluster at L4 and L5");

    assert_momentum_cancels(&s);
}

#[test]
fn slingshot_probes_are_deflected() {
    let mut s = sim(Scenario::Slingshot);
    assert_eq!(s.objects().len(), 4, "planet + 3 probes");
    // probes start with purely horizontal velocity
    assert!(s.objects()[1..].iter().all(|p| p.velocity.y.abs() < 1e-3));

    for _ in 0..50 {
        s.update(0.005);
    }

    let objs = s.objects();
    // heavy planet barely recoils
    assert!((objs[0].position - CENTER).length() < 30.0, "planet drifted too far");
    // at least one probe is clearly bent off the horizontal
    let max_vy = objs[1..].iter().map(|p| p.velocity.y.abs()).fold(0.0_f32, f32::max);
    assert!(max_vy > 200.0, "no visible deflection, max |vy|={max_vy}");
}
