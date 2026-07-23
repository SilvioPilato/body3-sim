use std::f32::consts::TAU;
use std::rc::Rc;

use macroquad::math::{Vec2, vec2};
use macroquad::rand::{gen_range, srand};

use crate::physics::{GRAVITY, Physics, PhysicsObject};

// Caps how much simulated time a single update() call can inject. Without this,
// an abnormally large frame_time (startup GPU/shader init, alt-tab, a debugger
// pause) fills the accumulator and forces a burst of catch-up substeps in one
// frame, causing a visible stutter.
const MAX_FRAME_TIME: f32 = 0.1;

// Reference swarm_size at which the annulus is [MIN_RADIUS, MAX_RADIUS]. Both
// bounds scale with sqrt(n / REF_N) so the annulus area grows proportionally
// to n and spawn density (bodies/area) stays constant — which keeps the
// Barnes-Hut opening-angle geometry, and its O(n log n) scaling, invariant.
const CENTRAL_SWARM_REF_N: f32 = 1000.0;
const CENTRAL_SWARM_MIN_RADIUS: f32 = 60.0;
const CENTRAL_SWARM_MAX_RADIUS: f32 = 280.0;
// Core and orbiter masses for CentralSwarm / GalaxyCollision. Named because
// integration_params needs the core mass to derive the softening.
pub const CENTRAL_SWARM_CORE_MASS: f32 = 20_000.0;
pub const CENTRAL_SWARM_LIGHT_MASS: f32 = 1.0;
// Quadtree::insert has no bounds check — bodies outside the root quadrant are
// silently misfiled into corner quadrants, unbalancing the tree. The root
// half-size must contain the whole swarm, with margin for orbital drift.
const WORLD_EXTENT_MARGIN: f32 = 1.1;

// Annulus radius bounds for a CentralSwarm of `n` bodies.
pub fn central_swarm_radii(n: usize) -> (f32, f32) {
    let scale = (n as f32 / CENTRAL_SWARM_REF_N).sqrt();
    (CENTRAL_SWARM_MIN_RADIUS * scale, CENTRAL_SWARM_MAX_RADIUS * scale)
}

// physics::min_softening ties dt and softening together, leaving one free
// choice: pick dt by cost. Force evaluation is trivial for a handful of
// bodies, so few-body presets afford a tiny dt and a small, geometry-preserving
// softening; swarms can't, so they take a large dt and pay for it with a
// large softening — self-consistent only because swarm orbits are built from
// the measured force field (see `circularize`), not an analytic Kepler speed.
const FEW_BODY_MAX: usize = 100;
const FEW_BODY_DT: f32 = 1.0e-4;
const SWARM_DT: f32 = 0.005;
// Margin above min_softening's equality point (examples/stability_sweep.rs
// is clean from ~32 upward at dt=0.005, where the criterion predicts 36.8).
const SOFTENING_SAFETY: f32 = 1.2;

/// Number of bodies `build_scenario` will produce. Drives the timestep choice.
pub fn body_count(scenario: &Scenario) -> usize {
    match scenario {
        Scenario::CentralSwarm { swarm_size } => swarm_size + 1,
        Scenario::DualCircle => 2,
        Scenario::TriangleCircle => 3,
        Scenario::Burrau1913 => 3,
        Scenario::SolarSystem => SOLAR_PLANETS.len() + 1,
        Scenario::FigureEight => 3,
        Scenario::Circumbinary => CIRCUMBINARY_PLANETS.len() + 2,
        Scenario::Trojan => 2 * TROJAN_COUNT_PER_POINT + 2,
        Scenario::Slingshot => SLINGSHOT_IMPACT_PARAMS.len() + 1,
        // galaxy_collision splits `swarm_size` in two and adds a core to each.
        Scenario::GalaxyCollision { swarm_size } => swarm_size + 2,
        Scenario::RandomSwarm(p) => p.swarm_size + 1,
        Scenario::RandomNBody(p) => p.count,
    }
}

/// Mass of the heaviest body in the scenario — the one that sets the shortest
/// encounter timescale, and therefore the softening floor.
pub fn dominant_mass(scenario: &Scenario) -> f32 {
    match scenario {
        Scenario::CentralSwarm { .. } | Scenario::GalaxyCollision { .. } => CENTRAL_SWARM_CORE_MASS,
        Scenario::DualCircle => 50.0,
        Scenario::TriangleCircle => 20.0,
        Scenario::Burrau1913 => 5.0,
        Scenario::SolarSystem => SOLAR_SUN_MASS,
        Scenario::FigureEight => FIG8_MASS,
        Scenario::Circumbinary => CIRCUMBINARY_STAR_A_MASS.max(CIRCUMBINARY_STAR_B_MASS),
        Scenario::Trojan => TROJAN_SUN_MASS,
        Scenario::Slingshot => SLINGSHOT_PLANET_MASS,
        Scenario::RandomSwarm(p) => p.central_mass_range.1,
        Scenario::RandomNBody(p) => p.mass_range.1,
    }
}

/// `(physics_dt, softening)` for a scenario, satisfying the stability
/// criterion by construction.
pub fn integration_params(scenario: &Scenario) -> (f32, f32) {
    let dt = if body_count(scenario) <= FEW_BODY_MAX { FEW_BODY_DT } else { SWARM_DT };
    (dt, crate::physics::min_softening(dt, dominant_mass(scenario)) * SOFTENING_SAFETY)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scenario {
    CentralSwarm { swarm_size: usize },
    DualCircle,
    TriangleCircle,
    Burrau1913,
    SolarSystem,
    FigureEight,
    Circumbinary,
    Trojan,
    Slingshot,
    GalaxyCollision { swarm_size: usize },
    RandomSwarm(RandomSwarmParams),
    RandomNBody(RandomNBodyParams),
}

// GalaxyCollision: two central_swarms launched at each other on a grazing
// trajectory. GALAXY_SEPARATION is the initial x-gap between the two cores;
// GALAXY_IMPACT is the y offset (impact parameter) that makes the pass grazing
// rather than head-on; GALAXY_APPROACH_SPEED is each core's bulk speed inward.
const GALAXY_SEPARATION: f32 = 500.0;
const GALAXY_IMPACT: f32 = 150.0;
const GALAXY_APPROACH_SPEED: f32 = 900.0;

// SolarSystem: a heavy central star plus planets on circular orbits at
// increasing radii. (orbital radius, mass) per planet, inner to outer.
const SOLAR_SUN_MASS: f32 = 40_000.0;
const SOLAR_PLANETS: [(f32, f32); 6] = [
    (90.0, 8.0),
    (150.0, 20.0),
    (220.0, 30.0),
    (300.0, 25.0),
    (390.0, 40.0),
    (470.0, 15.0),
];

// FigureEight: the Chenciner-Montgomery three-body choreography (three equal
// masses chasing each other along a figure-8). The canonical initial condition
// is stated for G=1, m=1, length ~1; we rescale it to our GRAVITY and on-screen
// size. Under position -> L*r, mass -> m, the orbit shape is preserved if
// velocity -> sqrt(GRAVITY*m/L) * v (time rescales by sqrt(L^3/(GRAVITY*m))).
const FIG8_SCALE: f32 = 150.0;
const FIG8_MASS: f32 = 50.0;

// Circumbinary: a tight two-star binary at the barycenter (dual_circle pattern)
// plus planets orbiting the pair far enough out to treat it as a point mass
// M = m_a + m_b. (orbital radius, mass) per planet.
const CIRCUMBINARY_STAR_A_MASS: f32 = 30_000.0;
const CIRCUMBINARY_STAR_B_MASS: f32 = 20_000.0;
const CIRCUMBINARY_SEPARATION: f32 = 80.0;
const CIRCUMBINARY_PLANETS: [(f32, f32); 3] = [(250.0, 20.0), (340.0, 30.0), (430.0, 15.0)];

// Trojan: a sun, one planet on a circular orbit, and small bodies clustered at
// the planet's L4 (+60 deg) and L5 (-60 deg) Lagrange points. Stable while the
// planet/sun mass ratio is well below the Gascheau limit (~1/25).
const TROJAN_SUN_MASS: f32 = 40_000.0;
const TROJAN_PLANET_MASS: f32 = 200.0;
const TROJAN_ORBIT_RADIUS: f32 = 250.0;
const TROJAN_COUNT_PER_POINT: usize = 6;
const TROJAN_MASS: f32 = 2.0;

// Slingshot: a heavy stationary planet and light probes flying past at
// different impact parameters, each bent onto a hyperbola. Demonstrates the
// close-encounter deflection physics (the same regime the softening length
// tames). Impact parameter = perpendicular offset of the incoming velocity.
const SLINGSHOT_PLANET_MASS: f32 = 20_000.0;
const SLINGSHOT_PROBE_SPEED: f32 = 4_000.0;
const SLINGSHOT_START_X: f32 = -450.0; // probe start x, relative to center
const SLINGSHOT_IMPACT_PARAMS: [f32; 3] = [60.0, 120.0, 240.0];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RandomSwarmParams {
    pub seed: u64,
    pub swarm_size: usize,
    pub radius_range: (f32, f32),
    pub central_mass_range: (f32, f32),
    pub light_mass_range: (f32, f32),
}

impl Default for RandomSwarmParams {
    fn default() -> Self {
        Self {
            seed: 42,
            swarm_size: 300,
            radius_range: (60.0, 280.0),
            central_mass_range: (5_000.0, 30_000.0),
            light_mass_range: (0.5, 2.0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RandomNBodyParams {
    pub seed: u64,
    pub count: usize,
    pub mass_range: (f32, f32),
    pub position_spread: f32,
    pub velocity_range: (f32, f32),
}

impl Default for RandomNBodyParams {
    fn default() -> Self {
        Self {
            seed: 42,
            count: 6,
            mass_range: (50.0, 2_000.0),
            position_spread: 300.0,
            velocity_range: (0.0, 40.0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    pub scenario: Scenario,
    pub screen_size: f32,
    pub physics_dt: f32,
    pub time_scale: f32,
    // Barnes-Hut opening-angle threshold: larger coarsens the force
    // approximation for better O(n log n) scaling, smaller stays closer to
    // exact O(n^2). Default is DEFAULT_TETHA_THRESHOLD; overridable from
    // benches/tests but not exposed as a UI slider.
    pub theta_threshold: f32,
    // Plummer softening length (see physics::DEFAULT_SOFTENING). Caps the
    // close-encounter force so fixed-dt Verlet stays energy-stable: too small
    // and close passes inject spurious energy, too large and far-field
    // dynamics blur. Overridable from benches/tests but not a UI slider.
    pub softening: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self::for_scenario(Scenario::CentralSwarm { swarm_size: 1000 })
    }
}

impl SimulationConfig {
    /// Config for a scenario with dt/softening derived from the stability
    /// criterion. Use this rather than mutating `scenario` on an existing
    /// config, which would leave the previous scenario's physics parameters in
    /// place.
    pub fn for_scenario(scenario: Scenario) -> Self {
        let (physics_dt, softening) = integration_params(&scenario);
        Self {
            scenario,
            screen_size: 1000.0,
            physics_dt,
            time_scale: 0.3,
            theta_threshold: crate::physics::DEFAULT_TETHA_THRESHOLD,
            softening,
        }
    }
}

fn central_swarm(n: usize, center: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    central_swarm_at(n, center, Vec2::ZERO, theta, softening)
}

// A central_swarm whose every body (core + orbiters) is additionally moving at
// `bulk` — the whole cluster translates rigidly, so the internal orbits are
// unchanged. Used to launch two swarms at each other in GalaxyCollision.
fn central_swarm_at(n: usize, center: Vec2, bulk: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    let central_mass = CENTRAL_SWARM_CORE_MASS;
    let light_mass = CENTRAL_SWARM_LIGHT_MASS;
    let (min_radius, max_radius) = central_swarm_radii(n);

    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject {
        position: center,
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // golden-angle spread: even radial/angular coverage, no rand dependency.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..n {
        let radius = min_radius + (max_radius - min_radius) * (i as f32 / n.max(1) as f32);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = center + dir * radius;
        objects.push(PhysicsObject { position, velocity: Vec2::ZERO, mass: light_mass });
    }

    // Circularize about this swarm's own center BEFORE the bulk boost, so a
    // GalaxyCollision core is circularized against its own swarm only and the
    // rigid translation leaves the internal orbits untouched.
    circularize(&mut objects, center, theta, softening);
    for obj in objects.iter_mut() {
        obj.velocity += bulk;
    }
    objects
}

// Sets each orbiter's speed to the circular speed for the acceleration it
// ACTUALLY feels (measured via compute_accelerations), rather than the
// analytic sqrt(G*M_core/r), which ignores the swarm's own mass and the
// softening. The shell theorem doesn't hold in 2D with a 1/r^2 force, so an
// enclosed-mass estimate would overestimate the required speed anyway; the
// measured form also needs no radius-ordered indices, so it works unchanged
// for random_swarm's randomly-drawn radii.
//
// `objects[0]` is the core and is skipped. Velocities are ignored by the
// force evaluation, so this may be called before or after they are set.
fn circularize(objects: &mut [PhysicsObject], center: Vec2, theta: f32, softening: f32) {
    let (root_center, half_size) = crate::quadtree::fitting_root(objects);
    let acc = Physics::compute_accelerations(objects, root_center, half_size, theta, softening);
    for (obj, a) in objects.iter_mut().zip(acc.iter()).skip(1) {
        let d = obj.position - center;
        let r = d.length();
        if r <= 0.0 {
            continue;
        }
        let dir = d / r;
        let a_radial = -a.dot(dir); // positive => net pull toward the center
        if a_radial <= 0.0 {
            continue; // net outward: no circular orbit exists here
        }
        obj.velocity = vec2(-dir.y, dir.x) * (a_radial * r).sqrt();
    }
}

fn dual_circle(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let m1 = 50.0_f32;
    let m2 = 20.0_f32;
    let d = 200.0_f32; // distance between bodies
    let r1 = d * m2 / (m1 + m2);
    let r2 = d * m1 / (m1 + m2);
    let v_factor = (GRAVITY / (d * (m1 + m2))).sqrt();
    let v1 = m2 * v_factor;
    let v2 = m1 * v_factor;
    let obj_a = PhysicsObject { position: Vec2 { x: cx - r1, y: cy }, mass: m1, velocity: Vec2 { x: 0.0, y: -v1 } };
    let obj_b = PhysicsObject { position: Vec2 { x: cx + r2, y: cy }, mass: m2, velocity: Vec2 { x: 0.0, y: v2 } };
    vec![obj_a, obj_b]
}

fn triangle_circle(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let m = 20.0_f32;
    let side = 200.0_f32;
    let r = side / 3.0_f32.sqrt();

    let v = (GRAVITY * m / side).sqrt();

    let p0 = Vec2 { x: cx, y: cy - r };
    let p1 = Vec2 { x: cx - side / 2.0, y: cy + r / 2.0 };
    let p2 = Vec2 { x: cx + side / 2.0, y: cy + r / 2.0 };

    let v0 = Vec2 { x: -v, y: 0.0 };
    let v1 = Vec2 { x: v / 2.0, y: v * 3.0_f32.sqrt() / 2.0 };
    let v2 = Vec2 { x: v / 2.0, y: -v * 3.0_f32.sqrt() / 2.0 };

    let obj_a = PhysicsObject { position: p0, mass: m, velocity: v0 };
    let obj_b = PhysicsObject { position: p1, mass: m, velocity: v1 };
    let obj_c = PhysicsObject { position: p2, mass: m, velocity: v2 };
    vec![obj_a, obj_b, obj_c]
}

fn burrau_1913(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let scale = 50.0_f32;

    let obj_a = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy - 3.0 * scale },
        mass: 3.0,
        velocity: Vec2::ZERO,
    };
    let obj_b = PhysicsObject {
        position: Vec2 { x: cx - 2.0 * scale, y: cy + 1.0 * scale },
        mass: 4.0,
        velocity: Vec2::ZERO,
    };
    let obj_c = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy + 1.0 * scale },
        mass: 5.0,
        velocity: Vec2::ZERO,
    };
    vec![obj_a, obj_b, obj_c]
}

fn solar_system(center: Vec2) -> Vec<PhysicsObject> {
    let golden_angle = TAU * 0.618_034_f32;
    let mut planets: Vec<PhysicsObject> = Vec::with_capacity(SOLAR_PLANETS.len());
    // Planets are spread by the golden angle, so their momenta don't cancel;
    // give the sun the counter-momentum so the whole system stays centered
    // instead of drifting off-screen.
    let mut planet_momentum = Vec2::ZERO;
    for (i, &(radius, mass)) in SOLAR_PLANETS.iter().enumerate() {
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = center + dir * radius;
        let speed = (GRAVITY * SOLAR_SUN_MASS / radius).sqrt();
        let velocity = Vec2 { x: -dir.y, y: dir.x } * speed; // prograde tangent
        planet_momentum += velocity * mass;
        planets.push(PhysicsObject { position, velocity, mass });
    }

    let mut objects = Vec::with_capacity(SOLAR_PLANETS.len() + 1);
    objects.push(PhysicsObject {
        position: center,
        velocity: -planet_momentum / SOLAR_SUN_MASS,
        mass: SOLAR_SUN_MASS,
    });
    objects.extend(planets);
    objects
}

fn figure_eight(center: Vec2) -> Vec<PhysicsObject> {
    // Canonical Chenciner-Montgomery initial condition (G=1, m=1, length ~1).
    let r1 = Vec2 { x: 0.970_004_36, y: -0.243_087_53 };
    let v3 = Vec2 { x: -0.932_407_37, y: -0.864_731_46 };
    let v12 = -v3 * 0.5; // bodies 1 and 2 share this velocity; total momentum is 0

    let v_scale = (GRAVITY * FIG8_MASS / FIG8_SCALE).sqrt();
    let body = |r: Vec2, v: Vec2| PhysicsObject {
        position: center + r * FIG8_SCALE,
        velocity: v * v_scale,
        mass: FIG8_MASS,
    };
    vec![
        body(r1, v12),
        body(-r1, v12),
        body(Vec2::ZERO, v3),
    ]
}

// A body on a prograde circular orbit of `radius` around a `central_mass` at
// `center`, positioned at `angle`.
fn orbiting_body(center: Vec2, central_mass: f32, angle: f32, radius: f32, mass: f32) -> PhysicsObject {
    let dir = Vec2 { x: angle.cos(), y: angle.sin() };
    let speed = (GRAVITY * central_mass / radius).sqrt();
    PhysicsObject {
        position: center + dir * radius,
        velocity: Vec2 { x: -dir.y, y: dir.x } * speed,
        mass,
    }
}

fn circumbinary(center: Vec2) -> Vec<PhysicsObject> {
    let m1 = CIRCUMBINARY_STAR_A_MASS;
    let m2 = CIRCUMBINARY_STAR_B_MASS;
    let m_total = m1 + m2;
    let d = CIRCUMBINARY_SEPARATION;
    let r1 = d * m2 / m_total;
    let r2 = d * m1 / m_total;
    // dual_circle's two-body circular solution: v_rel = sqrt(G*M/d), split by mass.
    let v_factor = (GRAVITY / (d * m_total)).sqrt();
    let mut star_a = PhysicsObject { position: center + vec2(r1, 0.0), velocity: vec2(0.0, m2 * v_factor), mass: m1 };
    let mut star_b = PhysicsObject { position: center + vec2(-r2, 0.0), velocity: vec2(0.0, -m1 * v_factor), mass: m2 };

    let golden_angle = TAU * 0.618_034_f32;
    let mut planet_momentum = Vec2::ZERO;
    let mut planets: Vec<PhysicsObject> = Vec::with_capacity(CIRCUMBINARY_PLANETS.len());
    for (i, &(radius, mass)) in CIRCUMBINARY_PLANETS.iter().enumerate() {
        let p = orbiting_body(center, m_total, golden_angle * i as f32, radius, mass);
        planet_momentum += p.velocity * mass;
        planets.push(p);
    }
    // Boost both stars equally to absorb the planets' net momentum; equal boost
    // is a pure translation of the binary, so its internal orbit is unchanged.
    let boost = -planet_momentum / m_total;
    star_a.velocity += boost;
    star_b.velocity += boost;

    let mut objects = vec![star_a, star_b];
    objects.extend(planets);
    objects
}

fn trojan(center: Vec2) -> Vec<PhysicsObject> {
    let m_sun = TROJAN_SUN_MASS;
    let r = TROJAN_ORBIT_RADIUS;
    let l4 = TAU / 6.0; // 60 degrees

    // (angle, radius, mass): the planet, then a small cluster around L4 and L5
    // with a deterministic +-8 deg spread so the trojans librate visibly.
    let mut specs: Vec<(f32, f32, f32)> = vec![(0.0, r, TROJAN_PLANET_MASS)];
    for base in [l4, -l4] {
        for j in 0..TROJAN_COUNT_PER_POINT {
            let t = j as f32 / (TROJAN_COUNT_PER_POINT as f32 - 1.0) - 0.5; // -0.5..0.5
            specs.push((base + t * 16.0_f32.to_radians(), r + t * 6.0, TROJAN_MASS));
        }
    }

    let mut momentum = Vec2::ZERO;
    let bodies: Vec<PhysicsObject> = specs
        .iter()
        .map(|&(angle, radius, mass)| {
            let b = orbiting_body(center, m_sun, angle, radius, mass);
            momentum += b.velocity * mass;
            b
        })
        .collect();

    let mut objects = vec![PhysicsObject { position: center, velocity: -momentum / m_sun, mass: m_sun }];
    objects.extend(bodies);
    objects
}

fn slingshot(center: Vec2) -> Vec<PhysicsObject> {
    let mut objects = vec![PhysicsObject { position: center, velocity: Vec2::ZERO, mass: SLINGSHOT_PLANET_MASS }];
    for &b in SLINGSHOT_IMPACT_PARAMS.iter() {
        objects.push(PhysicsObject {
            position: center + vec2(SLINGSHOT_START_X, -b),
            velocity: vec2(SLINGSHOT_PROBE_SPEED, 0.0),
            mass: 1.0,
        });
    }
    objects
}

fn galaxy_collision(total: usize, center: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    let per = total / 2;
    let a_center = center + vec2(-GALAXY_SEPARATION / 2.0, GALAXY_IMPACT / 2.0);
    let b_center = center + vec2(GALAXY_SEPARATION / 2.0, -GALAXY_IMPACT / 2.0);
    // opposite bulk velocities -> the pair's net momentum cancels (equal cores).
    let mut objects = central_swarm_at(per, a_center, vec2(GALAXY_APPROACH_SPEED, 0.0), theta, softening);
    objects.extend(central_swarm_at(total - per, b_center, vec2(-GALAXY_APPROACH_SPEED, 0.0), theta, softening));
    objects
}

fn random_swarm(params: &RandomSwarmParams, center: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    srand(params.seed);
    let central_mass = gen_range(params.central_mass_range.0, params.central_mass_range.1);

    let mut objects = Vec::with_capacity(params.swarm_size + 1);
    objects.push(PhysicsObject {
        position: center,
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // keep golden-angle angular spacing for even coverage; randomize radius per body.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..params.swarm_size {
        let radius = gen_range(params.radius_range.0, params.radius_range.1);
        let light_mass = gen_range(params.light_mass_range.0, params.light_mass_range.1);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = center + dir * radius;
        objects.push(PhysicsObject { position, velocity: Vec2::ZERO, mass: light_mass });
    }

    circularize(&mut objects, center, theta, softening);
    objects
}

fn random_n_body(params: &RandomNBodyParams, center: Vec2) -> Vec<PhysicsObject> {
    srand(params.seed);
    let mut objects = Vec::with_capacity(params.count);
    for _ in 0..params.count {
        let mass = gen_range(params.mass_range.0, params.mass_range.1);
        let offset = Vec2 {
            x: gen_range(-params.position_spread, params.position_spread),
            y: gen_range(-params.position_spread, params.position_spread),
        };
        let speed = gen_range(params.velocity_range.0, params.velocity_range.1);
        let angle = gen_range(0.0, TAU);
        let velocity = Vec2 { x: angle.cos(), y: angle.sin() } * speed;
        objects.push(PhysicsObject { position: center + offset, velocity, mass });
    }
    objects
}

fn build_scenario(scenario: &Scenario, center: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    match scenario {
        Scenario::CentralSwarm { swarm_size } => central_swarm(*swarm_size, center, theta, softening),
        Scenario::DualCircle => dual_circle(center),
        Scenario::TriangleCircle => triangle_circle(center),
        Scenario::Burrau1913 => burrau_1913(center),
        Scenario::SolarSystem => solar_system(center),
        Scenario::FigureEight => figure_eight(center),
        Scenario::Circumbinary => circumbinary(center),
        Scenario::Trojan => trojan(center),
        Scenario::Slingshot => slingshot(center),
        Scenario::GalaxyCollision { swarm_size } => galaxy_collision(*swarm_size, center, theta, softening),
        Scenario::RandomSwarm(params) => random_swarm(params, center, theta, softening),
        Scenario::RandomNBody(params) => random_n_body(params, center),
    }
}

pub struct Simulation {
    config: SimulationConfig,
    center: Vec2,
    world_half_size: f32,
    objects: Rc<Vec<PhysicsObject>>,
    accumulator: f32,
    // acc_new from the last substep, reused as the next substep's acc_old
    // instead of recomputing it — positions don't move between one step's
    // end and the next step's start, so it's still valid. See `update`.
    cached_acceleration: Option<Vec<Vec2>>,
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self {
        let center = vec2(config.screen_size / 2.0, config.screen_size / 2.0);
        let world_half_size = Self::world_extent(&config.scenario, config.screen_size);
        let objects = Rc::new(build_scenario(
            &config.scenario,
            center,
            config.theta_threshold,
            config.softening,
        ));
        Self { config, center, world_half_size, objects, accumulator: 0.0, cached_acceleration: None }
    }

    pub fn reset(&mut self, config: SimulationConfig) {
        *self = Self::new(config);
    }

    pub fn update(&mut self, frame_time: f32) {
        self.accumulator += frame_time.min(MAX_FRAME_TIME) * self.config.time_scale;
        let dt = self.config.physics_dt;
        let theta = self.config.theta_threshold;
        let softening = self.config.softening;
        while self.accumulator >= dt {
            // Inlined Verlet step (rather than Verlet::execute_cached) so the
            // quadtree root is refit on the POST-update positions for
            // acc_new, not just the pre-update positions for acc_old — bodies
            // move enough per substep that a pre-update root can fail to
            // contain them, and Quadtree::insert has no bounds check.
            let mut objects = (*self.objects).clone();

            let acc_old = match self.cached_acceleration.as_deref() {
                Some(acc) => acc.to_vec(),
                None => {
                    let (rc, rh) = crate::quadtree::fitting_root(&objects);
                    Physics::compute_accelerations(&objects, rc, rh, theta, softening)
                }
            };

            for (obj, acc) in objects.iter_mut().zip(acc_old.iter()) {
                obj.position += obj.velocity * dt + 0.5 * *acc * dt * dt;
            }

            let (rc_post, rh_post) = crate::quadtree::fitting_root(&objects);
            let acc_new = Physics::compute_accelerations(&objects, rc_post, rh_post, theta, softening);

            for ((obj, a_old), a_new) in objects.iter_mut().zip(acc_old.iter()).zip(acc_new.iter()) {
                obj.velocity += 0.5 * (*a_old + *a_new) * dt;
            }

            self.objects = Rc::new(objects);
            self.cached_acceleration = Some(acc_new);
            self.accumulator -= dt;
        }
    }

    pub fn objects(&self) -> &[PhysicsObject] {
        &self.objects
    }

    pub fn total_energy(&self) -> f32 {
        Physics::total_energy(&self.objects, self.config.softening)
    }

    pub fn total_energy_approx(&self) -> f32 {
        Physics::total_energy_approx(&self.objects, self.center, self.world_half_size, self.config.theta_threshold, self.config.softening)
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }

    pub fn set_time_scale(&mut self, time_scale: f32) {
        self.config.time_scale = time_scale;
    }

    pub fn set_physics_dt(&mut self, physics_dt: f32) {
        // update()'s accumulator loop never terminates if physics_dt <= 0.0.
        self.config.physics_dt = physics_dt.max(0.0001);
    }

    // Half-size of the square physics domain (quadtree root) for a scenario:
    // at least screen_size/2, grown to contain scenario extents that exceed it.
    // CentralSwarm radii scale with sqrt(n); RandomSwarm's radius max (UI slider,
    // up to 600) can also exceed the default 400. Single source of this rule so
    // benches/examples compute the same half-size production runs with.
    pub fn world_extent(scenario: &Scenario, screen_size: f32) -> f32 {
        let base = screen_size / 2.0;
        match scenario {
            Scenario::CentralSwarm { swarm_size } => {
                base.max(central_swarm_radii(*swarm_size).1 * WORLD_EXTENT_MARGIN)
            }
            Scenario::RandomSwarm(params) => base.max(params.radius_range.1 * WORLD_EXTENT_MARGIN),
            Scenario::SolarSystem => {
                let outer = SOLAR_PLANETS[SOLAR_PLANETS.len() - 1].0;
                base.max(outer * WORLD_EXTENT_MARGIN)
            }
            Scenario::Circumbinary => {
                let outer = CIRCUMBINARY_PLANETS[CIRCUMBINARY_PLANETS.len() - 1].0;
                base.max(outer * WORLD_EXTENT_MARGIN)
            }
            Scenario::Trojan => base.max(TROJAN_ORBIT_RADIUS * WORLD_EXTENT_MARGIN),
            Scenario::Slingshot => base.max(-SLINGSHOT_START_X * WORLD_EXTENT_MARGIN),
            Scenario::GalaxyCollision { swarm_size } => {
                // root must reach a core's offset plus its own swarm radius.
                let per = (*swarm_size / 2).max(1);
                let reach = GALAXY_SEPARATION / 2.0 + central_swarm_radii(per).1;
                base.max(reach * WORLD_EXTENT_MARGIN)
            }
            _ => base,
        }
    }

    pub fn world_half_size(&self) -> f32 {
        self.world_half_size
    }
}
