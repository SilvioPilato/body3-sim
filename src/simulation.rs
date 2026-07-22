use std::f32::consts::TAU;
use std::rc::Rc;

use macroquad::math::{Vec2, vec2};
use macroquad::rand::{gen_range, srand};

use crate::physics::{GRAVITY, Physics, PhysicsObject, Verlet};

// Caps how much simulated time a single update() call can inject. Without this,
// an abnormally large frame_time (startup GPU/shader init, alt-tab, a debugger
// pause) fills the accumulator and forces a burst of catch-up substeps in one
// frame, causing a visible stutter.
const MAX_FRAME_TIME: f32 = 0.1;

// Reference swarm_size at which the annulus is [MIN_RADIUS, MAX_RADIUS].
// Both bounds scale with sqrt(n / REF_N) so the annulus area grows
// proportionally to n and spawn density (bodies/area) stays constant.
// Constant density keeps the Barnes-Hut opening-angle geometry scale-invariant:
// without it, packing more bodies into a fixed area makes the force walk
// degrade past O(n log n) (measured: ~169x vs ~103x predicted, n=1000->64000).
const CENTRAL_SWARM_REF_N: f32 = 1000.0;
const CENTRAL_SWARM_MIN_RADIUS: f32 = 60.0;
const CENTRAL_SWARM_MAX_RADIUS: f32 = 280.0;
// Quadtree::insert has no bounds check — bodies outside the root quadrant are
// silently misfiled into corner quadrants, unbalancing the tree. The root
// half-size must contain the whole swarm, with margin for orbital drift.
const WORLD_EXTENT_MARGIN: f32 = 1.1;

// Annulus radius bounds for a CentralSwarm of `n` bodies.
pub fn central_swarm_radii(n: usize) -> (f32, f32) {
    let scale = (n as f32 / CENTRAL_SWARM_REF_N).sqrt();
    (CENTRAL_SWARM_MIN_RADIUS * scale, CENTRAL_SWARM_MAX_RADIUS * scale)
}

#[derive(Clone, Copy, Debug)]
pub enum Scenario {
    CentralSwarm { swarm_size: usize },
    DualCircle,
    TriangleCircle,
    Burrau1913,
    RandomSwarm(RandomSwarmParams),
    RandomNBody(RandomNBodyParams),
}

#[derive(Clone, Copy, Debug)]
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

#[derive(Clone, Copy, Debug)]
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

#[derive(Clone, Copy, Debug)]
pub struct SimulationConfig {
    pub scenario: Scenario,
    pub screen_size: f32,
    pub physics_dt: f32,
    pub time_scale: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            scenario: Scenario::CentralSwarm { swarm_size: 1000 },
            screen_size: 800.0,
            physics_dt: 0.005,
            time_scale: 0.3,
        }
    }
}

fn central_swarm(n: usize, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let central_mass = 20_000.0_f32;
    let light_mass = 1.0_f32;
    let (min_radius, max_radius) = central_swarm_radii(n);

    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject {
        position: Vec2 { x: cx, y: cy },
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // golden-angle spread: even radial/angular coverage, no rand dependency.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..n {
        let radius = min_radius + (max_radius - min_radius) * (i as f32 / n.max(1) as f32);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = Vec2 { x: cx, y: cy } + dir * radius;
        let speed = (GRAVITY * central_mass / radius).sqrt();
        let tangent = Vec2 { x: -dir.y, y: dir.x } * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: light_mass });
    }
    objects
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

fn random_swarm(params: &RandomSwarmParams, center: Vec2) -> Vec<PhysicsObject> {
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
        // derived circular-orbit speed from the randomized radius/central_mass, not a separate random draw.
        let speed = (GRAVITY * central_mass / radius).sqrt();
        let tangent = Vec2 { x: -dir.y, y: dir.x } * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: light_mass });
    }
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

fn build_scenario(scenario: &Scenario, center: Vec2) -> Vec<PhysicsObject> {
    match scenario {
        Scenario::CentralSwarm { swarm_size } => central_swarm(*swarm_size, center),
        Scenario::DualCircle => dual_circle(center),
        Scenario::TriangleCircle => triangle_circle(center),
        Scenario::Burrau1913 => burrau_1913(center),
        Scenario::RandomSwarm(params) => random_swarm(params, center),
        Scenario::RandomNBody(params) => random_n_body(params, center),
    }
}

pub struct Simulation {
    config: SimulationConfig,
    center: Vec2,
    world_half_size: f32,
    objects: Rc<Vec<PhysicsObject>>,
    accumulator: f32,
    // acc_new from the last substep, reused as next substep's acc_old
    // (Verlet::execute_cached) instead of recomputing it from scratch.
    cached_acceleration: Option<Vec<Vec2>>,
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self {
        let center = vec2(config.screen_size / 2.0, config.screen_size / 2.0);
        let world_half_size = Self::world_extent(&config.scenario, config.screen_size);
        let objects = Rc::new(build_scenario(&config.scenario, center));
        Self { config, center, world_half_size, objects, accumulator: 0.0, cached_acceleration: None }
    }

    pub fn reset(&mut self, config: SimulationConfig) {
        *self = Self::new(config);
    }

    pub fn update(&mut self, frame_time: f32) {
        self.accumulator += frame_time.min(MAX_FRAME_TIME) * self.config.time_scale;
        while self.accumulator >= self.config.physics_dt {
            let (objects, acc_new) = Verlet::execute_cached(
                self.objects.clone(),
                self.config.physics_dt,
                self.center,
                self.world_half_size,
                self.cached_acceleration.as_deref(),
            );
            self.objects = objects;
            self.cached_acceleration = Some(acc_new);
            self.accumulator -= self.config.physics_dt;
        }
    }

    pub fn objects(&self) -> &[PhysicsObject] {
        &self.objects
    }

    pub fn total_energy(&self) -> f32 {
        Physics::total_energy(&self.objects)
    }

    pub fn total_energy_approx(&self) -> f32 {
        Physics::total_energy_approx(&self.objects, self.center, self.world_half_size)
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
            _ => base,
        }
    }

    pub fn world_half_size(&self) -> f32 {
        self.world_half_size
    }
}
