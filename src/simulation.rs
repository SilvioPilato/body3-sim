use std::f32::consts::TAU;
use std::rc::Rc;

use macroquad::math::{Vec2, vec2};
use macroquad::rand::{gen_range, srand};

use crate::physics::{GRAVITY, Physics, PhysicsObject, PhysicsSystem, Verlet};

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
    let min_radius = 60.0_f32;
    let max_radius = 280.0_f32;

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
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self {
        let center = vec2(config.screen_size / 2.0, config.screen_size / 2.0);
        let world_half_size = config.screen_size / 2.0;
        let objects = Rc::new(build_scenario(&config.scenario, center));
        Self { config, center, world_half_size, objects, accumulator: 0.0 }
    }

    pub fn reset(&mut self, config: SimulationConfig) {
        *self = Self::new(config);
    }

    pub fn update(&mut self, frame_time: f32) {
        self.accumulator += frame_time * self.config.time_scale;
        while self.accumulator >= self.config.physics_dt {
            self.objects = Verlet::execute(self.objects.clone(), self.config.physics_dt, self.center, self.world_half_size);
            self.accumulator -= self.config.physics_dt;
        }
    }

    pub fn objects(&self) -> &[PhysicsObject] {
        &self.objects
    }

    pub fn total_energy(&self) -> f32 {
        Physics::total_energy(&self.objects)
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }
}
