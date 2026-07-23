use macroquad::math::Vec2;

use crate::physics::PhysicsObject;

// Fraction of bodies the view is required to contain. Deliberately not 1.0:
// once a swarm starts ejecting bodies, a handful of escapers run off to
// arbitrary distance and fitting the true bounding box would zoom the whole
// cluster down to a few pixels. Cutting at the 98th percentile keeps the
// view sized by the bulk of the system and lets outliers leave the frame.
pub const FIT_PERCENTILE: f32 = 0.98;

// Breathing room around the fitted radius so bodies near the percentile
// boundary are not drawn exactly on the frame edge.
pub const FIT_MARGIN: f32 = 1.15;

// Exponential smoothing rates (per second), asymmetric on purpose. Expansion
// is fast so the view stays ahead of bodies moving outward — a slow expand
// would let them cross the edge before the camera catches up, which is the
// exact symptom this camera exists to remove. Contraction is slow so a
// transient outward excursion (a core rebound, one wide orbit) does not make
// the view pump in and out.
pub const EXPAND_RATE: f32 = 8.0;
pub const CONTRACT_RATE: f32 = 2.0;

// Tracks a square view that follows the system's center of mass and grows to
// contain it. Pure geometry over positions/masses — no macroquad camera or
// rendering state — so it is unit-testable without a window.
pub struct CameraFit {
    center: Vec2,
    half_size: f32,
    // The view never zooms in tighter than this. Set from the scenario's
    // static `Simulation::world_half_size`, so every existing preset keeps its
    // original framing and the camera only ever reacts by zooming *out*.
    floor_half_size: f32,
    // Scratch buffer for the percentile selection, reused across frames: at
    // n=44000 a fresh Vec<f32> per frame is ~176KB of allocation per frame.
    dist_sq: Vec<f32>,
}

impl CameraFit {
    pub fn new(center: Vec2, floor_half_size: f32) -> Self {
        Self {
            center,
            half_size: floor_half_size,
            floor_half_size,
            dist_sq: Vec::new(),
        }
    }

    pub fn center(&self) -> Vec2 {
        self.center
    }

    pub fn half_size(&self) -> f32 {
        self.half_size
    }

    /// Jump straight to the fitted view, skipping smoothing. Use on scenario
    /// reset, where interpolating from the previous scenario's framing would
    /// show a pointless slide.
    pub fn snap(&mut self, objects: &[PhysicsObject]) {
        if let Some((center, half_size)) = self.target(objects) {
            self.center = center;
            self.half_size = half_size;
        }
    }

    /// Advance the smoothed view one frame. `dt` is real elapsed seconds; the
    /// smoothing is frame-rate independent, so the same motion is produced at
    /// 30 and 144 FPS.
    pub fn update(&mut self, objects: &[PhysicsObject], dt: f32) {
        let Some((target_center, target_half)) = self.target(objects) else {
            return;
        };
        if dt <= 0.0 {
            return;
        }

        let rate = if target_half > self.half_size { EXPAND_RATE } else { CONTRACT_RATE };
        let t = 1.0 - (-rate * dt).exp();
        self.center += (target_center - self.center) * t;
        self.half_size += (target_half - self.half_size) * t;
    }

    // Fitted (center, half_size), or None if the input can't produce a
    // usable view (empty, massless, or non-finite positions — the latter can
    // happen if the integration diverges; holding the previous view beats
    // propagating NaN into the projection).
    fn target(&mut self, objects: &[PhysicsObject]) -> Option<(Vec2, f32)> {
        if objects.is_empty() {
            return None;
        }

        let mut total_mass = 0.0f32;
        let mut weighted = Vec2::ZERO;
        for obj in objects {
            total_mass += obj.mass;
            weighted += obj.position * obj.mass;
        }
        if total_mass <= 0.0 {
            return None;
        }
        let center = weighted / total_mass;
        if !center.is_finite() {
            return None;
        }

        self.dist_sq.clear();
        self.dist_sq.extend(objects.iter().map(|o| (o.position - center).length_squared()));

        // Index of the FIT_PERCENTILE-th smallest distance. `total_cmp` rather
        // than `partial_cmp().unwrap()`: a NaN distance sorts last instead of
        // panicking the render loop.
        let k = (((self.dist_sq.len() - 1) as f32) * FIT_PERCENTILE).round() as usize;
        let (_, nth, _) = self.dist_sq.select_nth_unstable_by(k, f32::total_cmp);
        let radius = nth.sqrt();
        if !radius.is_finite() {
            return None;
        }

        Some((center, (radius * FIT_MARGIN).max(self.floor_half_size)))
    }
}
