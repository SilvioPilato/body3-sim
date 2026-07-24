//! Hand-rolled Web Worker thread pool for the wasm build (no wasm-bindgen).
//!
//! M2 scaffolding: a single shared atomic. A worker instantiates the same wasm
//! module against the main instance's imported SharedArrayBuffer memory and
//! calls `pool_worker_bump`; if the main instance then sees the incremented
//! `pool_counter_get`, the two instances genuinely share one address space —
//! the foundation the force-eval pool is built on. Kept minimal and leaf-ish
//! (a bare atomic add, no meaningful stack/TLS use) because per-worker stack
//! and TLS are not set up until M3.
use core::arch::wasm32;
use core::cell::Cell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use macroquad::math::Vec2;

use crate::physics::{Physics, PhysicsObject};
use crate::quadtree::Quadtree;

static SHARED_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Read the shared counter (called from the main instance).
#[unsafe(no_mangle)]
pub extern "C" fn pool_counter_get() -> u32 {
    SHARED_COUNTER.load(Ordering::SeqCst)
}

/// Increment the shared counter (called from a worker instance).
#[unsafe(no_mangle)]
pub extern "C" fn pool_worker_bump() {
    SHARED_COUNTER.fetch_add(1, Ordering::SeqCst);
}

// ---- M3: per-worker stack + TLS ----

/// Allocate a block in the shared heap for a worker's stack or TLS region.
/// Intentionally leaked — the pool lives for the whole program. The main
/// instance calls this (it is fully initialized); the returned address is in
/// shared memory and therefore valid in every worker instance too.
#[unsafe(no_mangle)]
pub extern "C" fn pool_alloc(size: usize, align: usize) -> *mut u8 {
    let layout = std::alloc::Layout::from_size_align(size.max(1), align.max(16))
        .expect("valid pool layout");
    // SAFETY: non-zero size; block is deliberately never freed.
    unsafe { std::alloc::alloc(layout) }
}

thread_local! {
    static TL_MARK: Cell<u32> = const { Cell::new(0) };
}

/// Exercises a worker's freshly set-up stack and TLS: recurses `depth` frames
/// (stack) and reads/writes a `#[thread_local]` (TLS). Returns a deterministic
/// value — for a thread whose TL_MARK starts at 0, `pool_selftest(1000)` is
/// sum(1..=1000) + 1 = 500501. A wrong TLS/stack setup yields a wrong value or
/// a trap, making M3 verifiable.
#[unsafe(no_mangle)]
pub extern "C" fn pool_selftest(depth: u32) -> u32 {
    fn sum_to(n: u32) -> u32 {
        if n == 0 { 0 } else { n.wrapping_add(sum_to(n - 1)) }
    }
    TL_MARK.with(|m| m.set(m.get().wrapping_add(1)));
    let s = sum_to(depth);
    TL_MARK.with(|m| s.wrapping_add(m.get()))
}

// ---- M4: work-stealing force-eval pool ----
//
// One job at a time. The main thread publishes a job (raw pointers to the
// objects slice, the Quadtree, and the output buffer, plus the theta/softening
// scalars) and bumps EPOCH; workers block on EPOCH via a wasm futex wait, then
// everyone — workers and the main thread — steals CHUNK-sized index ranges via
// NEXT_CHUNK.fetch_add and computes body_acceleration for each body. Each
// finished chunk bumps COMPLETED; the main thread returns only once
// COMPLETED == TOTAL_CHUNKS, so correctness never depends on how many workers
// happened to register (a registration race just shifts who does the work).
//
// Safety: the input (objects, tree) is read-only and shared; output ranges are
// disjoint by chunk; the main thread stays parked in pool_run until every chunk
// is done, so the pointers it published outlive all worker accesses.

const CHUNK: usize = 256; // bodies per steal

static EPOCH: AtomicU32 = AtomicU32::new(0);
static NEXT_CHUNK: AtomicU32 = AtomicU32::new(0);
static COMPLETED: AtomicU32 = AtomicU32::new(0);
static TOTAL_CHUNKS: AtomicU32 = AtomicU32::new(0);
static WORKER_COUNT: AtomicU32 = AtomicU32::new(0);

static N_BODIES: AtomicUsize = AtomicUsize::new(0);
static OBJECTS_PTR: AtomicUsize = AtomicUsize::new(0);
static TREE_PTR: AtomicUsize = AtomicUsize::new(0);
static OUT_PTR: AtomicUsize = AtomicUsize::new(0);
static THETA_SQ_BITS: AtomicU32 = AtomicU32::new(0);
static SOFTENING_BITS: AtomicU32 = AtomicU32::new(0);

// Steal and process chunks until the job is drained. Called by both the worker
// loop and the main thread. Reads the published job out of the shared statics.
fn drain_chunks() {
    let n = N_BODIES.load(Ordering::Relaxed);
    if n == 0 {
        return;
    }
    let objects = unsafe {
        core::slice::from_raw_parts(OBJECTS_PTR.load(Ordering::Relaxed) as *const PhysicsObject, n)
    };
    let tree = unsafe { &*(TREE_PTR.load(Ordering::Relaxed) as *const Quadtree) };
    let out = OUT_PTR.load(Ordering::Relaxed) as *mut Vec2;
    let theta_sq = f32::from_bits(THETA_SQ_BITS.load(Ordering::Relaxed));
    let softening = f32::from_bits(SOFTENING_BITS.load(Ordering::Relaxed));
    let total = TOTAL_CHUNKS.load(Ordering::Relaxed);

    loop {
        let c = NEXT_CHUNK.fetch_add(1, Ordering::Relaxed);
        if c >= total {
            break;
        }
        let start = (c as usize) * CHUNK;
        let end = (start + CHUNK).min(n);
        for i in start..end {
            let a = Physics::body_acceleration(i, objects, tree, theta_sq, softening);
            // SAFETY: `i < n`, and chunks partition [0, n) so writes never alias.
            unsafe { *out.add(i) = a };
        }
        COMPLETED.fetch_add(1, Ordering::Release);
    }
}

/// Worker entry point. Registers, then blocks on each new EPOCH and helps drain
/// the job. Never returns. Call once per worker after its stack/TLS are set up.
#[unsafe(no_mangle)]
pub extern "C" fn pool_worker_loop() {
    WORKER_COUNT.fetch_add(1, Ordering::Release);
    let mut last = EPOCH.load(Ordering::Acquire);
    let epoch_ptr = EPOCH.as_ptr() as *mut i32;
    loop {
        // Block until EPOCH changes. Returns immediately if it already has.
        unsafe { wasm32::memory_atomic_wait32(epoch_ptr, last as i32, -1) };
        let e = EPOCH.load(Ordering::Acquire);
        if e == last {
            continue; // spurious wake
        }
        last = e;
        drain_chunks();
        // Piggyback on the force job's wake: it fires every frame, so this is
        // the energy job's only wake signal too (see the M5 comment below).
        drain_energy_chunks(u32::MAX);
    }
}

/// Number of registered workers (diagnostic).
#[unsafe(no_mangle)]
pub extern "C" fn pool_worker_count() -> u32 {
    WORKER_COUNT.load(Ordering::Relaxed)
}

/// Run a force evaluation across the pool: fills `out_ptr[0..n]` with the
/// acceleration on each body. Publishes the job, wakes workers, participates on
/// the calling (main) thread, then spin-waits until every chunk is done — the
/// main thread cannot `Atomics.wait`, and it has nothing else to do meanwhile.
#[unsafe(no_mangle)]
pub extern "C" fn pool_run(
    objects_ptr: usize,
    n: usize,
    tree_ptr: usize,
    out_ptr: usize,
    theta_sq: f32,
    softening: f32,
) {
    if n == 0 {
        return;
    }
    OBJECTS_PTR.store(objects_ptr, Ordering::Relaxed);
    N_BODIES.store(n, Ordering::Relaxed);
    TREE_PTR.store(tree_ptr, Ordering::Relaxed);
    OUT_PTR.store(out_ptr, Ordering::Relaxed);
    THETA_SQ_BITS.store(theta_sq.to_bits(), Ordering::Relaxed);
    SOFTENING_BITS.store(softening.to_bits(), Ordering::Relaxed);

    let total = n.div_ceil(CHUNK) as u32;
    TOTAL_CHUNKS.store(total, Ordering::Relaxed);
    NEXT_CHUNK.store(0, Ordering::Relaxed);
    COMPLETED.store(0, Ordering::Relaxed);

    // Publish (Release) and wake all workers.
    EPOCH.fetch_add(1, Ordering::Release);
    unsafe { wasm32::memory_atomic_notify(EPOCH.as_ptr() as *mut i32, u32::MAX) };

    // Main thread participates, then waits for every chunk (Acquire pairs with
    // the Release in drain_chunks so the output writes are visible).
    drain_chunks();
    while COMPLETED.load(Ordering::Acquire) < total {
        core::hint::spin_loop();
    }
    // Give the main thread a bounded chance to help the energy job along too,
    // in case no workers ever registered (drain_energy_chunks is otherwise
    // only reached from pool_worker_loop).
    drain_energy_chunks(1);
}

/// Self-check: build a `swarm_size` central swarm, compute its accelerations
/// both serially and via the pool over the same tree, and return the number of
/// components that differ. Because both paths call the identical
/// `body_acceleration` per body and chunks only partition the index range, the
/// results must be bit-identical — any nonzero return means the threading
/// corrupted state. 0 = pass.
#[unsafe(no_mangle)]
pub extern "C" fn pool_selfcheck(swarm_size: usize) -> u32 {
    use crate::simulation::{Scenario, Simulation, SimulationConfig};
    let sim = Simulation::new(SimulationConfig::for_scenario(Scenario::CentralSwarm { swarm_size }));
    let objects = sim.objects();
    let cfg = sim.config();
    let (rc, rh) = crate::quadtree::fitting_root(objects);
    let tree = Quadtree::build(objects, rc, rh);
    let serial = Physics::walk_forces(objects, &tree, cfg.theta_threshold, cfg.softening);
    let parallel = pooled_walk(objects, &tree, cfg.theta_threshold, cfg.softening);
    let mut mismatches = 0u32;
    for (a, b) in serial.iter().zip(parallel.iter()) {
        if a.x.to_bits() != b.x.to_bits() || a.y.to_bits() != b.y.to_bits() {
            mismatches += 1;
        }
    }
    mismatches
}

/// Dispatch a Barnes-Hut force walk across the pool. Result is identical to the
/// serial `Physics::walk_forces` (same per-body summation order), so it can be
/// verified against it. With no workers registered the calling thread does all
/// the chunks itself.
pub fn pooled_walk(objects: &[PhysicsObject], tree: &Quadtree, theta: f32, softening: f32) -> Vec<Vec2> {
    let n = objects.len();
    let mut out = vec![Vec2::ZERO; n];
    pool_run(
        objects.as_ptr() as usize,
        n,
        tree as *const Quadtree as usize,
        out.as_mut_ptr() as usize,
        theta * theta,
        softening,
    );
    out
}

// ---- M5: async exact-energy job ----
//
// total_energy is exact O(n^2) — too slow to compute inline on the render
// thread (~1.1s serial at n=44000, see energy.rs). Unlike the force walk this
// job is NOT rendezvoused: `EnergyJob::request` publishes it and returns
// immediately, and workers drain it opportunistically whenever they wake for
// a force job (which happens every frame, so this piggybacks on that wake
// instead of needing its own futex address). The calling thread also steals a
// single chunk at the end of every `pool_run` so the job still finishes even
// if no workers ever registered (the same degraded fallback the force walk
// has). Only one energy job may be in flight at a time — the Rust-side
// `EnergyWorker` (energy.rs) enforces that by refusing a new `request()`
// while the previous `EnergyJob` hasn't reported `done()`.
//
// Chunked over rows of the i<j pair triangle, which is uneven (row i has n-i
// pairs), so the chunk is smaller than the force walk's and work-stealing
// (NEXT_CHUNK.fetch_add) evens it out across whichever threads pick up work.
// Each chunk reduces into its own disjoint slot of a per-chunk output array
// (float adds aren't atomic, so chunks can't share an accumulator); the
// requester sums that array once `done()` is true.
const ENERGY_CHUNK: usize = 64;

static ENERGY_NEXT_CHUNK: AtomicU32 = AtomicU32::new(0);
static ENERGY_COMPLETED: AtomicU32 = AtomicU32::new(0);
static ENERGY_TOTAL_CHUNKS: AtomicU32 = AtomicU32::new(0);
static ENERGY_N: AtomicUsize = AtomicUsize::new(0);
static ENERGY_OBJECTS_PTR: AtomicUsize = AtomicUsize::new(0);
static ENERGY_OUT_PTR: AtomicUsize = AtomicUsize::new(0);
static ENERGY_SOFTENING_BITS: AtomicU32 = AtomicU32::new(0);

// Steal up to `max_chunks` energy chunks (pass u32::MAX to drain fully — safe
// for a background worker, but the main thread caps it so a frame can never
// stall on a large leftover backlog).
fn drain_energy_chunks(max_chunks: u32) {
    let n = ENERGY_N.load(Ordering::Relaxed);
    let total = ENERGY_TOTAL_CHUNKS.load(Ordering::Relaxed);
    if n == 0 || total == 0 {
        return;
    }
    let objects = unsafe {
        core::slice::from_raw_parts(ENERGY_OBJECTS_PTR.load(Ordering::Relaxed) as *const PhysicsObject, n)
    };
    let out = ENERGY_OUT_PTR.load(Ordering::Relaxed) as *mut f32;
    let softening = f32::from_bits(ENERGY_SOFTENING_BITS.load(Ordering::Relaxed));

    for _ in 0..max_chunks {
        let c = ENERGY_NEXT_CHUNK.fetch_add(1, Ordering::Relaxed);
        if c >= total {
            break;
        }
        let start = (c as usize) * ENERGY_CHUNK;
        let end = (start + ENERGY_CHUNK).min(n);
        let mut partial = 0.0f32;
        for i in start..end {
            partial += 0.5 * objects[i].mass * objects[i].velocity.length_squared();
            for j in (i + 1)..n {
                let dist_sq = Vec2::distance_squared(objects[i].position, objects[j].position) + softening * softening;
                partial += -crate::physics::GRAVITY * objects[i].mass * objects[j].mass / dist_sq.sqrt();
            }
        }
        // SAFETY: `c < total` and chunks partition disjoint output slots.
        unsafe { *out.add(c as usize) = partial };
        ENERGY_COMPLETED.fetch_add(1, Ordering::Release);
    }
}

/// A published, in-flight exact-energy computation. Owns the snapshot and
/// output buffer for as long as the job may still be read by a worker —
/// dropping it before `done()` is true would be a use-after-free for whichever
/// thread is mid-chunk.
pub struct EnergyJob {
    snapshot: Vec<PhysicsObject>,
    partials: Vec<f32>,
}

impl EnergyJob {
    pub fn request(objects: &[PhysicsObject], softening: f32) -> Self {
        let snapshot: Vec<PhysicsObject> = objects.to_vec();
        let n = snapshot.len();
        let total = n.div_ceil(ENERGY_CHUNK) as u32;
        let mut partials = vec![0.0f32; total as usize];

        ENERGY_OBJECTS_PTR.store(snapshot.as_ptr() as usize, Ordering::Relaxed);
        ENERGY_OUT_PTR.store(partials.as_mut_ptr() as usize, Ordering::Relaxed);
        ENERGY_SOFTENING_BITS.store(softening.to_bits(), Ordering::Relaxed);
        ENERGY_NEXT_CHUNK.store(0, Ordering::Relaxed);
        ENERGY_COMPLETED.store(0, Ordering::Relaxed);
        ENERGY_N.store(n, Ordering::Relaxed);
        // Publish total last (Release) — it's the field drain_energy_chunks
        // checks before touching the others, so this must become visible only
        // after everything it depends on already is.
        ENERGY_TOTAL_CHUNKS.store(total, Ordering::Release);

        Self { snapshot, partials }
    }

    /// Non-blocking. Once true, every chunk's partial sum is in `self.partials`
    /// and no thread will touch `self.snapshot`/`self.partials` again.
    pub fn done(&self) -> bool {
        ENERGY_COMPLETED.load(Ordering::Acquire) >= ENERGY_TOTAL_CHUNKS.load(Ordering::Relaxed)
    }

    pub fn total_energy(&self) -> f32 {
        let kinetic_and_potential: f32 = self.partials.iter().sum();
        let _ = &self.snapshot; // kept alive only for the safety argument above
        kinetic_and_potential
    }
}
