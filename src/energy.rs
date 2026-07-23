#[cfg(not(target_arch = "wasm32"))]
use std::sync::mpsc;
#[cfg(not(target_arch = "wasm32"))]
use std::thread::JoinHandle;

#[cfg(not(target_arch = "wasm32"))]
use crate::physics::Physics;
use crate::physics::PhysicsObject;

// Exact total-energy computation off the render thread. The render loop calls
// `request()` with an immutable snapshot at whatever cadence it likes, and
// `try_recv()` every frame; the computation runs on a background thread
// (native) so the ~1s O(n^2) cost at large n never stalls rendering.
//
// On wasm32 this is a no-op stub (request ignored, try_recv always None):
// std::thread is unavailable without SharedArrayBuffer. The deployed site
// serves COOP/COEP headers (docs/web-deploy.md), so a future wasm backend
// could wire wasm-bindgen-rayon into this same struct API without touching
// main.rs.
pub struct EnergyWorker {
    #[cfg(not(target_arch = "wasm32"))]
    rx: Option<mpsc::Receiver<f32>>,
    #[cfg(not(target_arch = "wasm32"))]
    handle: Option<JoinHandle<()>>,
}

impl EnergyWorker {
    pub fn new() -> Self {
        Self {
            #[cfg(not(target_arch = "wasm32"))]
            rx: None,
            #[cfg(not(target_arch = "wasm32"))]
            handle: None,
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn busy(&self) -> bool {
        self.rx.is_some()
    }

    #[cfg(target_arch = "wasm32")]
    pub fn busy(&self) -> bool {
        false
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn request(&mut self, objects: &[PhysicsObject], softening: f32) {
        if self.busy() {
            return;
        }
        let snapshot: Vec<PhysicsObject> = objects.to_vec();
        let (tx, rx) = mpsc::channel();
        let handle = std::thread::spawn(move || {
            let energy = Physics::total_energy(&snapshot, softening);
            // If the receiver was dropped (e.g. worker cancelled), ignore.
            let _ = tx.send(energy);
        });
        self.rx = Some(rx);
        self.handle = Some(handle);
    }

    #[cfg(target_arch = "wasm32")]
    pub fn request(&mut self, _objects: &[PhysicsObject], _softening: f32) {
        // wasm stub: no background compute. See struct comment.
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn try_recv(&mut self) -> Option<f32> {
        let rx = self.rx.as_ref()?;
        match rx.try_recv() {
            Ok(energy) => {
                if let Some(handle) = self.handle.take() {
                    let _ = handle.join();
                }
                self.rx = None;
                Some(energy)
            }
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                self.rx = None;
                self.handle = None;
                None
            }
        }
    }

    #[cfg(target_arch = "wasm32")]
    pub fn try_recv(&mut self) -> Option<f32> {
        None
    }
}

impl Default for EnergyWorker {
    fn default() -> Self {
        Self::new()
    }
}
