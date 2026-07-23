use std::sync::mpsc;
use std::thread::JoinHandle;

use crate::physics::{Physics, PhysicsObject};

// Exact total-energy computation off the render thread. The render loop calls
// `request()` with an immutable snapshot at whatever cadence it likes, and
// `try_recv()` every frame; the computation itself happens on a background
// thread (native) so the ~1s O(n^2) cost at large n never stalls rendering.
//
// WASM note: std::thread does not work on wasm32 without wasm-bindgen-rayon +
// SharedArrayBuffer + COOP/COEP headers. The wasm backend below is therefore a
// documented no-op stub (try_recv always None, request ignored). A future wasm
// implementation should plug a web-worker / rayon backend into this same
// struct API so main.rs stays unchanged.
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
    pub fn request(&mut self, objects: &[PhysicsObject]) {
        if self.busy() {
            return;
        }
        let snapshot: Vec<PhysicsObject> = objects.to_vec();
        let (tx, rx) = mpsc::channel();
        let handle = std::thread::spawn(move || {
            let energy = Physics::total_energy(&snapshot);
            // If the receiver was dropped (e.g. worker cancelled), ignore.
            let _ = tx.send(energy);
        });
        self.rx = Some(rx);
        self.handle = Some(handle);
    }

    #[cfg(target_arch = "wasm32")]
    pub fn request(&mut self, _objects: &[PhysicsObject]) {
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
