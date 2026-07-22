use body3_sim::simulation::{Simulation, SimulationConfig};
use macroquad::prelude::*;

fn window_conf() -> Conf {
    let screen_size = SimulationConfig::default().screen_size;
    Conf {
        window_title: "Simulation".to_owned(),
        window_width: screen_size as i32,
        window_height: screen_size as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut sim = Simulation::new(SimulationConfig::default());
    loop {
        clear_background(BLACK);
        sim.update(get_frame_time());

        let total_energy = sim.total_energy();
        println!("total_energy={:.4}", total_energy);
        for obj in sim.objects() {
            draw_circle(obj.position.x, obj.position.y, 5.0, RED);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);

        next_frame().await
    }
}
