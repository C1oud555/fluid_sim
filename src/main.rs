mod app;
mod circle_mesh;
mod egui_tools;
mod simu_render;
mod simulation;
use winit::event_loop::{ControlFlow, EventLoop};

fn main() {
    pollster::block_on(run());
}

async fn run() {
    let event_loop = EventLoop::new().unwrap();

    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = app::App::new();

    event_loop.run_app(&mut app).expect("Failed to run app");
}
