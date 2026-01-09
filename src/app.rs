use crate::egui_tools::EguiRenderer;
use crate::simulation::{Particle, Simulation};

use egui::Vec2;
use egui_wgpu::{ScreenDescriptor, wgpu};
use rand::Rng;
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::{Window, WindowId};

pub const TEXTURE_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Bgra8UnormSrgb;

pub struct AppState {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub surface_config: wgpu::SurfaceConfiguration,
    pub surface: wgpu::Surface<'static>,
    pub egui_renderer: EguiRenderer,

    pub simu: Simulation,
}

impl AppState {
    async fn new(
        instance: &wgpu::Instance,
        surface: wgpu::Surface<'static>,
        window: &Window,
        width: u32,
        height: u32,
    ) -> Self {
        let power_pref = wgpu::PowerPreference::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: power_pref,
                force_fallback_adapter: false,
                compatible_surface: Some(&surface),
            })
            .await
            .expect("Failed to find an appropriate adapter");

        let required_features = wgpu::Features::PUSH_CONSTANTS;
        let required_limits = wgpu::Limits {
            max_push_constant_size: 8,
            ..Default::default()
        };
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: None,
                required_features,
                required_limits,
                memory_hints: Default::default(),
                trace: Default::default(),
                experimental_features: Default::default(),
            })
            .await
            .expect("Failed to create device");

        let swapchain_capabilities = surface.get_capabilities(&adapter);
        let selected_format = TEXTURE_FORMAT;
        let swapchain_format = swapchain_capabilities
            .formats
            .iter()
            .find(|d| **d == selected_format)
            .expect("failed to select proper surface texture format!");

        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: *swapchain_format,
            width,
            height,
            present_mode: wgpu::PresentMode::AutoVsync,
            desired_maximum_frame_latency: 0,
            alpha_mode: swapchain_capabilities.alpha_modes[0],
            view_formats: vec![],
        };

        surface.configure(&device, &surface_config);

        let egui_renderer = EguiRenderer::new(&device, surface_config.format, None, 1, window);

        let simu = Simulation::new(&device);

        Self {
            device,
            queue,
            surface,
            surface_config,
            egui_renderer,
            simu,
        }
    }

    fn resize_surface(&mut self, width: u32, height: u32) {
        self.surface_config.width = width;
        self.surface_config.height = height;
        self.surface.configure(&self.device, &self.surface_config);
    }
}

pub struct App {
    instance: wgpu::Instance,
    state: Option<AppState>,
    window: Option<Arc<Window>>,
}

impl App {
    pub fn new() -> Self {
        let instance = egui_wgpu::wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
        Self {
            instance,
            state: None,
            window: None,
        }
    }

    async fn set_window(&mut self, window: Window) {
        let window = Arc::new(window);
        let initial_width = 1360;
        let initial_height = 768;

        let _ = window.request_inner_size(PhysicalSize::new(initial_width, initial_height));

        let surface = self
            .instance
            .create_surface(window.clone())
            .expect("Failed to create surface!");

        let state = AppState::new(
            &self.instance,
            surface,
            &window,
            initial_width,
            initial_width,
        )
        .await;

        self.window.get_or_insert(window);
        self.state.get_or_insert(state);
    }

    fn handle_resized(&mut self, width: u32, height: u32) {
        if width > 0 && height > 0 {
            self.state.as_mut().unwrap().resize_surface(width, height);
        }
        if let Some(state) = self.state.as_mut() {
            state.simu.param.aspect_ratio = width as f32 / height as f32;
        }
    }

    fn handle_redraw(&mut self) {
        let Some(window) = self.window.as_ref() else {
            return;
        };
        // Attempt to handle minimizing window
        if let Some(true) = window.is_minimized() {
            println!("Window is minimized");
            return;
        }

        let Some(state) = self.state.as_mut() else {
            return;
        };

        let surface_texture = state.surface.get_current_texture();
        let Ok(surface_texture) = surface_texture else {
            println!("surface err: {:?}", surface_texture);
            return;
        };

        let surface_view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = state
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });

        state.simu.update();
        draw_particles(state, &mut encoder, &surface_view);

        draw_egui(window, state, &mut encoder, &surface_view);

        state.queue.submit(Some(encoder.finish()));
        surface_texture.present();
    }
}

fn draw_particles(
    state: &mut AppState,
    encoder: &mut wgpu::CommandEncoder,
    surface_view: &wgpu::TextureView,
) {
    let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("simu render pass"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: surface_view,
            depth_slice: None,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.1,
                    g: 0.2,
                    b: 0.3,
                    a: 1.0,
                }),
                store: wgpu::StoreOp::Store,
            },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
    });

    state.simu.render(&mut state.queue, &mut render_pass);
}

fn draw_egui(
    window: &Window,
    state: &mut AppState,
    encoder: &mut wgpu::CommandEncoder,
    surface_view: &wgpu::TextureView,
) {
    let screen_descriptor = ScreenDescriptor {
        size_in_pixels: [state.surface_config.width, state.surface_config.height],
        pixels_per_point: window.scale_factor() as f32,
    };
    state.egui_renderer.begin_frame(window);

    egui::Window::new("simulation params")
        .resizable(true)
        .vscroll(true)
        .default_open(true)
        .show(state.egui_renderer.context(), |ui| {
            ui.add(egui::Slider::new(&mut state.simu.param.radius, 0.01..=0.05).text("radius"));
            ui.separator();
            ui.add(
                egui::Slider::new(&mut state.simu.param.gravity_acc, -1.0..=1.0).text("gravity"),
            );
            ui.separator();
            ui.add(
                egui::Slider::new(&mut state.simu.param.collapse_loss, -0.9..=0.9)
                    .text("collapse_loss"),
            );
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Add Particle").clicked() {
                    let mut rng = rand::rng();
                    state.simu.add_paticle(
                        &state.device,
                        Particle {
                            position: Vec2::new(
                                rng.random_range(-1.0..1.0),
                                rng.random_range(-1.0..1.0),
                            ),
                            velocity: Vec2::new(
                                rng.random_range(-1.0..1.0),
                                rng.random_range(-1.0..1.0),
                            ),
                        },
                    );
                }
                ui.separator();
                if ui.button("clear Particle").clicked() {
                    state.simu.clear_paticle(&state.device);
                }
            });
        });

    state.egui_renderer.end_frame_and_draw(
        &state.device,
        &state.queue,
        encoder,
        window,
        surface_view,
        screen_descriptor,
    );
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window = event_loop
            .create_window(Window::default_attributes())
            .unwrap();
        let (width, height) = (window.inner_size().width, window.inner_size().height);
        pollster::block_on(self.set_window(window));
        if let Some(state) = self.state.as_mut() {
            state.simu.param.aspect_ratio = width as f32 / height as f32;
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _: WindowId, event: WindowEvent) {
        // let egui render to process the event first
        self.state
            .as_mut()
            .unwrap()
            .egui_renderer
            .handle_input(self.window.as_ref().unwrap(), &event);

        match event {
            WindowEvent::CloseRequested => {
                println!("The close button was pressed; stopping");
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                self.handle_redraw();

                self.window.as_ref().unwrap().request_redraw();
            }
            WindowEvent::Resized(new_size) => {
                self.handle_resized(new_size.width, new_size.height);
            }
            _ => (),
        }
    }
}
