use std::time::Instant;

use egui::Vec2;
use egui_wgpu::wgpu::{self, util::DeviceExt};
use winit::dpi::PhysicalSize;

use crate::simu_render::SimuRender;

pub struct SimuParams {
    pub radius: f32,
    pub mass: f32,

    pub gravity: f32,
    pub boundary_damping: f32,
}

impl Default for SimuParams {
    fn default() -> Self {
        Self {
            radius: 3.0,
            mass: 50.0,
            gravity: 9.8,
            boundary_damping: 0.95,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Particle {
    pub position: Vec2,
    pub velocity: Vec2,
}

pub struct Simulation {
    pub param: SimuParams,
    pub box_size: (f32, f32),
    particles: Vec<Particle>,

    render: SimuRender,
    lastframe_timestamp: Instant,
}

impl Simulation {
    pub fn new(device: &wgpu::Device, window_size: &PhysicalSize<u32>) -> Self {
        let mut particles = vec![];
        let spacing = 15.0;
        let start = Vec2::new(-200.0, -200.0);

        for x in 0..50 {
            for y in 0..50 {
                particles.push(Particle {
                    position: Vec2::new(x as f32 * spacing, y as f32 * spacing) + start,
                    velocity: Vec2::ZERO,
                })
            }
        }

        let param = Default::default();
        let render = SimuRender::new(device, &particles);

        Self {
            param,
            particles,
            box_size: (window_size.width as f32, window_size.height as f32),
            render,
            lastframe_timestamp: Instant::now(),
        }
    }

    pub fn update(&mut self) {
        // apply force
        let delta = self.lastframe_timestamp.elapsed().as_secs_f32();

        self.apply_force(Vec2::new(0.0, self.param.gravity), delta);

        self.move_particle(delta);

        self.lastframe_timestamp = Instant::now();
    }

    pub fn apply_force(&mut self, force: Vec2, delta: f32) {
        let (fx, fy) = (force.x * self.param.mass, force.y * self.param.mass);
        for particle in self.particles.iter_mut() {
            if particle.position.x >= self.box_size.0 {
                particle.position.x -= 3.0;
                particle.velocity.x = -particle.velocity.x * self.param.boundary_damping;
            } else if particle.position.x <= -self.box_size.0 {
                particle.position.x += 3.0;
                particle.velocity.x = -particle.velocity.x * self.param.boundary_damping;
            } else if particle.position.y >= self.box_size.1 {
                particle.position.y -= 3.0;
                particle.velocity.y = -particle.velocity.y * self.param.boundary_damping;
            } else if particle.position.y <= -self.box_size.1 {
                particle.position.y += 3.0;
                particle.velocity.y = -particle.velocity.y * self.param.boundary_damping;
            } else {
                particle.velocity.x += fx * delta;
                particle.velocity.y += fy * delta;
            }
        }
    }

    pub fn move_particle(&mut self, delta: f32) {
        for particle in self.particles.iter_mut() {
            particle.position.x += particle.velocity.x * delta;
            particle.position.y += particle.velocity.y * delta;
        }
    }

    pub fn add_paticle(&mut self, device: &wgpu::Device, par: Particle) {
        self.particles.push(par);
        // update buffer layout

        self.render.instance_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("instance buffer"),
                contents: bytemuck::cast_slice(&self.particles),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            });
    }

    pub fn clear_paticle(&mut self, device: &wgpu::Device) {
        self.particles.clear();
        // update buffer layout

        self.render.instance_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("instance buffer"),
                contents: bytemuck::cast_slice(&self.particles),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            });
    }

    pub fn render(&self, queue: &mut wgpu::Queue, render_pass: &mut wgpu::RenderPass) {
        self.render
            .render(queue, render_pass, &self.param, &self.particles);
    }

    pub fn update_size(&mut self, width: f32, height: f32) {
        self.render.uniforms = cgmath::Matrix4::from_cols(
            cgmath::Vector4::new(1.0 / width, 0.0, 0.0, 0.0),
            cgmath::Vector4::new(0.0, 1.0 / height, 0.0, 0.0),
            cgmath::Vector4::new(0.0, 0.0, 1.0, 0.0),
            cgmath::Vector4::new(0.0, 0.0, 0.0, 1.0),
        )
        .into();

        self.box_size = (width, height);
    }
}
