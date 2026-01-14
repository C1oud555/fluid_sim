use std::time::Instant;

use egui::Vec2;
use egui_wgpu::wgpu::{self, util::DeviceExt};
use winit::dpi::PhysicalSize;

use crate::simu_render::SimuRender;

use std::f32::consts::PI;
const H: f32 = 16.0;
const HSQ: f32 = H * H;

const REST_DENS: f32 = 300.0;
const GAS_CONST: f32 = 2000.0;
const VISC: f32 = 200.0;

static POLY6: f32 = 4.0 / (PI * H * H * H * H * H * H * H * H);
static SPIKY_GRAD: f32 = -10.0 / (PI * H * H * H * H * H);
static VISC_LAP: f32 = 40.0 / (PI * H * H * H * H * H);

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
}

#[derive(Default, Clone, Copy)]
pub struct ParProperty {
    pub velocity: Vec2,
    pub density: f32,
    pub pressure: f32,
    pub force: Vec2,
}

pub struct Simulation {
    pub param: SimuParams,
    pub box_size: (f32, f32),
    particles: Vec<Particle>,
    properties: Vec<ParProperty>,

    render: SimuRender,
    lastframe_timestamp: Instant,
}

impl Simulation {
    pub fn new(device: &wgpu::Device, window_size: &PhysicalSize<u32>) -> Self {
        let mut particles = vec![];
        let mut properties = vec![];
        let spacing = 15.0;
        let start = Vec2::new(-200.0, -200.0);

        for x in 0..50 {
            for y in 0..50 {
                particles.push(Particle {
                    position: Vec2::new(x as f32 * spacing, y as f32 * spacing) + start,
                });
                properties.push(ParProperty::default());
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
            properties,
        }
    }

    pub fn update(&mut self) {
        // apply force
        let delta = self.lastframe_timestamp.elapsed().as_secs_f32();

        self.compute_density_pressure();
        self.compute_force();

        self.apply_force(delta);

        self.move_particle(delta);

        self.lastframe_timestamp = Instant::now();
    }

    fn compute_density_pressure(&mut self) {
        for (prop, particle) in self.properties.iter_mut().zip(&self.particles) {
            prop.density = 0.0;
            for j in 0..self.particles.len() {
                let distance = particle.position - self.particles[j].position;
                let r2 = distance.length_sq();
                if r2 < HSQ {
                    prop.density += self.param.mass * POLY6 * f32::powf(HSQ - r2, 3.0);
                }
            }
            prop.pressure = GAS_CONST * (prop.density - REST_DENS);
        }
    }

    fn compute_force(&mut self) {
        for i in 0..self.particles.len() {
            let mut fpressure = Vec2::ZERO;
            let mut fvisc = Vec2::ZERO;
            for j in 0..self.particles.len() {
                if i == j {
                    continue;
                }
                let other = self.properties[j].clone();
                let this = &mut self.properties[i];
                let rji = self.particles[j].position - self.particles[i].position;
                let r = rji.length();
                if r < H {
                    fpressure +=
                        -rji.normalized() * self.param.mass * (this.pressure + other.pressure)
                            / (2.0 * other.density)
                            * SPIKY_GRAD
                            * f32::powf(H - r, 3.0);
                    fvisc = VISC * self.param.mass * (other.velocity - this.velocity)
                        / other.density
                        * VISC_LAP
                        * (H - r);
                }
            }
            let fgrav =
                Vec2::new(0.0, self.param.gravity) * self.param.mass / self.properties[i].density;
            self.properties[i].force = fpressure + fvisc + fgrav;
        }
    }

    pub fn apply_force(&mut self, delta: f32) {
        for (particle, prop) in self.particles.iter_mut().zip(self.properties.iter_mut()) {
            let (fx, fy) = (prop.force.x, prop.force.y);
            if particle.position.x >= self.box_size.0 {
                particle.position.x -= 3.0;
                prop.velocity.x = -prop.velocity.x * self.param.boundary_damping;
            } else if particle.position.x <= -self.box_size.0 {
                particle.position.x += 3.0;
                prop.velocity.x = -prop.velocity.x * self.param.boundary_damping;
            } else if particle.position.y >= self.box_size.1 {
                particle.position.y -= 3.0;
                prop.velocity.y = -prop.velocity.y * self.param.boundary_damping;
            } else if particle.position.y <= -self.box_size.1 {
                particle.position.y += 3.0;
                prop.velocity.y = -prop.velocity.y * self.param.boundary_damping;
            } else {
                prop.velocity.x += fx * delta;
                prop.velocity.y += fy * delta;
            }
        }
    }

    pub fn move_particle(&mut self, delta: f32) {
        for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
            particle.position.x += prop.velocity.x * delta;
            particle.position.y += prop.velocity.y * delta;
        }
    }

    pub fn add_paticle(&mut self, device: &wgpu::Device, par: Particle) {
        self.particles.push(par);
        self.properties.push(ParProperty::default());
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
