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
            radius: 5.0,
            mass: 2.5,
            gravity: -9.8,
            boundary_damping: 0.5,
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
    pub lastframe_timestamp: Instant,
}

impl Simulation {
    pub fn new(device: &wgpu::Device, window_size: &PhysicalSize<u32>) -> Self {
        let mut particles = vec![];
        let mut properties = vec![];
        let spacing = 10.0;
        let start = Vec2::new(-200.0, -200.0);

        for x in 0..100 {
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
        let delta = 0.0007;

        self.compute_density_pressure();
        self.compute_force();

        self.apply_force(delta);

        self.move_particle(delta);
    }

    fn compute_density_pressure(&mut self) {
        for (prop, particle) in self.properties.iter_mut().zip(&self.particles) {
            prop.density = 0.0;
            for j in 0..self.particles.len() {
                let distance = self.particles[j].position - particle.position;
                let r2 = distance.length() * distance.length();
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
                let other = self.properties[j];
                let this = self.properties[i];
                let rji = self.particles[j].position - self.particles[i].position;
                let r = rji.length();
                if r < H {
                    fpressure +=
                        -rji.normalized() * self.param.mass * (this.pressure + other.pressure)
                            / (2.0 * other.density)
                            * SPIKY_GRAD
                            * f32::powf(H - r, 3.0);
                    fvisc += VISC * self.param.mass * (other.velocity - this.velocity)
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
            prop.velocity += prop.force * delta / prop.density;
            if particle.position.x >= self.box_size.0 {
                particle.position.x -= 3.0;
                prop.velocity.x *= -self.param.boundary_damping;
            } else if particle.position.x <= -self.box_size.0 {
                particle.position.x += 3.0;
                prop.velocity.x *= -self.param.boundary_damping;
            } else if particle.position.y >= self.box_size.1 {
                particle.position.y -= 3.0;
                prop.velocity.y *= -self.param.boundary_damping;
            } else if particle.position.y <= -self.box_size.1 {
                particle.position.y += 3.0;
                prop.velocity.y *= -self.param.boundary_damping;
            }
        }
    }

    pub fn move_particle(&mut self, delta: f32) {
        for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
            particle.position += prop.velocity * delta;
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
