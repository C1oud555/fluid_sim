use std::time::Instant;

use rustc_hash::FxHashMap;
use std::hash::Hash;
use std::sync::RwLock;

use rayon::prelude::*;

use egui_wgpu::wgpu::{self, util::DeviceExt};
use winit::dpi::PhysicalSize;

use crate::simu_render::SimuRender;

const H: f32 = 16.0;
const HSQ: f32 = H * H;

const REST_DENS: f32 = 300.0;
const GAS_CONST: f32 = 2000.0;
const VISC: f32 = 200.0;

// Compute SPH constants at runtime
fn poly6() -> f32 {
    4.0 / (std::f32::consts::PI * H.powi(8))
}

fn spiky_grad() -> f32 {
    -10.0 / (std::f32::consts::PI * H.powi(5))
}

fn visc_lap() -> f32 {
    40.0 / (std::f32::consts::PI * H.powi(5))
}

// Color visualization mode
#[derive(Clone, Copy, PartialEq)]
pub enum ColorMode {
    Pressure,
    Velocity,
    Density,
}

impl Default for ColorMode {
    fn default() -> Self {
        ColorMode::Pressure
    }
}

#[derive(Default, Clone, Copy)]
pub struct SimuParams {
    pub radius: f32,
    pub mass: f32,
    pub gravity: f32,
    pub boundary_damping: f32,
    pub color_mode: ColorMode,
}

impl SimuParams {
    pub fn new() -> Self {
        Self {
            radius: 5.0,
            mass: 2.5,
            gravity: -9.8,
            boundary_damping: 0.5,
            color_mode: ColorMode::Pressure,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Particle {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub density: f32,
    pub pressure: f32,
    pub color: [f32; 3],
}

#[derive(Default, Clone, Copy)]
pub struct ParProperty {
    pub velocity: [f32; 2],
    pub density: f32,
    pub pressure: f32,
    pub force: [f32; 2],
}

/// 空间哈希网格 - 用于加速邻居查找
struct SpatialHash {
    grid: RwLock<FxHashMap<GridCell, Vec<usize>>>,
    grid_size: f32,
    neighbor_buffer: RwLock<Vec<usize>>,
}

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
struct GridCell {
    x: i32,
    y: i32,
}

impl SpatialHash {
    fn new(grid_size: f32, max_particles: usize) -> Self {
        Self {
            grid: RwLock::new(FxHashMap::default()),
            grid_size,
            neighbor_buffer: RwLock::new(Vec::with_capacity(max_particles)),
        }
    }

    fn clear(&self) {
        self.grid.write().unwrap().clear();
        self.neighbor_buffer.write().unwrap().clear();
    }

    fn insert(&self, idx: usize, pos: [f32; 2]) {
        let cell = self.get_cell(pos);
        self.grid
            .write()
            .unwrap()
            .entry(cell)
            .or_insert_with(Vec::new)
            .push(idx);
    }

    #[inline]
    fn get_cell(&self, pos: [f32; 2]) -> GridCell {
        GridCell {
            x: (pos[0] / self.grid_size).floor() as i32,
            y: (pos[1] / self.grid_size).floor() as i32,
        }
    }

    #[inline]
    fn get_neighbor_cells(&self, cell: GridCell) -> [GridCell; 9] {
        [
            GridCell {
                x: cell.x - 1,
                y: cell.y - 1,
            },
            GridCell {
                x: cell.x,
                y: cell.y - 1,
            },
            GridCell {
                x: cell.x + 1,
                y: cell.y - 1,
            },
            GridCell {
                x: cell.x - 1,
                y: cell.y,
            },
            GridCell {
                x: cell.x,
                y: cell.y,
            },
            GridCell {
                x: cell.x + 1,
                y: cell.y,
            },
            GridCell {
                x: cell.x - 1,
                y: cell.y + 1,
            },
            GridCell {
                x: cell.x,
                y: cell.y + 1,
            },
            GridCell {
                x: cell.x + 1,
                y: cell.y + 1,
            },
        ]
    }

    fn fill_neighbors(&self, pos: [f32; 2], buffer: &mut Vec<usize>) {
        let cell = self.get_cell(pos);
        let neighbor_cells = self.get_neighbor_cells(cell);

        let guard = self.grid.read().unwrap();
        buffer.clear();

        for cell in neighbor_cells {
            if let Some(indices) = guard.get(&cell) {
                buffer.extend(indices.iter().copied());
            }
        }
    }
}

pub struct Simulation {
    pub param: SimuParams,
    pub box_size: (f32, f32),
    particles: Vec<Particle>,
    properties: Vec<ParProperty>,
    spatial_hash: SpatialHash,
    render: SimuRender,
    pub lastframe_timestamp: Instant,
}

impl Simulation {
    pub fn new(device: &wgpu::Device, window_size: &PhysicalSize<u32>) -> Self {
        let mut particles = vec![];
        let mut properties = vec![];
        let spacing = 10.0;
        let start = [-200.0, -200.0];

        for x in 0..100 {
            for y in 0..50 {
                particles.push(Particle {
                    position: [x as f32 * spacing + start[0], y as f32 * spacing + start[1]],
                    velocity: [0.0, 0.0],
                    density: 0.0,
                    pressure: 0.0,
                    color: [0.0, 0.5, 1.0],
                });
                properties.push(ParProperty::default());
            }
        }

        let param = SimuParams::new();
        let render = SimuRender::new(device, &particles);

        Self {
            param,
            particles,
            box_size: (window_size.width as f32, window_size.height as f32),
            render,
            lastframe_timestamp: Instant::now(),
            properties,
            spatial_hash: SpatialHash::new(H, 10000),
        }
    }

    pub fn update(&mut self) {
        let delta = 0.0007;

        // 构建空间哈希网格
        self.spatial_hash.clear();
        self.particles
            .iter()
            .enumerate()
            .for_each(|(idx, p)| self.spatial_hash.insert(idx, p.position));

        // 并行计算密度和压力
        self.compute_density_pressure();

        // 并行计算力
        self.compute_force();

        self.apply_force(delta);

        self.move_particle(delta);

        // 更新颜色
        self.update_colors();
    }

    fn compute_density_pressure(&mut self) {
        let particles = &self.particles;
        let param = self.param;
        let spatial_hash = &self.spatial_hash;

        let new_densities: Vec<f32> = particles
            .par_iter()
            .enumerate()
            .map(|(_i, particle)| {
                let mut buffer = Vec::with_capacity(64);
                spatial_hash.fill_neighbors(particle.position, &mut buffer);

                let mut density = 0.0;
                for &j in &buffer {
                    let other = particles[j];
                    let dx = other.position[0] - particle.position[0];
                    let dy = other.position[1] - particle.position[1];
                    let r2 = dx * dx + dy * dy;

                    if r2 < HSQ {
                        density += param.mass * poly6() * (HSQ - r2).powi(3);
                    }
                }
                density
            })
            .collect();

        for (i, density) in new_densities.iter().enumerate() {
            self.properties[i].density = *density;
            self.properties[i].pressure = GAS_CONST * (density - REST_DENS);
        }
    }

    fn compute_force(&mut self) {
        let particles = &self.particles;
        let properties = &self.properties;
        let param = self.param;
        let spatial_hash = &self.spatial_hash;

        let new_forces: Vec<[f32; 2]> = particles
            .par_iter()
            .enumerate()
            .map(|(i, particle)| {
                let mut fpressure = [0.0, 0.0];
                let mut fvisc = [0.0, 0.0];

                let this_prop = properties[i];
                let mut buffer = Vec::with_capacity(64);
                spatial_hash.fill_neighbors(particle.position, &mut buffer);

                for &j in &buffer {
                    if i == j {
                        continue;
                    }

                    let other = particles[j];
                    let other_prop = properties[j];

                    let dx = other.position[0] - particle.position[0];
                    let dy = other.position[1] - particle.position[1];
                    let r = (dx * dx + dy * dy).sqrt();

                    if r < H && r > 0.0001 {
                        let r_normalized = [dx / r, dy / r];
                        let pressure_force = -param.mass
                            * (this_prop.pressure + other_prop.pressure)
                            / (2.0 * other_prop.density)
                            * spiky_grad()
                            * (H - r).powi(3);

                        fpressure[0] += r_normalized[0] * pressure_force;
                        fpressure[1] += r_normalized[1] * pressure_force;

                        let visc_scalar =
                            VISC * param.mass / other_prop.density * visc_lap() * (H - r);
                        fvisc[0] += (other_prop.velocity[0] - this_prop.velocity[0]) * visc_scalar;
                        fvisc[1] += (other_prop.velocity[1] - this_prop.velocity[1]) * visc_scalar;
                    }
                }

                let fgrav = [0.0, param.gravity * param.mass / this_prop.density];

                [
                    fpressure[0] + fvisc[0] + fgrav[0],
                    fpressure[1] + fvisc[1] + fgrav[1],
                ]
            })
            .collect();

        for (i, force) in new_forces.iter().enumerate() {
            self.properties[i].force = *force;
        }
    }

    pub fn apply_force(&mut self, delta: f32) {
        let box_half_w = self.box_size.0;
        let box_half_h = self.box_size.1;
        let damping = self.param.boundary_damping;
        let push_back = 3.0;

        for (particle, prop) in self.particles.iter_mut().zip(self.properties.iter_mut()) {
            prop.velocity[0] += prop.force[0] * delta / prop.density;
            prop.velocity[1] += prop.force[1] * delta / prop.density;

            if particle.position[0] >= box_half_w {
                particle.position[0] -= push_back;
                prop.velocity[0] *= -damping;
            } else if particle.position[0] <= -box_half_w {
                particle.position[0] += push_back;
                prop.velocity[0] *= -damping;
            }

            if particle.position[1] >= box_half_h {
                particle.position[1] -= push_back;
                prop.velocity[1] *= -damping;
            } else if particle.position[1] <= -box_half_h {
                particle.position[1] += push_back;
                prop.velocity[1] *= -damping;
            }
        }
    }

    pub fn move_particle(&mut self, delta: f32) {
        for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
            particle.position[0] += prop.velocity[0] * delta;
            particle.position[1] += prop.velocity[1] * delta;
        }
    }

    /// 根据压力/速度/密度更新粒子颜色
    /// 值越大越明亮，值越小越暗淡
    fn update_colors(&mut self) {
        match self.param.color_mode {
            ColorMode::Pressure => {
                for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
                    let t = (prop.pressure / 3000.0).clamp(0.0, 1.0);
                    let t = t * t * (3.0 - 2.0 * t);
                    particle.color = [t * 1.0, t * 0.6, t * 0.1];
                }
            }
            ColorMode::Velocity => {
                for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
                    let speed = (prop.velocity[0].powi(2) + prop.velocity[1].powi(2)).sqrt();
                    let t = (speed / 150.0).clamp(0.0, 1.0);
                    let t = t * t * (3.0 - 2.0 * t);
                    particle.color = [t * 0.5, t * 1.0, t * 1.0];
                }
            }
            ColorMode::Density => {
                for (particle, prop) in self.particles.iter_mut().zip(&self.properties) {
                    let t = ((prop.density - REST_DENS * 0.8) / (REST_DENS * 0.4)).clamp(0.0, 1.0);
                    let t = t * t * (3.0 - 2.0 * t);
                    particle.color = [t * 0.2, t * 1.0, t * 0.4];
                }
            }
        }
    }

    pub fn add_paticle(&mut self, device: &wgpu::Device, par: Particle) {
        self.particles.push(par);
        self.properties.push(ParProperty::default());

        self.render.instance_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("instance buffer"),
                contents: bytemuck::cast_slice(&self.particles),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            });
    }

    pub fn clear_paticle(&mut self, device: &wgpu::Device) {
        self.particles.clear();

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
