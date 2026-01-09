use egui_wgpu::wgpu::{self, util::DeviceExt};

use crate::{app::TEXTURE_FORMAT, circle_mesh::CircleMesh};

pub struct SimuParams {
    pub radius: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Particle {
    position: [f32; 2],
    velocity: [f32; 2],
}

pub struct Simulation {
    pub param: SimuParams,
    render_pipeline: wgpu::RenderPipeline,
    particles: Vec<Particle>,

    index_count: u32,
    vertex_buffer: wgpu::Buffer,
    indice_buffer: wgpu::Buffer,
    instance_buffer: wgpu::Buffer,
}

impl Simulation {
    pub fn new(device: &wgpu::Device) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("simu Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let push_constant_ranges = &[wgpu::PushConstantRange {
            stages: wgpu::ShaderStages::VERTEX,
            range: 0..4,
        }];

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("simu render pipeline layout"),
                bind_group_layouts: &[],
                push_constant_ranges,
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("simu render pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<[f32; 2]>() as wgpu::BufferAddress,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x2],
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Particle>() as wgpu::BufferAddress,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            1 => Float32x2,
                            2 => Float32x2,
                        ],
                    },
                ],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: TEXTURE_FORMAT,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });

        let particles = vec![
            Particle {
                position: [0.0, 0.0],
                velocity: [0.0, 0.0],
            },
            Particle {
                position: [0.2, 0.2],
                velocity: [0.0, 0.0],
            },
        ];

        let param = SimuParams { radius: 0.1 };
        let circle_mesh = CircleMesh::new(16);
        let index_count = circle_mesh.indices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("vertex buffer"),
            contents: bytemuck::cast_slice(&circle_mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let indice_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("indice buffer"),
            contents: bytemuck::cast_slice(&circle_mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("instance buffer"),
            contents: bytemuck::cast_slice(&particles),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        dbg!(circle_mesh.vertices);
        dbg!(circle_mesh.indices);
        dbg!(&particles);

        Self {
            param,
            particles,
            render_pipeline,
            vertex_buffer,
            indice_buffer,
            instance_buffer,
            index_count,
        }
    }

    pub fn update(&mut self) {}

    pub fn add_paticle(&mut self, par: Particle) {
        self.particles.push(par);
    }

    pub fn render(&self, render_pass: &mut wgpu::RenderPass) {
        let particle_count = self.particles.len() as u32;

        render_pass.set_pipeline(&self.render_pipeline);

        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));

        render_pass.set_index_buffer(self.indice_buffer.slice(..), wgpu::IndexFormat::Uint16);

        render_pass.set_push_constants(
            wgpu::ShaderStages::VERTEX,
            0,
            bytemuck::bytes_of(&self.param.radius),
        );
        render_pass.draw_indexed(0..self.index_count, 0, 0..particle_count);
    }
}
