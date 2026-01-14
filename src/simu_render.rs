use cgmath::SquareMatrix;
use egui_wgpu::wgpu::util::DeviceExt;

use egui_wgpu::wgpu;

use crate::{
    app::TEXTURE_FORMAT,
    circle_mesh::CircleMesh,
    simulation::{Particle, SimuParams},
};

pub struct SimuRender {
    pub render_pipeline: wgpu::RenderPipeline,
    pub index_count: u32,
    pub vertex_buffer: wgpu::Buffer,
    pub indice_buffer: wgpu::Buffer,
    pub instance_buffer: wgpu::Buffer,
    pub uniform_buffer: wgpu::Buffer,
    pub uniform_bind_group: wgpu::BindGroup,
    pub uniforms: [[f32; 4]; 4],
}

impl SimuRender {
    pub fn new(device: &wgpu::Device, particles: &[Particle]) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("simu Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let push_constant_ranges = &[wgpu::PushConstantRange {
            stages: wgpu::ShaderStages::VERTEX,
            range: 0..4,
        }];

        let uniforms = cgmath::Matrix4::identity().into();

        let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("uniform buffer"),
            contents: bytemuck::cast_slice(&[uniforms]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("uniform buffer bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("uniform bind group"),
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("simu render pipeline layout"),
                bind_group_layouts: &[&uniform_bind_group_layout],
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
            contents: bytemuck::cast_slice(particles),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        Self {
            render_pipeline,
            index_count,
            vertex_buffer,
            indice_buffer,
            instance_buffer,
            uniform_buffer,
            uniform_bind_group,
            uniforms,
        }
    }

    pub fn render(
        &self,
        queue: &mut wgpu::Queue,
        render_pass: &mut wgpu::RenderPass,
        param: &SimuParams,
        particles: &[Particle],
    ) {
        let particle_count = particles.len() as u32;

        if particle_count > 0 {
            queue.write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(particles));
            // TODO: performance opt
            queue.write_buffer(
                &self.uniform_buffer,
                0,
                bytemuck::cast_slice(&[self.uniforms]),
            );

            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, &self.uniform_bind_group, &[]);

            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));

            render_pass.set_index_buffer(self.indice_buffer.slice(..), wgpu::IndexFormat::Uint16);

            render_pass.set_push_constants(
                wgpu::ShaderStages::VERTEX,
                0,
                bytemuck::bytes_of(&param.radius),
            );
            render_pass.draw_indexed(0..self.index_count, 0, 0..particle_count);
        }
    }
}
