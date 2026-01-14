struct PushConstants {
  radius: f32,
};

var<push_constant> push_constants: PushConstants;

struct Uniform {
  proj: mat4x4<f32>,
}

@group(0) @binding(0)
var<uniform> puniform: Uniform;

struct VertexInput {
  @location(0) position: vec2<f32>,
};

struct InstanceInput {
  @location(1) position: vec2<f32>,
  @location(2) velocity: vec2<f32>,
  @location(3) density: f32,
  @location(4) pressure: f32,
  @location(5) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(
  mesh: VertexInput,
  instance: InstanceInput,
) -> VertexOutput {
  var out: VertexOutput;

  let world_pos = instance.position + mesh.position * push_constants.radius;

  out.clip_position = puniform.proj * vec4<f32>(world_pos, 0.0, 1.0);
  out.color = instance.color;

  return out;
}


@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
  return vec4<f32>(in.color, 1.0);
}
