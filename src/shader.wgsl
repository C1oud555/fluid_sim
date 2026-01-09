struct PushConstants {
  radius: f32,
  aspect_ratio: f32,
};

var<push_constant> push_constants: PushConstants;

struct VertexInput {
  @location(0) position: vec2<f32>,
};

struct InstanceInput {
  @location(1) position: vec2<f32>,
  @location(2) velocity: vec2<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(
  particle: VertexInput,
  instance: InstanceInput,
) -> VertexOutput {
  var out: VertexOutput;

  let corrected_radius_x = push_constants.radius / push_constants.aspect_ratio;
  let world_pos = vec2<f32>(
    instance.position.x + particle.position.x * corrected_radius_x,
    instance.position.y + particle.position.y * push_constants.radius,
  );

  out.clip_position = vec4<f32>(world_pos, 0.0, 1.0);
  out.color = vec3<f32>(1.0, 0.0, 0.0);

  return out;
}


@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
  return vec4<f32>(in.color, 1.0);
}
