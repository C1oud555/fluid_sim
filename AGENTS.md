# AGENTS.md - Development Guidelines for SPH Fluid Simulator

This document provides essential information for agentic coding agents working on this Rust-based SPH (Smoothed Particle Hydrodynamics) fluid simulator codebase.

## Project Overview

This is a real-time fluid simulation application built with Rust, featuring:
- SPH physics simulation with particle systems
- Interactive GUI using egui for parameter control
- Modern GPU rendering via wgpu and WGSL shaders
- Real-time visualization and manipulation of fluid properties

## Build Commands

### Standard Cargo Commands
```bash
# Debug build
cargo build

# Release build (optimized)
cargo build --release

# Run the simulator
cargo run

# Cross-platform Windows build
./win.sh
```

### Development Workflow
```bash
# Build and run in debug mode for development
cargo run

# Build optimized version for performance testing
cargo build --release && ./target/release/sph-sim
```

## Testing

**No automated test suite exists** - Testing is currently manual through the interactive GUI. When adding tests:

1. Use standard Rust `#[test]` attributes
2. Place unit tests in the same modules as the code they test
3. Use `cargo test` to run all tests
4. For single test: `cargo test test_name`

## Code Style Guidelines

### Naming Conventions
- **Structs/Enums**: PascalCase (`AppState`, `Particle`, `Simulation`)
- **Functions/Methods**: snake_case (`add_particle`, `update_simulation`, `render_particles`)
- **Constants**: SCREAMING_SNAKE_CASE (`TEXTURE_FORMAT`, `WINDOW_WIDTH`)
- **Modules**: snake_case (`egui_tools`, `circle_mesh`, `simulation`)
- **Variables**: snake_case (`particle_count`, `delta_time`, `gravity_force`)

### Import Organization
```rust
// Standard library first
use std::time::Instant;
use std::sync::Arc;

// External crates next
use egui::Vec2;
use egui_wgpu::wgpu::{self, util::DeviceExt};
use winit::application::ApplicationHandler;

// Local crate imports last
use crate::egui_tools::EguiRenderer;
use crate::simulation::{Particle, Simulation};
```

### Code Structure Patterns

#### Struct Definition Pattern
```rust
#[repr(C)] // For GPU-compatible structures
pub struct Particle {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub density: f32,
    pub pressure: f32,
}
// Implement bytemuck for GPU buffer casting
unsafe impl bytemuck::Pod for Particle {}
unsafe impl bytemuck::Zeroable for Particle {}
```

#### Constructor Pattern
```rust
impl Simulation {
    pub fn new(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) -> Self {
        // Initialization logic
        Self { /* fields */ }
    }
}
```

#### Error Handling Style
- Use `.expect()` for critical failures during initialization
- Use `.unwrap()` for non-critical operations where failure is unlikely
- Prefer explicit error messages that help with debugging:
  ```rust
  .expect("Failed to create wgpu device")
  .expect("Failed to compile shader")
  ```

### Memory Management Guidelines

#### GPU Buffer Patterns
```rust
// Use bytemuck for safe casting to GPU buffers
let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
    label: Some("Particle Vertex Buffer"),
    contents: bytemuck::cast_slice(&particles),
    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
});
```

#### Data Structure Alignment
- Use `#[repr(C)]` for structures sent to GPU
- Implement `bytemuck::Pod` and `bytemuck::Zeroable` for GPU-compatible types
- Ensure proper memory alignment for uniform buffers

### Graphics Programming Guidelines

#### WGSL Shader Structure
```wgsl
// Vertex shader
@vertex
fn vs_main(@location(0) position: vec2<f32>, @location(1) color: vec3<f32>) -> @builtin(position) vec4<f32> {
    // Vertex processing
}

// Fragment shader  
@fragment
fn fs_main(@location(0) color: vec3<f32>) -> @location(0) vec4<f32> {
    // Fragment processing
}
```

#### Pipeline Configuration
- Use push constants for uniform data (camera transforms, simulation parameters)
- Implement proper bind group layouts for resources
- Use instance rendering for particle systems

### GUI Integration (egui)

#### Window Management
```rust
impl ApplicationHandler for AppState {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        // Window and surface creation
    }
    
    fn window_event(&mut self, event_loop: &winit::event_loop::ActiveEventLoop, window_id: winit::window::WindowId, event: winit::event::WindowEvent) {
        // Event handling
    }
}
```

#### Control Panels
- Group related controls using `egui::CollapsingHeader` or `egui::Group`
- Use sliders for continuous parameters (`ui.add(egui::Slider::new(&mut value, min..=max))`)
- Use checkboxes for boolean toggles
- Provide immediate visual feedback for parameter changes

### Performance Considerations

#### Simulation Updates
```rust
// Use delta time for frame-rate independent physics
pub fn update(&mut self, delta_time: f32) {
    // SPH simulation logic
    self.apply_forces(delta_time);
    self.integrate(delta_time);
    self.handle_collisions();
}
```

#### Rendering Optimization
- Batch particle rendering using instanced drawing
- Minimize CPU-GPU synchronization points
- Use appropriate buffer usage flags
- Consider compute shaders for heavy simulation work

### Module Organization

#### Core Modules
- `main.rs`: Application entry point and async runtime setup
- `app.rs`: Main application state and window management (`AppState`)
- `simulation.rs`: SPH physics engine and particle system
- `egui_tools.rs`: GUI rendering and interaction utilities
- `circle_mesh.rs`: Mesh generation for particle visualization
- `shader.wgsl`: Vertex and fragment shaders for particle rendering

#### Module Dependencies
```
main.rs -> app.rs -> simulation.rs
                    -> egui_tools.rs -> wgpu
                    -> circle_mesh.rs -> wgpu
```

## Development Practices

### Adding New Features
1. Follow the existing module structure and naming conventions
2. Add GPU resources through the `Simulation::new()` constructor
3. Integrate GUI controls in the egui rendering loop
4. Update the WGSL shaders if rendering changes are needed
5. Test manually using the interactive GUI controls

### Physics Simulation Guidelines
- Use frame-rate independent physics with delta time
- Maintain particle count invariants
- Implement proper boundary conditions
- Use appropriate SPH kernel functions for density/pressure calculations

### Code Review Checklist
- [ ] Proper import organization
- [ ] Correct memory alignment for GPU structures
- [ ] Appropriate error handling with descriptive messages
- [ ] Consistent naming conventions
- [ ] GPU buffer creation follows bytemuck patterns
- [ ] GUI controls provide immediate feedback
- [ ] Performance considerations for particle systems

## Common Issues and Solutions

### Shader Compilation Failures
- Check WGSL syntax validity
- Ensure struct layouts match between Rust and WGSL
- Verify all bound resources are properly declared

### Performance Degradation
- Profile GPU rendering with appropriate tools
- Check for unnecessary buffer copies between CPU and GPU
- Optimize particle count vs. quality tradeoffs
- Consider compute shaders for heavy simulation work

### GUI Integration Issues
- Ensure proper event forwarding between winit and egui
- Handle window resize events for surface recreation
- Maintain proper state synchronization between GUI and simulation