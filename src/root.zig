const std = @import("std");
const rl = @import("raylib");

pub const Simu = @This();

pub const Params = struct {
    radius: f32,
};

pub const Particle = struct {
    x: i32,
    y: i32,
};

// field
params: Params,
allocator: std.mem.Allocator,
particles: std.ArrayList(Particle),

// methods
pub fn init(allocator_: std.mem.Allocator) !Simu {
    return Simu{
        .params = .{ .radius = 10.0 },
        .allocator = allocator_,
        .particles = try std.ArrayList(Particle).initCapacity(allocator_, 2),
    };
}

pub fn addParticle(self: *Simu, item: Particle) void {
    self.particles.append(self.allocator, item) catch {
        std.debug.print("push particle failed\n", .{});
    };
}

pub fn deinit(self: *Simu) void {
    self.particles.deinit(self.allocator);
}

pub fn render(self: Simu) void {
    for (self.particles.items) |particle| {
        rl.drawCircle(particle.x, particle.y, self.params.radius, .red);
    }
}
