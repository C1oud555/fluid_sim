const std = @import("std");
const z_render = @import("z_render");
const rl = @import("raylib");
const rgui = @import("raygui");

const Particle = struct {
    x: i32,
    y: i32,
};

pub fn main() anyerror!void {
    var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    const screenWidth = 800;
    const screenHeight = 450;

    var radius: f32 = 30.0;

    var particles = try std.ArrayList(Particle).initCapacity(allocator, 2);
    defer particles.deinit(allocator);
    try particles.append(allocator, .{ .x = 400, .y = 200 });

    rl.initWindow(screenWidth, screenHeight, "raylib-zig [core] example - basic window");
    defer rl.closeWindow();

    rl.setTargetFPS(60);

    const control_rect = rl.Rectangle{ .x = 10, .y = 10, .width = 50, .height = 50 };

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.white);
        _ = rgui.slider(control_rect, "", "", &radius, 10.0, 20.0);
        for (particles.items) |particle| {
            rl.drawCircle(particle.x, particle.y, radius, .red);
        }
    }
}
