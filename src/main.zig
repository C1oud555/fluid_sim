const std = @import("std");

const rgui = @import("raygui");
const rl = @import("raylib");
const sim = @import("fluid_sim").Simu;

pub fn main() anyerror!void {
    var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
    defer arena.deinit();

    const screenWidth = 800;
    const screenHeight = 450;

    var simu = try sim.init(arena.allocator());
    defer simu.deinit();
    simu.addParticle(.{ .x = 50, .y = 50 });

    rl.initWindow(screenWidth, screenHeight, "raylib-zig [core] example - basic window");
    defer rl.closeWindow();

    rl.setTargetFPS(60);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.white);
        simu.render();
    }
}
