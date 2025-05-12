const std = @import("std");
const run = std.process.Child.run;

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});

    const joltc = b.dependency("joltc_zig", .{
        .target = target,
        .optimize = .ReleaseFast,
        .shared = true,
    });

    //Copies library to root directory
    b.getInstallStep().dependOn(&b.addInstallArtifact(joltc.artifact("joltc"), .{
        .dest_dir = .{ .override = .{ .custom = "../" } },
    }).step);

    const bindgen_step = b.step("bindgen", "Generate bindings");
    bindgen_step.makeFn = bindgen_fn;
}

pub fn bindgen_fn(step: *std.Build.Step, options: std.Build.Step.MakeOptions) !void {
    _ = step;
    _ = options;

    //TODO: Support for Windows bindgen.exe

    const alloc = std.heap.page_allocator;
    const cwd = std.fs.cwd();

    //Run bindgen
    const argv = [_][]const u8{ "./bindgen/bindgen.bin", "bindgen" };
    const proc = try run(.{
        .argv = &argv,
        .allocator = alloc,
    });

    defer alloc.free(proc.stdout);
    defer alloc.free(proc.stderr);

    //Copy resultant file
    try std.fs.cwd().copyFile(
        "bindgen/temp/joltc.odin",
        cwd,
        "jolt.odin",
        .{},
    );

    //Clean-up. Try not allowed in a defer...
    try cwd.deleteTree("bindgen/temp");
}
